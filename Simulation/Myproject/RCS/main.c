

#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>

#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"


char filename[] = "../Myproject/RCS/box.xml";
// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}
void getRotationAngles(mjModel* model, mjData* data, const char* body_name, double angles[3]) {
    // Znajdź indeks ciała na podstawie jego nazwy
    int body_id = mj_name2id(model, mjOBJ_BODY, body_name);

    // Symulacja
    mj_forward(model, data);

    // Quaternion reprezentujący rotację ciała
    double quaternion[4];
    for (int i = 0; i < 4; i++) {
        quaternion[i] = data->xquat[4 * body_id + i];
    }

    // Oblicz macierz rotacji z quaterniona
    double rotation_matrix[9];
    mju_quat2Mat(rotation_matrix, quaternion);

    // Oblicz kąty obrotu w osiach x, y i z
    angles[0] = atan2(rotation_matrix[7], rotation_matrix[8]); // Oś X
    angles[1] = atan2(-rotation_matrix[2], sqrt(rotation_matrix[5] * rotation_matrix[5] + rotation_matrix[8] * rotation_matrix[8])); // Oś Y
    angles[2] = atan2(rotation_matrix[3], rotation_matrix[0]); // Oś Z

    // Przelicz kąty z radianów na stopnie
    for (int i = 0; i < 3; i++) {
        angles[i] = angles[i] * 180.0 / M_PI;           
    }
}
void rotateBodyAroundXAxis(mjModel* model, mjData* data, const char* body_name, double rotation_angle) {
    // Oblicz kwaternion reprezentujący obrót wokół osi X
    double cos_half_angle = cos(rotation_angle / 2);
    double sin_half_angle = sin(rotation_angle / 2);
    double quaternion[4] = {cos_half_angle, sin_half_angle, 0.0, 0.0};

    // Znajdź indeks ciała na podstawie jego nazwy
    int body_id = mj_name2id(model, mjOBJ_BODY, body_name);

    // Ustaw kwaternion w danych
    for (int i = 0; i < 2; i++) {
        data->xquat[4 * body_id + i] = quaternion[i];
    }
}


// main function
int main(int argc, const char** argv)
{

    // activate software
    mj_activate("mjkey.txt");
    int x = 0;
    int y = 0;
    double angles[3];
    double endTime;
    FILE *fptr;
    fptr = fopen("../Myproject/RCS/data.txt", "w");

    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(filename, 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);


    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
    // use the first while condition if you want to simulate for a period.
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;  
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);
            d->qfrc_applied[2] = 3;
            if(d->time < 5) {
                if(angles[0] < 2) d->qfrc_applied[3] = 0.04;
                else d->qfrc_applied[3] = -0.04;
            }
            else{
                if(angles[0] > 0) d->qfrc_applied[3] = -0.01;
                else d->qfrc_applied[3] = 0.01;
            }
           // else if(angles[0] > 0) d->qfrc_applied[3] = -0.001;
            //else d->qfrc_applied[3] = 0.001;
             /* if(d->time > 5){
                if(angles[0] > 0.01) d->qfrc_applied[3] = -0.01;
                if(angles[0] < -0.01) d->qfrc_applied[3] = 0.01;
                else d->qfrc_applied[3] = 0; 
            }
            */
            getRotationAngles(m, d, "parachute", angles);
            fprintf(fptr, "%f, %f, %f;\n",d->time,angles[0],angles[2]);
             printf("%f, %f, %f;\n",d->time,angles[0],angles[2]);
             if(x == 10){
             d->qvel[1] = sin(angles[2] * (M_PI / 180));
             d->qvel[0] = cos(angles[2]* (M_PI / 180));
             d->qvel[7] = 0.9*sin(angles[2] * (M_PI / 180));
             d->qvel[6] = 0.9*cos(angles[2]* (M_PI / 180));
             x = 0;
             }
             x++;
        }

       // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

          // update scene and render
        cam.lookat[0] = d->qpos[0];
        cam.lookat[1] = d->qpos[1];
        cam.lookat[2] = d->qpos[2];
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif  
    fclose(fptr);

    return 1;
}

// czas trwania jednego kroku symulacji to 0,002 sekundy !!!!
