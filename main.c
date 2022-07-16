

#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>

#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

//simulation end time
double simend = 15;

//related to writing data to a file
FILE *fid;
int loop_index = 0;
const int data_frequency = 10; //frequency at which data is written to a file


// char xmlpath[] = "../myproject/template_writeData/pendulum.xml";
// char datapath[] = "../myproject/template_writeData/data.csv";


//Change the path <template_writeData>
//Change the xml file
char path[] = "../myproject/penddulum/";
char xmlfile[] = "doublependulum.xml";


char datafile[] = "data.csv";


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


//****************************
//This function is called once and is used to get the headers
void init_save_data()
{
  //write name of the variable here (header)
   fprintf(fid,"t, ");
   

   //Don't remove the newline
   fprintf(fid,"\n");
}

//***************************
//This function is called at a set frequency, put data here
void save_data(const mjModel* m, mjData* d)
{
  //data here should correspond to headers in init_save_data()
  //seperate data by a space %f followed by space
  fprintf(fid,"%f ,",d->time);

  fprintf(fid,"%f,%f",d->qpos[2],d->qpos[3]);

  //Don't remove the newline
  fprintf(fid,"\n");
}

//**************************
void mycontroller(const mjModel* m, mjData* d)
{
  //write control here
mj_energyPos(m,d);
mj_energyVel(m,d);
// This deals with the mass matrix of the system 
/* const int nv = 2;
double dense_M[16] = {0};
mj_fullM(m, dense_M, d->qM);
double M[4][4] = {0};
M[0][0]=dense_M[0];
M[0][1]=dense_M[1];
M[0][2]=dense_M[2];
M[0][3]=dense_M[3];

M[1][0]=dense_M[4];
M[1][1]=dense_M[5];
M[1][2]=dense_M[6];
M[1][3]=dense_M[7];

M[2][0]=dense_M[8];
M[2][1]=dense_M[9];
M[2][2]=dense_M[10];
M[2][3]=dense_M[11];

M[3][0]=dense_M[12];
M[3][1]=dense_M[13];
M[3][2]=dense_M[14];
M[3][3]=dense_M[15];
printf("\n%f	%f	 %f	%f   %f\n",d->time,M[0][0],M[0][1],M[0][2],M[0][3]);
printf("\n%f	%f	 %f	%f   %f\n",d->time,M[1][0],M[1][1],M[1][2],M[1][3]);
printf("\n%f	%f	 %f	%f   %f\n",d->time,M[2][0],M[2][1],M[2][2],M[2][3]);
printf("\n%f	%f	 %f	%f   %f\n*********************",d->time,M[3][0],M[3][1],M[3][2],M[3][3]);  */
//printf("**********************\n",d->time,M[0][0],M[0][1],M[1][0],M[1][1]);
//printf("%f  %f   %f \n",m->body_inertia[6],m->body_inertia[7],m->body_inertia[8]);
//printf("%d\n",m->nq);//get the number of qs 
//
/* mj_inverse(m,d);
printf("%f  %f  %f   %f\n",d-> qfrc_inverse[0],d-> qfrc_inverse[1],d-> qfrc_inverse[2],d-> qfrc_inverse[3]); */

/* mj_fwdPosition(m, d);
mj_comPos(m,d);
double Cen[3][6] = {0};
Cen[0][0] = d->cdof[12];
Cen[0][1] = d->cdof[13];
Cen[0][2] = d->cdof[14];
Cen[0][3] = d->cdof[15];
Cen[0][4] = d->cdof[16];
Cen[0][5] = d->cdof[17];
printf("%f  %f %f %f  %f %f \n",Cen[0][0] ,Cen[0][1] ,Cen[0][2] ,Cen[0][3] ,Cen[0][4] ,Cen[0][5] ); */


//get the states of the system
//printf(" positions %f   %f  %f %f %f \n",d->time,d->qpos[0],d->qpos[1],d->qpos[2],d->qpos[3],d->qpos[3]);
//printf(" velocities %f   %f  %f %f %f \n",d->time,d->qpos[0],d->qpos[1],d->qpos[2],d->qpos[3],d->qpos[3]);
//printf("%f %f %f \n%f %f %f \n%**********************\n",m->body_inertia[3],m->body_inertia[4],m->body_inertia[5],m->body_mass[6],m->body_mass[7],m->body_mass[8]);

//Control The robot when theta1 and theat2 are zeros 
//Kx 
//0                          ,1,2                            ,3,                 
//double k1x = 31.6228*d->qpos[0]+0+78.6222*d->qpos[2]+ 0 + 40.8058*d->qpos[4]+0.0000 +   1.7148 * d->qpos[6] +0*d->qpos[7];
//double k2x = 0*d->qpos[0]+31.6228*d->qpos[1]+0*d->qpos[2]+ -107.7085 *d->qpos[3]+ 0*d->qpos[4]+ 43.0392*d->qpos[5]+   0* d->qpos[6] +-13.0516*d->qpos[7];
double x = d->qpos[0];
double y = d->qpos[1];
double theta1 = d->qpos[2];
double theta2 = d->qpos[3];
//
double xdot = d->qvel[0];
double ydot = d->qvel[1];
double theta1dot = d->qvel[2];
double theta2dot = d->qvel[3];

double k1x = 31.6228*x+0+78.6222*theta1+ 0 + 40.8058*xdot +0.0000 +   1.7148 * theta1dot +0*theta2dot;
double k2x = 0*x+31.6228*y+0*theta1+ -107.7085 *theta2+ 0*xdot+ 43.0392*ydot+   0*theta1dot+-13.0516*theta2dot;
//double k1x = 31.6228*d->qpos[0]+0+78.6222*d->qpos[2]+ 0 + 40.8058*d->qpos[4]+0.0000 +   1.7148 * d->qvel[6] +0*d->qpos[7];
//double k2x = 0*d->qpos[0]+31.6228*d->qpos[1]+0*d->qpos[2]+ -107.7085 *d->qpos[3]+ 0*d->qpos[4]+ 43.0392*d->qpos[5]+   0* d->qpos[6] +-13.0516*d->qpos[7];
//double k1x  =  14.142135623731*d->qpos[0] + 28.8427997853313*d->qpos[3] + 18.8114452251434*d->qvel[0] +1.18596249141894*d->qvel[3]; 
//double k2x  =  14.142135623731*d->qpos[1] + -42.330533 *d->qpos[4] +19.818133 *d->qvel[1]  + -7.0204 *d->qvel[4] ;



double qref_0 = 0;
double qref_1 = 0;


double e1 = qref_0-k1x;
double e2 = qref_1-k2x;

double force_max = 80;

if(e1>force_max){
	e1 = force_max;
}
if(e1<-force_max){
	e1 = -force_max;
}


if(e2>force_max){
	e2 = force_max;
}
if(e2<-force_max){
	e2 = -force_max;
}

d->qfrc_applied[0] = e1;
d->qfrc_applied[1] = e2;
//printf(" positions %f [s]  %f  %f %f %f %f  %d \n",d->time,d->qpos[0],d->qpos[1],d->qpos[2],d->qpos[3],d->qpos[4],m->nq);
//printf(" velocities %f [s]  %f  %f %f %f %f\n",d->time,d->qvel[0],d->qvel[1],d->qvel[2],d->qvel[3],d->qvel[4]);
//double qref_0  = .1*mjPI;
//double qref_1 =  .2*mjPI;

//1.0000 0.0000 -56.6959 0.0000 -2.7495 0.0000 -8.8824 -0.0000
//-0.0000 1.0000 0.0000 0.7521 -0.0000 2.4806 -0.0000 0.5268
/* double k1x = 1*d->qpos[0]+0*d->qpos[1]+-56.6959*d->qpos[2]+ 0.0 *d->qpos[3]+ -2.7495*d->qpos[4]+0.0000*d->qpos[5]+   -8.8824* d->qpos[6] +-0.0000*d->qpos[7];
//double k2x = 0*d->qpos[0]+1*d->qpos[1]+0*d->qpos[2]+ 0.7521*d->qpos[3]+ 0*d->qpos[4]+ 2.4806*d->qpos[5]+   0* d->qpos[6] +0.5268*d->qpos[7];


double e1 = qref_0-k1x;
double e2 = qref_1-k2x;

double force_max = 80;

if(e1>force_max){
	e1 = force_max;
}
if(e1<=-force_max){
	e1 = -force_max;
}


if(e2>=force_max){
	e2 = force_max;
}
if(e2<-force_max){
	e2 = -force_max;
}
d->qfrc_applied[0] = e1;
d->qfrc_applied[1] = e2; */
//printf("Control Signals are %f [n]    %f [n]\n",e1,e2);

  //write data here (dont change/dete this function call; instead write what you need to save in save_data)
  if ( loop_index%data_frequency==0)
    {
      save_data(m,d);
    }
  loop_index = loop_index + 1;
}


//************************
// main function
int main(int argc, const char** argv)
{

    // activate software
    mj_activate("mjkey.txt");

    char xmlpath[100]={};
    char datapath[100]={};

    strcat(xmlpath,path);
    strcat(xmlpath,xmlfile);

    strcat(datapath,path);
    strcat(datapath,datafile);


    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(xmlpath, 0, error, 1000);

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

    double arr_view[] = {89.608063, -11.588379, 5, 0.000000, 0.000000, 1.000000};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    // install control callback
    mjcb_control = mycontroller;

    fid = fopen(datapath,"w");
    init_save_data();
	d->qpos[0] =  0;//0.0555555556*10;x
	d->qpos[1] = 0;// 0.0555555556*10;y
	d->qpos[2]  = .5;//theta1
	d->qpos[3] = 1;//theta 2
	//d->qpos[4]  = .0;//theta1
	//d->qpos[5]  = .2;
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
        }

        if (d->time>=simend)
        {
           fclose(fid);
           break;
         }

       // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

          // update scene and render
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

    return 1;
}
