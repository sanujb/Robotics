/****************************************************************/
/** xrobot.c: simulates and renders mobile manipulator         **/
/**           version of Roger-the-Crab                        **/
/** author:   Grupen                                           **/
/** date:     April, 2010                                      **/
/****************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "Xkw/Xkw.h"

#include "include/roger.h"
#include "include/simulate.h"
#include "include/control.h"
#include "include/modes.h"
#include "include/matrix_math.h"

// simulator data structures that comprise Roger
Base mobile_base;
extern Base mobile_base_home;
Eye eyes[NEYES];
extern Eye eyes_home[NEYES];
Arm arms[NARMS][NARM_FRAMES];
extern Arm arms_home[NARMS][NARM_FRAMES];
PolyBall objects[NBODY];  // NBODY=5: BASE, ARM1, ARM2, OBJECT, occupancy grid

// Roger + room model + one inertial object "toy"
PolyBall toy;                  // assigned either CIRCLE || TRIANGLE 
extern PolyBall toy_home;   // just the red toy (ball|triangle)

Robot Roger; // the user application interface data structure

SimColor world_colors[113];

int draw_visual = FALSE;

History history[MAX_HISTORY]; // a diagnostic tool to illustrate a trajectory
int history_ptr=0;

int init_control_flag = TRUE;

// global Boolean reset flag - eliminates user defined obstacles and goals
// reconfigures Roger and single object to starting configuration
int reset;

// global simulator clock
double simtime = 0.0;

void x_canvas_proc(), x_start_proc(), x_params_proc(), x_input_mode_proc();
void x_control_mode_proc(), x_quit_proc(), x_timer_proc(), x_visualize_proc();
void x_room_proc();

void project1_enter_params(), project1_reset(),
  project1_visualize(), project1_control();
void project2_enter_params(), project2_reset(), 
  project2_visualize(), project2_control();
void project3_enter_params(), project3_reset(), 
  project3_visualize(), project3_control();
void project4_enter_params(), project4_reset(), 
  project4_visualize(), project4_control();
void project5_enter_params(), project5_reset(),
  project5_visualize(), project5_control();
void project6_enter_params(), project6_reset(), 
  project6_visualize(), project6_control();
void project7_enter_params(), project7_reset(), 
  project7_visualize(), project7_control();
void project8_enter_params(), project8_reset(), 
  project8_visualize(), project8_control();
void project9_enter_params(), project9_reset(), 
  project9_visualize(), project9_control();
void project10_enter_params(), project10_reset(), 
  project10_visualize(), project10_control();
void project11_enter_params(), project11_reset(), 
  project11_visualize(), project11_control();

void HT_invert(), construct_wTb();

void control_roger(), simulate_roger(), simulate_object();
//void simulate_base(), simulate_arm(), simulate_eyes(), simulate_object(), 
void inv_arm_kinematics();

Display          *display;
Window           window;
Pixmap           pixmap;
XtAppContext     app_con;
GC               gc;
int              screen;
Widget           canvas_w, input_mode_w, control_mode_w, params_w, start_w, popup_w, room_w = NULL;
Widget           intensity_w, stream_w;
XtIntervalId     timer = 0;
int              width = WIDTH, height = HEIGHT, depth;
unsigned long    foreground, background;

int zoom = ZOOM_SCALE; 

void x_init_colors() 
{
  int i;

  //  printf("initializing grey scale colors..."); fflush(stdout);
  for (i = 0; i <= 100; i++) { // 0 => black; 100 => white
    sprintf(world_colors[i].name, "grey%d", i);
    world_colors[i].display_color=XkwParseColor(display,world_colors[i].name);
    world_colors[i].red = (int) i*2.55;
    world_colors[i].green = (int) i*2.55;
    world_colors[i].blue = (int) i*2.55;
  }
  strcpy(world_colors[101].name, "dark red");
  world_colors[101].display_color = XkwParseColor(display, "dark red");
  world_colors[101].red = 139;
  world_colors[101].green = 0;
  world_colors[101].blue = 0;

  strcpy(world_colors[102].name, "red");
  world_colors[102].display_color = XkwParseColor(display, "red");
  world_colors[102].red = 255;
  world_colors[102].green = 0;
  world_colors[102].blue = 0;

  strcpy(world_colors[103].name, "hot pink");
  world_colors[103].display_color = XkwParseColor(display, "hot pink");
  world_colors[103].red = 255;
  world_colors[103].green = 105;
  world_colors[103].blue = 180;

  strcpy(world_colors[104].name, "navy");
  world_colors[104].display_color = XkwParseColor(display, "navy");
  world_colors[104].red = 65;
  world_colors[104].green = 105;
  world_colors[104].blue = 225;

  strcpy(world_colors[105].name, "blue");
  world_colors[105].display_color = XkwParseColor(display, "blue");
  world_colors[105].red = 0;
  world_colors[105].green = 0;
  world_colors[105].blue = 255;

  strcpy(world_colors[106].name, "light sky blue");
  world_colors[106].display_color = XkwParseColor(display, "light sky blue");
  world_colors[106].red = 250;
  world_colors[106].green = 128;
  world_colors[106].blue = 114;

  strcpy(world_colors[107].name, "dark green");
  world_colors[107].display_color = XkwParseColor(display, "dark green");
  world_colors[107].red = 244;
  world_colors[107].green = 164;
  world_colors[107].blue = 96;

  strcpy(world_colors[108].name, "green");
  world_colors[108].display_color = XkwParseColor(display, "green");
  world_colors[108].red = 0;
  world_colors[108].green = 255;
  world_colors[108].blue = 0;

  strcpy(world_colors[109].name, "light green");
  world_colors[109].display_color = XkwParseColor(display, "light green");
  world_colors[109].red = 46;
  world_colors[109].green = 139;
  world_colors[109].blue = 87;

  strcpy(world_colors[110].name, "gold");
  world_colors[110].display_color = XkwParseColor(display, "gold");
  world_colors[110].red = 160;
  world_colors[110].green = 82;
  world_colors[110].blue = 45;

  strcpy(world_colors[111].name, "yellow");
  world_colors[111].display_color = XkwParseColor(display, "yellow");
  world_colors[111].red = 255;
  world_colors[111].green = 255;
  world_colors[111].blue = 0;

  strcpy(world_colors[112].name, "light goldenrod");
  world_colors[112].display_color = XkwParseColor(display, "light goldenrod");
  world_colors[112].red = 192;
  world_colors[112].green = 192;
  world_colors[112].blue = 192;
}

void write_interface(reset)
int reset;
{
  int i,j;

  // pass in afferents (read only)
  Roger.eye_theta[0] = eyes[0].theta;
  Roger.eye_theta_dot[0] = eyes[0].theta_dot;
  Roger.eye_theta[1] = eyes[1].theta;
  Roger.eye_theta_dot[1] = eyes[1].theta_dot;
  for (i=0;i<NPIXELS;++i) {
    Roger.image[LEFT][i][RED_CHANNEL] = world_colors[eyes[LEFT].image[i]].red;
    Roger.image[LEFT][i][GREEN_CHANNEL] =
      world_colors[eyes[LEFT].image[i]].green;
    Roger.image[LEFT][i][BLUE_CHANNEL] = 
      world_colors[eyes[LEFT].image[i]].blue;
    Roger.image[RIGHT][i][RED_CHANNEL] = 
      world_colors[eyes[RIGHT].image[i]].red;
    Roger.image[RIGHT][i][GREEN_CHANNEL] = 
      world_colors[eyes[RIGHT].image[i]].green;
    Roger.image[RIGHT][i][BLUE_CHANNEL] = 
      world_colors[eyes[RIGHT].image[i]].blue;
  }
  Roger.arm_theta[0][0] = arms[0][1].theta;
  Roger.arm_theta[0][1] = arms[0][2].theta;
  Roger.arm_theta[1][0] = arms[1][1].theta;
  Roger.arm_theta[1][1] = arms[1][2].theta;
  Roger.arm_theta_dot[0][0] = arms[0][1].theta_dot;
  Roger.arm_theta_dot[0][1] = arms[0][2].theta_dot;
  Roger.arm_theta_dot[1][0] = arms[1][1].theta_dot;
  Roger.arm_theta_dot[1][1] = arms[1][2].theta_dot;
  Roger.ext_force[0][X] = arms[0][NARM_FRAMES - 1].extForce[X];
  Roger.ext_force[0][Y] = arms[0][NARM_FRAMES - 1].extForce[Y];
  Roger.ext_force[1][X] = arms[1][NARM_FRAMES - 1].extForce[X];
  Roger.ext_force[1][Y] = arms[1][NARM_FRAMES - 1].extForce[Y];

  Roger.base_position[0] = mobile_base.x;
  Roger.base_position[1] = mobile_base.y;
  Roger.base_position[2] = mobile_base.theta;
  Roger.base_velocity[0] = mobile_base.x_dot;
  Roger.base_velocity[1] = mobile_base.y_dot;
  Roger.base_velocity[2] = mobile_base.theta_dot;
  Roger.wheel_theta_dot[LEFT] = mobile_base.wheel_theta_dot[LEFT];
  Roger.wheel_theta_dot[RIGHT] = mobile_base.wheel_theta_dot[RIGHT];

  // zero efferents (write only)
  Roger.eye_torque[0] = Roger.eye_torque[1] = 0.0;
  Roger.arm_torque[0][0] = Roger.arm_torque[0][1] = Roger.arm_torque[1][0]
    = Roger.arm_torque[1][1] = 0.0;
  Roger.wheel_torque[0] = Roger.wheel_torque[1] = 0.0;
}  

void read_interface()
{
  double motor_model();

  // pass back torques (write only)
  arms[0][1].torque = 
    motor_model(Roger.arm_torque[0][0], Roger.arm_theta_dot[0][0],
		SHOULDER_TS, SHOULDER_W0);
  arms[1][1].torque = 
    motor_model(Roger.arm_torque[1][0], Roger.arm_theta_dot[1][0],
		SHOULDER_TS, SHOULDER_W0);
  arms[0][2].torque = 
    motor_model(Roger.arm_torque[0][1], Roger.arm_theta_dot[0][1],
		ELBOW_TS, ELBOW_W0);
  arms[1][2].torque = 
    motor_model(Roger.arm_torque[1][1], Roger.arm_theta_dot[1][1],
		ELBOW_TS, ELBOW_W0);

  eyes[0].torque = 
    motor_model(Roger.eye_torque[0], Roger.eye_theta_dot[0], EYE_TS, EYE_W0);
  eyes[1].torque = 
    motor_model(Roger.eye_torque[1], Roger.eye_theta_dot[1], EYE_TS, EYE_W0);

  mobile_base.wheel_torque[0] = 
    motor_model(Roger.wheel_torque[0], Roger.wheel_theta_dot[0],
		WHEEL_TS, WHEEL_W0);
  mobile_base.wheel_torque[1] = 
    motor_model(Roger.wheel_torque[1], Roger.wheel_theta_dot[1],
		WHEEL_TS, WHEEL_W0);
}

double motor_model(tau, omega, tau_s, omega_0)
double tau, omega, tau_s, omega_0;
{
  int i;
  double tau_max, tau_min;

  if(omega >= 0) { // motor velocity positive
    tau_min = -tau_s;
    tau_max = tau_s - (tau_s/omega_0)*omega;
  }
  else { // motor velocity negative
    tau_min = -tau_s - (tau_s/omega_0)*omega;
    tau_max = tau_s;
  }
  if(tau < tau_min) tau = tau_min;
  if(tau > tau_max) tau = tau_max;

  return(tau);
}

void initialize_simulator(rst) 
int rst;
{
  void initialize_roger(), initialize_inertial_objects();

  if (rst) {
    initialize_roger(); // assign "home" values into configuration variables 
    initialize_inertial_objects(); // all params and data structures needed
                                   // to compute collision forces between 
                                   // base, hands, object (circle||"triangle")
                                   // and OBSTACLEs in the occupancy grid.
    // reset room geometry as well? to get rid of user inputs and
    // become Room 0 again?

    // reset the applications interface for user control code
    write_interface(rst);
  }
}

void initialize_roger()
{
  int i,j,k,l;

  /************************************************************************/
  // MOBILE BASE
  for (i=0;i<4;++i){
    for (j=0;j<4;++j) {
      mobile_base.wTb[i][j] = mobile_base_home.wTb[i][j];
    }
  }
  mobile_base.x = mobile_base_home.x;
  mobile_base.x_dot = mobile_base_home.x_dot;
  mobile_base.y = mobile_base_home.y;
  mobile_base.y_dot = mobile_base_home.y_dot;
  mobile_base.theta = mobile_base_home.theta;
  mobile_base.theta_dot = mobile_base_home.theta_dot;
  for (i=0;i<2;++i) {
    mobile_base.wheel_torque[i] = mobile_base_home.wheel_torque[i];
  }
  mobile_base.contact_theta = mobile_base_home.contact_theta;
  for (i=0;i<2;++i) {
    mobile_base.extForce[i] = mobile_base_home.extForce[i];
  }
  
  /************************************************************************/
  // LEFT AND RIGHT EYE
  for (i = LEFT; i <= RIGHT; ++i) {
    eyes[i].position[X] = eyes_home[i].position[X];
    eyes[i].position[Y] = eyes_home[i].position[Y];
    eyes[i].theta = eyes_home[i].theta;
    eyes[i].theta_dot = eyes[i].theta_dot;
    for (j=0; j<NPIXELS; ++j) {
      eyes[i].image[j] = eyes_home[i].image[j];
    }
    eyes[i].torque = eyes_home[i].torque;
  }

  /************************************************************************/
  // LEFT AND RIGHT ARM
  for (i=0;i<NARMS;++i) {
    for (j=0;j<NARM_FRAMES;++j) {
      for (k=0;k<4;++k) {
	for (l=0;l<4;++l) {
	  arms[i][j].iTj[k][l] = arms_home[i][j].iTj[k][l];
	}
      }
      arms[i][j].dof_type = arms_home[i][j].dof_type;
      arms[i][j].axis = arms_home[i][j].axis;
      arms[i][j].theta = arms_home[i][j].theta;
      arms[i][j].theta_dot = arms_home[i][j].theta_dot;
      arms[i][j].torque = arms_home[i][j].torque;
      arms[i][j].extForce[0] = arms_home[i][j].extForce[0];
      arms[i][j].extForce[1] = arms_home[i][j].extForce[1];
    }
  }
}

// define geometrical and inertial parameters for the NBODY dynamical system
void initialize_inertial_objects()
{
  double pb[4], pw[4]; // homogeneous position vectors in base and world coords
  double vb[4], vw[4]; // homogeneous velocity vectors in base and world coords
  double x, y, J[2][2];

  void SIMfwd_kinematics(), SIMarm_Jacobian();


  // initialize an array "PolyBall objects[NBODY]" that support computing 
  //    collision forces between "bodies" which are rigid body arrangements
  //    of elastic circular objects moving in the Cartesian world plane
  //  typedef struct _polyball {
  //    int id;                // CIRCLE || TRIANGLE
  //    int N;                 // a  composite of N elastic spheres
  //    double Rsphere;        // the radius of the elastic spheres
  //    double radius;         // distance from sphere center to body frame
  //    double mass;           // total mass of polyball
  //    double total_moment;   // moment of inertia (mass*SQR(radius) )
  //    double position[3];    // position (x,y,theta) of the object
  //    double velocity[3];    // velocity of the object
  //    double net_extForce[3]; // from collisions with other objects
  //  } PolyBall;

  /****************************** 0: BASE ************************************/
     objects[BASE].N = 1;
     objects[BASE].Rsphere = R_BASE;
     objects[BASE].radius = 0.0;
     objects[BASE].mass = M_BASE;
     objects[BASE].moi = I_BASE;

     objects[BASE].position[X] = mobile_base.x; 
     objects[BASE].position[Y] = mobile_base.y; 
     objects[BASE].position[THETA] = mobile_base.theta;

     objects[BASE].velocity[X] = mobile_base.x_dot;
     objects[BASE].velocity[Y] = mobile_base.y_dot; 
     objects[BASE].velocity[THETA] = mobile_base.theta_dot;

     objects[BASE].net_extForce[X] = objects[BASE].net_extForce[Y] =
       objects[BASE].net_extForce[THETA] = 0.0;

  /************************** 1: LEFT ARM ************************************/
  // left hand position in world coordinates
     SIMfwd_kinematics(LEFT, arms[LEFT][1].theta, arms[LEFT][2].theta, &x, &y);
     pb[0] = x; pb[1] = y; pb[2] = 0.0; pb[3] = 1.0;
     matrix_mult(4, 4, mobile_base.wTb, 1, pb, pw);

  // left hand velocity relative to base written in base coordinates
  //          (figure 2 in RogerDynamics document defines frames)
  // ^w(v)_10 = wRb[ ^b(v)_7 +  J_arm theta_dot_arm ]
  //    = ^w(v)_b + wRb[ -ARM_OFFSET omega_0 xhat_b + J_arm theta_dot_arm ]
     SIMarm_Jacobian(arms[LEFT][1].theta, arms[LEFT][2].theta, J);
     vb[X] = J[0][0]*arms[LEFT][1].theta_dot + 
       J[0][1]*arms[LEFT][2].theta_dot - ARM_OFFSET*mobile_base.theta_dot;
     vb[Y] = J[1][0]*arms[LEFT][1].theta_dot + J[1][1]*arms[LEFT][2].theta_dot;
     vb[2] = 0.0;
     vb[3] = 0.0; // homogeneous vector

     matrix_mult(4, 4, mobile_base.wTb, 1, vb, vw);
  //  v[1][X] = base->x_dot + v_w[X];
  //  v[1][Y] = base->y_dot + v_w[Y];
  //  R[1] = R_TACTILE;

     objects[ARM1].N = 1;
     objects[ARM1].Rsphere = R_TACTILE;
     objects[ARM1].radius = 0.0;
     objects[ARM1].mass = M_ARM1;
     objects[ARM1].moi = I_ARM1;
     
     objects[ARM1].position[X] = pw[X];
     objects[ARM1].position[Y] = pw[Y];
     objects[ARM1].position[THETA] = 0.0;
     // hand orientation is not relevant to system dynamics

     objects[ARM1].velocity[X] = mobile_base.x_dot + vw[X];
     objects[ARM1].velocity[Y] = mobile_base.y_dot + vw[Y]; 
     objects[ARM1].velocity[THETA] = 0.0; 
     // angular acceleration in the hand is not relevant to system dynamics

     objects[ARM1].net_extForce[X] = objects[ARM1].net_extForce[Y] =
       objects[ARM1].net_extForce[THETA] = 0.0;

  /************************** 2: RIGHT ARM ************************************/
  // right hand position in world coordinates
     SIMfwd_kinematics(RIGHT,arms[RIGHT][1].theta,arms[RIGHT][2].theta, &x,&y);
     pb[0] = x; pb[1] = y; pb[2] = 0.0; pb[3] = 1.0;
     matrix_mult(4, 4, mobile_base.wTb, 1, pb, pw);

  // left hand velocity relative to base written in base coordinates
  //          (figure 2 in RogerDynamics document defines frames)
  // ^w(v)_10 = wRb[ ^b(v)_7 +  J_arm theta_dot_arm ]
  //    = ^w(v)_b + wRb[ -ARM_OFFSET omega_0 xhat_b + J_arm theta_dot_arm ]
     SIMarm_Jacobian(arms[RIGHT][1].theta, arms[RIGHT][2].theta, J);
     vb[X] = J[0][0]*arms[RIGHT][1].theta_dot + 
       J[0][1]*arms[RIGHT][2].theta_dot - ARM_OFFSET*mobile_base.theta_dot;
     vb[Y] = J[1][0]*arms[RIGHT][1].theta_dot+J[1][1]*arms[RIGHT][2].theta_dot;
     vb[2] = 0.0;
     vb[3] = 0.0; // homogeneous vector

     matrix_mult(4, 4, mobile_base.wTb, 1, vb, vw);

     objects[ARM2].N = 1;
     objects[ARM2].Rsphere = R_TACTILE;
     objects[ARM2].radius = 0.0;
     objects[ARM2].mass = M_ARM1;
     objects[ARM2].moi = I_ARM1;

     objects[ARM2].position[X] = pw[X];
     objects[ARM2].position[Y] = pw[Y];
     objects[ARM2].position[THETA] = 0.0; 
     // hand orientation not relevant to system dynamics

     objects[ARM2].velocity[X] = mobile_base.x_dot + vw[X];
     objects[ARM2].velocity[Y] = mobile_base.y_dot + vw[Y]; 
     objects[ARM2].velocity[THETA] = 0.0;
     // angular acceleration of hand is not relevant to system dynamics

     objects[ARM2].net_extForce[X] = objects[ARM2].net_extForce[Y] =
       objects[ARM2].net_extForce[THETA] = 0.0;

  /****************** 3: TOY OBJECT - CIRCLE || TRIANGLE *********************/
     objects[TOY].id = toy.id = toy_home.id;
     objects[TOY].N = toy.N = toy_home.N;
     objects[TOY].Rsphere = toy.Rsphere = toy_home.Rsphere;
     objects[TOY].radius = toy.radius = toy_home.radius;
     objects[TOY].mass = toy.mass = toy_home.mass;
     objects[TOY].moi = toy.moi = toy_home.moi;

     objects[TOY].position[X] = toy.position[X] = toy_home.position[X]; 
     objects[TOY].position[Y] = toy.position[Y] = toy_home.position[Y];
     objects[TOY].position[THETA]=toy.position[THETA]=toy_home.position[THETA];

     objects[TOY].velocity[X] = toy.velocity[X] = toy_home.velocity[X];
     objects[TOY].velocity[Y] = toy.velocity[Y] = toy_home.velocity[Y];
     objects[TOY].velocity[THETA]=toy.velocity[THETA]=toy_home.velocity[THETA];

     objects[TOY].net_extForce[X] = objects[TOY].net_extForce[Y] =
       objects[TOY].net_extForce[THETA] = 0.0;
}

// #ifndef MAX_RAND
// #define MAX_RAND   2147483647.0
// #endif

#define R_GOAL 0.45

void initialize_random_object()
{
  // ORIGINAL (SINGLE) OBJECT
  toy.id = CIRCLE;
  toy.mass = toy_home.mass;
  toy.moi = toy_home.moi;
  // srand((unsigned int)(time(NULL)));
  double N1 = ((double)(rand() % 1000 + 1))/1000.0;
  double N2 = ((double)(rand() % 1000 + 1))/1000.0;

  double dx = MAX_X - MIN_X - 2.0*XDELTA - 2.0*R_BALL;
  double dy = MAX_Y - MIN_Y - 2.0*YDELTA - 2.0*R_BALL - R_GOAL;

  toy.position[X] = (MIN_X + XDELTA + R_BALL) + N1*dx;
  toy.position[Y] = (MIN_Y + YDELTA + R_BALL + R_GOAL) + N2*dy;
  toy.position[THETA] = 0.0;

  // printf("%lf %lf %lf %lf\n",
  //        N1, N2, object.position[X], object.position[Y]);

  toy.velocity[X] = toy.velocity[Y] = toy.velocity[THETA] = 0.0;
  toy.net_extForce[X] = toy.net_extForce[Y] = toy.net_extForce[THETA] = 0.0;
}

void place_object(x,y,id)
double x,y;
int id;
{
  if (id==CIRCLE) {
    toy.id = id;
    toy.N = 1;
    toy.Rsphere = R_BALL;;
    toy.radius = 0.0;
    toy.mass = M_BALL;
    toy.moi = I_BALL;
    toy.position[X] = x;
    toy.position[Y] = y;
    toy.position[THETA] = 0.0;
    toy.velocity[X] = toy.velocity[Y] = toy.velocity[THETA] = 0.0;
    toy.net_extForce[X] = toy.net_extForce[Y] = toy.net_extForce[THETA] = 0.0;
  }
  else if (id==TRIANGLE) {
    toy.id = id;
    toy.N = 3;
    toy.Rsphere = 0.1; //R_VERTEX;;
    toy.radius = R_SPOKE;
    toy.mass = M_TRIANGLE;
    toy.moi = I_TRIANGLE;
    toy.position[X] = x;
    toy.position[Y] = y;
    toy.position[THETA] = 0.0;
    toy.velocity[X] = toy.velocity[Y] = toy.velocity[THETA] = 0.0;
    toy.net_extForce[X] = toy.net_extForce[Y] = toy.net_extForce[THETA] = 0.0;
  }
}    

void x_start_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  if (!timer) {
    XkwSetWidgetLabel(start_w, "Stop");
    timer = XtAppAddTimeOut(app_con,TIMER_UPDATE,x_timer_proc,(XtPointer)NULL);
    //    initialize_random_object();
  }
  else {
    XkwSetWidgetLabel(start_w, "Start");
    XtRemoveTimeOut(timer);
    timer = 0;
  }
}

void x_params_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  switch (Roger.control_mode) {
     case PROJECT2: project2_enter_params(); break;
     case PROJECT3: project3_enter_params(); break;
     case PROJECT4: project4_enter_params(); break;
     case PROJECT5: project5_enter_params(); break;
     case PROJECT6: project6_enter_params(); break;
     case PROJECT7: project7_enter_params(); break;
     case PROJECT8: project8_enter_params(); break;
     case PROJECT9: project9_enter_params(); break;
     case PROJECT10: project10_enter_params(); break;
     case PROJECT11: project11_enter_params(); break;
     default:
       project1_enter_params();
  }
}

int change_input_mode()
{
   static int input_mode;

   input_mode = (input_mode + 1) % N_INPUT_MODES;
   //init_input_flag = TRUE;
   return (input_mode);
}

void x_input_mode_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  Roger.input_mode = change_input_mode();

  switch (Roger.input_mode) {
     case JOINT_ANGLE_INPUT:
       XkwSetWidgetLabel(input_mode_w, "Input: Joint angles"); break;
     case BASE_GOAL_INPUT:
       XkwSetWidgetLabel(input_mode_w, "Input: Base goal"); break;
     case ARM_GOAL_INPUT:
       XkwSetWidgetLabel(input_mode_w, "Input: Arm goals"); break;
     case TOY_INPUT:
       XkwSetWidgetLabel(input_mode_w, "Input: Ball position"); break;
     case MAP_INPUT:
       XkwSetWidgetLabel(input_mode_w, "Input: Map Editor"); break;
     default: break;
  }
}

// initialize all setpoints to reflect the current posture, set the Cartesian
//    button_reference as appropriate for the new mode (base, left arm,
//       right arm, stereo head, harmonic function, integrated app)
void initialize_control_mode(roger)
Robot * roger;
{
  double wTb[4][4],ref_w[4],ref_b[4];
  int i,j;

  // define all setpoints to current positions and zero velocities
  roger->base_setpoint[X] = roger->base_position[X];
  roger->base_setpoint[Y] = roger->base_position[Y];
  roger->base_setpoint[THETA] = roger->base_position[THETA];

  roger->arm_setpoint[LEFT][0] = roger->arm_theta[LEFT][0];
  roger->arm_setpoint[LEFT][1] = roger->arm_theta[LEFT][1];
  roger->arm_setpoint[RIGHT][0] = roger->arm_theta[RIGHT][0];
  roger->arm_setpoint[RIGHT][1] = roger->arm_theta[RIGHT][1];

  roger->eyes_setpoint[LEFT] = roger->eye_theta[LEFT];
  roger->eyes_setpoint[RIGHT] = roger->eye_theta[RIGHT];

  // call the reset method of the last mode --> clear everything you modified
  switch ((roger->control_mode + N_CONTROL_MODES - 1) % N_CONTROL_MODES) {
    // "+ N_CONTROL_MODES" needed as modulo of negative numbers incorrect
     case PROJECT3:
       project3_reset(roger);
       break;
     case PROJECT4:
       project4_reset(roger);
       break;
     case PROJECT5:
       project5_reset(roger);
       break;
     case PROJECT6:
       project6_reset(roger);
       break;
     case PROJECT7:
       project7_reset(roger);
       break;
     case PROJECT8:
       project8_reset(roger);
       break;
     case PROJECT9:
       project9_reset(roger);
       break;
     case PROJECT10:
       project10_reset(roger);
       break;
     case PROJECT11:
       project11_reset(roger);
       break;
     default:
       break;
  }
  init_control_flag = FALSE;
}

int change_control_mode()
{
  static int control_mode;

  printf("control mode=%d", control_mode);
  control_mode = (control_mode + 1) % N_CONTROL_MODES;
  init_control_flag = TRUE;
  return (control_mode);
}

void x_control_mode_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  Roger.control_mode = change_control_mode();

  switch (Roger.control_mode) {
     case PROJECT1:
       XkwSetWidgetLabel(control_mode_w, "1-MotorUnits"); break;
     case PROJECT2:
       XkwSetWidgetLabel(control_mode_w, "2-ArmKinematics"); break;
     case PROJECT3:
       XkwSetWidgetLabel(control_mode_w, "3-Vision"); break;
     case PROJECT4:
       XkwSetWidgetLabel(control_mode_w, "4-SearchTrack"); break;
     case PROJECT5:
       XkwSetWidgetLabel(control_mode_w, "5-StereoKinematics"); break;
     case PROJECT6:
       XkwSetWidgetLabel(control_mode_w, "6-Kalman"); break;
     case PROJECT7:
       XkwSetWidgetLabel(control_mode_w, "7-ChasePunch"); break;
     case PROJECT8:
       XkwSetWidgetLabel(control_mode_w, "8-PathPlanning"); break;
     case PROJECT9:
       XkwSetWidgetLabel(control_mode_w, "9-Grasp"); break;
     case PROJECT10:
       XkwSetWidgetLabel(control_mode_w, "10-Model"); break;
     case PROJECT11:
       XkwSetWidgetLabel(control_mode_w, "11-Belief"); break;
     default: break;
  }
  //call init here makes it independent of timer running
  initialize_control_mode(&Roger);
}

void initialize_room(roger)
Robot * roger;
{
  int i,j;
  FILE *fp;
  char line[NBINS+2];

  // read in appropriate Room file selected by user
  switch (roger->room_num) {
     case R0:
       fp = fopen("ROOMS/R0.txt", "r");
       break;
     case R1:
       fp = fopen("ROOMS/R1.txt", "r");
       break;
     case R2:
       fp = fopen("ROOMS/R2.txt", "r");
       break;
     default:
       break;
  }

  // Initialize world geometry according to Room file
  for (i=0; i<NBINS; i++) {
    fgets(line, NBINS+2, fp);
    for (j=0; j<NBINS; j++) {
      roger->world_map.occupancy_map[i][j] = FREESPACE;
      roger->world_map.potential_map[i][j] = 1.0;
      roger->arm_map[LEFT].occupancy_map[i][j] = FREESPACE;
      roger->arm_map[LEFT].potential_map[i][j] = 1.0;
      roger->arm_map[RIGHT].occupancy_map[i][j] = FREESPACE;
      roger->arm_map[RIGHT].potential_map[i][j] = 1.0;

      switch (line[j]) {
         case 'B':
	   roger->world_map.occupancy_map[i][j] = OBSTACLE;
	   roger->world_map.potential_map[i][j] = 1.0;
	   roger->world_map.color_map[i][j] = DARKBLUE;
	   break;
         case 'G':
	   roger->world_map.occupancy_map[i][j] = OBSTACLE;
	   roger->world_map.potential_map[i][j] = 1.0;
	   roger->world_map.color_map[i][j] = LIGHTGREEN;
	   break;
         case 'K':
	   roger->world_map.occupancy_map[i][j] = OBSTACLE;
	   roger->world_map.potential_map[i][j] = 1.0;
	   roger->world_map.color_map[i][j] = 0;
	   break;
         default:
	   break;
      }
    }
  }
  fclose(fp);
}

int change_room_num()
{
  static int room_num;

  room_num = (room_num + 1) % N_ROOMS;
  return (room_num);
}

void x_room_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  Roger.room_num = change_room_num();

  switch (Roger.room_num) {
     case R0:
       XkwSetWidgetLabel(room_w, "Room: 0");
       break;
     case R1:
       XkwSetWidgetLabel(room_w, "Room: 1");
       break;
     case R2:
       XkwSetWidgetLabel(room_w, "Room: 2");
       break;
     default:
       break;
  }
  initialize_room(&Roger);
}

void x_visualize_proc(w,client_data,call_data)
{
  if (draw_visual == TRUE) draw_visual=FALSE;
  else draw_visual = TRUE;
}

void x_quit_proc(w, client_data, call_data)
Widget w;
XtPointer call_data, client_data;
{
  XFreePixmap(display, pixmap);
  XFreeGC(display, gc);
  XtDestroyApplicationContext(app_con);
  exit(0);
}

void x_canvas_proc(w, client_data, call_data)
Widget w;
XtPointer call_data, client_data;
{
  XEvent *event = ((XEvent *) call_data);
  char text[10];
  KeySym key;
  double x, y, theta1, theta2;
  int i, j, xbin, ybin;
  int c;
  static Dimension nwidth = WIDTH, nheight = HEIGHT;
  void x_expose(), x_clear();

  switch (event->type) {
     case ConfigureNotify:
       nwidth = event->xconfigure.width;
       nheight = event->xconfigure.height;
       break;
     case Expose:
       if (nwidth == width && nheight == height)
	 x_expose();
       else {
	 /* width = nwidth; height = nheight; */
	 x_expose();
       }
       break;

  case ButtonPress:
    Roger.button_reference[X] = D2WX(zoom,event->xbutton.x);
    Roger.button_reference[Y] = D2WY(zoom,event->xbutton.y);
    
    Roger.button_event = event->xbutton.button;
    if (event->xbutton.button == LEFT_BUTTON) { }
    break;

  case ButtonRelease:
    break;
    
  case KeyPress:
    c = XLookupString((XKeyEvent *) event, text, 10, &key, 0);
    if (c == 1)
      switch (text[0]) {
         case 'h':
	   break;
         case 'c':
	   x_clear();
	   x_expose();
	   break;
         case 'q':
	   x_quit_proc(w, client_data, call_data);
      }
  }
}

void x_draw_line(color, start_x, start_y, end_x, end_y)
int color;
double start_x, start_y, end_x, end_y;
{
  XSetForeground(display, gc, world_colors[color].display_color);
  XDrawLine(display, pixmap, gc,
	    W2DX(zoom, start_x), W2DY(zoom, start_y),
	    W2DX(zoom, end_x), W2DY(zoom, end_y));
}

void x_expose()
{
  XCopyArea(display, pixmap, window, gc, 0, 0, width, height, 0, 0);
}

void x_clear()
{
  XSetForeground(display, gc, background);
  XFillRectangle(display, pixmap, gc, 0, 0, width, height);
}

/* forward kinematics in base frame **************************************/
void SIMfwd_kinematics(arm_id, theta1, theta2, x, y)
int arm_id;
double theta1, theta2;
double *x, *y;
{
  *x = L_ARM1 * cos(theta1) + L_ARM2 * cos(theta1 + theta2);
  *y = L_ARM1 * sin(theta1) + L_ARM2 * sin(theta1 + theta2);

  if (arm_id == LEFT) *y += ARM_OFFSET;
  else *y -= ARM_OFFSET;
}

void SIMarm_Jacobian(theta1,theta2, Jacobian)
double theta1,theta2;
double Jacobian[2][2];
{
  Jacobian[0][0] = -L_ARM1*sin(theta1) - L_ARM2*sin(theta1 + theta2);
  Jacobian[0][1] = -L_ARM2*sin(theta1 + theta2);
  Jacobian[1][0] =  L_ARM1*cos(theta1) + L_ARM2*cos(theta1 + theta2);
  Jacobian[1][1] =  L_ARM2*cos(theta1 + theta2);
}

void compute_external_forces()
{
  int i, j, ii, jj, row, col, thetai, thetaj;
  double ri[2], rj[2];
  double dr[2], mag, theta, dij, rx, ry, fx, fy, tz;
  PolyBall obji, objj;

  void copy_object(), update_objects(), SIMfwd_kinematics(), SIMarm_Jacobian();

  // copy the current pos/vel info from device (mobile_base, arms, and toy
  // structures) into inertial "PolyBall objects[NBODY]" array
  update_objects();

  // initialize net_extForce sums
  objects[BASE].net_extForce[X] = objects[BASE].net_extForce[Y] =
    objects[BASE].net_extForce[THETA] = 0.0;

  objects[ARM1].net_extForce[X] = objects[ARM1].net_extForce[Y] =
    objects[ARM1].net_extForce[THETA] = 0.0;

  objects[ARM2].net_extForce[X] = objects[ARM2].net_extForce[Y] =
    objects[ARM2].net_extForce[THETA] = 0.0;

  objects[TOY].net_extForce[X] = objects[TOY].net_extForce[Y] =
    objects[TOY].net_extForce[THETA] = 0.0;
     
  for (i=0; i<(NBODY-1); ++i) { // compute force on body i by body j
    for (j=(i+1); j<NBODY; ++j) {

      copy_object(i, &obji); copy_object(j, &objj);

      // body #4 is the occupancy grid - sum compression      
      if (j==4){ // needs to be upgraded too
	//	printf("checking body i=%d bouncing on OBSTACLE j=%d\n", i,j);
	for (ii=0; ii<obji.N; ++ii) {
	  thetai = (double)ii * (2*M_PI/obji.N);
          
	  // position of the first vertex
	  //	  theta = (double)k*(2.0*M_PI/3.0);
	  //	  xt = RT*cos((double)k*one_twenty); 
	  //	  yt = RT*sin((double)k*one_twenty); 
	  ri[X] = objects[i].position[X] 
	    + (objects[i].radius * cos(thetai))
	    * cos(objects[i].position[THETA]) 
	    - (objects[i].radius * sin(thetai))
	    * sin(objects[i].position[THETA]);
	  ri[Y] = objects[i].position[Y] 
	    + (objects[i].radius * cos(thetai))
	    * sin(objects[i].position[THETA]) 
	    + (objects[i].radius * sin(thetai))
	    * cos(objects[i].position[THETA]);

	  for (row=0; row<NBINS; ++row) {
	    for (col=0; col<NBINS; ++col) {
	      if (Roger.world_map.occupancy_map[row][col] == OBSTACLE) {
		dr[X] = ri[X] - (MIN_X + (col+0.5)*XDELTA);
		dr[Y] = ri[Y] - (MAX_Y - (row+0.5)*YDELTA);
		mag = sqrt(SQR(dr[X]) + SQR(dr[Y]));
		dij = MAX(0.0, (obji.Rsphere + R_OBSTACLE - mag));
		fx = K_COLLIDE*dij*(dr[X]/mag);
		fy = K_COLLIDE*dij*(dr[Y]/mag);

		rx = obji.radius * cos(obji.position[THETA] + thetai);
		ry = obji.radius * sin(obji.position[THETA] + thetai);
		tz = rx * fy - fx * ry;

		objects[i].net_extForce[X] += fx;
		objects[i].net_extForce[Y] += fy;
		objects[i].net_extForce[THETA] += tz;
	    
		//    rx = objj.radius * cos(objj.position[THETA] + thetaj);
		//    ry = objj.radius * sin(objj.position[THETA] + thetaj);
		//    tz = rx * fy - fx * ry;
		//
		//    objects[j].net_extForce[X] -= fx;
		//    objects[j].net_extForce[Y] -= fy;
		//    objects[j].net_extForce[THETA] -= tz;

	      }
	    }
	  }
	}
	//	printf("\tforce on body i=%d  f = [%6.4lf %6.4lf %6.4lf]\n", i,
	//           objects[i].net_extForce[X], objects[i].net_extForce[Y], 
	//	       objects[i].net_extForce[THETA]);
      }
      else { // j not equal to 4: BASE || ARM#1 || ARM#2 || circle object
	for (ii=0; ii<obji.N; ++ii) {
	  thetai = (double)ii * (2*M_PI/obji.N);
          
	  // position of the first vertex
	  //	  theta = (double)k*(2.0*M_PI/3.0);
	  //	  xt = RT*cos((double)k*one_twenty); 
	  //	  yt = RT*sin((double)k*one_twenty); 
	  ri[X] = obji.position[X] + 
	    obji.radius * cos(obji.position[THETA] + thetai);
	  ri[Y] = obji.position[Y] + 
	    obji.radius * sin(obji.position[THETA] + thetai);
	  
	  for (jj=0; jj<objj.N; ++jj) {
	    thetaj = (double)jj * (2*M_PI/objj.N);
          
	    rj[X] = objj.position[X] + 
	      objj.radius * cos(objj.position[THETA] + thetaj);
	    rj[Y] = objj.position[Y] + 
	      objj.radius * sin(objj.position[THETA] + thetaj);

	    dr[X] = ri[X] - rj[X];
	    dr[Y] = ri[Y] - rj[Y];
	    mag = sqrt(SQR(dr[X]) + SQR(dr[Y]));
	    dij = MAX(0.0, (obji.Rsphere + objj.Rsphere - mag)); 
	    fx = K_COLLIDE*dij*(dr[X]/mag);
	    fy = K_COLLIDE*dij*(dr[Y]/mag);

	    rx = obji.radius * cos(obji.position[THETA] + thetai);
	    ry = obji.radius * sin(obji.position[THETA] + thetai);
	    tz = rx * fy - fx * ry;

	    objects[i].net_extForce[X] += fx;
	    objects[i].net_extForce[Y] += fy;
	    objects[i].net_extForce[THETA] += tz;
	    
	    rx = objj.radius * cos(objj.position[THETA] + thetaj);
	    ry = objj.radius * sin(objj.position[THETA] + thetaj);
	    tz = rx * fy - fx * ry;

	    objects[j].net_extForce[X] -= fx;
	    objects[j].net_extForce[Y] -= fy;
	    objects[j].net_extForce[THETA] -= tz;
	  }
	}
      }
    }
  }

  // BASE
  mobile_base.extForce[X] = objects[BASE].net_extForce[X];
  mobile_base.extForce[Y] = objects[BASE].net_extForce[Y];

  // ARM #1
  //  reality check: why do you need the negative of fb?
  arms[LEFT][NARM_FRAMES - 1].extForce[X] = -objects[ARM1].net_extForce[X];
  arms[LEFT][NARM_FRAMES - 1].extForce[Y] = -objects[ARM1].net_extForce[Y];

  // ARM #2
  //  reality check: why do you need the negative of fb?
  arms[RIGHT][NARM_FRAMES - 1].extForce[X] = -objects[ARM2].net_extForce[X];
  arms[RIGHT][NARM_FRAMES - 1].extForce[Y] = -objects[ARM2].net_extForce[Y];

  // TOY OBJECT
  toy.net_extForce[X] = objects[TOY].net_extForce[X];
  toy.net_extForce[Y] = objects[TOY].net_extForce[Y];
  toy.net_extForce[THETA] = objects[TOY].net_extForce[THETA];

  //  printf("exiting compute_external_forces()\n"); fflush(stdout);
}

void copy_object(i, obj)
int i;
PolyBall * obj;
{
  obj->id = objects[i].id;
  obj->N = objects[i].N;
  obj->Rsphere = objects[i].Rsphere;
  obj->radius = objects[i].radius;
  obj->mass = objects[i].mass;
  obj->moi = objects[i].moi;
  obj->position[X] = objects[i].position[X];
  obj->position[Y] = objects[i].position[Y];
  obj->position[THETA] = objects[i].position[THETA];

  obj->velocity[X] = objects[i].velocity[X];
  obj->velocity[Y] = objects[i].velocity[Y];
  obj->velocity[THETA] = objects[i].velocity[THETA];

  obj->net_extForce[X] = objects[i].net_extForce[X];
  obj->net_extForce[Y] = objects[i].net_extForce[Y];
  obj->net_extForce[THETA] = objects[i].net_extForce[THETA];
}

// copy the pos/vel info from device (mobile_base, arms, toy) structures
//    into inertial "PolyBall objects[NBODY]" arrayv
void update_objects() 
{
  double x, y, pb[4], pw[4], vb[4], vw[4], J[2][2];

  /****************************** 0: BASE ************************************/
  //     objects[BASE].N = 1;
  //     objects[BASE].Rsphere = R_BASE;
  //     objects[BASE].radius = 0.0;
  //     objects[BASE].mass = M_BASE;
  //     objects[BASE].moi = I_BASE;

  objects[BASE].position[X] = mobile_base.x; 
  objects[BASE].position[Y] = mobile_base.y; 
  objects[BASE].position[THETA] = mobile_base.theta;

  objects[BASE].velocity[X] = mobile_base.x_dot;
  objects[BASE].velocity[Y] = mobile_base.y_dot; 
  objects[BASE].velocity[THETA] = mobile_base.theta_dot;

  /************************** 1: LEFT ARM ************************************/
  // left hand position in world coordinates
  SIMfwd_kinematics(LEFT, arms[LEFT][1].theta, arms[LEFT][2].theta, &x, &y);
  pb[0] = x; pb[1] = y; pb[2] = 0.0; pb[3] = 1.0;
  matrix_mult(4, 4, mobile_base.wTb, 1, pb, pw);

  // left hand velocity relative to base written in base coordinates
  //          (figure 2 in RogerDynamics document defines frames)
  // ^w(v)_10 = wRb[ ^b(v)_7 +  J_arm theta_dot_arm ]
  //    = ^w(v)_b + wRb[ -ARM_OFFSET omega_0 xhat_b + J_arm theta_dot_arm ]
  SIMarm_Jacobian(arms[LEFT][1].theta, arms[LEFT][2].theta, J);
  vb[X] = J[0][0]*arms[LEFT][1].theta_dot + 
    J[0][1]*arms[LEFT][2].theta_dot - ARM_OFFSET*mobile_base.theta_dot;
  vb[Y] = J[1][0]*arms[LEFT][1].theta_dot + J[1][1]*arms[LEFT][2].theta_dot;
  vb[2] = 0.0;
  vb[3] = 0.0; // homogeneous vector

  matrix_mult(4, 4, mobile_base.wTb, 1, vb, vw);
  //  v[1][X] = base->x_dot + v_w[X];
  //  v[1][Y] = base->y_dot + v_w[Y];
  //  R[1] = R_TACTILE;

  //  objects[ARM1].N = 1;
  //  objects[ARM1].Rsphere = R_TACTILE;
  //  objects[ARM1].radius = 0.0;
  //  objects[ARM1].mass = M_ARM1;
  //  objects[ARM1].moi = I_ARM1;
     
  objects[ARM1].position[X] = pw[X];
  objects[ARM1].position[Y] = pw[Y];
  objects[ARM1].position[THETA] = 0.0;
  // hand orientation is not relevant to system dynamics

  objects[ARM1].velocity[X] = mobile_base.x_dot + vw[X];
  objects[ARM1].velocity[Y] = mobile_base.y_dot + vw[Y]; 
  objects[ARM1].velocity[THETA] = 0.0; 
  // angular acceleration in the hand is not relevant to system dynamics

  /************************** 2: RIGHT ARM ************************************/
  // right hand position in world coordinates
  SIMfwd_kinematics(RIGHT,arms[RIGHT][1].theta,arms[RIGHT][2].theta, &x,&y);
  pb[0] = x; pb[1] = y; pb[2] = 0.0; pb[3] = 1.0;
  matrix_mult(4, 4, mobile_base.wTb, 1, pb, pw);

  // left hand velocity relative to base written in base coordinates
  //          (figure 2 in RogerDynamics document defines frames)
  // ^w(v)_10 = wRb[ ^b(v)_7 +  J_arm theta_dot_arm ]
  //    = ^w(v)_b + wRb[ -ARM_OFFSET omega_0 xhat_b + J_arm theta_dot_arm ]
  SIMarm_Jacobian(arms[RIGHT][1].theta, arms[RIGHT][2].theta, J);
  vb[X] = J[0][0]*arms[RIGHT][1].theta_dot + 
    J[0][1]*arms[RIGHT][2].theta_dot - ARM_OFFSET*mobile_base.theta_dot;
  vb[Y] = J[1][0]*arms[RIGHT][1].theta_dot+J[1][1]*arms[RIGHT][2].theta_dot;
  vb[2] = 0.0;
  vb[3] = 0.0; // homogeneous vector

  matrix_mult(4, 4, mobile_base.wTb, 1, vb, vw);

  //  objects[ARM2].N = 1;
  //  objects[ARM2].Rsphere = R_TACTILE;
  //  objects[ARM2].radius = 0.0;
  //  objects[ARM2].mass = M_ARM1;
  //  objects[ARM2].moi = I_ARM1;
  
  objects[ARM2].position[X] = pw[X];
  objects[ARM2].position[Y] = pw[Y];
  objects[ARM2].position[THETA] = 0.0; 
  // hand orientation not relevant to system dynamics

  objects[ARM2].velocity[X] = mobile_base.x_dot + vw[X];
  objects[ARM2].velocity[Y] = mobile_base.y_dot + vw[Y]; 
  objects[ARM2].velocity[THETA] = 0.0;
  // angular acceleration of hand is not relevant to system dynamics


  /****************** 3: TOY OBJECT - CIRCLE || TRIANGLE *********************/
  objects[TOY].id = toy.id;
  objects[TOY].N = toy.N;
  objects[TOY].Rsphere = toy.Rsphere;
  objects[TOY].radius = toy.radius;
  objects[TOY].mass = toy.mass;
  objects[TOY].moi = toy.moi;

  objects[TOY].position[X] = toy.position[X]; 
  objects[TOY].position[Y] = toy.position[Y];
  objects[TOY].position[THETA] = toy.position[THETA];

  objects[TOY].velocity[X] = toy.velocity[X];
  objects[TOY].velocity[Y] = toy.velocity[Y];
  objects[TOY].velocity[THETA] = toy.velocity[THETA];
}

#define NSAMPLES 10

void update_cspace_arms(roger)
Robot * roger;
{ 
  int i,j, arm, xbin, ybin, dl, link, collide;
  double t1, t2, x, y, dx[3], dy[3];
  double wTb[4][4], bTw[4][4], b[4], w[4];

  // MAP CARTESIAN OBSTACLES TO OBSTACLES IN ARM CSPACES
  construct_wTb(roger->base_position, wTb);

  for (arm=0; arm < NARMS; ++arm) {
    for (i=0;i<NBINS;++i) {
      t2 = T2_MAX - (i+0.5)*TDELTA;
      for (j=0;j<NBINS;++j) {
	t1 = T1_MIN + (j+0.5)*TDELTA;
	
	// default status
	roger->arm_map[arm].occupancy_map[i][j] = FREESPACE;
	collide=FALSE;
	if ((j <= 0) || (j >= (NBINS - 1))) {
	  Roger.arm_map[arm].occupancy_map[i][j] = OBSTACLE;
	  Roger.arm_map[arm].potential_map[i][j] = 1.0;
	}
	else if ((i <= 0) || (i >= (NBINS - 1))) {
	  Roger.arm_map[arm].occupancy_map[i][j] = OBSTACLE;
	  Roger.arm_map[arm].potential_map[i][j] = 1.0;
	}
	else {
	  // in base coordinates
	  if (arm==LEFT) {x = 0.0; y = ARM_OFFSET;}
	  else  {x = 0.0; y = -ARM_OFFSET;}
	  dx[1] = L_ARM1*cos(t1)/NSAMPLES;
	  dx[2] = L_ARM2*cos(t1+t2)/NSAMPLES;
	  dy[1] = L_ARM1*sin(t1)/NSAMPLES;
	  dy[2] = L_ARM2*sin(t1+t2)/NSAMPLES;

	  // walk the length of the manipulator in Cartesian space
	  for (link=1; ((link <= 2) && (!collide)); ++link) {
	    for (dl=0; (dl<NSAMPLES); ++dl) {
	      x += dx[link]; y += dy[link];
	      
	      b[0]=x; b[1]=y; b[2]=0.0; b[3]=1.0;
	      
	      matrix_mult(4, 4, wTb, 1, b, w);
	      
	      xbin = (int)((w[0]-MIN_X)/XDELTA);
	      ybin = (int)((MAX_Y-w[1])/YDELTA);

	      if (roger->world_map.occupancy_map[ybin][xbin] == OBSTACLE)
		collide = TRUE;
	    }
	  }
	  if (collide) {
	    roger->arm_map[arm].occupancy_map[i][j] = OBSTACLE;
	    roger->arm_map[arm].potential_map[i][j] = 1.0;
	  }
	}
	//map goals
	SIMfwd_kinematics(arm,t1,t2,&x,&y);
	b[0]=x; b[1]=y; b[2]=0.0; b[3]=1.0;
	matrix_mult(4, 4, wTb, 1, b, w);

	xbin = (int)((w[0]-MIN_X)/XDELTA);
	ybin = (int)((MAX_Y-w[1])/YDELTA);
	if (roger->world_map.occupancy_map[ybin][xbin] == GOAL) {
	  roger->arm_map[arm].occupancy_map[i][j] = GOAL;
	  roger->arm_map[arm].potential_map[i][j] = 0.0;
	}
      }
    }
  }
}

//update_cspace_base(roger)
//Robot * roger;
//{ 
//  int i,j, xbin, ybin, dl, link, collide;
//  double t1, t2, x, y, dx[3], dy[3];
//  double wTb[4][4], bTw[4][4], b[4], w[4];
//	
//  // MAP CARTESIAN OBSTACLES TO DIRICHLET CONSTRAINTS IN CSPACE
//  // for each CSpace bin:
//  //	printf("\n Updating CSPACE for BASE");
//  //construct_wTb(roger->base_position, wTb);
//	
//  for (i=0;i<NBINS;++i) {
//    for (j=0;j<NBINS;++j) {
//      cspace[BASE].boundary_map[i][j] = FREESPACE;
//      if (CartesianMap[i][j] == DIRICHLET) {
//	  cspace[BASE].boundary_map[i][j] = DIRICHLET;
//	  cspace[BASE].potential_map[i][j] = 1.0;
//	}
//      if (CartesianMap[i][j] == GOAL) {
//	  cspace[BASE].boundary_map[i][j] = GOAL;
//	  cspace[BASE].potential_map[i][j] = 0.0;
//	}
//      cspace[BASE].boundary_map[0][j] = 
//	cspace[BASE].boundary_map[(NBINS-1)][j] = DIRICHLET;
//      cspace[BASE].potential_map[0][j] = 
//	   cspace[BASE].potential_map[(NBINS-1)][j] = 1.0;
//    }
//		
//    // joint angle limits:
//    // theta1: is toroidal   theta2: -3.1 rad < theta2 < +3.1 rad
//    cspace[BASE].boundary_map[i][0] = 
//       cspace[BASE].boundary_map[i][(NBINS-1)] = DIRICHLET;
//    cspace[BASE].potential_map[i][0] = 
//       cspace[BASE].potential_map[i][(NBINS-1)] = 1.0;
//  }
//  //	printf("\n Updating CSPACE for BASE");
//  fflush(stdout);
//}

void draw_history()
{
  int h;
  
  // draw history of all Cartesian arm postures
  XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);

  for (h = 0; h < history_ptr; ++h) {
    // draw Cartesian history of the mobile platform
    XFillRectangle(display, pixmap, gc, 
		   W2DX(zoom, (history[h].base_pos[0]-XDELTA/4.0)),
		   W2DY(zoom, (history[h].base_pos[1]-YDELTA/4.0)),
		   W2DR(zoom,XDELTA/2.0), W2DR(zoom,YDELTA/2.0));
  }
}

void draw_all() 
{
  int n;
  char buffer[64];
  void draw_frames(), draw_boundaries(), draw_potential_maps();
  void draw_object(), draw_roger();

  x_clear();

  // draw localizability/manipulability ellipses on top (i.e. render them last)
  // draw harmonic function streamlines on bottom (i.e. render them first)

  //  draw_ellipse(manipulator(LEFT));
  //  draw_ellipse(manipulator(RIGHT));

  // XSetForeground(display, gc, goal_color);
  //  draw_ellipse(observation);
  //  draw_ellipse(spatial_goals[CENTROID]);
  //  draw_ellipse(spatial_goals[LEFT_EDGE]);
  //  draw_ellipse(spatial_goals[RIGHT_EDGE]);

  //  if (p_index==6) draw_ellipse(grasp_goal);
  //  XSetForeground(display, gc, target_color);
  //  draw_ellipse(target);

  draw_potential_maps();
  draw_boundaries();
  draw_frames();

  switch (Roger.control_mode) {

  case PROJECT2: // VISION
    draw_object(objects[TOY]);
    draw_roger(mobile_base, arms, eyes);
    if (draw_visual==TRUE) {
      project2_visualize(&Roger);
    }
    break;
  case PROJECT3: // SEARCHTRACK
    draw_object(objects[TOY]);
    draw_roger(mobile_base, arms, eyes);
    if (draw_visual==TRUE) {
      project3_visualize(&Roger);
    }
    break;
  case PROJECT4: // LOCALIZE
    draw_object(objects[TOY]);
    draw_roger(mobile_base, arms, eyes);
    if (draw_visual==TRUE) {
      project4_visualize(&Roger);
    }
    break;
  case PROJECT5: // KALMAN FILTER
    draw_object(objects[TOY]);
    draw_roger(mobile_base, arms, eyes);
    if (draw_visual==TRUE) {
      project5_visualize(&Roger);
    }
    break;
  case PROJECT6:  // KINEMATICS
    draw_object(objects[TOY]);
    draw_roger(mobile_base, arms, eyes);
    if (draw_visual==TRUE) {
      project6_visualize(&Roger);
    }
    break;
  case PROJECT7: // CHASEPUNCH
    draw_object(objects[TOY]);
    draw_roger(mobile_base, arms, eyes);
    if (draw_visual==TRUE) {
      project7_visualize(&Roger);
    }
    break;
  case PROJECT8: // PATHPLANNING
    // harmonic functions - put object and Roger on top of
    // the potential field and streamlines in project6-visualize
    if (draw_visual==TRUE) project8_visualize(&Roger);
    draw_object(objects[TOY]);
    draw_roger(mobile_base, arms, eyes);
    break;
  case PROJECT9: // GRASP
    draw_object(objects[TOY]);
    draw_roger(mobile_base, arms, eyes);
    if (draw_visual==TRUE) {
      project9_visualize(&Roger);
    }
    break;
  case PROJECT10: // MODEL
    draw_object(objects[TOY]);
    draw_roger(mobile_base, arms, eyes);
    if (draw_visual==TRUE) {
      project10_visualize(&Roger);
    }
    break;
  case PROJECT11: // BELIEF
    draw_object(objects[TOY]);
    draw_roger(mobile_base, arms, eyes);
    if (draw_visual==TRUE) {
      project11_visualize(&Roger);
    }
    break;
  default:
    draw_object(objects[TOY]);
    draw_roger(mobile_base, arms, eyes);
    break;
  }

  if (HISTORY) draw_history();

  XSetForeground(display, gc, foreground);
  n = sprintf(buffer, "total elapsed time = %6.3lf", simtime);
  XDrawString(display, pixmap, gc, W2DX(zoom,2.2),W2DY(zoom,1.65),buffer, n);

  if (draw_visual) {
    n = sprintf(buffer, "VISUALIZE: ON");
    XDrawString(display,pixmap,gc,W2DX(zoom,2.6),W2DY(zoom,1.8),buffer, n);
  }
  else {
    n = sprintf(buffer, "VISUALIZE: OFF");
    XDrawString(display,pixmap,gc,W2DX(zoom,2.6),W2DY(zoom,1.8),buffer, n);
  }
  
  if (COUPLED_DYNAMICS) {
    n = sprintf(buffer, "WHOLE-BODY DYNAMICS");
    XDrawString(display,pixmap,gc,W2DX(zoom,-3.625),W2DY(zoom,1.8),buffer, n);
  }
  else {
    n = sprintf(buffer, "BLOCK DIAGONAL DYNAMICS");
    XDrawString(display,pixmap,gc,W2DX(zoom,-3.875),W2DY(zoom,1.8),buffer, n);
  }

  if (ACTUATE_BASE) {
    n = sprintf(buffer, "BASE: ON");
    XDrawString(display,pixmap,gc,W2DX(zoom,-4.0),W2DY(zoom,1.65),buffer, n);
  }
  else {
    n = sprintf(buffer, "BASE: OFF");
    XDrawString(display,pixmap,gc,W2DX(zoom,-4.0),W2DY(zoom,1.65),buffer, n);
  }
  
  if (ACTUATE_ARMS) {
    n = sprintf(buffer, "ARMS: ON");
    XDrawString(display,pixmap,gc,W2DX(zoom,-3.3125),W2DY(zoom,1.65),buffer,n);
  }
  else {
    n = sprintf(buffer, "ARMS: OFF");
    XDrawString(display,pixmap,gc,W2DX(zoom,-3.3125),W2DY(zoom,1.65),buffer,n);
  }
  
  if (ACTUATE_EYES) {
    n = sprintf(buffer, "EYES: ON");
    XDrawString(display,pixmap,gc,W2DX(zoom,-2.625),W2DY(zoom,1.65),buffer, n);
  }
  else {
    n = sprintf(buffer, "EYES: OFF");
    XDrawString(display,pixmap,gc,W2DX(zoom,-2.625),W2DY(zoom,1.65),buffer, n);
  }
  

  x_expose();
}

void draw_circle(cu, cv, r, fill)
int cu, cv, r, fill;
{
  if (fill == NOFILL)
    XDrawArc(display, pixmap, gc, cu - r, cv - r, 2* r , 2* r , 0, 64*360);
  else
    XFillArc(display, pixmap, gc, cu-r, cv-r, 2*r, 2*r, 0, 64*360);
}

void draw_frames()
{
#define FRAME_L 0.04
#define FRAME_T 0.045

  XSetForeground(display, gc, foreground);

  // the Cartesian frame
  /* x-axis */
  XDrawLine(display, pixmap, gc, 
	    W2DX(zoom,0.0), W2DY(zoom,0.0), 
	    W2DX(zoom,(FRAME_L*4.0)), W2DY(zoom,0.0));
  XDrawString(display,pixmap,gc,W2DX(zoom,FRAME_T*4.0),W2DY(zoom,0.0),"x",1);

  /* y-axis */
  XDrawLine(display, pixmap, gc, 
	    W2DX(zoom,0.0), W2DY(zoom,0.0), 
	    W2DX(zoom,0.0), W2DY(zoom,FRAME_L*4.0));
  XDrawString(display,pixmap,gc,W2DX(zoom,0.0),W2DY(zoom,FRAME_T*4.0),"y",1);

  // the LEFT CSpace frame
  /* q1-axis */
  XDrawLine(display, pixmap, gc, 
	    T12LD(0.0), T22LD(0.0), 
	    T12LD(FRAME_L*2.0*M_PI), T22LD(0.0));
  XDrawString(display,pixmap,gc,T12LD(FRAME_T*2.0*M_PI),T22LD(0.0),"q1",2);

  /* q2-axis */
  XDrawLine(display, pixmap, gc, 
	    T12LD(0.0), T22LD(0.0),
	    T12LD(0.0), T22LD(FRAME_L*2.0*M_PI));
  XDrawString(display,pixmap,gc,T12LD(0.0),T22LD(FRAME_T*2.0*M_PI),"q2",2);

  XDrawString(display, pixmap, gc, T12LD(-0.75), T22LD(-3.5), "left    /", 9);
  XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
  XDrawString(display, pixmap, gc, T12LD(-0.2), T22LD(-3.5), "arm", 3);
  XSetForeground(display, gc, world_colors[EYE_COLOR].display_color);
  XDrawString(display, pixmap, gc, T12LD(0.35), T22LD(-3.5), "eye", 3);

  XSetForeground(display, gc, foreground);
  // the RIGHT CSpace frame
  /* q1-axis */
  XDrawLine(display, pixmap, gc, 
	    T12RD(0.0), T22RD(0.0), T12RD(FRAME_L*2.0*M_PI), T22RD(0.0));
  XDrawString(display,pixmap,gc,T12RD(FRAME_T*2.0*M_PI),T22RD(0.0),"q1",2);

  /* q2-axis */
  XDrawLine(display, pixmap, gc, 
	    T12RD(0.0), T22RD(0.0), T12RD(0.0), T22RD(FRAME_L*2.0*M_PI));
  XDrawString(display,pixmap,gc,T12RD(0.0),T22RD(FRAME_T*2.0*M_PI),"q2",2);

  XDrawString(display,pixmap,gc,T12RD(-0.85),T22RD(-3.5), "right    /", 10);
  XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
  XDrawString(display, pixmap, gc, T12RD(-0.15), T22RD(-3.5), "arm", 3);
  XSetForeground(display, gc, world_colors[EYE_COLOR].display_color);
  XDrawString(display, pixmap, gc, T12RD(0.4), T22RD(-3.5), "eye", 3);

#undef FRAME_L // 0.04
#undef FRAME_T // 0.045
}

void draw_frame(xform)
double xform[4][4];
{
#define FRAME_L 0.04
#define FRAME_T 0.045

  XSetForeground(display, gc, foreground);

  /* x-axis */
  XDrawLine(display, pixmap, gc, 
	    W2DX(zoom,xform[0][3]), W2DY(zoom,xform[1][3]),
	    W2DX(zoom,xform[0][3]+FRAME_L*xform[0][0]),
	    W2DY(zoom,xform[1][3]+FRAME_L*xform[1][0]));
  XDrawString(display, pixmap, gc, 
	      W2DX(zoom,xform[0][3]+FRAME_T*xform[0][0]),
	      W2DY(zoom,xform[1][3]+FRAME_T*xform[1][0]), "x", 1);

  /* y-axis */
  XDrawLine(display, pixmap, gc, 
	    W2DX(zoom,xform[0][3]), W2DY(zoom,xform[1][3]),
	    W2DX(zoom,xform[0][3]+FRAME_L*xform[0][1]),
	    W2DY(zoom,xform[1][3]+FRAME_L*xform[1][1]));
  XDrawString(display,pixmap,gc,
	      W2DX(zoom,xform[0][3]+FRAME_T*xform[0][1]),
	      W2DY(zoom,xform[1][3]+FRAME_T*xform[1][1]), "y", 1);
#undef FRAME_L // 0.04
#undef FRAME_T // 0.045
}

void draw_boundaries()
{
  /******************************************************************/
  /**  draw world                                                  **/
  XSetForeground(display, gc, foreground);
  XDrawLine(display, pixmap, gc, 
	    W2DX(zoom,MIN_X), W2DY(zoom,MAX_Y),
	    W2DX(zoom,MAX_X), W2DY(zoom,MAX_Y));
  XDrawLine(display, pixmap, gc,
	    W2DX(zoom,MAX_X), W2DY(zoom,MAX_Y),
	    W2DX(zoom,MAX_X), W2DY(zoom,MIN_Y));
  XDrawLine(display, pixmap, gc, 
	    W2DX(zoom,MAX_X), W2DY(zoom,MIN_Y),
	    W2DX(zoom,MIN_X), W2DY(zoom,MIN_Y));
  XDrawLine(display, pixmap, gc,
	    W2DX(zoom,MIN_X), W2DY(zoom,MIN_Y),
	    W2DX(zoom,MIN_X), W2DY(zoom,MAX_Y));

  /* draw LEFT boundaries */
  XSetForeground(display, gc, foreground);
  XDrawLine(display, pixmap, gc, 
	    T12LD(T1_MIN), T22LD(T2_MAX), T12LD(T1_MAX), T22LD(T2_MAX));
  XDrawLine(display, pixmap, gc, 
	    T12LD(T1_MAX), T22LD(T2_MAX), T12LD(T1_MAX), T22LD(T2_MIN));
  XDrawLine(display, pixmap, gc, 
	    T12LD(T1_MAX), T22LD(T2_MIN), T12LD(T1_MIN), T22LD(T2_MIN));
  XDrawLine(display, pixmap, gc,
	    T12LD(T1_MIN), T22LD(T2_MIN), T12LD(T1_MIN), T22LD(T2_MAX));

  /* draw RIGHT boundaries */
  XSetForeground(display, gc, foreground);
  XDrawLine(display, pixmap, gc,
	    T12RD(T1_MIN), T22RD(T2_MAX), T12RD(T1_MAX), T22RD(T2_MAX));
  XDrawLine(display, pixmap, gc,
	    T12RD(T1_MAX), T22RD(T2_MAX), T12RD(T1_MAX), T22RD(T2_MIN));
  XDrawLine(display, pixmap, gc,
	    T12RD(T1_MAX), T22RD(T2_MIN), T12RD(T1_MIN), T22RD(T2_MIN));
  XDrawLine(display, pixmap, gc,
	    T12RD(T1_MIN), T22RD(T2_MIN), T12RD(T1_MIN), T22RD(T2_MAX));
}

// draw the NBINSxNBINS potential maps in their respective areas of the canvas
void draw_potential_maps()
{
  int i, j, Cart_grey_index, left_arm_grey_index, right_arm_grey_index;
  double x, y, t1, t2;
  double Cart_bin_potential, left_arm_bin_potential, right_arm_bin_potential;

  for (i = 0; i < NBINS; ++i) {
    y = MAX_Y - i*YDELTA;
    t2 = T2_MAX - i*TDELTA;
    for (j = 0; j < NBINS; ++j) {
      x = MIN_X + j*XDELTA;
      t1 = T1_MIN + j*TDELTA;
      // user map grey level fill
      Cart_bin_potential = Roger.world_map.potential_map[i][j];
      left_arm_bin_potential = Roger.arm_map[LEFT].potential_map[i][j];
      right_arm_bin_potential = Roger.arm_map[RIGHT].potential_map[i][j];

      // 0 <= grey indices <= 100
      Cart_grey_index = (int) (Cart_bin_potential * 100.0);
      left_arm_grey_index = (int) (left_arm_bin_potential * 100.0);
      right_arm_grey_index = (int) (right_arm_bin_potential * 100.0);

      // Cartesian Map
      // fill is either:
      //   a grey level depicting the user defined potential
      XSetForeground(display, gc, world_colors[Cart_grey_index].display_color);
      //   a user map perceived obstacle color, or
      if (Roger.world_map.occupancy_map[i][j] == OBSTACLE)
	XSetForeground(display, gc, 
		 world_colors[Roger.world_map.color_map[i][j]].display_color);
      else if (Roger.world_map.occupancy_map[i][j] == DILATED_OBSTACLE)
	XSetForeground(display, gc, 
		 world_colors[Roger.world_map.color_map[i][j]].display_color);
      //   a user defined goal
      else if (Roger.world_map.occupancy_map[i][j] == GOAL)
	XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);
      XFillRectangle(display, pixmap, gc,
		     W2DX(zoom,x), W2DY(zoom,y),
		     (W2DR(zoom,XDELTA) + 1), (W2DR(zoom,YDELTA) + 1));

      //      // each real obstacle should be outlined in the obstacle color
      //      if (real_world.occupancy_map[i][j] == OBSTACLE) {
      //	XSetForeground(display, gc,
      //	       world_colors[real_world.color_map[i][j]].display_color);
      //	x = MIN_X + j*XDELTA; y = MAX_Y - i*YDELTA;
      //	XDrawRectangle(display, pixmap, gc, W2DX(zoom,x), W2DY(zoom,y),
      //		     (W2DR(zoom,XDELTA)), (W2DR(zoom,YDELTA)));
      //      }
      
      // Left Arm Map
      XSetForeground(display, gc, 
		     world_colors[left_arm_grey_index].display_color);
      if (Roger.arm_map[LEFT].occupancy_map[i][j] == OBSTACLE)
	XSetForeground(display, gc, world_colors[RED].display_color);
      else if (Roger.arm_map[LEFT].occupancy_map[i][j] == GOAL)
	XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);
      XFillRectangle(display, pixmap, gc,
		     T12LD(t1),T22LD(t2),(T2DR(TDELTA)+1),(T2DR(TDELTA)+1));

      // Right Arm Map
      XSetForeground(display, gc, 
		     world_colors[right_arm_grey_index].display_color);
      if (Roger.arm_map[RIGHT].occupancy_map[i][j] == OBSTACLE)
	XSetForeground(display, gc, world_colors[RED].display_color);
      else if (Roger.arm_map[RIGHT].occupancy_map[i][j] == GOAL)
	XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);
      XFillRectangle(display, pixmap, gc, 
		     T12RD(t1),T22RD(t2),(T2DR(TDELTA)+1),(T2DR(TDELTA)+1));
    }
  }
}

void draw_object(obj)
PolyBall obj;
{
  int k;
  double xw, yw, theta;

  XSetForeground(display, gc, world_colors[OBJECT_COLOR].display_color);

  for (k=0; k<obj.N; ++k) {
    theta = (double)k * 2.0*M_PI/obj.N;
    xw = obj.position[X] 
      + cos(obj.position[THETA]) * (obj.radius*cos(theta))
      - sin(obj.position[THETA]) * (obj.radius*sin(theta));
    yw = obj.position[Y] 
      + sin(obj.position[THETA]) * (obj.radius*cos(theta))
      + cos(obj.position[THETA]) * (obj.radius*sin(theta));
    draw_circle(W2DX(zoom, xw), W2DY(zoom, yw), 
		W2DR(zoom,obj.Rsphere),FILL);
  }
}

void draw_object2(obj)
PolyBall obj;
{
  double x, y, theta, c, s, wTo[4][4], v[4][2], vw[4][2];
  double vertex_o[4], vertex_w[4];
  double oT1[4][4], oT2[4][4], wT1[4][4], wT2[4][4];
  XPoint polygon[6];

  XSetForeground(display, gc, world_colors[OBJECT_COLOR].display_color);

  switch(obj.id) {
  case CIRCLE:
    draw_circle(W2DX(zoom,obj.position[X]), W2DY(zoom,obj.position[Y]),
	      W2DR(zoom,R_BALL), FILL);
    break;
  case TRIANGLE:
    x = obj.position[X]; y = obj.position[Y]; theta = obj.position[THETA];
    c = cos(theta); s = sin(theta);

    // world-to-object (object frame is frame 0)
    wTo[0][0] =   c; wTo[0][1] =  -s; wTo[0][2] = 0.0; wTo[0][3] = x;
    wTo[1][0] =   s; wTo[1][1] =   c; wTo[1][2] = 0.0; wTo[1][3] = y;
    wTo[2][0] = 0.0; wTo[2][1] = 0.0; wTo[2][2] = 1.0; wTo[2][3] = 0.0;
    wTo[3][0] = 0.0; wTo[3][1] = 0.0; wTo[3][2] = 0.0; wTo[3][3] = 1.0;

    // vertices are the same with respect to their local frame
    v[0][0] = obj.Rsphere*cos(M_PI/12.0);
    v[0][1] = -v[0][0];
    v[1][0] = obj.radius + obj.Rsphere*sin(M_PI/12.0);
    v[1][1] = v[1][0];

    v[2][0] = 0.0;                    v[2][1] = 0.0;
    v[3][0] = 1.0;                    v[3][1] = 1.0;

    matrix_mult(4, 4, wTo, 2, v, vw);
    polygon[0].x = (short) (W2DX(zoom, vw[0][0]));
    polygon[0].y = (short) (W2DY(zoom, vw[1][0]));
    polygon[1].x = (short) (W2DX(zoom, vw[0][1]));
    polygon[1].y = (short) (W2DY(zoom, vw[1][1]));

    vertex_o[0] = 0.0; vertex_o[1] = obj.radius;
    vertex_o[2] = 0.0; vertex_o[3] = 1.0; 
    matrix_mult(4, 4, wTo, 1, vertex_o, vertex_w);
    draw_circle(W2DX(zoom, vertex_w[0]), W2DY(zoom, vertex_w[1]),
		W2DR(zoom,obj.Rsphere),FILL);

    c = cos(2.0*M_PI/3.0); s = sin(2.0*M_PI/3.0);
    oT1[0][0] =  c;  oT1[0][1] = -s;  oT1[0][2]= 0.0; oT1[0][3]= 0.0;
    oT1[1][0] =  s;  oT1[1][1] =  c;  oT1[1][2]= 0.0; oT1[1][3]= 0.0;
    oT1[2][0] = 0.0; oT1[2][1] = 0.0; oT1[2][2]= 1.0; oT1[2][3]= 0.0;
    oT1[3][0] = 0.0; oT1[3][1] = 0.0; oT1[3][2]= 0.0; oT1[3][3]= 1.0;

    matrix_mult(4, 4, wTo, 4, oT1, wT1);
    matrix_mult(4, 4, wT1, 2, v, vw);
    polygon[2].x = (short) (W2DX(zoom, vw[0][0]));
    polygon[2].y = (short) (W2DY(zoom, vw[1][0]));
    polygon[3].x = (short) (W2DX(zoom, vw[0][1]));
    polygon[3].y = (short) (W2DY(zoom, vw[1][1]));

    matrix_mult(4, 4, wT1, 1, vertex_o, vertex_w);
    draw_circle(W2DX(zoom, vertex_w[0]), W2DY(zoom, vertex_w[1]),
		W2DR(zoom,obj.Rsphere),FILL);

    c = cos(4.0*M_PI/3.0); s = sin(4.0*M_PI/3.0);
    oT2[0][0] =  c;  oT2[0][1] = -s;  oT2[0][2]= 0.0; oT2[0][3]= 0.0;
    oT2[1][0] =  s;  oT2[1][1] =  c;  oT2[1][2]= 0.0; oT2[1][3]= 0.0;
    oT2[2][0] = 0.0; oT2[2][1] = 0.0; oT2[2][2]= 1.0; oT2[2][3]= 0.0;
    oT2[3][0] = 0.0; oT2[3][1] = 0.0; oT2[3][2]= 0.0; oT2[3][3]= 1.0;

    matrix_mult(4, 4, wTo, 4, oT2, wT2);
    matrix_mult(4, 4, wT2, 2, v, vw);
    polygon[4].x = (short) (W2DX(zoom, vw[0][0]));
    polygon[4].y = (short) (W2DY(zoom, vw[1][0]));
    polygon[5].x = (short) (W2DX(zoom, vw[0][1]));
    polygon[5].y = (short) (W2DY(zoom, vw[1][1]));

    matrix_mult(4, 4, wT2, 1, vertex_o, vertex_w);
    draw_circle(W2DX(zoom, vertex_w[0]), W2DY(zoom, vertex_w[1]),
		W2DR(zoom,obj.Rsphere),FILL);

    XFillPolygon(display, pixmap, gc, polygon, 6, Convex, CoordModeOrigin);
  }
}

void draw_eye(base, eye)
Base base;
Eye eye;
{
  double px, py;
  double rx, ry, from_x, from_y, to_x, to_y;
  double lambda_x, lambda_y;
  int xbin, ybin;

  px = base.wTb[0][0]*eye.position[0] + base.wTb[0][1]*eye.position[1] +
    base.wTb[0][3];
  py = base.wTb[1][0]*eye.position[0] + base.wTb[1][1]*eye.position[1] +
    base.wTb[1][3];

  from_x = px; from_y = py;
  rx = cos(base.theta + eye.theta);
  ry = sin(base.theta + eye.theta);

  // old code to cut off eye gaze at map boundary
  //  lambda_x = lambda_y = 1000.0;
  //  if (rx > 0.1)	lambda_x = (MAX_X - px) / rx;
  //  else if (rx < -0.1) lambda_x = (MIN_X - px) / rx;
  //  if (ry > 0.1) lambda_y = (MAX_Y - py) / ry;
  //  else if (ry < -0.1) lambda_y = (MIN_Y - py) / ry;
  //
  //  if (lambda_x < lambda_y) {
  //  if (rx > 0.0) to_x = MAX_X - XDELTA;
  //  else to_x = MIN_X + XDELTA;
  //  to_y = py + lambda_x * ry;
  //  } 
  //  else {
  //  if (ry > 0.0) to_y = MAX_Y - YDELTA;
  //  else to_y = MIN_Y + YDELTA;
  //  to_x = px + lambda_y * rx;
  //  }

  //trace the eye direction till you hit an obstacle
  to_x = from_x;
  to_y = from_y;

  while (to_x < MAX_X && to_x > MIN_X && to_y < MAX_Y && to_y > MIN_Y) {
    //get bin for location
    ybin = (int)((MAX_Y - to_y)/YDELTA);
    xbin = (int)((to_x - MIN_X)/XDELTA);

    //check for obstacle collision
    if (Roger.world_map.occupancy_map[ybin][xbin] == OBSTACLE) {
      break;
    }
    to_x += rx * 0.001;
    to_y += ry * 0.001;
  }

  XSetForeground(display, gc, world_colors[GAZE_COLOR].display_color);
  XDrawLine(display, pixmap, gc, 
	    W2DX(zoom,from_x), W2DY(zoom,from_y),
	    W2DX(zoom,to_x), W2DY(zoom,to_y));

  XSetForeground(display, gc, foreground);
  draw_circle(W2DX(zoom,px), W2DY(zoom,py), W2DR(zoom,R_EYE), NOFILL);
  draw_circle(W2DX(zoom, px+(R_EYE-R_PUPIL)*rx),
	      W2DY(zoom, py+(R_EYE-R_PUPIL)*ry), W2DR(zoom,R_PUPIL), FILL);
}

void draw_image(eye)
int eye; 
{
  register int i, color, dx;

  XSetForeground(display, gc, foreground);
  if (eye == LEFT)
    XDrawRectangle(display, pixmap, gc,
		   (LEFT_IMAGE_X-1), (IMAGE_Y-1),
		   (IMAGE_WIDTH+1), (PIXEL_HEIGHT+1));
  else if (eye == RIGHT)
    XDrawRectangle(display, pixmap, gc, 
		   (RIGHT_IMAGE_X-1), (IMAGE_Y-1),
		   (IMAGE_WIDTH+1), (PIXEL_HEIGHT+1));

  for (i = 0, dx = 0; i < NPIXELS; i++, dx += PIXEL_WIDTH) {
    color = eyes[eye].image[i];
    XSetForeground(display, gc, world_colors[color].display_color);
    
    if (eye == LEFT)
      XFillRectangle(display, pixmap, gc, 
		     (LEFT_IMAGE_X+dx), IMAGE_Y, PIXEL_WIDTH, PIXEL_HEIGHT);
    else if (eye == RIGHT)
      XFillRectangle(display, pixmap, gc,
		     (RIGHT_IMAGE_X+dx), IMAGE_Y, PIXEL_WIDTH, PIXEL_HEIGHT);
  }
}

void draw_roger(mobile_base, arms, eyes)
Base mobile_base;
Arm arms[NARMS][NARM_FRAMES];
Eye eyes[NEYES];
{
  register int i, j;
  double r_b[4], r_w[4], fhat[2];
  double theta1, theta2, mag;
  double temp0[4][4], temp1[4][4];
  XPoint rect[4];
  //void draw_history();

  /******************************************************************/
  /* draw mobile base */
  XSetForeground(display, gc, foreground);
  draw_circle(W2DX(zoom,mobile_base.wTb[0][3]),
	      W2DY(zoom,mobile_base.wTb[1][3]), W2DR(zoom,R_BASE), NOFILL);

  // draw contact forces on object from body
  mag = sqrt(SQR(mobile_base.extForce[X]) + SQR(mobile_base.extForce[Y]));

  if (mag > 0.0) {
    fhat[X] = mobile_base.extForce[X] / mag;
    fhat[Y] = mobile_base.extForce[Y] / mag;

    XDrawLine(display, pixmap, gc, 
	      W2DX(zoom, mobile_base.x - R_BASE*fhat[X]),
	      W2DY(zoom, mobile_base.y - R_BASE*fhat[Y]),
	      W2DX(zoom, mobile_base.x - (R_BASE+0.08)*fhat[X]),
	      W2DY(zoom, mobile_base.y - (R_BASE+0.08)*fhat[Y]));

    //    XDrawLine(display, pixmap, gc,
    //      W2DX(zoom, mobile_base.x + R_BASE*cos(mobile_base.contact_theta)),
    //      W2DY(zoom, mobile_base.y + R_BASE*sin(mobile_base.contact_theta)),
    //      W2DX(zoom, mobile_base.x + R_BASE*cos(mobile_base.contact_theta)
    //	   + 0.08*mobile_base.extforce[Y]/mag),
    //      W2DY(zoom, mobile_base.y + R_BASE*sin(mobile_base.contact_theta)
    //	   - 0.08*mobile_base.extforce[X]/mag));
  }

  //  draw_wheels();
  r_b[0] = R_BASE/2.0; r_b[1] = R_BASE+WHEEL_THICKNESS; 
  r_b[2] = 0.0; r_b[3] = 1.0;
  matrix_mult(4, 4, mobile_base.wTb, 1, r_b, r_w);
  draw_circle(W2DX(zoom,r_w[0]), W2DY(zoom,r_w[1]), W2DR(zoom,WHEEL_THICKNESS),
	      FILL);
  r_b[0] = -R_BASE/2.0; r_b[1] = R_BASE+WHEEL_THICKNESS;
  r_b[2] = 0.0; r_b[3] = 1.0;
  matrix_mult(4, 4, mobile_base.wTb, 1, r_b, r_w);
  draw_circle(W2DX(zoom,r_w[0]), W2DY(zoom,r_w[1]), W2DR(zoom,WHEEL_THICKNESS),
	      FILL);

  r_b[0] = R_BASE/2.0; r_b[1] = R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  matrix_mult(4, 4, mobile_base.wTb, 1, r_b, r_w);
  rect[0].x = (short) (W2DX(zoom, r_w[0]));
  rect[0].y = (short) (W2DY(zoom, r_w[1]));

  r_b[0] = R_BASE/2.0; r_b[1] = (R_BASE+2*WHEEL_THICKNESS);
  r_b[2] = 0.0; r_b[3] = 1.0;
  matrix_mult(4, 4, mobile_base.wTb, 1, r_b, r_w);
  rect[1].x = (short) (W2DX(zoom, r_w[0]));
  rect[1].y = (short) (W2DY(zoom, r_w[1]));

  r_b[0] = -R_BASE/2.0; r_b[1] = (R_BASE+2*WHEEL_THICKNESS); 
  r_b[2]=0.0; r_b[3]=1.0;
  matrix_mult(4, 4, mobile_base.wTb, 1, r_b, r_w);
  rect[2].x = (short) (W2DX(zoom, r_w[0]));
  rect[2].y = (short) (W2DY(zoom, r_w[1]));

  r_b[0] = -R_BASE / 2.0; r_b[1] = R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  matrix_mult(4, 4, mobile_base.wTb, 1, r_b, r_w);
  rect[3].x = (short) (W2DX(zoom, r_w[0]));
  rect[3].y = (short) (W2DY(zoom, r_w[1]));

  XFillPolygon(display, pixmap, gc, rect, 4, Convex, CoordModeOrigin);

  r_b[0] = R_BASE/2.0; r_b[1] = -R_BASE-WHEEL_THICKNESS;
  r_b[2] = 0.0; r_b[3] = 1.0;
  matrix_mult(4, 4, mobile_base.wTb, 1, r_b, r_w);
  draw_circle(W2DX(zoom, r_w[0]), W2DY(zoom, r_w[1]),
	      W2DR(zoom,WHEEL_THICKNESS),FILL);
  r_b[0] = -R_BASE/2.0; r_b[1] = -R_BASE-WHEEL_THICKNESS; r_b[2] = 0.0;
  r_b[3] = 1.0;
  matrix_mult(4, 4, mobile_base.wTb, 1, r_b, r_w);
  draw_circle(W2DX(zoom, r_w[0]), W2DY(zoom, r_w[1]),
	      W2DR(zoom,WHEEL_THICKNESS),FILL);

  r_b[0] = R_BASE/2.0; r_b[1] = -R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  matrix_mult(4, 4, mobile_base.wTb, 1, r_b, r_w);
  rect[0].x = (short) (W2DX(zoom, r_w[0]));
  rect[0].y = (short) (W2DY(zoom, r_w[1]));

  r_b[0] = R_BASE/2.0; r_b[1] = -(R_BASE+2*WHEEL_THICKNESS);
  r_b[2] = 0.0; r_b[3]=1.0;
  matrix_mult(4, 4, mobile_base.wTb, 1, r_b, r_w);
  rect[1].x = (short) (W2DX(zoom, r_w[0]));
  rect[1].y = (short) (W2DY(zoom, r_w[1]));

  r_b[0] = -R_BASE/2.0; r_b[1] = -(R_BASE+2*WHEEL_THICKNESS);
  r_b[2] = 0.0; r_b[3]=1.0;
  matrix_mult(4, 4, mobile_base.wTb, 1, r_b, r_w);
  rect[2].x = (short) (W2DX(zoom, r_w[0]));
  rect[2].y = (short) (W2DY(zoom, r_w[1]));

  r_b[0] = -R_BASE/2.0; r_b[1] = -R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  matrix_mult(4, 4, mobile_base.wTb, 1, r_b, r_w);
  rect[3].x = (short) (W2DX(zoom, r_w[0]));
  rect[3].y = (short) (W2DY(zoom, r_w[1]));

  XFillPolygon(display, pixmap, gc, rect, 4, Convex, CoordModeOrigin);
  /******************************************************************/
  /* draw eyes */
  for (i = 0; i < NEYES; i++) {
    draw_eye(mobile_base, eyes[i]);

    /******************************************************************/
    /* draw displays **************************************************/
    /* draw coordinate in configuration space for left and right eyes */
    XSetForeground(display, gc, world_colors[EYE_COLOR].display_color);
    if (i == LEFT)
      XFillRectangle(display, pixmap, gc,
		     T12LD(eyes[i].theta), T22LD(0.0), 
		     (T2DR(TDELTA)+1), (T2DR(TDELTA)+1));
    else if (i == RIGHT)
      XFillRectangle(display, pixmap, gc,
		     T12RD(eyes[i].theta), T22RD(0.0), 
		     (T2DR(TDELTA)+1), (T2DR(TDELTA)+1));
  }
  /******************************************************************/
  /* draw arms */
  for (j=0; j<NARMS; j++) {
    XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
    matrix_mult(4, 4, mobile_base.wTb, 4, arms[j][0].iTj, temp0);

    for (i=1; i<NARM_FRAMES; i++) {
      matrix_mult(4, 4, temp0, 4, arms[j][i].iTj, temp1);
      XDrawLine(display, pixmap, gc, 
		W2DX(zoom,temp0[0][3]), W2DY(zoom,temp0[1][3]),
		W2DX(zoom,temp1[0][3]), W2DY(zoom,temp1[1][3]));
      if (i==(NARM_FRAMES-1))
	draw_circle(W2DX(zoom,temp1[0][3]), W2DY(zoom,temp1[1][3]),
		    W2DR(zoom,R_TACTILE), FILL);
      else {
	draw_circle(W2DX(zoom,temp1[0][3]), W2DY(zoom,temp1[1][3]),
		    W2DR(zoom,R_JOINT), NOFILL);
	matrix_copy(4, 4, temp1, temp0);
      }
    }

    // draw endpoint forces
    mag = sqrt(SQR(arms[j][NARM_FRAMES-1].extForce[X]) + 
	       SQR(arms[j][NARM_FRAMES-1].extForce[Y]));
    if (mag>0.0) {
      // XSetForeground(display, gc, world_colors[GAZE_COLOR].display_color);
      XSetForeground(display, gc, foreground);
      fhat[X] = arms[j][NARM_FRAMES-1].extForce[X]/mag;
      fhat[Y] = arms[j][NARM_FRAMES-1].extForce[Y]/mag;
      XDrawLine(display, pixmap, gc,
		W2DX(zoom,(temp1[0][3]+R_TACTILE*fhat[X])),
		W2DY(zoom,(temp1[1][3]+R_TACTILE*fhat[Y])),
		W2DX(zoom,(temp1[0][3]+(R_TACTILE+0.08)*fhat[X])),
		W2DY(zoom,(temp1[1][3]+(R_TACTILE+0.08)*fhat[Y])));

      // XDrawLine(display, pixmap, gc,
      //  W2DX(zoom,temp1[0][3]), W2DY(zoom,temp1[1][3]),
      //  W2DX(zoom,temp1[0][3]+0.125*arms[j][NARM_FRAMES-1].extForce[Y]/mag),
      //  W2DY(zoom,temp1[1][3]-0.125*arms[j][NARM_FRAMES-1].extForce[X]/mag));
    }

    /******************************************************************/
    /* draw displays **************************************************/
    /* draw coordinate in configuration space for left and right arms */
    if (j == LEFT)
      XFillRectangle(display, pixmap, gc,
		     T12LD(arms[j][1].theta), T22LD(arms[j][2].theta),
		     (T2DR(TDELTA)+1), (T2DR(TDELTA)+1));
    else if (j == RIGHT)
      XFillRectangle(display, pixmap, gc,
		     T12RD(arms[j][1].theta), T22RD(arms[j][2].theta),
		     (T2DR(TDELTA)+1), (T2DR(TDELTA)+1));
  }
  if (HISTORY) draw_history();

  /* visual images *************************************************/
  draw_image(LEFT);
  draw_image(RIGHT);
}

#define FRAME_L         0.08

// observations in world coordinates
void draw_observation(obs)
Observation obs;
{
  double a, b, c, root[2];
  double dx, dy, mag, eigenvalue[2], eigenvector[2][2];
  double ref_w[4], ref_b[4], wTb[4][4], bTw[4][4];
  double theta, dx0, dy0, dx1, dy1;

  // cov = [A  B] => det |JJt| = a(lambda)^2 + b(lambda) + c
  //       [B  C]
  a = 1.0;
  b = -(obs.cov[0][0] + obs.cov[1][1]);
  c = obs.cov[0][0] * obs.cov[1][1] - obs.cov[1][0] * obs.cov[0][1];

  root[0] = (-b + sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);
  root[1] = (-b - sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);

  if (fabs(obs.cov[1][0]) > 0.0001) {
    dy = 1.0;
    dx = -(obs.cov[1][1]-root[0])/obs.cov[1][0];

    mag = sqrt(SQR(dx) + SQR(dy));
    //    eigenvalue[0] = scale*sqrt(root[0]);
    eigenvalue[0] = sqrt(root[0]);
    eigenvector[0][0] = dx / mag;
    eigenvector[0][1] = dy / mag;

    // the eigenvector for root 1
    //    eigenvalue[1] = scale*sqrt(root[1]);
    eigenvalue[1] = sqrt(root[1]);
    eigenvector[1][0] = -eigenvector[0][1];
    eigenvector[1][1] = eigenvector[0][0];
  }
  // when ball is directly in front of Roger:
  else{
    // set eigenvalue 0 to be the greater root
    if (fabs(root[0]) > fabs(root[1])) {
      eigenvalue[0] = sqrt(root[0]);
      eigenvalue[1] = sqrt(root[1]);
    }
    else {
      eigenvalue[0] = sqrt(root[1]);
      eigenvalue[1] = sqrt(root[0]);
    }

    // when cov item A > cov item C, Roger is facing either of the green
    // walls; when cov item A < cov item C, Roger is facing either of the
    // blue walls
    if (fabs(obs.cov[0][0]) > fabs(obs.cov[1][1])) {
      eigenvector[0][0] = 1.0;
      eigenvector[0][1] = 0.0;
      eigenvector[1][0] = -0.0;
      eigenvector[1][1] = 1.0;
    }
    else {
      eigenvector[0][0] = 0.0;
      eigenvector[0][1] = 1.0;
      eigenvector[1][0] = -1.0;
      eigenvector[1][1] = 0.0;
    }
  }

  // all observations will be displayed in green goal_color
  XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);
  
  // draw cross hair
  XDrawLine(display, pixmap, gc, 
	    W2DX(zoom,(obs.pos[X]-(FRAME_L/2.0)*eigenvector[0][X])),
	    W2DY(zoom,(obs.pos[Y]-(FRAME_L/2.0)*eigenvector[0][Y])),
	    W2DX(zoom,(obs.pos[X]+(FRAME_L/2.0)*eigenvector[0][X])),
	    W2DY(zoom,(obs.pos[Y]+(FRAME_L/2.0)*eigenvector[0][Y])));
  XDrawLine(display, pixmap, gc, 
	    W2DX(zoom,(obs.pos[X]-(FRAME_L/2.0)*eigenvector[1][X])),
	    W2DY(zoom,(obs.pos[Y]-(FRAME_L/2.0)*eigenvector[1][Y])),
	    W2DX(zoom,(obs.pos[X]+(FRAME_L/2.0)*eigenvector[1][X])),
	    W2DY(zoom,(obs.pos[Y] + (FRAME_L/2.0)*eigenvector[1][Y])));
  dx0 = eigenvalue[0] * eigenvector[0][X];
  dy0 = eigenvalue[0] * eigenvector[0][Y];

  //printf("observation cov:\n");
  //printf("\t%lf %lf\n\t%lf %lf\n", obs.cov[0][0], obs.cov[0][1],
  //                                 obs.cov[1][0], obs.cov[1][1]);

  for (theta = 0.0; theta <= 2*M_PI; theta += M_PI / 20.0) {
    dx1 = (eigenvalue[0]*cos(theta))*eigenvector[0][X] + 
      (eigenvalue[1]*sin(theta))*eigenvector[1][X];
    dy1 = (eigenvalue[0]*cos(theta))*eigenvector[0][Y] + 
      (eigenvalue[1]*sin(theta))*eigenvector[1][Y];
    XDrawLine(display, pixmap, gc,
	      W2DX(zoom,(obs.pos[X] + dx0)), W2DY(zoom,(obs.pos[Y] + dy0)),
	      W2DX(zoom,(obs.pos[X] + dx1)), W2DY(zoom,(obs.pos[Y] + dy1)));
    dx0 = dx1;
    dy0 = dy1;
  }
}

#define STEP         0.01
#define STREAM_SPACE 1

/* the streamline visualization is not yet implemented in this version */
void draw_streamlines(color)
int color;
{
  void sor();
  double from[2], to[2];

  int i, row, col, current_row, current_col, already_used[NBINS][NBINS], count;
  double grad[2], oldx, oldy, x, y, mag, compute_gradient();
  void mark_used(), draw_current_streamline();

  /* make sure the harmonic function is fully converged */
  sor(&Roger);
  
  // initialize auxilliary structure for controlling the
  // density of streamlines rendered
  for (row=0; row<NBINS; ++row) {
    for (col=0; col<NBINS; ++col) {
      already_used[row][col] = FALSE;
    }
  }

  XSetForeground(display, gc, world_colors[GREEN].display_color);

  // if [col,row] is FREESPACE and at least one of its neighbors 
  // is OBSTACLE (could be DIRICHLET and NEUMANN), then draw a streamline
  for (row=1;row<(NBINS-1); ++row) {
    for (col=1;col<(NBINS-1); ++col) {
      if ((Roger.world_map.occupancy_map[row][col] == FREESPACE) &&
	  ((Roger.world_map.occupancy_map[row-1][col-1] == OBSTACLE) ||
	   (Roger.world_map.occupancy_map[row-1][col] == OBSTACLE)   ||
	   (Roger.world_map.occupancy_map[row-1][col+1] == OBSTACLE) ||
	   (Roger.world_map.occupancy_map[row][col-1] == OBSTACLE)   ||
	   (Roger.world_map.occupancy_map[row][col+1] == OBSTACLE)   ||
	   (Roger.world_map.occupancy_map[row+1][col-1] == OBSTACLE) ||
	   (Roger.world_map.occupancy_map[row+1][col] == OBSTACLE)   ||
	   (Roger.world_map.occupancy_map[row+1][col+1] == OBSTACLE) ) &&
	  (!already_used[row][col]) ) {
	  
	/* follow a stream line */
	oldx = MIN_X + (col+0.5)*XDELTA;
	oldy = MAX_Y - (row+0.5)*YDELTA;
	//	draw_current_streamline(row,col,oldx, oldy);

	count = 0;
	current_row = row;
	current_col = col;
	while ((Roger.world_map.occupancy_map[current_row][current_col]!=GOAL)
	       && (mag=compute_gradient(oldx, oldy, &Roger, grad))
	       && (count++ < 500)) {
	  
	  x = oldx - STEP*grad[X];
	  y = oldy - STEP*grad[Y];

	  XDrawLine(display, pixmap, gc, W2DX(zoom,oldx), W2DY(zoom,oldy),
		    W2DX(zoom,x), W2DY(zoom,y));
	  /*  circle((int)x,(int)y,2,background);  */
	  oldx=x; oldy=y;
	  current_row = (int)(y - MAX_Y)/YDELTA;
	  current_col = (int)(MIN_X - x)/XDELTA;
	}

	//	mark_used((col+1),(row+1),already_used); 
	mark_used(row, col, already_used); 
      }
    }
  }
  //  draw_history();
  x_expose();
}
      
void draw_current_streamline()
{
  int count, current_row, current_col;
  double x,y,grad[2];
  double compute_gradient();
  double oldx = Roger.base_position[X];
  double oldy = Roger.base_position[Y];
  double mag = compute_gradient(oldx, oldy, &Roger, grad);

  XSetForeground(display, gc, world_colors[BLUE].display_color);

  count = 0;
  current_row = (int)(oldy - MAX_Y)/YDELTA;
  current_col = (int)(MIN_X - oldx)/XDELTA;
  while ((Roger.world_map.occupancy_map[current_row][current_col] != GOAL)
	 && (mag=compute_gradient(oldx, oldy, &Roger, grad))
	 && (count++ < 500)) {
	  
    x = oldx - STEP*grad[X];
    y = oldy - STEP*grad[Y];

    XDrawLine(display, pixmap, gc, W2DX(zoom,oldx), W2DY(zoom,oldy),
		    W2DX(zoom,x), W2DY(zoom,y));
    oldx=x; oldy=y;
    current_row = (int)(y - MAX_Y)/YDELTA;
    current_col = (int)(MIN_X - x)/XDELTA;
  }
  x_expose();
}

void mark_used(r,c,aux)
int r, c;
int aux[NBINS][NBINS];
{
  int i,j;
  double dist;

  for (i=-STREAM_SPACE; i<=STREAM_SPACE; ++i) {
    for (j=-STREAM_SPACE; j<=STREAM_SPACE; ++j) {
      dist = sqrt(SQR((double)i) + SQR((double)j));
      if ((dist < (2.0*STREAM_SPACE + 1.0)) 
	     && ((r+i) >= 0) && ((r+i) < NBINS) 
	     && ((c+j) >= 0) && ((c+j) < NBINS))
	aux[r+i][c+j] = TRUE;
    }
  }
}

#define VMAG 0.5

// observations in world coordinates
void draw_estimate0(scale, est)
double scale;
Estimate est;
{
  draw_circle(W2DX(ZOOM_SCALE,est.state[X]), W2DY(ZOOM_SCALE,est.state[Y]),
	      W2DR(ZOOM_SCALE,R_BASE), NOFILL);

  x_draw_line(BLUE,est.state[X],est.state[Y],
	      (est.state[X]+VMAG*est.state[XDOT]),
	      (est.state[Y]+VMAG*est.state[YDOT]));
}

// estimates in world coordinates
void draw_estimate(scale, est)
double scale;
Estimate est;
{
  double a, b, c, root[2];
  double dx, dy, mag, eigenvalue[2], eigenvector[2][2];
  double ref_w[2], ref_b[2], wTb[4][4], bTw[4][4];
  double theta, dx0, dy0, dx1, dy1;

  //  printf("inside draw_estimate()\n");
  //  printf("x=%6.4lf y=%6.4lf xdot=%6.4lf ydot=%6.4lf\n\n",
  //	 est.state[X], est.state[Y], est.state[XDOT], est.state[YDOT]);

  //printf("   %lf %lf\n", est.cov[0][0], est.cov[0][1]);
  //printf("   %lf %lf\n", est.cov[1][0], est.cov[1][1]);

  // cov = [A  B] => det |JJt| = a(lambda)^2 + b(lambda) + c
  //       [B  C]
  a = 1.0;
  b = -(est.cov[0][0] + est.cov[1][1]);
  c = est.cov[0][0] * est.cov[1][1] - est.cov[1][0] * est.cov[0][1];

  root[0] = (-b + sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);
  root[1] = (-b - sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);

  if (fabs(est.cov[1][0]) > 0.000001) {
    dy = 1.0;
    dx = -(est.cov[1][1]-root[0])/est.cov[1][0];

    mag = sqrt(SQR(dx) + SQR(dy));
    //      eigenvalue[0] = scale*0.0221*sqrt(root[0]);
    eigenvalue[0] = scale*sqrt(root[0]);
    eigenvector[0][0] = dx / mag;
    eigenvector[0][1] = dy / mag;

    // the eigenvector for root 1
    //      eigenvalue[1] = scale*0.0221*sqrt(root[1]);
    eigenvalue[1] = scale*sqrt(root[1]);
    eigenvector[1][0] = -eigenvector[0][1];
    eigenvector[1][1] = eigenvector[0][0];
  }
  else{
    // set eigenvalue[0] to be the greater root
    if (fabs(root[0]) > fabs(root[1])) {
      eigenvalue[0] = scale*sqrt(root[0]);
      eigenvalue[1] = scale*sqrt(root[1]);
    }
    else {
      eigenvalue[0] = scale*sqrt(root[1]);
      eigenvalue[1] = scale*sqrt(root[0]);
    }

    // when cov item A > cov item C, Roger is facing either of the green
    // walls; when cov item A < cov item C, Roger is facing either of the
    // blue walls
    if (fabs(est.cov[0][0]) > fabs(est.cov[1][1])) {
      eigenvector[0][0] = 1.0;
      eigenvector[0][1] = 0.0;
      eigenvector[1][0] = -0.0;
      eigenvector[1][1] = 1.0;
    }
    else {
      eigenvector[0][0] = 0.0;
      eigenvector[0][1] = 1.0;
      eigenvector[1][0] = -1.0;
      eigenvector[1][1] = 0.0;
    }
  }

  // all "estimates" will be displayed in blue
  XSetForeground(display, gc, world_colors[BLUE].display_color);

  // draw cross hair
  XDrawLine(display, pixmap, gc,
	    W2DX(zoom,(est.state[X]-(FRAME_L/2.0)*eigenvector[0][X])),
	    W2DY(zoom,(est.state[Y]-(FRAME_L/2.0)*eigenvector[0][Y])),
	    W2DX(zoom,(est.state[X]+(FRAME_L/2.0)*eigenvector[0][X])),
	    W2DY(zoom,(est.state[Y]+(FRAME_L/2.0)*eigenvector[0][Y])));
  XDrawLine(display, pixmap, gc, 
	    W2DX(zoom,(est.state[X]-(FRAME_L/2.0)*eigenvector[1][X])),
	    W2DY(zoom,(est.state[Y]-(FRAME_L/2.0)*eigenvector[1][Y])),
	    W2DX(zoom,(est.state[X]+(FRAME_L/2.0)*eigenvector[1][X])),
	    W2DY(zoom,(est.state[Y]+(FRAME_L/2.0)*eigenvector[1][Y])));
  dx0 = eigenvalue[0] * eigenvector[0][X];
  dy0 = eigenvalue[0] * eigenvector[0][Y];

  //printf("estimate cov:\n");
  //printf("\t%lf %lf\n\t%lf %lf\n", est.cov[0][0], est.cov[0][1],
  //                                 est.cov[1][0], est.cov[1][1]);

  for (theta = 0.0; theta <= 2*M_PI; theta += M_PI / 20.0) {
    dx1 = (eigenvalue[0]*cos(theta))*eigenvector[0][X] +
      (eigenvalue[1]*sin(theta))*eigenvector[1][X];
    dy1 = (eigenvalue[0]*cos(theta))*eigenvector[0][Y] +
      (eigenvalue[1]*sin(theta))*eigenvector[1][Y];
    XDrawLine(display, pixmap, gc, 
	      W2DX(zoom,(est.state[X]+dx0)), W2DY(zoom,(est.state[Y]+dy0)),
	      W2DX(zoom,(est.state[X]+dx1)),
	      W2DY(zoom,(est.state[Y] + dy1)));
    dx0 = dx1;
    dy0 = dy1;
  }
}

void draw_ellipse(est)
Estimate est;
{
  double m[2][2], a, b, c, root[2];
  double dx, dy, mag, eigenvalue[2], eigenvector[2][2];
  double theta, dx0, dy0, dx1, dy1;

  //  printf("observation time = %lf  time = %lf\n", est.t, simtime);
  
  //if ((est.t == simtime)) {

  // DRAW THE CURRENT ESTIMATED STATE VARIABLES AND THEIR VELOCITIES
  draw_circle(W2DX(zoom,est.state[X]), W2DY(zoom,est.state[Y]),
	      W2DR(zoom,R_JOINT), FILL);

  m[0][0] = est.cov[0][0];
  m[0][1] = est.cov[0][1];
  m[1][0] = est.cov[1][0];
  m[1][1] = est.cov[1][1];

  // cov = [A  B] => det |JJt| = a(lambda)^2 + b(lambda) +c
  //       [B  C]
  a = 1.0;
  b = -(m[0][0] + m[1][1]);
  c = m[0][0] * m[1][1] - m[1][0] * m[0][1];

  root[0] = (-b + sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);
  root[1] = (-b - sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);

  // the eigenvector for root 0
  dy = 1.0;
  dx = -m[0][1] / (m[0][0] - root[0]);
  mag = sqrt(SQR(dx) + SQR(dy));
  eigenvalue[0] = sqrt(root[0]);
  eigenvector[0][0] = dx / mag;
  eigenvector[0][1] = dy / mag;

  // the eigenvector for root 1
  dy = 1.0;
  dx = -m[0][1] / (m[0][0] - root[1]);
  mag = sqrt(SQR(dx) + SQR(dy));
  eigenvalue[1] = sqrt(root[1]);
  eigenvector[1][0] = dx / mag;
  eigenvector[1][1] = dy / mag;

  dx0 = eigenvalue[0] * eigenvector[0][X];
  dy0 = eigenvalue[0] * eigenvector[0][Y];
  for (theta = M_PI / 20.0; theta < 2*M_PI; theta += M_PI / 20.0) {
    dx1 = (eigenvalue[0]*cos(theta))*eigenvector[0][X] +
      (eigenvalue[1]*sin(theta))*eigenvector[1][X];
    dy1 = (eigenvalue[0]*cos(theta))*eigenvector[0][Y] +
      (eigenvalue[1]*sin(theta))*eigenvector[1][Y];
    XDrawLine(display, pixmap, gc, 
	      W2DX(zoom,(est.state[X]+dx0)), W2DY(zoom,(est.state[Y]+dy0)),
	      W2DX(zoom,(est.state[X]+dx1)),
	      W2DY(zoom,(est.state[Y]+dy1)));
    dx0 = dx1;
    dy0 = dy1;
  }
  // }
}

typedef struct _visible_object {
  double dx, dy;
  double radius;
  int color;
} VisibleObject;

typedef struct _circle {
  double x, y;
  double radius;
  int color;
} Circle;

typedef struct _edge {
  double endpt1[2];
  double endpt2[2];
  int color;
} Edge;

typedef struct _polygon {
  int n; // n>2
  Circle * vertices;
  Edge * sides;
  int color;
} Polygon;

void insertion_sort(vob, sort, num)
VisibleObject *vob;
int *sort, num;
{
  int i,j, temp;

  for(i=0; i<num; i++) sort[i] = i;

  for (i=1; i<num; i++) {
    j = i - 1;
    temp = sort[i];
    while ((j>=0) && 
	   (sqrt(SQR(vob[sort[j]].dx) + SQR(vob[sort[j]].dy)) <= 
	    sqrt(SQR(vob[temp].dx) + SQR(vob[temp].dy)))) {
      sort[j+1] = sort[j];
      j--;
    }
    sort[j+1] = temp;
  }
}

void pinhole_camera(vob, i)
VisibleObject vob;
int i;
{
  int j, low_bin_index, high_bin_index, paint_image;
  double phi, alpha;

  phi = atan2(vob.dy, vob.dx) - eyes[i].theta; // eye frame heading eye->object
  //  printf("      phi for eye %d = %6.4lf\n", i, phi);
  alpha = atan2(vob.radius, sqrt(SQR(vob.dx)+SQR(vob.dy) - SQR(vob.radius)));

  paint_image = FALSE;

  if (fabs(phi + alpha) < FOV) { /* CCW edge of the feature projects in FOV */
    high_bin_index = (int) (NPIXELS / 2.0 * (1.0 + tan(phi + alpha)));
    paint_image = TRUE;
  }
  else high_bin_index = 127;

  if (fabs(phi - alpha) < FOV) { /* CW edge of the feature projects in FOV */
    low_bin_index = (int) (NPIXELS / 2.0 * (1.0 + tan(phi - alpha)));
    paint_image = TRUE;
  }
  else low_bin_index = 0;

  // low_bin_index = MAX(low_bin_index,0);
  // high_bin_index = MIN(high_bin_index,(NPIXELS-1));

  if (paint_image) {
    for (j = low_bin_index; j <= high_bin_index; j++) {
      eyes[i].image[j] = vob.color;
    }
  }
  //  else {
  //     printf("FEATURE OUTSIDE FIELD OF VIEW \n");
  //  }
}

void make_images()
{
  int i, j, eye, o_index; // intensity[3];
  double x, y;
  double pb[4], pw[4], bTw[4][4];
  //  int feature_id,

  int sort[NBINS*NBINS];
  VisibleObject vobject[NBINS*NBINS];

  /* initialize image white */
  // make sure eye's images keep changing when roger is moving
  for (i = 0; i < NPIXELS; i++)
    eyes[LEFT].image[i] = eyes[RIGHT].image[i] = 100;

  for (eye = LEFT; eye <= RIGHT; ++eye) {
    //    first 3 elements in the range array are the hands and the ball
    /* LEFT_ARM - in body frame */
    SIMfwd_kinematics(LEFT, arms[LEFT][1].theta, arms[LEFT][2].theta, &x, &y);
    // convert to eye coordinates
    vobject[0].dx = x - eyes[eye].position[X];
    vobject[0].dy = y - eyes[eye].position[Y];
    vobject[0].radius = R_JOINT;
    vobject[0].color = ARM_COLOR;

    /* RIGHT_ARM - in body frame */
    SIMfwd_kinematics(RIGHT,arms[RIGHT][1].theta,arms[RIGHT][2].theta,&x,&y);
    vobject[1].dx = x - eyes[eye].position[X];
    vobject[1].dy = y - eyes[eye].position[Y];
    vobject[1].radius = R_JOINT;
    vobject[1].color = ARM_COLOR;

    /* OBJECT */
    HT_invert(mobile_base.wTb, bTw);
    pw[0] = toy.position[X]; pw[1] = toy.position[Y];
    pw[2] = 0.0; pw[3] = 1.0;
    //    SIMmatXvec(bTw, p_w, p_b);
    matrix_mult(4, 4, bTw, 1, pw, pb);
    vobject[2].dx = pb[X] - eyes[eye].position[X];
    vobject[2].dy = pb[Y] - eyes[eye].position[Y];
    vobject[2].radius = toy.Rsphere;
    vobject[2].color = OBJECT_COLOR;

    // after the first three, the rest are colored obstacles in the occupancy
    // grid
    o_index = 3; // points at the next empty element of the range array
    for (i=0;i<NBINS;++i) {
      y = MAX_Y - (i+0.5)*YDELTA;
      for (j=0;j<NBINS; ++j) {
	if (Roger.world_map.occupancy_map[i][j] == OBSTACLE) {
	  pw[0] = MIN_X + (j+0.5)*XDELTA; pw[1] = y;
	  pw[2] = 0.0; pw[3] = 1.0;
	  matrix_mult(4, 4, bTw, 1, pw, pb);
	  vobject[o_index].dx = pb[X] - eyes[eye].position[X];
	  vobject[o_index].dy = pb[Y] - eyes[eye].position[Y];
	  vobject[o_index].radius = R_OBSTACLE;
	  vobject[o_index++].color = Roger.world_map.color_map[i][j];
	}
      }
    }
    insertion_sort(vobject, sort, o_index);
    for (i=0; i<o_index; ++i)
      pinhole_camera(vobject[sort[i]], eye);
  }
}

void x_timer_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  int i, j, reset;
  static int render = RENDER_RATE, servo = SERVO_RATE;
  void goal_detection();

  if (servo++ == SERVO_RATE) {
    reset=FALSE; // reset eliminates user boundaries from occupancy grids
    write_interface(reset);
    control_roger(&Roger, simtime);
    read_interface();
    servo = 1;
  }

  // compute_external_forces(&mobile_base, arms, objects); updates "objects[]"
  // array, writes forces into mobile_base, arms, and toy data structures
  compute_external_forces(); 

  simulate_roger(&mobile_base, arms, eyes);

  simulate_object(&toy);

  if (render++ == RENDER_RATE) {
    if ((HISTORY) && (history_ptr < MAX_HISTORY)) {
      history[history_ptr].arm_pos[LEFT][0] = arms[LEFT][1].theta;
      history[history_ptr].arm_pos[LEFT][1] = arms[LEFT][2].theta;
      
      history[history_ptr].arm_pos[RIGHT][0] = arms[RIGHT][1].theta;
      history[history_ptr].arm_pos[RIGHT][1] = arms[RIGHT][2].theta;
      
      history[history_ptr].base_pos[0] = mobile_base.x;
      history[history_ptr].base_pos[1] = mobile_base.y;
      history[history_ptr].base_pos[2] = mobile_base.theta;

      ++history_ptr;
    }
    make_images();
    draw_all();
    render = 1;
  }
  simtime += DT;

  timer=XtAppAddTimeOut(app_con,TIMER_UPDATE,x_timer_proc,(XtPointer) NULL);
  // goal_detection(); // for shooting goals in soccer app
}

void goal_detection()
{
  char *temp, buf[20];

  if (toy.position[Y] < (MIN_Y + R_GOAL)) {
    printf("Scored at: %f\n",simtime);
    
    temp = gcvt(simtime,5,buf);
    printf("temp=%s\n",temp);

    XSetForeground(display, gc, foreground);
    XDrawString(display, pixmap, gc, W2DX(zoom,3.0), W2DY(zoom,1.8), temp, 5);
    XDrawString(display, pixmap, gc, T12RD(-0.45), T22RD(-3.6), "time=", 5);

    XkwSetWidgetLabel(start_w, "Start");
    XtRemoveTimeOut(timer);
    timer = 0;
  }
}

void Teleoperation_Cartesian_input_arms(roger, x, y, button)
Robot* roger;
double x;
double y;
int button;
{
  int limb = LEFT;

  printf("Arm goal input - x: %4.3f, y: %4.3f - button: %d\n", x, y, button);

  if (button == LEFT_BUTTON) {
    limb = LEFT;
    printf("world frame input for LEFT arm x=%6.4lf y=%6.4lf\n", x, y);
  }
  else if (button == RIGHT_BUTTON) {
    limb = RIGHT;
    printf("world frame input for RIGHT arm x=%6.4lf y=%6.4lf\n", x, y);
  }
  else return;
  inv_arm_kinematics(roger, limb, x,y); //PROJECT #2 - ArmKinematics/project2.c
}

void Teleoperation_Cartesian_input_base(roger, x, y, button)
Robot* roger;
double x;
double y;
int button;
{
  double dx, dy, theta;

  roger->base_setpoint[X] = x;
  roger->base_setpoint[Y] = y;
  dx = x - roger->base_position[X];
  dy = y - roger->base_position[Y];
  theta = roger->base_setpoint[THETA] = atan2(dy,dx);
  printf("world frame Base goal: x=%6.4lf y=%6.4lf theta=%6.4lf\n", x,y,theta);
}

//check if input is in configuration space area and return angles
int isConfigurationInput(x, y, q1, q2, side) 
double x, y;
double *q1, *q2;
int *side;
{
  double range = 0.0;

  *q1 = 0.0;
  *q2 = 0.0;
  *side = LEFT;

  //printf("Conf space check: %6.4lf, %6.4lf \n", x, y);

  //check if click is in configuration space area
  //X-Direction
  if (x < D2WX(100.0, T12LD(T1_MIN)) || (x > D2WX(100.0, T12LD(T1_MAX)) && 
      x < D2WX(100.0, T12RD(T1_MIN))) || x > D2WX(100.0, T12RD(T1_MAX))) {
    //printf("x-location out of bounds!!! %6.4lf \n", x);
    return FALSE;
  }
  //Y-Direction
  else if (y < D2WY(100.0, T22LD(T2_MIN)) || y > D2WY(100.0, T22LD(T2_MAX)) ) {
    //printf("y-location out of bounds!!! %6.4lf\n", y);
    return FALSE;
  }

  //calculate joint angles from click locations
  //left arm
  if (x < D2WX(100.0, T12LD(T1_MAX))) {
    *side = LEFT;
    range = D2WX(100.0, T12LD(T1_MAX)) - D2WX(100.0, T12LD(T1_MIN));
    *q1 = (x - (D2WX(100.0, T12LD(T1_MIN)) + range/2.0)) / (range/2.0) * M_PI;
    range = D2WY(100.0, T22LD(T2_MAX)) - D2WY(100.0, T22LD(T2_MIN));
    *q2 = (y - (D2WY(100.0, T22LD(T2_MIN)) + range/2.0)) / (range/2.0) * M_PI;
  }
  //right arm
  else {
    *side = RIGHT;
    range = D2WX(100.0, T12RD(T1_MAX)) - D2WX(100.0, T12RD(T1_MIN));
    *q1 = (x - (D2WX(100.0, T12RD(T1_MIN)) + range/2.0)) / (range/2.0) * M_PI;
    range = D2WY(100.0, T22RD(T2_MAX)) - D2WY(100.0, T22RD(T2_MIN));
    *q2 = (y - (D2WY(100.0, T22RD(T2_MIN)) + range/2.0)) / (range/2.0) * M_PI;
  }
  return TRUE;
}

//check if input is in cartesian space area
int isCartesianInput(x_input, y_input, x, y) 
double x_input, y_input;
double *x, *y;
{
  if (x_input<MIN_X || x_input>MAX_X || y_input<MIN_Y || y_input>MAX_Y) {
    //printf("Location out of bounds!!!\n");
    return FALSE;
  }
  *x = x_input;
  *y = y_input;

  return TRUE;
}

/***********************************************************************/
/** Input modes --- update the setpoint data structure:            *****/ 
/**    JOINT_ANGLE_INPUT mode: mouse(q1,q2) -> arm (q1, q2) ref    *****/
/**      BASE_GOAL_INPUT mode: mouse(x,y) -> base (x,y) ref	   *****/
/**      ARM_GOALS_INPUT mode: mouse(x,y) -> arm (x,y) ref  	   *****/
/**        BALL_POSITION mode: mouse(x,y) -> red ball (x,y)        *****/
/**           MAP_EDITOR mode: left button(x,y) -> obstacle        *****/
/**                            right button(x,y) -> goal           *****/
/***********************************************************************/
void update_setpoints(roger)
Robot * roger;
{
  int xbin, ybin;
  //void rrt_planning();
  double wTb[4][4], bTw[4][4], ref_b[4], ref_w[4], theta0, theta1, avg_theta;
  //cartesian coordinates
  double x, y;
  //configuration coordinates
  double q1, q2;
  //body side
  int body_side = 0;
  double range = 0.0;

  //*******************************************************************
  // initialize the configuration of the robot at each mode change
  //*******************************************************************
  if (init_control_flag) initialize_control_mode(roger);
  init_control_flag = FALSE;

  //*******************************************************************
  //***** process button input based on selected input mode ***********
  //*******************************************************************
  // Mode dependent button interfaces
  if (roger->button_event) {
    switch(roger->input_mode) {
    case JOINT_ANGLE_INPUT:
      //check if inputs are valid
      if (isConfigurationInput(roger->button_reference[X],
			       roger->button_reference[Y], &q1, &q2,
			       &body_side) == FALSE)
	break;

      if (roger->control_mode == PROJECT1) { // GUI inputs to Motor Units
	//left mouse button -> arms
	if (roger->button_event == LEFT_BUTTON) {
	  printf("q1=%6.4lf q2=%6.4lf\n", q1, q2);
	  
	  roger->arm_setpoint[body_side][0] = q1;
	  roger->arm_setpoint[body_side][1] = q2;
	}
	//right mouse button -> eyes
	else if (roger->button_event == RIGHT_BUTTON) {
	  printf("q1=%6.4lf \n", q1);
	  roger->eyes_setpoint[body_side] = q1;
	}
      }
      break;

    case BASE_GOAL_INPUT:
      //check if inputs are valid
      if (isCartesianInput(roger->button_reference[X], 
			   roger->button_reference[Y], &x, &y) == FALSE)
	break;

      if (roger->control_mode == PROJECT1)
	Teleoperation_Cartesian_input_base(roger, x, y, roger->button_event);
      break;

    case ARM_GOAL_INPUT:
      // BUTTON INTERFACE - ARM TELEOPERATOR - CARTESIAN CONTROL
      //check if inputs are valid
      if (isCartesianInput(roger->button_reference[X],
			   roger->button_reference[Y],&x,&y) == FALSE)
	break;
      //      printf("inside arm_goal_input case");
      if ((roger->control_mode == PROJECT1) ||
	  (roger->control_mode == PROJECT2))
	Teleoperation_Cartesian_input_arms(roger, x, y, roger->button_event);
      break;

    case TOY_INPUT:
      //check if inputs are valid
      if (isCartesianInput(roger->button_reference[X],
			   roger->button_reference[Y], &x, &y) == FALSE)
	break;
      else {
	if (roger->button_event == LEFT_BUTTON) {
	  place_object(x,y,CIRCLE);
	}
	else if (roger->button_event == RIGHT_BUTTON) {
	  place_object(x,y,TRIANGLE);
	}
	break;
      }
    case MAP_INPUT:
      if (isCartesianInput(roger->button_reference[X],
			   roger->button_reference[Y], &x, &y) == FALSE)
	break;

      //      int xbin, ybin; already defined above
      printf("Map editor input - x: %4.3f, y: %4.3f - button: %d\n",
	     x, y, roger->button_event);

      xbin = (x - MIN_X) / XDELTA;
      ybin = NBINS - (y - MIN_Y) / YDELTA;
      if ((xbin<0) || (xbin>(NBINS-1)) || (ybin<0) || (ybin>(NBINS-1))) {
	printf("Out of the boundary!!!\n");
      }
      else {
	if (roger->button_event==LEFT_BUTTON) {// obstacles in Cartesian space
	  if (roger->world_map.occupancy_map[ybin][xbin] == OBSTACLE) {
	    printf("deleting an obstacle xbin=%d  ybin=%d\n", xbin, ybin);
	    fflush(stdout);
	    roger->world_map.occupancy_map[ybin][xbin] = FREESPACE;
	    //	    delete_bin_bumper(xbin,ybin);
	  }
	  else if (roger->world_map.occupancy_map[ybin][xbin] == FREESPACE) {
	    printf("inserting an obstacle xbin=%d  ybin=%d\n", xbin, ybin);
	    fflush(stdout);
	    roger->world_map.occupancy_map[ybin][xbin] = OBSTACLE;
	    roger->world_map.potential_map[ybin][xbin] = 1.0;
	    roger->world_map.color_map[ybin][xbin] = LIGHTBLUE;
	  }
	}
	else if (roger->button_event == MIDDLE_BUTTON) { }
	else if (roger->button_event == RIGHT_BUTTON) {
	  if (roger->world_map.occupancy_map[ybin][xbin] == GOAL) {
	    printf("deleting an goal xbin=%d  ybin=%d\n", xbin, ybin);
	    fflush(stdout);
	    roger->world_map.occupancy_map[ybin][xbin] = FREESPACE;
	  }
	  else if (roger->world_map.occupancy_map[ybin][xbin] == FREESPACE) {
	    printf("inserting an goal xbin=%d  ybin=%d\n", xbin, ybin);
	    fflush(stdout);
	    roger->world_map.occupancy_map[ybin][xbin] = GOAL;
	    roger->world_map.potential_map[ybin][xbin] = 0.0;
	  }
	}
	//update harmonic map
	//sor(roger);
      }
      
      break;
    default:
      break;
    }
    roger->button_event = FALSE;
  }

  //*******************************************************************
  //******** perform control based on selected control mode ***********
  //*******************************************************************

  switch(roger->control_mode) {
     case PROJECT2:
       project2_control(roger, simtime);
       break;
     case PROJECT3:
       project3_control(roger, simtime);
       break;
     case PROJECT4:
       project4_control(roger,simtime);
       break;
     case PROJECT5:
       project5_control(roger, simtime);
       break;
     case PROJECT6:
       project6_control(roger, simtime);
       break;
     case PROJECT7:
       project7_control(roger, simtime);
       break;
     case PROJECT8:
       project8_control(roger, simtime);
       break;
     case PROJECT9:
       project9_control(roger, simtime);
       break;
     default:
       break;
  }
}

int main(argc, argv)
int argc;char **argv;
{
  int i;
  static String fallback_resources[] = {"*title:	Roger-the-Crab",
					"*Roger-the-Crab*x:	100",
					"*Roger-the-Crab*y:	100",NULL,};
  Widget toplevel, form, widget;
  void x_clear();

  //  void intersect_line_circle();
  //  intersect_line_circle();

  toplevel = XtAppInitialize(&app_con, "Roger-the-Crab", NULL, ZERO,
			     &argc, argv, fallback_resources, NULL, ZERO);
  form = XkwMakeForm(toplevel);
  widget = NULL;
  start_w = widget = 
    XkwMakeCommand(form,NULL,widget,x_start_proc,"Start",BOXW,BOXH);
  input_mode_w = widget = 
    XkwMakeCommand(form,NULL,widget,x_input_mode_proc,
		   "Input: Joint angles", BOXW, BOXH);
  control_mode_w = widget = 
    XkwMakeCommand(form,NULL,widget,x_control_mode_proc,
		   "1-Motor Units",BOXW,BOXH);
  room_w = widget = 
    XkwMakeCommand(form,NULL,widget,x_room_proc,"Room: 0", BOXW, BOXH);
  params_w = widget = 
    XkwMakeCommand(form,NULL,widget,x_params_proc,"Enter Params",BOXW,BOXH);
  stream_w = widget = 
    XkwMakeCommand(form,NULL,widget,x_visualize_proc,"Visualize",BOXW,BOXH);
  widget = XkwMakeCommand(form,NULL,widget,x_quit_proc,"Quit",BOXW,BOXH);
  canvas_w = widget = 
    XkwMakeCanvas(form, widget, NULL, x_canvas_proc, width, height);
  XtRealizeWidget(toplevel);
  display = XtDisplay(canvas_w);
  window = XtWindow(canvas_w);
  screen = DefaultScreen(display);
  depth = DefaultDepth(display, screen);
  foreground = BlackPixel(display, screen);
  background = WhitePixel(display, screen);

  gc = XCreateGC(display, window, 0, NULL);
  XSetFunction(display, gc, GXcopy);
  XSetForeground(display, gc, foreground);
  XSetBackground(display, gc, background);

  pixmap = XCreatePixmap(display, window, width, height, depth);
  x_clear();

  x_init_colors();

  reset=TRUE;
  initialize_room(&Roger);
  initialize_simulator(reset); // initializes world boundaries,
                               // mobile_base, eyes[2], arms[2], the object and
                               // the Roger interface structure
  simulate_roger(&mobile_base, arms, eyes);
  simulate_object(&toy);
  make_images();
  draw_all();

  XtAppMainLoop(app_con);
}
