/*************************************************************************/
/* File:        project1.c                                               */
/* Description: PD control analogs of the biological motor units for     */
/*              every degree of freedom: eyes, arms, base rotate and     */
/*              base translate --- motor units execute every simulated   */
/*              millisecond and are never disengaged. Higher-level       */
/*              control applications submit sequences of setpoints to    */
/*              combinations of motorunits.                              */
/* Date:        1-2015                                                   */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "matrix_math.h"

void update_setpoints();

// int write = 0;
// FILE * fpx, * fpy, * fptheta;
int getParamsFlag = 0;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int startFlag_arms = 0; //NOTE: Sets the default arm position at startup. Set to 0 to disable ~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void home_position(Robot * roger){
  roger->arm_setpoint[0][0] = 3.05;
  roger->arm_setpoint[1][0] = -3.05;

  roger->arm_setpoint[0][1] = -2.8;
  roger->arm_setpoint[1][1] = 2.8;
}

/*************************************************************************/
/* PROJECT #1 - COMPLETE THE FOLLOWING CONTROL PROCEDURES                */
// gains for the PD controllers EYES
double Kp_eye = 15;
double Kd_eye = 0.079;
double passive_Kd_eye = 0.001;

// ARMS 
double Kp_arm =  80;//60
double Kd_arm =  11.2;//10.5
double passive_Kd_arm = 1.0;

// BASE TRANSLATION
double Kp_base_trans = 185;
double Kd_base_trans = 36;
double passive_Kd_base_trans = 2.0;

// BASE ROTATION
double Kp_base_rot =  580;
double Kd_base_rot =  36.9;
double passive_Kd_base_rot = 1.0;
/*************************************************************************/

/* PROJECT #1.1 - PDController_armsf CONTROLLER FOR THE EYES                             */
/* setpoints are joint angle values in radians for the eyes              */
void PDController_eyes(roger, time)
Robot * roger;
double time;
{
  int i;
  double theta_error, theta_dot_error;

  for (i = 0; i < NEYES; i++) {
    theta_error = roger->eyes_setpoint[i] - roger->eye_theta[i];
    theta_dot_error = 0.0 - roger->eye_theta_dot[i];
    if (ACTUATE_EYES) {
      // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE EYE
      // USING YOUR GAINS
      roger->eye_torque[i] = Kp_eye*theta_error + Kd_eye*theta_dot_error;
    } 
    else roger->eye_torque[i] = passive_Kd_eye*theta_dot_error;
  }
}

/* PROJECT #1.2 - PD CONTROLLER FOR THE ARMS                             */
/* setpoints - joint angles in radians for the shoulders and elbows      */
void PDController_arms(roger, time)
Robot * roger;
double time;
{
  int i, j;
  double theta_error, theta_dot_error;
  if (startFlag_arms == 1){
  	roger->arm_setpoint[0][0] = 3.05;
  	roger->arm_setpoint[1][0] = -3.05;

  	roger->arm_setpoint[0][1] = -2.8;
  	roger->arm_setpoint[1][1] = 2.8;
  	startFlag_arms = 0;
  }
  
  for (i=LEFT; i<=RIGHT; ++i) {
    for (j=0; j<NARM_JOINTS; ++j) {

      theta_error = roger->arm_setpoint[i][j] - roger->arm_theta[i][j];
      theta_dot_error = 0.0 - roger->arm_theta_dot[i][j];

      while (theta_error > M_PI) theta_error -= 2.0 * M_PI;
      while (theta_error < -M_PI) theta_error += 2.0 * M_PI;

      // tune kp_arm and kd_arm by changing their value using enter_params()
      if (ACTUATE_ARMS) {
      // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE ARM
      // USING YOUR GAINS
	    	roger->arm_torque[i][j] = Kp_arm*theta_error + Kd_arm * theta_dot_error;
      }
      else {
	    	roger->arm_torque[i][j] = passive_Kd_arm * theta_dot_error;
      }
    }
  }
}

/* Base PD controller, Cartesian reference */
double PDBase_translate(roger, time) 
Robot * roger;
double time;
{ 
  double Fx, error[2], trans_error, trans_vel;

  error[X] = roger->base_setpoint[X] - roger->base_position[X];
  error[Y] = roger->base_setpoint[Y] - roger->base_position[Y];

  trans_error = error[X]*cos(roger->base_position[THETA]) +
    error[Y]*sin(roger->base_position[THETA]);

  trans_vel = roger->base_velocity[X]*cos(roger->base_position[THETA]) +
    roger->base_velocity[Y]*sin(roger->base_position[THETA]);

  if (ACTUATE_BASE) {
    // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE BASE
    // USING YOUR GAINS
    Fx = Kp_base_trans * trans_error - Kd_base_trans * trans_vel;
  }
  else {
    Fx = - passive_Kd_base_trans * trans_vel;
  }
	// if (write==1){
	// 	fprintf(fpx, "%6.10lf\n", error[X]);
	// 	fprintf(fpy, "%6.10lf\n", error[Y]);
	// }
  return(Fx);
}

/* Base PD controller, Cartesian reference */
double PDBase_rotate(roger, time) 
Robot * roger;
double time;
{
  double Mz, theta_error, theta_dot_error;

  theta_error = roger->base_setpoint[THETA] - roger->base_position[THETA];
  theta_dot_error = 0.0 - roger->base_velocity[THETA];
  while (theta_error > M_PI) theta_error -= 2.0 * M_PI;
  while (theta_error < -M_PI) theta_error += 2.0 * M_PI;

  if (ACTUATE_BASE) {
    // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE BASE
    // USING YOUR GAINS
    Mz = Kp_base_rot * theta_error + Kd_base_rot * theta_dot_error;
  }
  else {
    Mz = passive_Kd_base_rot * theta_dot_error;
  }

	// if (write==1){
	// 	fprintf(fptheta, "%6.10lf\n", theta_error);
	// }
  
  return(Mz);
}

/* PROJECT #1.3 - PD CONTROLLER FOR THE BASE                             */
/* setpoints - (xy) location for translation heading in radians          */

/*   the base differential drive Jacobian:                               */
/*    |tau_l|     |Fx|      |  x_dot  |     |theta_dot_left |            */
/*    |     |= JT |  |      |         | = J |               |            */
/*    |tau_r|     |Mz|      |theta_dot|     |theta_dot_right|            */

double baseJT[2][2] = 
  {{(1.0/2.0), -(1.0/(2.0*R_AXLE))}, {(1.0/2.0), (1.0/(2.0*R_AXLE))}};

void PDController_base(roger, time)
Robot * roger;
double time;
{ 
  double Fx, Mz, PDBase_translate(), PDBase_rotate();

  //  Fx = PDBase_translate(roger,time); // translate along current heading
  //  Mz = 0.0;

  //  Fx = 0.0;                          // rotate in current footprint
  //  Mz = PDBase_rotate(roger,time);
  if(getParamsFlag == 1){
		// roger->base_setpoint[X] = 0.75;
		// roger->base_setpoint[Y] = 0;
		roger->base_setpoint[THETA] = 0;
    getParamsFlag = 0;
  }
  Fx = PDBase_translate(roger,time);     // translate and rotate 
  Mz = PDBase_rotate(roger,time);

  // integrated wheel torque control
  roger->wheel_torque[LEFT] = baseJT[0][0]*Fx + baseJT[0][1]*Mz;
  roger->wheel_torque[RIGHT] = baseJT[1][0]*Fx + baseJT[1][1]*Mz;
}

/*************************************************************************/
/*       THE SIMULATOR EXECUTES control_roger() EVERY CONTROL CYCLE      */
/*                        *** DO NOT ALTER ***                           */
/*************************************************************************/
void control_roger(roger, time)
Robot * roger;
double time;
{
  update_setpoints(roger); // check_GUI_inputs(roger)

  // turn setpoint references into torques
  PDController_eyes(roger, time);
  PDController_arms(roger, time);
  PDController_base(roger,time);

}

/*************************************************************************/
void project1_reset(roger)
Robot* roger;
{ }

/*************************************************************************/
/* prompt for and read user customized input values                      */
/*************************************************************************/
void project1_enter_params()
{
  // put anything in here that you would like to change at run-time
  // without re-compiling user project codes

  //EYE
  //printf("Eye: K=%6.4lf  B=%6.4lf\n", Kp_eye, Kd_eye);
  //printf("Eye: enter 'K B'\n"); fflush(stdout);
  //scanf("%lf %lf", &Kp_eye, &Kd_eye);

  //ARM
  //printf("ARM: K=%6.4lf  B=%6.4lf\n", Kp_arm, Kd_arm);
  //printf("ARM: enter 'K B'\n"); fflush(stdout);
  //scanf("%lf %lf", &Kp_arm, &Kd_arm);

  //BASE Trans
	// printf("Base Trans: K=%6.4lf  B=%6.4lf\n", Kp_base_trans, Kd_base_trans);
	// printf("Base Trans: enter 'K B'\n"); fflush(stdout);
	// scanf("%lf %lf", &Kp_base_trans, &Kd_base_trans);

	// BASE Rot
	// printf("Base Rot: K=%6.4lf  B=%6.4lf\n", Kp_base_rot, Kd_base_rot);
	// printf("Base Rot: enter 'K B'\n"); fflush(stdout);
	// scanf("%lf %lf", &Kp_base_rot, &Kd_base_rot);


	//Code to write outputs to file. Not removed to enable future tests.
	// printf("Base: enter 'WriteToFile'\n"); fflush(stdout);
	// scanf("%d", &write);

	// if (write == 1){
	// 	fpx = fopen("x.txt", "w");
	// 	fpy = fopen("y.txt", "w");
	// 	fptheta = fopen("theta.txt", "w");
	// 	getParamsFlag = 1;
	// }
}

/*************************************************************************/
// function called when the 'visualize' button on the gui is pressed            
void project1_visualize(roger)
Robot* roger;
{ }