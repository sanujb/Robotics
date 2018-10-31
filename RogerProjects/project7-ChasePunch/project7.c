/*************************************************************************/
/* File:        project7.c                                               */
/* Description: User project #7 - empty project directory for project    */
/*              development                                             */
/* Date:        01-2015                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

double * stereo_observation(Robot* roger, double time);
int SEARCHTRACK(), inv_arm_kinematics(), fwd_arm_kinematics();
void draw_observation(), home_position();
double chase_dist;

Observation obs;
double recommended_setpoints_chase[2];
		// base-x,y

/*************************************************************************/
/* CHASE - Moves roger towards the ball */
/*************************************************************************/
int CHASE(roger, time)
Robot* roger;
double time;
{	
	double dist_to_ball, slope;
	static int return_state = NO_REFERENCE;
	double * pos = stereo_observation(roger, time);
	recommended_setpoints_chase[0] = roger->base_setpoint[X];
	recommended_setpoints_chase[1] = roger->base_setpoint[Y];
		
	if (isfinite(pos[0]) && isfinite(pos[1])){
		dist_to_ball = pow(pow(pos[0] - roger->base_position[X],2) + pow(pos[1] - roger->base_position[Y],2), 0.5);
		chase_dist = dist_to_ball;
		if (dist_to_ball < 0.5){
			chase_dist = 0.5;
			return_state = CONVERGED;
			roger->base_setpoint[X] = roger->base_position[X];
			roger->base_setpoint[Y] = roger->base_position[Y];
			recommended_setpoints_chase[0] = roger->base_position[X];
			recommended_setpoints_chase[1] = roger->base_position[Y];
		} else{
			slope = atan2((pos[1]-roger->base_position[Y]), (pos[0] - roger->base_position[X]));
			recommended_setpoints_chase[0] = roger->base_position[X] + (dist_to_ball - 0.6)*cos(slope);
			recommended_setpoints_chase[1] = roger->base_position[Y] + (dist_to_ball - 0.6)*sin(slope);
			return_state = TRANSIENT;
		}		
		
	} else{
		return_state = NO_REFERENCE;
	}
	// if(return_state != CONVERGED) home_position(roger);
	return return_state;
}

/*************************************************************************/
/* TOUCH - touching the ball (any way possible)  ************************/
/*************************************************************************/
int TOUCH(roger, time)
Robot* roger;
double time;
{
	int i,j;
	static int return_state = NO_REFERENCE;
	double * pos = stereo_observation(roger, time);

	if (isfinite(pos[0]) && isfinite(pos[1])) {
		return_state = 1;
		int l_inRange = inv_arm_kinematics(roger, LEFT, pos[0], pos[1]);
		int r_inRange = inv_arm_kinematics(roger, RIGHT, pos[0], pos[1]);

		double wTb[4][4], ref_bl[4], ref_br[4], ref_wl[4], ref_wr[4];
		construct_wTb(roger->base_position, wTb);
		double lx,ly,rx,ry;
		fwd_arm_kinematics(roger, LEFT, &lx, &ly);
		fwd_arm_kinematics(roger, RIGHT, &rx, &ry);
		ly+=ARM_OFFSET;
		ry-=ARM_OFFSET;
		ref_bl[0] = lx;
		ref_bl[1] = ly;
		ref_bl[2] = 0.0;
		ref_bl[3] = 1.0;
		ref_br[0] = rx;
		ref_br[1] = ry;
		ref_br[2] = 0.0;
		ref_br[3] = 1.0;
		matrix_mult(4, 4, wTb, 1, ref_bl, ref_wl);
		matrix_mult(4, 4, wTb, 1, ref_br, ref_wr);
		double distance_l = pow(pow(ref_wl[0]-pos[0], 2) + pow(ref_wl[1]-pos[1], 2), 0.5);
		double distance_r = pow(pow(ref_wr[0]-pos[0], 2) + pow(ref_wr[1]-pos[1], 2), 0.5);

		if(l_inRange == FALSE && r_inRange == FALSE){
			return_state = NO_REFERENCE;
			return return_state;
		}
		// Reading tactile sensor measurements on the left and right end effectors
		double fx_left = roger->ext_force[LEFT][0];
		double fy_left = roger->ext_force[LEFT][1];
		double fx_right = roger->ext_force[RIGHT][0];
		double fy_right = roger->ext_force[RIGHT][1];

		if((fabs(fx_left) > 0 || fabs(fy_left) > 0) && distance_l < 0.28){
			return_state = CONVERGED;
		}

		if((fabs(fx_right) > 0 || fabs(fy_right) > 0) && distance_r < 0.28){
			return_state = CONVERGED;
		}
	} else{
		return_state = NO_REFERENCE;
	}
	// if (return_state == NO_REFERENCE) home_position(roger);
	return(return_state);
}

/*************************************************************************/
// CHASETOUCH - Uses SEARCHTRACK, CHASE, and TOUCH to find the ball, move
// towards the ball, and touch it.
/*************************************************************************/
int CHASETOUCH(roger, time)
Robot* roger;
double time;
{
	static int internal_state[2] = {NO_REFERENCE, NO_REFERENCE};
	static int return_state = NO_REFERENCE;

	internal_state[0] = CHASE(roger, time);
	internal_state[1] = TOUCH(roger, time);
	int d = internal_state[0] + 3*internal_state[1];

	if (SEARCHTRACK(roger, time) == 0){
		return_state = NO_REFERENCE;
	}
	// printf("%d\n", d);
	switch(d){
		
		case 0: 
		case 3: 
		case 6: return_state = NO_REFERENCE;
				home_position(roger);
				break;
		
		case 1: return_state = TRANSIENT;
				home_position(roger);
				roger->base_setpoint[X] = recommended_setpoints_chase[0];
				roger->base_setpoint[Y] = recommended_setpoints_chase[1];
				break;
		
		case 2: //home_position(roger);
		case 4: 
		case 5: return_state = TRANSIENT;
				break;

		case 7:
		case 8: return_state = CONVERGED;

		break;
	}
	return return_state;
}


void project7_control(roger, time)
Robot* roger;
double time;
{
  CHASETOUCH(roger, time);
}

/************************************************************************/
void project7_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project7_enter_params() 
{
  printf("Project 7 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project7_visualize(roger)
Robot* roger;
{
  draw_observation(obs); 
}
