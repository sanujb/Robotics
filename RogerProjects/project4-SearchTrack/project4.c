/*************************************************************************/
/* File:        project4.c                                               */
/* Description: User project #4 - empty project directory for project    */
/*              developement                                             */
/* Date:        01-2015                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Xkw/Xkw.h"
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"


void home_position();
double recommended_setpoints_ST[2][3], compute_average_red_pixel();
		//s,t,st | base_theta, left_eye, right_eye
double track_error;

int SEARCH(roger, time)
Robot* roger;
double time;
{	
	int return_state = NO_REFERENCE;
	double search_heading;
	double heading_error_base;

	heading_error_base = roger->base_setpoint[THETA] - roger->base_position[THETA];
	while (heading_error_base > M_PI) heading_error_base -= 2.0 * M_PI;
	while (heading_error_base < -M_PI) heading_error_base += 2.0 * M_PI;
	if (fabs(heading_error_base) < 0.1) return_state = CONVERGED;

	if (return_state == CONVERGED){
		sample_gaze_direction(&search_heading);
		return_state = TRANSIENT;
		recommended_setpoints_ST[0][0] = search_heading;
		recommended_setpoints_ST[0][1] = 0;
		recommended_setpoints_ST[0][2] = 0;
	}
	// printf("base heading in search %f\n", search_heading);
	// printf("%d %f %f %f\n", return_state, recommended_setpoints_ST[0][0], recommended_setpoints_ST[0][1], recommended_setpoints_ST[0][2]);
	return(return_state);
}

int TRACK(roger, time)
Robot* roger;
double time;
{
	double ul, ur, eye[2], error_base;
	static int return_state = NO_REFERENCE;

	ul = compute_average_red_pixel(roger->image[LEFT]);
	ur = compute_average_red_pixel(roger->image[RIGHT]);
	if (ul >= 0 && ur >= 0){
		ul = 63.5 - ul;
		ur = 63.5 - ur;

		eye[LEFT] = roger->eye_theta[LEFT] - atan2(ul, FOCAL_LENGTH);
	    eye[RIGHT] = roger->eye_theta[RIGHT] - atan2(ur, FOCAL_LENGTH);

		recommended_setpoints_ST[1][1] = eye[LEFT];
		recommended_setpoints_ST[1][2] = eye[RIGHT];
		recommended_setpoints_ST[1][0] = roger->base_position[THETA];
		

		if ( (eye[RIGHT]*eye[LEFT] <= 0) && (fabs(ul) < 1.0) && (fabs(ur) < 1.0) ){
			return_state = CONVERGED;
			track_error = 0;
		} else {
			if (eye[LEFT] < 0 || eye[RIGHT] < 0){
				float x = eye[LEFT] < eye[RIGHT] ? eye[LEFT] : eye[RIGHT];
				recommended_setpoints_ST[1][0] = roger->base_position[THETA] + x*0.8;
				track_error = fabs(0.8*x);
			}
			if (eye[LEFT] > 0 || eye[RIGHT] > 0){
				float x = eye[LEFT] > eye[RIGHT] ? eye[LEFT] : eye[RIGHT];
				recommended_setpoints_ST[1][0] = roger->base_position[THETA] + x*0.8;
				track_error = fabs(0.8*x);
			}
			return_state = TRANSIENT;

			while(recommended_setpoints_ST[1][0] > M_PI) recommended_setpoints_ST[1][0] -= 2.0*M_PI;
			while(recommended_setpoints_ST[1][0] < -M_PI) recommended_setpoints_ST[1][0] += 2.0*M_PI;
		}
	} else{
		return_state = NO_REFERENCE;
	}
	return(return_state);
}

int SEARCHTRACK(roger, time)
Robot* roger;
double time;
{
	static int return_state = NO_REFERENCE;
	static int internal_state[2] = { NO_REFERENCE, NO_REFERENCE };
	//                               [0]->SEARCH    [1]->TRACK
	internal_state[0] = SEARCH(roger, time);
	internal_state[1] = TRACK(roger, time);
	int state = 3*internal_state[1] + internal_state[0];
	//printf("%d\n", state);
	switch(state){
		case 0:
		case 1:
		case 2:
			return_state = NO_REFERENCE;
			roger->base_setpoint[THETA] = recommended_setpoints_ST[0][0];
			roger->eyes_setpoint[LEFT] = recommended_setpoints_ST[0][1];
			roger->eyes_setpoint[RIGHT] = recommended_setpoints_ST[0][2];
			break;

		case 3:
		case 4:
		case 5:
			return_state = TRANSIENT;
			roger->base_setpoint[THETA] = recommended_setpoints_ST[1][0];
			roger->eyes_setpoint[LEFT] = recommended_setpoints_ST[1][1];
			roger->eyes_setpoint[RIGHT] = recommended_setpoints_ST[1][2];
			break;
		
		case 6:
		case 7:
		case 8:
			return_state = CONVERGED;
			break;
	}
	return(return_state);
}

void project4_control(roger, time)
Robot* roger;
double time;
{	
	SEARCHTRACK(roger);
}

/************************************************************************/
void project4_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project4_enter_params() 
{
  printf("Project 4 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project4_visualize(roger)
Robot* roger;
{ }
