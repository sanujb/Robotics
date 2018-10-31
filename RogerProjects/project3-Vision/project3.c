/*************************************************************************/
/* File:        project3.c                                               */
/* Description: User project #3 - empty project directory for project    */
/*              developement                                             */
/* Date:        01-2015                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"


double compute_average_red_pixel(int image[NPIXELS][3])
{
	int i;
	double start = -1.0, end = -1.0;
	for (i = 0; i < NPIXELS; ++i) {
		if (image[i][RED_CHANNEL] == 255) {
			if (start == -1){
				start = i;
			}
			end = i;
		}
	}
	return (start+end)/2;
}

void project3_control(roger, time)
Robot* roger;
double time;
{
	float target_location_left = compute_average_red_pixel(roger->image[LEFT]);
	float target_location_right = compute_average_red_pixel(roger->image[RIGHT]);

	if (target_location_right < 0 || target_location_left < 0){
		roger->eyes_setpoint[LEFT] = 0;
		roger->eyes_setpoint[RIGHT] = 0;
		return;
	}

	float u_l = 63.5 - target_location_left;
	float u_r = 63.5 - target_location_right;

	roger->eyes_setpoint[LEFT] = roger->eye_theta[LEFT] - atan2(u_l, FOCAL_LENGTH);
	roger->eyes_setpoint[RIGHT] = roger->eye_theta[RIGHT] - atan2(u_r, FOCAL_LENGTH);
}

/************************************************************************/
void project3_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project3_enter_params() 
{
  printf("Project 4 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project3_visualize(roger)
Robot* roger;
{ }
