/*************************************************************************/
/* File:        project5.c                                               */
/* Description: User project #5 - empty project directory for project    */
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

Observation obs;

float ball_position(int image[NPIXELS][3]){

	int i;
	float start = -1.0, end = -1.0;
	for (i = 0; i < NPIXELS; ++i) {
		if (image[i][RED_CHANNEL] == 255) {
			
			if (start == -1){
				start = i;
				end = i;
			} else {
				end += 1;
			}
		} else {
			if (start > -1){
				break;
			}
		}
	}

	return (start+end)/2;
}

double * stereo_observation(roger, time)
Robot* roger;
double time;
{
	static double pos[2];
	float target_location_left = ball_position(roger->image[LEFT]);
	float target_location_right = ball_position(roger->image[RIGHT]);

	if (target_location_right < 0 || target_location_left < 0){
		roger->eyes_setpoint[LEFT] = 0;
		roger->eyes_setpoint[RIGHT] = 0;
		pos[0] = 0.0/0.0;
		pos[1] = 0.0/0.0;
		return pos;
	}

	float d = BASELINE;

	float u_l = 63.5 - target_location_left;
	float u_r = 63.5 - target_location_right;

	double phi_l = atan2(u_l, FOCAL_LENGTH);
	double phi_r = atan2(u_r, FOCAL_LENGTH);

	roger->eyes_setpoint[LEFT] = roger->eye_theta[LEFT] - phi_l;
	roger->eyes_setpoint[RIGHT] = roger->eye_theta[RIGHT] - phi_r;

	double gam_l = phi_l + roger->eye_theta[LEFT];
	double gam_r = phi_r + roger->eye_theta[RIGHT];

	double wTb[4][4], ref_b[4], ref_w[4], J_b[2][2], J_w[2][2], J_wT[2][2], JJT[2][2], R[2][2];
	construct_wTb(roger->base_position, wTb);

	ref_b[0] = 2*d*cos(gam_r)*cos(gam_l)/sin(gam_r - gam_l);
	ref_b[1] = d + 2*d*cos(gam_r)*sin(gam_l)/sin(gam_r - gam_l);
	ref_b[2] = 0.0;
	ref_b[3] = 1.0;

	double scaling = 0.005;
	double J_mult = 2*d/(sin(gam_r - gam_l)*sin(gam_r - gam_l));

	J_b[0][0] = J_mult*cos(gam_r)*cos(gam_r);
	J_b[0][1] = -1*J_mult*cos(gam_l)*cos(gam_l);
	J_b[1][0] = J_mult*sin(gam_r)*cos(gam_r);
	J_b[1][1] = -1*J_mult*sin(gam_l)*cos(gam_l);

	R[0][0] = wTb[0][0];
	R[0][1] = wTb[0][1];
	R[1][0] = wTb[1][0];
	R[1][1] = wTb[1][1];

	matrix_mult(4, 4, wTb, 1, ref_b, ref_w);
	matrix_mult(2, 2, R, 2, J_b, J_w);

	J_wT[0][0] = J_w[0][0];
	J_wT[0][1] = J_w[1][0];
	J_wT[1][0] = J_w[0][1];
	J_wT[1][1] = J_w[1][1];

	matrix_mult(2, 2, J_w, 2, J_wT, JJT);

	obs.pos[0] = ref_w[0];
	obs.pos[1] = ref_w[1];
	obs.cov[0][0] = JJT[0][0]*scaling;
	obs.cov[0][1] = JJT[0][1]*scaling;
	obs.cov[1][0] = JJT[1][0]*scaling;
	obs.cov[1][1] = JJT[1][1]*scaling;
	obs.time = time;

	pos[0] = ref_w[0];
	pos[1] = ref_w[1];
	return pos;

	// int i = 0, j = 0;
	// for (i=0;i<2;i++){
	// 	for(j=0;j<2;j++){
	// 		printf("%d,%d,%lf,%lf\n",i,j,J_w[i][j],J_b[i][j]);
	// 	}
	// }
}

void project5_control(roger, time)
Robot* roger;
double time;
{
	stereo_observation(roger, time);
}

/************************************************************************/
void project5_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project5_enter_params() 
{
  printf("Project 5 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project5_visualize(roger)
Robot* roger;
{
	draw_observation(obs);
}
