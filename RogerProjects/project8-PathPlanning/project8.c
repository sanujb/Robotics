/*************************************************************************/
/* File:        project8.c                                               */
/* Description: User project #8 - harmonic function code                 */
/* Date:        12-2014                                               */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"

#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

/************************************************************************/
/************************************************************************/

#define CONTROL_STEP 0.2

void sor(roger)
Robot * roger;
{
  int i, j, sor_count=0, converged = FALSE;
  double sor_once();

  while (!converged && (sor_count < 5000)) {
    ++sor_count;
    if (sor_once(roger) < THRESHOLD) converged = TRUE;
  }
  if (sor_count > 1)
    printf("completed harmonic function --- %d iterations\n", sor_count);
}

// one complete backup, only dirichlet boundary conditions
double sor_once(roger)
Robot * roger;
{
  int i, j, ipos, ineg, jpos, jneg;
  double residual, max, front, back, up, down;

  // iterate over entire map once
  // return the  maximum change in the potential value over the entire
  // occupancy map as a means of determining convergence
  max = 0.0;
  for (i = 0; i < NBINS; ++i) {
    ipos = (i + 1) % NBINS;
    ineg = (i - 1 + NBINS) % NBINS;
    for (j = 0; j < NBINS; ++j) {
      jpos = (j + 1) % NBINS;
      jneg = (j - 1 + NBINS) % NBINS;
    }
  }
  return(max);
}

double compute_gradient(x, y, roger, grad)
double x, y;
Robot * roger;
double grad[2]; // grad = [ d(phi)/dx  d(phi)/dy ] ~ [ d(phi)/dj  -d(phi)/di ]
{
  int i0,i1,j0,j1;
  double mag, dphi_di, dphi_dj, del_x, del_y;

  j0 = (int) ((x-MIN_X)/XDELTA);
  j1 = (j0+1);
  i1 = NBINS - (int) ((y - MIN_Y)/YDELTA); // (int) ((MAX_Y - y)/YDELTA);?
  i0 = (i1-1);

  del_x = (x-MIN_X)/XDELTA - j0;
  del_y = (NBINS - (y - MIN_Y)/YDELTA) - i0;
  
  dphi_dj = ((1.0-del_y)*(roger->world_map.potential_map[i0][j1] 
			  - roger->world_map.potential_map[i0][j0] ) +
	     (del_y)*(roger->world_map.potential_map[i1][j1] 
		      - roger->world_map.potential_map[i1][j0]  ) );
  dphi_di = ((1.0-del_x)*(roger->world_map.potential_map[i1][j0] 
			  - roger->world_map.potential_map[i0][j0] ) +
	     (del_x)*(roger->world_map.potential_map[i1][j1] 
		      - roger->world_map.potential_map[i0][j1]  ) );
  
  grad[0] = dphi_dj; grad[1] = -dphi_di;

  mag = sqrt(SQR(grad[0])+SQR(grad[1]));

  if (mag>THRESHOLD) {
    grad[0] /= mag; grad[1] /= mag;
  }
  else grad[0] = grad[1] = 0;
  return(mag);
}

void follow_path(roger)
Robot *roger;
{
  int xbin, ybin;
  double x, y, bb, mag, grad[2], compute_gradient();

  x = roger->base_position[X];
  y = roger->base_position[Y];

  ybin = (int)((MAX_Y - y)/YDELTA);
  xbin = (int)((x - MIN_X)/XDELTA);

  grad[X] = grad[Y] = 0.0;
  if (roger->world_map.occupancy_map[ybin][xbin]!=GOAL) {
    mag = compute_gradient(x, y, roger, grad);
    roger->base_setpoint[THETA] = atan2(-grad[Y], -grad[X]);
  }
  roger->base_setpoint[X] = x - CONTROL_STEP*grad[X];
  roger->base_setpoint[Y] = y - CONTROL_STEP*grad[Y];
}

/************************************************************************/
/************************************************************************/

void sor(), follow_path();

void project8_control(roger, time)
Robot *roger;
double time;
{ 
  sor(roger);

  follow_path(roger);
}

/************************************************************************/
void project8_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project8_enter_params() 
{
  printf("Project 6 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project8_visualize(roger)
Robot* roger;
{
  void draw_streamlines();
  draw_streamlines();
}

