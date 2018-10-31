/*************************************************************************/
/* File:        object.c                                                 */
/* Description: all structures and dynamics specific to the objects      */
/* Author:      Rod Grupen                                               */
/* Date:        11-1-2009                                                */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "include/roger.h"
#include "include/simulate.h"
#include "include/control.h"

// "object" data structure definition can be found in include/simulate.h
PolyBall toy_home =
  { CIRCLE,                  // object id (CIRCLE||TRIANGLE)
    1,                       // number of vertices
    R_BALL,                  // circle radius
    0.0,                     // spoke_length
    M_BALL,                  // mass
    I_BALL,                  // mass moment of inertia
    { 0.0, (2*MAX_Y), 0.0 }, // position in world coordinates 
                             // (initially outside/north of drawable canvas
    { 0.0, 0.0, 0.0 },       // velocity in world coordinates
    { 0.0, 0.0, 0.0 } };     // default external forces

void simulate_object(obj)
PolyBall * obj;
{
  int i;
  double acc[3];
  double mag, vmag, fnorm[2];

  //  printf("net force on object = %6.4lf %6.4lf\n",
  //	 obj->net_extForce[X], obj->net_extForce[Y]);

  mag = sqrt(SQR(obj->net_extForce[X])+SQR(obj->net_extForce[Y]));
  fnorm[X] = fnorm[Y] = 0.0;

  if (mag < STATIC_FORCE_THRESHOLD) mag=0.0;
  else {
    fnorm[X] = obj->net_extForce[X]/mag; fnorm[Y] = obj->net_extForce[Y]/mag;
    mag -= STATIC_FORCE_THRESHOLD;
  }

  //  printf("\t force mag=%6.4lf  fhat=[%6.4lf %6.4lf]\n", 
  //	 mag, fnorm[X], fnorm[Y]);

  acc[X] = (mag*fnorm[X] - VISCOSITY*obj->velocity[X])/obj->mass;
  acc[Y] = (mag*fnorm[Y] - VISCOSITY*obj->velocity[Y])/obj->mass;
  //  acc[THETA] = (obj->extForce[THETA] 
  //		- VISCOSITY*obj->velocity[THETA])/obj->moment;
  acc[THETA] = (obj->net_extForce[THETA])/obj->moi;

  // experimental velocity governor to make collisions behave better
  //  vmag = sqrt(SQR(obj->velocity[X])+SQR(obj->velocity[Y]));
  //  if (vmag < 10.0) {
  obj->velocity[X] += acc[X] * DT;
  obj->velocity[Y] += acc[Y] * DT;
  obj->velocity[THETA] += acc[THETA] * DT;
  //  }
  //  else {
  //    acc[X] = acc[Y] = 0.0;
  //  }

  // printf("velocity=%lf\n",sqrt(SQR(obj->velocity[X]) +
  //                              SQR(obj->velocity[Y])));

  obj->position[X] += 0.5*acc[X]*SQR(DT) + obj->velocity[X]*DT;
  obj->position[Y] += 0.5*acc[Y]*SQR(DT) + obj->velocity[Y]*DT;
  obj->position[THETA] += 0.5*acc[THETA]*SQR(DT) + obj->velocity[THETA]*DT;
  //  if ((obj->position[X] < MIN_X + R_OBJ) && (obj->velocity[X] < 0.0))
  //    obj->velocity[X] *= -1.0;
  //  if ((obj->position[X] > MAX_X - R_OBJ) && (obj->velocity[X] > 0.0))
  //    obj->velocity[X] *= -1.0;
  //  if ((obj->position[Y] < MIN_Y + R_OBJ) && (obj->velocity[Y] < 0.0))
  //    obj->velocity[Y] *= -1.0;
  //  if ((obj->position[Y] > MAX_Y - R_OBJ) && (obj->velocity[Y] > 0.0))
  //    obj->velocity[Y] *= -1.0;

  //  printf("\t\t X: acc=%lf vel=%lf pos=%lf\n", 
  //	 acc[X], obj->velocity[X], obj->position[X]);
  //  printf("\t\t Y: acc=%lf vel=%lf pos=%lf\n", 
  //	 acc[Y], obj->velocity[Y], obj->position[Y]);
  //  printf("\t\t THETA: acc=%lf vel=%lf pos=%lf\n", 
  //	 acc[THETA], obj->velocity[THETA], obj->position[THETA]);
}
