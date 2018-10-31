/*************************************************************************/
/* File:        whole_body.c                                             */
/* Description: dynamic simulation of the 8 DOF mmRoger                  */
/*              uncoupled base, eyes, arms - first draft                 */
/* Author:      Rod Grupen                                               */
/* Date:        10-2014                                                  */
/*************************************************************************/
#include <math.h>
#include <stdlib.h>
#include "include/roger.h"
#include "include/simulate.h"
#include "include/control.h"

/*************************************************************************/
/***** the eye ***********************************************************/
/*************************************************************************/
Eye eyes_home[NEYES] = { { {0.0, BASELINE}, 0.0, 0.0,
                           {99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
                            99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
                            99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
                            99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
                            99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
                            99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
                            99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
                            99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99},
                           0.0},
			 { {0.0, -BASELINE}, 0.0, 0.0,
                           {99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
                            99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
                            99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
                            99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
                            99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
                            99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
                            99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
                            99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99},
			   0.0} };

/*************************************************************************/
/***** the base **********************************************************/
/*************************************************************************/
Base mobile_base_home =
  { { {  0.0, 1.0, 0.0,  0.0 },  /* wTb */
      { -1.0, 0.0, 0.0,  0.0 },
      {  0.0, 0.0, 1.0,  0.0 },
      {  0.0, 0.0, 0.0,  1.0 } },
    0.0, /* x */
    0.0, /* x_dot */
    0.0, /* y */
    0.0, /* y_dot */
    -M_PI/2.0, /* theta */
    0.0, /* theta_dot */
    { 0.0, 0.0 }, /* wheel_torque */
    0.0, /* contact_theta */
    { 0.0, 0.0 }, /* extForce (fx, fy) */
    { 0.0, 0.0 }  /* wheel_theta_dot */
  };

// for motor models
void wheel_speed(mobile_base)
Base * mobile_base;
{
  mobile_base->wheel_theta_dot[LEFT] = 
    (mobile_base->x_dot*cos(mobile_base->theta) +
     mobile_base->y_dot*sin(mobile_base->theta) -
     mobile_base->theta_dot * R_AXLE) / R_WHEEL;

  mobile_base->wheel_theta_dot[RIGHT] = 
    (mobile_base->x_dot*cos(mobile_base->theta) +
     mobile_base->y_dot*sin(mobile_base->theta) +
     mobile_base->theta_dot * R_AXLE) / R_WHEEL;
}

/*************************************************************************/
/***** the arm ***********************************************************/
/*************************************************************************/
static double l[2] = {L_ARM1, L_ARM2};
static double m[2] = {M_ARM1, M_ARM2};

Arm arms_home[NARMS][NARM_FRAMES] = 
  { { { { { 1.0, 0.0, 0.0, 0.0 },         /* LEFT ARM         */
	  { 0.0, 1.0, 0.0, ARM_OFFSET },  /* mobile base to frame 0 */
	  { 0.0, 0.0, 1.0, 0.0 },
	  { 0.0, 0.0, 0.0, 1.0 } }, NOTYPE,NOAXIS, 0.0,0.0,
	0.0, {0.0,0.0} },
      { { { 1.0, 0.0, 0.0, 0.0 },  /* frame 0 to frame 1 */
	  { 0.0, 1.0, 0.0, 0.0  },
	  { 0.0, 0.0, 1.0, 0.0 },
	  { 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE,ZAXIS, (M_PI/2.0),0.0,
//	  { 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE,ZAXIS, (9.0*M_PI/10.0),0.0,
	0.0, {0.0,0.0} },
      { { { 1.0, 0.0, 0.0, L_ARM1 },  /* frame 1 to frame 2 */
	  { 0.0, 1.0, 0.0, 0.0 },
	  { 0.0, 0.0, 1.0, 0.0 },
	  { 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE,ZAXIS, (0.0),0.0,
//	  { 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE,ZAXIS, (-9.0*M_PI/10.0),0.0,
	0.0, {0.0,0.0} },
      { { { 1.0, 0.0, 0.0, L_ARM2 },  /* frame 2 to frame 3 */
	  { 0.0, 1.0, 0.0, 0.0 },
	  { 0.0, 0.0, 1.0, 0.0 },
	  { 0.0, 0.0, 0.0, 1.0 } }, NOTYPE,NOAXIS, 0.0,0.0,
	0.0, {0.0,0.0} } },
    { { { { 1.0, 0.0, 0.0, 0.0 },           /* RIGHT ARM        */
	  { 0.0, 1.0, 0.0, -ARM_OFFSET },  /* world to frame 0 */
	  { 0.0, 0.0, 1.0, 0.0 },
	  { 0.0, 0.0, 0.0, 1.0 } }, NOTYPE,NOAXIS, 0.0,0.0,
	0.0, {0.0,0.0} },
      { { { 1.0, 0.0, 0.0, 0.0 },  /* frame 0 to frame 1 */
	  { 0.0, 1.0, 0.0, 0.0 },
	  { 0.0, 0.0, 1.0, 0.0 },
	  { 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE,ZAXIS, (-M_PI/2.0),0.0,
//	  { 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE,ZAXIS, (-9.0*M_PI/10.0),0.0,
	0.0, {0.0,0.0} },
      { { { 1.0, 0.0, 0.0, L_ARM1 },  /* frame 1 to frame 2 */
	  { 0.0, 1.0, 0.0, 0.0 },
	  { 0.0, 0.0, 1.0, 0.0 },
	  { 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE,ZAXIS, (0.0),0.0,
//	  { 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE,ZAXIS, (9.0*M_PI/10.0),0.0,
	0.0, {0.0,0.0} },
      { { { 1.0, 0.0, 0.0, L_ARM2 },  /* frame 2 to frame 3 */
	  { 0.0, 1.0, 0.0, 0.0 },
	  { 0.0, 0.0, 1.0, 0.0 },
	  { 0.0, 0.0, 0.0, 1.0 } }, NOTYPE,NOAXIS, 0.0,0.0,
	0.0, {0.0,0.0} } } };

void arm_dynamics(base,arm,M,V,G,F)
Base * base;
Arm arm[NARM_FRAMES];
double M[NARM_JOINTS][NARM_JOINTS],V[NARM_JOINTS],G[NARM_JOINTS];
double F[NARM_JOINTS];
{
  double s1, c1, s2, c2, s12, c12, s012, c012;
  double fW[2], f3[2], R3_W[2][2];

  s1 = sin(arm[1].theta);
  c1 = cos(arm[1].theta);
  s2 = sin(arm[2].theta);
  c2 = cos(arm[2].theta);
  s12 = sin(arm[1].theta + arm[2].theta);
  c12 = cos(arm[1].theta + arm[2].theta);

  s012 = sin(base->theta + arm[1].theta + arm[2].theta);
  c012 = cos(base->theta + arm[1].theta + arm[2].theta);

  R3_W[0][0] = c012;     R3_W[0][1] = s012;
  R3_W[1][0] = -s012;    R3_W[1][1] = c012;

  // f3 = 3Rw arm{NARM_FRAMES-1].extForce
  fW[X] = arm[NARM_FRAMES-1].extForce[X];
  fW[Y] = arm[NARM_FRAMES-1].extForce[Y];

  f3[X] = R3_W[0][0]*fW[X] + R3_W[0][1]*fW[Y];
  f3[Y] = R3_W[1][0]*fW[X] + R3_W[1][1]*fW[Y];

  /* fill in non-zero terms for M */
  M[0][0] = SQR(l[1])*m[1] + 2.0*l[0]*l[1]*m[1]*c2 + SQR(l[0])*(m[0]+m[1]);
  M[1][1] = SQR(l[1])*m[1];
  M[1][0] = M[1][1] + l[0]*l[1]*m[1]*c2;
  M[0][1] = M[1][0];

  V[0] = -m[1]*l[0]*l[1]*s2*SQR(arm[2].theta_dot) 
    - 2.0*m[1]*l[0]*l[1]*s2*arm[1].theta_dot*arm[2].theta_dot;
  V[1] = m[1]*l[0]*l[1]*s2*SQR(arm[1].theta_dot);

  G[1] = m[1]*l[1]*GRAVITY*c12;
  G[0] = G[1] + (m[0]+m[1])*l[0]*GRAVITY*c1;
  //G[1] = 0.0;
  //G[0] = 0.0;

  /* torques due to endpoint forces tau = J^T force */
  // arm[NARM_FRAMES-1].extForce[] is in world frame...it should be in base
  // frame to use the Jacobian like this

  //  F[0] = -(l[0]*s1+l[1]*s12)*arm[NARM_FRAMES-1].extForce[0] 
  //    + (l[0]*c1+l[1]*c12)*arm[NARM_FRAMES-1].extForce[1];
  //  F[1] = -(l[0]*s12)*arm[NARM_FRAMES-1].extForce[0] 
  //    + (l[1]*c12)*arm[NARM_FRAMES-1].extForce[1];
  F[0] = f3[X]*(l[0]*s2) + f3[Y]*(l[1] + l[0]*c2);
  F[1] = f3[Y]*l[1];
}

void update_state(arm)
Arm arm[NARM_FRAMES];
{
  int i;

  /* update transforms that describe position of roger's manipulator */
  for(i=1;i<NARM_FRAMES;++i) {
    switch(arm[i].axis) {
    case(XAXIS):   /* dof axis is local x axis */
      if (arm[i].dof_type == REVOLUTE) {
	arm[i].iTj[1][1] =  cos(arm[i].theta);
	arm[i].iTj[1][2] = -sin(arm[i].theta);
	arm[i].iTj[2][1] = -arm[i].iTj[1][2];
	arm[i].iTj[2][2] = 	arm[i].iTj[1][1];
      }
      else if (arm[i].dof_type == PRISMATIC) {
	arm[i].iTj[0][3] = arm[i].theta;
      }
      break;

    case(YAXIS):   /* dof axis is local y axis */
      if (arm[i].dof_type == REVOLUTE) {
	arm[i].iTj[0][0] = cos(arm[i].theta);
	arm[i].iTj[0][2] = sin(arm[i].theta);
	arm[i].iTj[2][0] = -arm[i].iTj[0][2];
	arm[i].iTj[2][2] =  arm[i].iTj[0][0];
      }
      else if (arm[i].dof_type == PRISMATIC) {
	arm[i].iTj[1][3] = arm[i].theta;
      }
      break;
      
    case(ZAXIS):   /* dof axis is local z axis */
      if (arm[i].dof_type == REVOLUTE) {
	arm[i].iTj[0][0] = cos(arm[i].theta);
	arm[i].iTj[0][1] = -sin(arm[i].theta);
	arm[i].iTj[1][0] = -arm[i].iTj[0][1];
	arm[i].iTj[1][1] =  arm[i].iTj[0][0];
      }
      else if (arm[i].dof_type == PRISMATIC) {
	arm[i].iTj[2][3] = l[i] + arm[i].theta;
      }
      break;
    }
  }
}

void rectify_theta(arm)
Arm arm[NARM_FRAMES];
{
  while (arm[1].theta < (-M_PI)) arm[1].theta += 2.0 * M_PI;
  while (arm[1].theta > M_PI) arm[1].theta -= 2.0 * M_PI;
  while (arm[2].theta < (-M_PI)) arm[2].theta += 2.0 * M_PI;
  while (arm[2].theta > M_PI) arm[2].theta -= 2.0 * M_PI;
}

/**********************************************************************/
//void old_invert(A,Ainv)
//double A[NARM_JOINTS][NARM_JOINTS], Ainv[NARM_JOINTS][NARM_JOINTS];
//{
//  double det;
//
//  switch(NARM_JOINTS) {
//  case (1):
//    Ainv[0][0] = 1.0/A[0][0];
//    break;
//  case (2):
//    det = 1.0 / (A[0][0]*A[1][1] - A[1][0]*A[0][1]);
//    
//    Ainv[0][0] = det * A[1][1];
//    Ainv[1][0] = -1.0 * det * A[1][0];
//    Ainv[0][1] = -1.0 * det * A[0][1];
//    Ainv[1][1] = det * A[0][0];
//    break;
//
//  case(3):
//    det = 1.0 / (A[0][0]*A[1][1]*A[2][2] + A[0][1]*A[1][2]*A[2][0] 
//		  + A[0][2]*A[1][0]*A[2][1] - A[2][0]*A[1][1]*A[0][2]
//		  - A[2][1]*A[1][2]*A[0][0] - A[2][2]*A[1][0]*A[0][1]) ;
//
//    Ainv[0][0] = det * (A[1][1]*A[2][2]-A[2][1]*A[1][2]);
//    Ainv[0][1] = det * (A[2][1]*A[0][2]-A[0][1]*A[2][2]);
//    Ainv[0][2] = det * (A[0][1]*A[1][2]-A[1][1]*A[0][2]);
//    Ainv[1][0] = det * (A[2][0]*A[1][2]-A[1][0]*A[2][2]);
//    Ainv[1][1] = det * (A[0][0]*A[2][2]-A[2][0]*A[0][2]);
//    Ainv[1][2] = det * (A[1][0]*A[0][2]-A[0][0]*A[1][2]);
//    Ainv[2][0] = det * (A[1][0]*A[2][1]-A[2][0]*A[1][1]);
//    Ainv[2][1] = det * (A[2][0]*A[0][1]-A[0][0]*A[2][1]);
//    Ainv[2][2] = det * (A[0][0]*A[1][1]-A[1][0]*A[0][1]);
//    break;
//  }
//}
//
//void old_matrix_mult(A,x,y)
//double A[NARM_JOINTS][NARM_JOINTS],x[NARM_JOINTS],y[NARM_JOINTS];
//{
//  int i,j;
//  for (i=0; i<NARM_JOINTS; ++i) {
//    y[i] = 0.0;
//    for (j=0; j<NARM_JOINTS; ++j) {
//      y[i] += A[i][j] * x[j];
//    }
//  }
//}
/**********************************************************************/

// 8x8 dynamics
void simulate_roger(mobile_base, arms, eyes)
Base *mobile_base;
Arm arms[NARMS][NARM_FRAMES];
Eye eyes[NEYES];
{
  int i,j;
  double M[8][8], V[8], G[8], F[8], Minv[8][8]; // NDOF = 8?
  double gForce[8], q_ddot[8], vmag;
  void roger_dynamics(), roger_dynamics_BD();
  //  void invert_WBM();
  int matrix_invert();

  if (COUPLED_DYNAMICS) {  // the coupled (whole body) computed torque equation
    roger_dynamics(mobile_base, arms, eyes, M, V, G, F);  
  }
  else {  // the uncoupled (block diagonal) computed torque equation
    roger_dynamics_BD(mobile_base, arms, eyes, M, V, G, F);
  }
  matrix_invert(M, 8, Minv);

  // base - elements 0 and 1
  gForce[0]=(mobile_base->wheel_torque[LEFT]+mobile_base->wheel_torque[RIGHT])
    - V[0] - G[0] - F[0];
  gForce[1]=R_BASE * (mobile_base->wheel_torque[RIGHT]
		      - mobile_base->wheel_torque[LEFT]) - V[1] - G[1] - F[1];
  // left eye - element 2
  gForce[2]=eyes[LEFT].torque - V[2] - G[2] - F[2];

  // right eye - element 3
  gForce[3]=eyes[RIGHT].torque - V[3] - G[3] - F[3];

  // left arm - elements 4 and 5
  gForce[4] = arms[LEFT][1].torque - V[4] - G[4] - F[4];
  gForce[5] = arms[LEFT][2].torque - V[5] - G[5] - F[5];

  // right arm - elements 6 and 7
  gForce[6] = arms[RIGHT][1].torque - V[6] - G[6] - F[6];
  gForce[7] = arms[RIGHT][2].torque - V[7] - G[7] - F[7];

  // matrixXvec() not yet implemented in matrix lib
  // 8x8 matrix times 8x1 vector yields accelerations
  for (i=0;i<NDOF;++i) {
    q_ddot[i] = 0;
    for (j=0;j<NDOF;++j) {
      q_ddot[i] += Minv[i][j] * gForce[j];
    }
  }

  // EULER INTEGRATION FOR BASE, ARMS, AND EYES
  // base translation q_ddot[0] --- nonholonomic, homo transform update, wheel
  //      speed (for motor models)
  vmag = mobile_base->x_dot * cos(mobile_base->theta) +
    mobile_base->y_dot * sin(mobile_base->theta);
  mobile_base->x_dot = (vmag + q_ddot[0]*DT)*cos(mobile_base->theta);
  mobile_base->y_dot = (vmag + q_ddot[0]*DT)*sin(mobile_base->theta);
  mobile_base->x += (0.5*q_ddot[0]*SQR(DT)+vmag*DT)*cos(mobile_base->theta);
  mobile_base->y += (0.5*q_ddot[0]*SQR(DT)+vmag*DT)*sin(mobile_base->theta);

  // base rotation q_ddot[1]
  mobile_base->theta_dot += q_ddot[1] * DT;
  mobile_base->theta += 0.5*q_ddot[1] *SQR(DT) + mobile_base->theta_dot*DT;

  while (mobile_base->theta > M_PI) mobile_base->theta -= 2.0 * M_PI;
  while (mobile_base->theta < -M_PI) mobile_base->theta += 2.0 * M_PI;

  // update permanent base configuration
  mobile_base->wTb[0][0] = mobile_base->wTb[1][1] = cos(mobile_base->theta);
  mobile_base->wTb[1][0] = sin(mobile_base->theta);
  mobile_base->wTb[0][1] = -mobile_base->wTb[1][0];
  mobile_base->wTb[0][3] =   mobile_base->x;
  mobile_base->wTb[1][3] =   mobile_base->y;
 
  wheel_speed(mobile_base);

  // left eye q_ddot[2]
  eyes[LEFT].theta_dot += q_ddot[2]*DT;
  eyes[LEFT].theta += 0.5*q_ddot[2]*DT*DT + eyes[LEFT].theta_dot*DT;
  if (eyes[LEFT].theta > M_PI/2.0) eyes[LEFT].theta = M_PI/2.0;
  else if (eyes[LEFT].theta < -M_PI/2.0) eyes[LEFT].theta = -M_PI/2.0;

  // right eye q_ddot[3]
  eyes[RIGHT].theta_dot += q_ddot[3]*DT;
  eyes[RIGHT].theta += 0.5*q_ddot[3]*DT*DT + eyes[RIGHT].theta_dot*DT;
  if (eyes[RIGHT].theta > M_PI/2.0) eyes[RIGHT].theta = M_PI/2.0;
  else if (eyes[RIGHT].theta < -M_PI/2.0) eyes[RIGHT].theta = -M_PI/2.0;

  // left arm q_ddot[4] and q_ddot[5]
  /* update positions and velocities of roger's manipulator */
  arms[LEFT][1].theta += 0.5*q_ddot[4]*DT*DT + arms[LEFT][1].theta_dot*DT;
  arms[LEFT][1].theta_dot += q_ddot[4]*DT;
  arms[LEFT][2].theta += 0.5*q_ddot[5]*DT*DT + arms[LEFT][2].theta_dot*DT;
  arms[LEFT][2].theta_dot += q_ddot[5]*DT;
  rectify_theta(arms[LEFT]);
  update_state(arms[LEFT]);
  
  // right arm q_ddot[6] and q_ddot[7]
  /* update positions and velocities of roger's manipulator */
  arms[RIGHT][1].theta += 0.5*q_ddot[6]*DT*DT + arms[RIGHT][1].theta_dot*DT;
  arms[RIGHT][1].theta_dot += q_ddot[6]*DT;
  arms[RIGHT][2].theta += 0.5*q_ddot[7]*DT*DT + arms[RIGHT][2].theta_dot*DT;
  arms[RIGHT][2].theta_dot += q_ddot[7]*DT;
  rectify_theta(arms[RIGHT]);
  update_state(arms[RIGHT]);
}

// the uncoupled, block-diagonal (8x8) dynamics
void roger_dynamics_BD(mobile_base, arms, eyes, M, V, G, F)
Eye eyes[NEYES];
Arm arms[NARMS][NARM_FRAMES];
Base *mobile_base;
double M[NDOF][NDOF], V[NDOF], G[NDOF], F[NDOF];
{
  int i,j;
  double Marm[2][2], Varm[2], Garm[2], Farm[2];
  for (i=0;i<8;++i) {
    for (j=0;j<8;++j) {
      M[i][j] = 0.0;
    }
    V[i] = G[i] = F[i] = 0.0;
  }

  // the BASE
  //  M[0][0] = M_BASE;
  //  M[0][1] = 0;
  //  M[1][0] = 0;
  //  M[1][1] = I_BASE;

  M[0][0] = M_WBODY;
  M[0][1] = 0;
  M[1][0] = 0;
  M[1][1] = I_WBODY;

  G[0] = G[1] = 0.0;
  V[0] = V[1] = 0.0;
  F[0] = -1*(mobile_base->extForce[X]*cos(mobile_base->theta) +
             mobile_base->extForce[Y]*sin(mobile_base->theta));
  F[1] = 0.0;

  // the LEFT EYE
  M[2][2] = I_EYE;
  G[2] = V[2] = 0.0;
  F[2] = 0.0;


  // the RIGHT EYE
  M[3][3] = I_EYE;
  G[3] = V[3] = 0.0;
  F[3] = 0.0;

  // the LEFT ARM
  arm_dynamics(mobile_base, arms[LEFT], Marm, Varm, Garm, Farm);
  M[4][4] = Marm[0][0];
  M[4][5] = Marm[0][1];
  M[5][4] = Marm[1][0];
  M[5][5] = Marm[1][1];
  V[4] = Varm[0]; G[4] = Garm[0]; F[4] = Farm[0];
  V[5] = Varm[1]; G[5] = Garm[1]; F[5] = Farm[1];

  // the RIGHT arm
  arm_dynamics(mobile_base, arms[RIGHT], Marm, Varm, Garm, Farm);
  M[6][6] = Marm[0][0];
  M[6][7] = Marm[0][1];
  M[7][6] = Marm[1][0];
  M[7][7] = Marm[1][1];
  V[6] = Varm[0]; G[6] = Garm[0]; F[6] = Farm[0];
  V[7] = Varm[1]; G[7] = Garm[1]; F[7] = Farm[1];
}
  
// the whole-body coupled (8x8) dynamics
void roger_dynamics(mobile_base, arms, eyes, M, V, G, F)
Base *mobile_base;
Arm arms[NARMS][NARM_FRAMES];
Eye eyes[NEYES];
double M[NDOF][NDOF], V[NDOF], G[NDOF], F[NDOF];
{
  int i,j;

  double m0, me, m1, m2;
  double l0, le, l1, l2;
  double a, b;
  double s1,c1,s2,c2,s3,c3,s4,c4,s34,c34,s5,c5,s6,c6,s56,c56;
  double s034, c034, s056, c056;
  double fW[2], f10[2], f17[2], R10_W[2][2], R17_W[2][2];
  double theta0_dot, theta1_dot, theta2_dot, theta3_dot, theta4_dot;
  double theta5_dot, theta6_dot;

  m0 = M_BASE; me = M_EYE; m1 = M_ARM1; m2 = M_ARM2;
  l0 = R_BASE; le = L_EYE; l1 = L_ARM1; l2 = L_ARM2; 
  a = ARM_OFFSET;
  b = BASELINE;
  s1 = sin(eyes[LEFT].theta); 
  c1 = cos(eyes[LEFT].theta); 
  s2 = sin(eyes[RIGHT].theta);
  c2 = cos(eyes[RIGHT].theta);
  s3 = sin(arms[LEFT][1].theta); 
  c3 = cos(arms[LEFT][1].theta);   
  s4 = sin(arms[LEFT][2].theta); 
  c4 = cos(arms[LEFT][2].theta); 
  s34 = sin(arms[LEFT][1].theta + arms[LEFT][2].theta);
  c34 = cos(arms[LEFT][1].theta + arms[LEFT][2].theta);
  s5 = sin(arms[RIGHT][1].theta); 
  c5 = cos(arms[RIGHT][1].theta); 
  s6 = sin(arms[RIGHT][2].theta); 
  c6 = cos(arms[RIGHT][2].theta); 
  s56 = sin(arms[RIGHT][1].theta + arms[RIGHT][2].theta);
  c56 = cos(arms[RIGHT][1].theta + arms[RIGHT][2].theta);

  s034 = sin(mobile_base->theta + arms[LEFT][1].theta + arms[LEFT][2].theta);
  c034 = cos(mobile_base->theta + arms[LEFT][1].theta + arms[LEFT][2].theta);

  R10_W[0][0] = c034;     R10_W[0][1] = s034;
  R10_W[1][0] = -s034;    R10_W[1][1] = c034;

  s056 = sin(mobile_base->theta + arms[RIGHT][1].theta + arms[RIGHT][2].theta);
  c056 = cos(mobile_base->theta + arms[RIGHT][1].theta + arms[RIGHT][2].theta);

  R17_W[0][0] = c056;     R17_W[0][1] = s056;
  R17_W[1][0] = -s056;    R17_W[1][1] = c056;

  theta0_dot = mobile_base->theta_dot;
  theta1_dot = eyes[LEFT].theta_dot;
  theta2_dot = eyes[RIGHT].theta_dot;
  theta3_dot = arms[LEFT][1].theta_dot;
  theta4_dot = arms[LEFT][2].theta_dot;
  theta5_dot = arms[RIGHT][1].theta_dot;
  theta6_dot = arms[RIGHT][2].theta_dot;
  
  // f10 = 10Rw arms[LEFT]{NARM_FRAMES-1].extForce
  fW[X] = arms[LEFT][NARM_FRAMES-1].extForce[X];
  fW[Y] = arms[LEFT][NARM_FRAMES-1].extForce[Y];

  f10[X] = R10_W[0][0]*fW[X] + R10_W[0][1]*fW[Y];
  f10[Y] = R10_W[1][0]*fW[X] + R10_W[1][1]*fW[Y];

  // f17 = 17Rw arms[RIGHT]{NARM_FRAMES-1].extForce
  fW[X] = arms[RIGHT][NARM_FRAMES-1].extForce[X];
  fW[Y] = arms[RIGHT][NARM_FRAMES-1].extForce[Y];

  f17[X] = R17_W[0][0]*fW[X] + R17_W[0][1]*fW[Y];
  f17[Y] = R17_W[1][0]*fW[X] + R17_W[1][1]*fW[Y];

  // the BASE translate ddot{r}_0
  M[0][0] = m0 + 2*me + 2*(m1+m2);
  M[0][1] = -me*le*(s1+s2)-(m1+m2)*l1*(s3+s5)-m2*l2*(s34+s56);
  M[0][2] = -me*le*s1;
  M[0][3] = -me*le*s2;
  M[0][4] = -(m1+m2)*l1*s3 - m2*l2*s34 ;
  M[0][5] = -m2*l2*s34;
  M[0][6] = -(m1+m2)*l1*s5 - m2*l2*s56;
  M[0][7] = -m2*l2*s56;

  V[0] = SQR(theta0_dot)*(-me*le*(c1+c2)-(m1+m2)*l1*(c3+c5)-m2*l2*(c34+c56)) +
    SQR(theta1_dot)*(-me*le*c1) +
    SQR(theta2_dot)*(-me*le*c2) +
    SQR(theta3_dot)*(-(m1+m2)*l1*c3 - m2*l2*c34) +
    SQR(theta4_dot)*(-m2*l2*c34) +
    SQR(theta5_dot)*(-(m1+m2)*l1*c5-m2*l2*c56) +
    SQR(theta6_dot)*(-m2*l2*c56) +
    theta0_dot*theta1_dot*(-2*me*le*c1) +
    theta0_dot*theta2_dot*(-2*me*le*c2) +
    theta0_dot*theta3_dot*(-2*(m1+m2)*l1*c3 - 2*m2*l2*c34) +
    theta0_dot*theta4_dot*(-2*m2*l2*c34) +
    theta0_dot*theta5_dot*(-2*(m1+m2)*l1*c5 -2*m2*l2*c56) +
    theta0_dot*theta6_dot*(-2*m2*l2*c56) +
    theta3_dot*theta4_dot*(-2*m2*l2*c34) +
    theta5_dot*theta6_dot*(-2*m2*l2*c56);
    
  G[0] = 0; 
  F[0] = -1*(mobile_base->extForce[X]*cos(mobile_base->theta) + 
	     mobile_base->extForce[Y]*sin(mobile_base->theta))
    + (f10[X]*c34 - f10[Y]*s34) + (f17[X]*c56 - f17[Y]*s56);


  // the BASE rotate ddot{theta}_0
  M[1][0] = -me*le*(s1+s2) - (m1+m2)*l1*(s3+s5) - m2*l2*(s34+s56);
  M[1][1] = m0*SQR(l0) + 4*me*SQR(le) + 2*me*b*le*(s1-s2) + 2*me*SQR(b)
    + (4*m1 + 2*m2)*SQR(l1) + 2*m2*l1*l2*(c4+c6) + 4*m2*SQR(l2)
    + 2*(m1+m2)*l1*a*(s3-s5) + 2*m2*l2*a*(s34-s56) + 2*(m1+m2)*SQR(a);
  M[1][2] = 2*me*SQR(le) + me*le*b*s1;
  M[1][3] = 2*me*SQR(le) - me*le*b*s2;
  M[1][4] = 2*m1*SQR(l1) + 2*m2*SQR(l2) + 2*m2*l1*l2*c4 + m2*SQR(l1) +
    m2*a*l2*s34  + (m1+m2)*a*l1*s3;
  M[1][5] = 2*m2*SQR(l2) + m2*l1*l2*c4 + m2*a*l2*s34;
  M[1][6] = 2*m1*SQR(l1) + 2*m2*SQR(l2)+2*m2*l1*l2*c6 + m2*SQR(l1) -
    m2*a*l2*s56 - (m1+m2)*a*l1*s5;
  M[1][7] = 2*m2*SQR(l2) + m2*l1*l2*c6 - m2*a*l2*s56;

  V[1] = SQR(theta1_dot)*(me*b*le*c1) +
    SQR(theta2_dot)*(-me*b*le*c2) +
    SQR(theta3_dot)*((m1+m2)*a*l1*c3 + m2*a*l2*c34) +
    SQR(theta4_dot)*(-m2*l1*l2*s4 + m2*a*l2*c34) +
    SQR(theta5_dot)*(-(m1+m2)*a*l1*c5 - m2*a*l2*c56) +
    SQR(theta6_dot)*(-m2*l1*l2*s6 - m2*a*l2*c56) +
    theta0_dot*theta1_dot*(2*me*b*le*c1) +
    theta0_dot*theta2_dot*(-2*me*b*le*c2) +
    theta0_dot*theta3_dot*(2*(m1+m2)*a*l1*c3 + 2*m2*a*l2*c34) +
    theta0_dot*theta4_dot*(-2*m2*l1*l2*s4 + 2*m2*a*l2*c34) +
    theta0_dot*theta5_dot*(-2*(m1+m2)*a*l1*c5 - 2*m2*a*l2*c56) +
    theta0_dot*theta6_dot*(-2*m2*l1*l2*s6 - 2*m2*a*l2*c56) +
    theta3_dot*theta4_dot*(-2*m2*l1*l2*s4 + 2*m2*a*l2*c34) +
    theta5_dot*theta6_dot*(-2*m2*l1*l2*s6 - 2*m2*a*l2*c56);
    
  G[1] = 0.0;
  F[1] = f10[X]*(l1*s4 - a*c34) + f10[Y]*(l1*c4 + l2 + a*s34) 
    + f17[X]*(l1*s6 + a*c56) + f17[Y]*(l1*c6 + l2 - a*s56);

  // the LEFT EYE
  M[2][0] = -me*le*s1;
  M[2][1] = 2*me*SQR(le) + me*le*b*s1;
  M[2][2] = 2*me*SQR(le);
  M[2][3] = 0.0;
  M[2][4] = 0.0;
  M[2][5] = 0.0;
  M[2][6] = 0.0;
  M[2][7] = 0.0;

  V[2] = SQR(theta0_dot)*(-me*le*b*c1);
  G[2] = F[2] = 0.0;

  // the RIGHT EYE
  M[3][0] = -me*le*s2;
  M[3][1] = 2*me*SQR(le) - me*le*b*s2;
  M[3][2] = 0.0;
  M[3][3] = 2*me*SQR(le);
  M[3][4] = 0.0;
  M[3][5] = 0.0;
  M[3][6] = 0.0;
  M[3][7] = 0.0;

  V[3] = SQR(theta0_dot)*(me*le*b*c2);
  G[3] = 0.0;
  F[3] = 0.0;

  // the LEFT ARM
  M[4][0] = -(m1+m2)*l1*s3 - m2*l2*s34;
  M[4][1] = (2*m1 + m2)*SQR(l1) + 2*m2*l1*l2*c4 + 2*m2*SQR(l2) 
    + (m1 + m2)*l1*a*s3 + m2*l2*a*s34;
  M[4][2] = 0.0;
  M[4][3] = 0.0;
  M[4][4] = (2*m1+m2)*SQR(l1) + 2*m2*l1*l2*c4 + 2*m2*SQR(l2);
  M[4][5] = 2*m2*SQR(l2) + m2*l1*l2*c4;
  M[4][6] = 0.0;
  M[4][7] = 0.0;

  V[4] = SQR(theta0_dot)*(-(m1+m2)*l1*a*c3) - m2*l2*a*c34 
    + SQR(theta4_dot)*(-m2*l1*l2*s4) +
    theta0_dot*theta4_dot *(-2*m2*l1*l2*s4) +
    theta3_dot*theta4_dot*(-2*m2*l1*l2*s4);
  
  G[4] = 0.0;
  F[4] = f10[X]*(l1*s4) + f10[Y]*(l1*c4 + l2);


  M[5][0] = -m2*l2*s34;
  M[5][1] = 2*m2*SQR(l2) + m2*l2*a*s34 + m2*l1*l2*c4;
  M[5][2] = 0.0;
  M[5][3] = 0.0;
  M[5][4] = 2*m2*SQR(l2) + m2*l1*l2*c4;
  M[5][5] = 2*m2*SQR(l2);
  M[5][6] = 0.0;
  M[5][7] = 0.0;

  V[5] = SQR(theta0_dot)*(m2*l1*l2*s4 - m2*l2*a*c34) 
    + SQR(theta3_dot)*(m2*l1*l2*s4) 
    + theta0_dot*theta3_dot*(2*m2*l1*l2*s4);

  G[5] = 0;
  
  F[5] = l2*f10[Y];

  // the RIGHT arm
  M[6][0] = -(m1+m2)*l1*s5 - m2*l2*s56 ;
  M[6][1] = (2*m1 + m2)*SQR(l1) + 2*m2*l1*l2*c6 + 2*m2*SQR(l2)
    - (m1+m2)*l1*a*s5 - m2*l2*a*s56;
  M[6][2] = 0.0;
  M[6][3] = 0.0;
  M[6][4] = 0.0;
  M[6][5] = 0.0;
  M[6][6] = (2*m1 + m2)*SQR(l1) + 2*m2*l1*l2*c6 + 2*m2*SQR(l2);
  M[6][7] = 2*m2*SQR(l2) + m2*l1*l2*c6;

  V[6] = SQR(theta0_dot)*( (m1+m2)*l1*a*c5 + m2*l2*a*c56) + 
    SQR(theta6_dot)*(-m2*l1*l2*s6) +
    theta0_dot*theta6_dot*(-2*m2*l1*l2*s6) +
    theta5_dot*theta6_dot*(-2*m2*l1*l2*s6);

  G[6] = 0.0;
  F[6] = f17[X]*(l1*s6) + f17[Y]*(l1*c6 + l2);


  M[7][0] = -m2*l2*s56;
  M[7][1] = 2*m2*SQR(l2) - m2*l2*a*s56 + m2*l1*l2*c6;
  M[7][2] = 0.0;
  M[7][3] = 0.0;
  M[7][4] = 0.0;
  M[7][5] = 0.0;
  M[7][6] = 2*m2*SQR(l2) + m2*l1*l2*c6;
  M[7][7] = 2*m2*SQR(l2);

  V[7] = SQR(theta0_dot)*(m2*l1*l2*s6 + m2*l2*a*c56)
    + SQR(theta5_dot)*(m2*l1*l2*s6)
    + theta0_dot*theta5_dot*(2*m2*l1*l2*s6);

  G[7] = 0.0;

  F[7] = f17[Y]*l2;
}
  
