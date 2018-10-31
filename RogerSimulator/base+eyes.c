/*************************************************************************/
/* File:        base+eyes.c ---> "platform.c" option                     */
/* Description: dynamic simulation of the nonholonomic base              */
/*              fixed arms - first draft                                 */
/* Author:      Rod Grupen                                               */
/* Date:        10-2014                                                  */
/*************************************************************************/
#include <math.h>
#include <stdlib.h>
#include "include/Roger.h"
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
static double l[2] = {LARM_1, LARM_2};
static double m[2] = {MARM_1, MARM_2};

Arm arms_home[NARMS][NARM_FRAMES] = 
  { { { { { 1.0, 0.0, 0.0, 0.0 },         /* LEFT ARM         */
	  { 0.0, 1.0, 0.0, ARM_OFFSET },  /* mobile base to frame 0 */
	  { 0.0, 0.0, 1.0, 0.0 },
	  { 0.0, 0.0, 0.0, 1.0 } }, NOTYPE,NOAXIS, 0.0,0.0,
	0.0, {0.0,0.0} },
      { { { 1.0, 0.0, 0.0, 0.0 },  /* frame 0 to frame 1 */
	  { 0.0, 1.0, 0.0, 0.0  },
	  { 0.0, 0.0, 1.0, 0.0 },
	  { 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE,ZAXIS, (9.0*M_PI/10.0),0.0,
	0.0, {0.0,0.0} },
      { { { 1.0, 0.0, 0.0, LARM_1 },  /* frame 1 to frame 2 */
	  { 0.0, 1.0, 0.0, 0.0 },
	  { 0.0, 0.0, 1.0, 0.0 },
	  { 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE,ZAXIS, (-9.0*M_PI/10.0),0.0,
	0.0, {0.0,0.0} },
      { { { 1.0, 0.0, 0.0, LARM_2 },  /* frame 2 to frame 3 */
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
	  { 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE,ZAXIS, (-9.0*M_PI/10.0),0.0,
	0.0, {0.0,0.0} },
      { { { 1.0, 0.0, 0.0, LARM_1 },  /* frame 1 to frame 2 */
	  { 0.0, 1.0, 0.0, 0.0 },
	  { 0.0, 0.0, 1.0, 0.0 },
	  { 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE,ZAXIS, (9.0*M_PI/10.0),0.0,
	0.0, {0.0,0.0} },
      { { { 1.0, 0.0, 0.0, LARM_2 },  /* frame 2 to frame 3 */
	  { 0.0, 1.0, 0.0, 0.0 },
	  { 0.0, 0.0, 1.0, 0.0 },
	  { 0.0, 0.0, 0.0, 1.0 } }, NOTYPE,NOAXIS, 0.0,0.0,
	0.0, {0.0,0.0} } } };

void arm_dynamics(arm,M,V,G,F)
Arm arm[NARM_FRAMES];
double M[NARM_JOINTS][NARM_JOINTS],V[NARM_JOINTS],G[NARM_JOINTS];
double F[NARM_JOINTS];
{
  double s1, c1, s2, c2, s12, c12;

  s1 = sin(arm[1].theta);
  c1 = cos(arm[1].theta);
  s2 = sin(arm[2].theta);
  c2 = cos(arm[2].theta);
  s12 = sin(arm[1].theta + arm[2].theta);
  c12 = cos(arm[1].theta + arm[2].theta);

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
  F[0] = -(l[0]*s1+l[1]*s12)*arm[NARM_FRAMES-1].extForce[0] 
    + (l[0]*c1+l[1]*c12)*arm[NARM_FRAMES-1].extForce[1];
  F[1] = -(l[0]*s12)*arm[NARM_FRAMES-1].extForce[0] 
    + (l[1]*c12)*arm[NARM_FRAMES-1].extForce[1];
}

void invert(A,Ainv)
double A[NARM_JOINTS][NARM_JOINTS], Ainv[NARM_JOINTS][NARM_JOINTS];
{
  double det;

  switch(NARM_JOINTS) {
  case (1):
    Ainv[0][0] = 1.0/A[0][0];
    break;
  case (2):
    det = 1.0 / (A[0][0]*A[1][1] - A[1][0]*A[0][1]);
    
    Ainv[0][0] = det * A[1][1];
    Ainv[1][0] = -1.0 * det * A[1][0];
    Ainv[0][1] = -1.0 * det * A[0][1];
    Ainv[1][1] = det * A[0][0];
    break;

  case(3):
    det = 1.0 / (A[0][0]*A[1][1]*A[2][2] + A[0][1]*A[1][2]*A[2][0] 
		  + A[0][2]*A[1][0]*A[2][1] - A[2][0]*A[1][1]*A[0][2]
		  - A[2][1]*A[1][2]*A[0][0] - A[2][2]*A[1][0]*A[0][1]) ;

    Ainv[0][0] = det * (A[1][1]*A[2][2]-A[2][1]*A[1][2]);
    Ainv[0][1] = det * (A[2][1]*A[0][2]-A[0][1]*A[2][2]);
    Ainv[0][2] = det * (A[0][1]*A[1][2]-A[1][1]*A[0][2]);
    Ainv[1][0] = det * (A[2][0]*A[1][2]-A[1][0]*A[2][2]);
    Ainv[1][1] = det * (A[0][0]*A[2][2]-A[2][0]*A[0][2]);
    Ainv[1][2] = det * (A[1][0]*A[0][2]-A[0][0]*A[1][2]);
    Ainv[2][0] = det * (A[1][0]*A[2][1]-A[2][0]*A[1][1]);
    Ainv[2][1] = det * (A[2][0]*A[0][1]-A[0][0]*A[2][1]);
    Ainv[2][2] = det * (A[0][0]*A[1][1]-A[1][0]*A[0][1]);
    break;
  }
}

void matrix_mult(A,x,y)
double A[NARM_JOINTS][NARM_JOINTS],x[NARM_JOINTS],y[NARM_JOINTS];
{
  int i,j;
  for (i=0; i<NARM_JOINTS; ++i) {
    y[i] = 0.0;
    for (j=0; j<NARM_JOINTS; ++j) {
      y[i] += A[i][j] * x[j];
    }
  }
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

// HACK
void rectify_theta(arm)
Arm arm[NARM_FRAMES];
{
  while (arm[1].theta < (-M_PI)) arm[1].theta += 2.0 * M_PI;
  while (arm[1].theta > M_PI) arm[1].theta -= 2.0 * M_PI;
  while (arm[2].theta < (-M_PI)) arm[2].theta += 2.0 * M_PI;
  while (arm[2].theta > M_PI) arm[2].theta -= 2.0 * M_PI;
}

// 4x4 dynamics
void simulate_roger(mobile_base, arms, eyes)
Base *mobile_base;
Arm arms[NARMS][NARM_FRAMES];
Eye eyes[NEYES];
{
  int i,j;
  double M[4][4], V[4], G[4], F[4], Minv[4][4]; // NDOF = 8?
  double gForce[4], q_ddot[4], vmag;
  void roger_dynamics();
  void invert_WBM();
  int invert_M();

  roger_dynamics(mobile_base, arms, eyes, M, V, G, F);  
  invert_M(M, 4, Minv);

  // base - elements 0 and 1
  gForce[0]=(mobile_base->wheel_torque[LEFT]+mobile_base->wheel_torque[RIGHT])
    - V[0] -G[0] -F[0];
  gForce[1]=R_BASE * (mobile_base->wheel_torque[RIGHT]
		      - mobile_base->wheel_torque[LEFT]) - V[1] - G[1] - F[1];
  gForce[2] = eyes[LEFT].torque - V[2] - G[2] - F[2];
  gForce[3] = eyes[RIGHT].torque - V[3] - G[3] - F[3];

  // 4x4 matrix times 4x1 vector yields accelerations
  for (i=0;i<4;++i) {
    q_ddot[i] = 0;
    for (j=0;j<4;++j) {
      q_ddot[i] += Minv[i][j] * gForce[j];
    }
  }

  // EULER INTEGRATION FOR BASE, ARMS, AND EYES
  // base translation q_ddot[0] --- nonholonomic, homo transform update, wheel
  //      speed (for motor models)
  vmag = mobile_base->x_dot * cos(mobile_base->theta) +
    mobile_base->y_dot * sin(mobile_base->theta);
  //  mobile_base->x_dot += q_ddot[0]*DT*cos(mobile_base->theta);
  //  mobile_base->y_dot += q_ddot[0]*DT*sin(mobile_base->theta);
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
  if (eyes[LEFT].theta > M_PI/2.0) {
    eyes[LEFT].theta = M_PI/2.0;
    eyes[LEFT].theta_dot = 0.0;
  }    
  else if (eyes[LEFT].theta < -M_PI/2.0) {
    eyes[LEFT].theta = -M_PI/2.0;
    eyes[LEFT].theta_dot = 0.0;
  }

  // right eye q_ddot[3]
  eyes[RIGHT].theta_dot += q_ddot[3]*DT;
  eyes[RIGHT].theta += 0.5*q_ddot[3]*DT*DT + eyes[RIGHT].theta_dot*DT;
  if (eyes[RIGHT].theta > M_PI/2.0) {
    eyes[RIGHT].theta = M_PI/2.0;
    eyes[RIGHT].theta_dot = 0.0;
  }    
  else if (eyes[RIGHT].theta < -M_PI/2.0) {
    eyes[RIGHT].theta = -M_PI/2.0;
    eyes[RIGHT].theta_dot = 0.0;
  }

  update_state(arms[LEFT]);
  update_state(arms[RIGHT]);
}

// special purpose for the existing block diagonal dynamics
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
  M[0][0] = M_BASE;
  M[0][1] = 0;
  M[1][0] = 0;
  M[1][1] = I_BASE;

  // the LEFT EYE
  M[2][2] = I_EYE;

  // the LEFT ARM
  arm_dynamics(arms[LEFT], Marm, Varm, Garm, Farm);
  M[3][3] = Marm[0][0];
  M[3][4] = Marm[0][1];
  M[4][3] = Marm[1][0];
  M[4][4] = Marm[1][1];
  V[3] = Varm[0]; G[3] = Garm[0]; F[3] = Farm[0];
  V[4] = Varm[1]; G[4] = Garm[1]; F[4] = Farm[1];

  // the RIGHT EYE
  M[5][5] = I_EYE;

  // the RIGHT arm
  arm_dynamics(arms[RIGHT], Marm, Varm, Garm, Farm);
  M[6][6] = Marm[0][0];
  M[6][7] = Marm[0][1];
  M[7][6] = Marm[1][0];
  M[7][7] = Marm[1][1];
  V[6] = Varm[0]; G[6] = Garm[0]; F[6] = Farm[0];
  V[6] = Varm[1]; G[6] = Garm[1]; F[6] = Farm[1];
}
  
// experimental version of the base dynamics
void roger_dynamics(mobile_base, arms, eyes, M, V, G, F)
Base *mobile_base;
Arm arms[NARMS][NARM_FRAMES];
Eye eyes[NEYES];
double M[4][4], V[4], G[4], F[4];
{
  int i,j;
  double Marm[2][2], Varm[2], Garm[2], Farm[2];
  double m0, me, m1, m2;
  double l0, le, l1, l2;
  double a, b;
  double s1,c1,s2,c2,s3,c3,s23,c23,s4,c4,s5,c5,s6,c6,s56,c56;
  double theta0_dot, theta1_dot, theta2_dot, theta3_dot, theta4_dot;
  double theta5_dot, theta6_dot;
  double f10x, f10y, f17x, f17y;

  m0 = M_BASE; me = M_EYE; m1 = MARM_1; m2 = MARM_2;
  l0 = R_BASE; le = L_EYE; l1 = LARM_1; l2 = LARM_2; 
  a = ARM_OFFSET;
  b = BASELINE;
  s1 = sin(eyes[LEFT].theta); 
  c1 = cos(eyes[LEFT].theta); 
  s2 = sin(eyes[RIGHT].theta); 
  c2 = cos(eyes[RIGHT].theta);   
  //  s3 = sin(arms[LEFT][2].theta); 
  //  c3 = cos(arms[LEFT][2].theta); 
  //  s23 = sin(arms[LEFT][1].theta + arms[LEFT][2].theta);
  //  c23 = cos(arms[LEFT][1].theta + arms[LEFT][2].theta);
  //  s4 = sin(eyes[RIGHT].theta);
  //  c4 = cos(eyes[RIGHT].theta);
  //  s5 = sin(arms[RIGHT][1].theta); 
  //  c5 = cos(arms[RIGHT][1].theta); 
  //  s6 = sin(arms[RIGHT][2].theta); 
  //  c6 = cos(arms[RIGHT][2].theta); 
  //  s56 = sin(arms[RIGHT][1].theta + arms[RIGHT][2].theta);
  //  c56 = cos(arms[RIGHT][1].theta + arms[RIGHT][2].theta);

  theta0_dot = mobile_base->theta_dot;
  theta1_dot = eyes[LEFT].theta_dot;
  theta2_dot = eyes[RIGHT].theta_dot;
  //  theta3_dot = arms[LEFT][2].theta_dot;
  //  theta4_dot = eyes[RIGHT].theta_dot;
  //  theta5_dot = arms[RIGHT][1].theta_dot;
  //  theta6_dot = arms[RIGHT][2].theta_dot;
  
  // the BASE f_x
  M[0][0] = m0 + 2*me;  
     M[0][1] = -me*le*(s1+s2);  
        M[0][2] = -me*le*s1;
           M[0][3] = -me*le*s2;

  V[0] = SQR(theta0_dot)*(-me*le*(c1+c2)) + SQR(theta1_dot)*(-me*le*c1)
    + SQR(theta2_dot)*(-me*le*c2) + theta0_dot*theta1_dot*(-2*me*le*c1)
    + theta0_dot*theta2_dot*(-2*me*le*c2);

  F[0] = -1.0*(mobile_base->extForce[X]*cos(mobile_base->theta) + 
	     mobile_base->extForce[Y]*sin(mobile_base->theta));


  // the BASE tau_z
  M[1][0] = -me*le*(s1+s2);  
     M[1][1] = m0*SQR(l0) + 4*me*SQR(le) + 2*me*le*b*(s1-s2) + 2*me*SQR(b);
        M[1][2] = 2*me*SQR(le) + me*le*b*s1;
           M[1][3] = 2*me*SQR(le) - me*le*b*s2;

	   V[1] = SQR(theta1_dot)*(me*le*b*c1) + SQR(theta2_dot)*(-me*le*b*c2)
	     + theta0_dot*theta1_dot*(2*me*le*b*c1)
	     + theta0_dot*theta2_dot*(-2*me*le*b*c2);

  F[1] = 0.0;



  // the LEFT EYE tau_z
  M[2][0] = -me*le*s1;
     M[2][1] = 2*me*SQR(le) + me*le*b*s1;
        M[2][2] = 2*me*SQR(le);
           M[2][3] = 0.0;

  V[2] = SQR(theta0_dot)*(-me*le*b*c1);

  F[2] = 0.0;



  // the RIGHT EYE tau_z
  M[3][0] = -me*le*s2;
     M[3][1] = 2*me*SQR(le) - me*le*b*s2;
        M[3][2] = 0.0;
           M[3][3] = 2*me*SQR(le);

  V[3] = SQR(theta0_dot)*(me*le*b*c2);

  F[3] = 0.0;


  // UNUSED GRAVITATIONAL tau_z
  G[0] = G[1] = G[2] = G[3] = 0.0;
}

//**********sample invocation************
// double* m = M[0];
// double* mi = MI[0];
// invert_M(m, 8 , mi);

//****************inverse Method*********************
int invert_M(double* A, int n, double* AInverse)
{
  // A = input matrix (n x n)
  // n = dimension of A
  // AInverse = inverted matrix (n x n)
  // This function inverts a matrix based on the Gauss Jordan method.
  // The function returns 1 on success, 0 on failure.
  int i, j, iPass, imx, icol, irow;
  double det, temp, pivot, factor;
  double* ac = (double*)calloc(n*n, sizeof(double));
  det = 1.0;

  for (i = 0; i < n; i++) {
    for (j = 0; j < n; j++) {
      AInverse[n*i+j] = 0;
      ac[n*i+j] = A[n*i+j];
    }
    AInverse[n*i+i] = 1;
  }

  // The current pivot row is iPass.
  // For each pass, first find the maximum element in the pivot column.
  for (iPass = 0; iPass < n; iPass++) {
    imx = iPass;
    for (irow = iPass; irow < n; irow++) {
      if (fabs(A[n*irow+iPass]) > fabs(A[n*imx+iPass])) imx = irow;
    }
    // Interchange the elements of row iPass and row imx in both A and
    //    AInverse.
    if (imx != iPass) {
      for (icol = 0; icol < n; icol++) {
	temp = AInverse[n*iPass+icol];
	AInverse[n*iPass+icol] = AInverse[n*imx+icol];
	AInverse[n*imx+icol] = temp;

	if (icol >= iPass) {
	  temp = A[n*iPass+icol];
	  A[n*iPass+icol] = A[n*imx+icol];
	  A[n*imx+icol] = temp;
	}
      }
    }
    // The current pivot is now A[iPass][iPass].
    // The determinant is the product of the pivot elements.
    pivot = A[n*iPass+iPass];
    det = det * pivot;
    if (det == 0) {
      free(ac);
      return 0;
    }
    for (icol = 0; icol < n; icol++) {
      // Normalize the pivot row by dividing by the pivot element.
      AInverse[n*iPass+icol] = AInverse[n*iPass+icol] / pivot;
      if (icol >= iPass) A[n*iPass+icol] = A[n*iPass+icol] / pivot;
    }
    for (irow = 0; irow < n; irow++) {
      // Add a multiple of the pivot row to each row.  The multiple factor
      // is chosen so that the element of A on the pivot column is 0.
      if (irow != iPass) factor = A[n*irow+iPass];
      for (icol = 0; icol < n; icol++) {
	if (irow != iPass) {
	  AInverse[n*irow+icol] -= factor * AInverse[n*iPass+icol];
	  A[n*irow+icol] -= factor * A[n*iPass+icol];
	}
      }
    }
  }
  free(ac);
  return 1;
}
