/*************************************************************************/
/* File:        project9.c                                               */
/* Description: User project #9 - empty project directory for project    */
/*              developement                                             */
/* Date:        04-2018                                                 */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "Xkw/Xkw.h"
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

// ********************************************
// Modify the following macros according to your implementation
// *******************************************

// Number of states, modify the value according to your implementation
// {[0]->SEARCH  [1]->TRACK  [2]->SEARCHTRACK  [3]->CHASE  [4]->TOUCH  [5]->CHASETOUCH  [6]->FCLOSURE}
#define STATE_NUM  27 //2187 // 3^7

// Number of actions, modify the value according to your implementation
// [0]->SEARCH  [1]->TRACK  [2]->SEARCHTRACK  [3]->CHASE  [4]->TOUCH  [5]->CHASETOUCH  [6]->FCLOSURE
#define ACTION_NUM       3

// Number of time steps to run exploration
#define ITERMAX          4000

// If set to 1, your previously saved policy will be loaded at startup
#define LOAD_POLICY_FROM_FILE  1

// *********************************************
// *********************************************

// Quality function
double Q_func[STATE_NUM][ACTION_NUM];

// Policy
int policy[STATE_NUM];

//Rewards
double reward_func[ACTION_NUM] = {-5, -5, -5, -2, -2, -2, -2};

// Declare the functions that you need from previous projects
int stereo_observation(), fwd_arm_kinematics(), inv_arm_kinematics();
int SEARCHTRACK(), SEARCH(), TRACK(), CHASE(), TOUCH(), CHASETOUCH();
void home_position();

// explore vs. exploit
int explore = 0;


extern double recommended_setpoints_ST[2][3];
		//s,t | base_theta, left_eye, right_eye
extern double recommended_setpoints_chase[2];
		// base-x,y
double phi_w;
extern double track_error;
extern double chase_dist;

// Variables to save the original value of motor setpoints before calling
// the skills
double eyes_setpoint_initial[NEYES];
double arm_setpoint_initial[NARMS][NARM_JOINTS];
double base_setpoint_initial[NWHEELS];
double left_arm_shoulder;


// ***********************************************************************
// ***********************************************************************
// ********************* Helper Functions ********************************

// Saves the current policy to policy.txt in the current directory
void SavePolicy(int fileNum) {
  int i;
  char buf[15];
  sprintf(buf, "policy_%d.txt", fileNum);
  FILE* file_out = fopen(buf, "w");
  if (!file_out) {
    printf("Error saving the policy to file!\n");
  } else {
    for (i = 0 ; i < STATE_NUM; i++) {
      fprintf(file_out, "%d ",policy[i]);
    }

    fclose(file_out);
  }
}

// Loads the previously saved policy from policy.txt in the current directory
void LoadPolicy() {
  int i;
  FILE* file_in = fopen("policy_20.txt", "r");
  if (!file_in) {
    printf("Error loading the policy from file!\n");
  } else {
    for (i = 0; i < STATE_NUM; i++) {
      fscanf(file_in, "%d ",&policy[i]);
    }

    fclose(file_in);
  }
}

// Picks a random integer from {0, 1, ..., max_num}
int RandomInteger(max_num)
int max_num;
{
  int random_int = 0;
  double random_double = 0.0;

  if (max_num > 0) {
    // Generate a random number between 0 and 1
    random_double = ((double)rand() / (double)RAND_MAX) * (double)(max_num + 1);
    random_int = floor(random_double);

    if (random_int > max_num) random_int = max_num;
  }
  return random_int;
}


void InitializeQualityFunc() {
  int i, j;
  for (i = 0; i < STATE_NUM; i++) {
    for (j = 0; j < ACTION_NUM; j++) {
      Q_func[i][j] = 0;
    }
  }
}

void InitializePolicy() {
  int i;
  for (i = 0; i < STATE_NUM; i++) {
    policy[i] = 0;
  }
}


// Backup the setpoint values from the roger structure
void BackupCommands(roger)
Robot* roger;
{
	// printf("Roger before backup: %f %f %f %f\n", roger->arm_setpoint[LEFT][0], roger->arm_setpoint[LEFT][1], roger->arm_setpoint[RIGHT][0], roger->arm_setpoint[RIGHT][1]);
  int i, j;
  // Backup eye motors setpoint values
  for (i = 0; i < NEYES; i++) {
    eyes_setpoint_initial[i] = roger->eyes_setpoint[i];
  }


  // Backup arm motors setpoint values
  left_arm_shoulder = roger->arm_setpoint[LEFT][0];
  arm_setpoint_initial[LEFT][1] = roger->arm_setpoint[LEFT][1];
  arm_setpoint_initial[RIGHT][0] = roger->arm_setpoint[RIGHT][0];
  arm_setpoint_initial[RIGHT][1] = roger->arm_setpoint[RIGHT][1];

  // Backup wheel motor setpoint values
  for (i = 0; i < 3; i++) {
    base_setpoint_initial[i] = roger->base_setpoint[i];
  }
	// printf("Backed-up after backup: %f %f %f %f\n",left_arm_shoulder, arm_setpoint_initial[LEFT][1], arm_setpoint_initial[RIGHT][0], arm_setpoint_initial[RIGHT][1]);
}

// Restore the previously backed up setpoint values
void RestoreCommands(roger)
Robot* roger;
{
	// printf("Roger before restore: %f %f %f %f\n", roger->arm_setpoint[LEFT][0], roger->arm_setpoint[LEFT][1], roger->arm_setpoint[RIGHT][0], roger->arm_setpoint[RIGHT][1]);
	// printf("Backed-up before restore: %f %f %f %f\n", arm_setpoint_initial[LEFT][0], arm_setpoint_initial[LEFT][1], arm_setpoint_initial[RIGHT][0], arm_setpoint_initial[RIGHT][1]);
  int i, j;
  // Restore eye motors setpoint values
  for (i = 0; i < NEYES; i++) {
    roger->eyes_setpoint[i] = eyes_setpoint_initial[i];
  }

  // Restore arm motors setpoint values
  roger->arm_setpoint[LEFT][0] = left_arm_shoulder;
  roger->arm_setpoint[LEFT][1] = arm_setpoint_initial[LEFT][1];
  roger->arm_setpoint[RIGHT][0] = arm_setpoint_initial[RIGHT][0];
  roger->arm_setpoint[RIGHT][1] = arm_setpoint_initial[RIGHT][1];

  // Restore wheel motor setpoint values
  for (i = 0; i < 3; i++) {
    roger->base_setpoint[i] = base_setpoint_initial[i];
  }
	// printf("Backed-up after restore: %f %f %f %f\n", arm_setpoint_initial[LEFT][0], arm_setpoint_initial[LEFT][1], arm_setpoint_initial[RIGHT][0], arm_setpoint_initial[RIGHT][1]);
}

// ********************* Helper Functions ********************************
// ***********************************************************************
// ***********************************************************************


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// >>>>>>>>>>>>>>>> Complete this function <<<<<<<<
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Call the action associated with the given index
// [0]->SEARCH  [1]->TRACK  [2]->SEARCHTRACK  ...
void RunAction(roger, time, action_index)
Robot* roger;
double time;
int action_index;
{
	// printf("Action %d\n", action_index);
	switch(action_index){
		// case 0: SEARCH(roger, time);
		// 		roger->base_setpoint[THETA] = recommended_setpoints_ST[0][0];
		// 		roger->eyes_setpoint[LEFT] = recommended_setpoints_ST[0][1];
		// 		roger->eyes_setpoint[RIGHT] = recommended_setpoints_ST[0][2];
		// 		// printf("Base heading in action: %f\n", recommended_setpoints_ST[0][0]);
		// 		break;

		// case 1: TRACK(roger, time);
		// 		roger->base_setpoint[THETA] = recommended_setpoints_ST[1][0];
		// 		roger->eyes_setpoint[LEFT] = recommended_setpoints_ST[1][1];
		// 		roger->eyes_setpoint[RIGHT] = recommended_setpoints_ST[1][2];
		// 		break;

		case 0: SEARCHTRACK(roger, time);
				break;

		// case 2: CHASE(roger, time);
		// 		roger->base_setpoint[X] = recommended_setpoints_chase[0];
		// 		roger->base_setpoint[Y] = recommended_setpoints_chase[1];
		// 		break;

		// case 3: TOUCH(roger, time);
		// 		break;

		case 1: CHASETOUCH(roger, time);
				break;

		case 2: FCLOSURE(roger, time);
				break;
	}
}


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// >>>>>>>>>>>>>>>> Complete this function <<<<<<<<
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Generate a policy given a Q function (Q table)
void GeneratePolicy(policy, Q_func)
int policy[STATE_NUM];
double Q_func[STATE_NUM][ACTION_NUM];
{
  int i, j;
  for (i = 0; i < STATE_NUM; i++) {
    policy[i] = 0;
    for (j = 0; j < ACTION_NUM; j++){
    	if (Q_func[i][j] > Q_func[i][policy[i]]){
    		policy[i] = j;
    	}
    	// printf("Q_func State:%d Action: %d Value: %f\n", i, j, Q_func[i][j]);
    }
    // printf("Policy for above: %d\n", policy[i]);
  }
}


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// >>>>>>>>>>>>>>>> Complete this function <<<<<<<<
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int FCLOSURE(roger, time)
Robot* roger;
double time;
{
	static int return_state = NO_REFERENCE;
	double * pos = stereo_observation(roger, time);
	double left[2], right[2];

	if (isfinite(pos[0]) && isfinite(pos[1])){
		return_state = TRANSIENT;
		double left_arm_forces[2], right_arm_forces[2], base_forces[2], estimated_theta[3];

		left_arm_forces[0] = roger->ext_force[LEFT][0];
		left_arm_forces[1] = roger->ext_force[LEFT][1];
		right_arm_forces[0] = roger->ext_force[RIGHT][0];
		right_arm_forces[1] = roger->ext_force[RIGHT][1];
		base_forces[0] = roger->ext_force_body[0];
		base_forces[1] = roger->ext_force_body[1];

		if (fabs(base_forces[0]) > 0 || fabs(base_forces[1]) > 0){
			estimated_theta[0] = M_PI + atan2(base_forces[1], base_forces[0]);
		}else{
			estimated_theta[0] = M_PI + atan2(pos[1] - roger->base_position[1], pos[0] - roger->base_position[0]);
		}

		if (fabs(left_arm_forces[0]) > 0 || fabs(left_arm_forces[1]) > 0){
			estimated_theta[1] = M_PI + atan2(left_arm_forces[1], left_arm_forces[0]);
		}else{
			double wTb[4][4], ref_bl[4], ref_wl[4];
			construct_wTb(roger->base_position, wTb);
			double lx,ly;
			fwd_arm_kinematics(roger, LEFT, &lx, &ly);
			ly += ARM_OFFSET;
			ref_bl[0] = lx;
			ref_bl[1] = ly;
			ref_bl[2] = 0.0;
			ref_bl[3] = 1.0;
			matrix_mult(4, 4, wTb, 1, ref_bl, ref_wl);
			estimated_theta[1] = M_PI + atan2(pos[1]-ref_wl[1], pos[0]-ref_wl[0]);
		}

		if (fabs(right_arm_forces[0]) > 0 || fabs(right_arm_forces[1]) > 0){
			estimated_theta[2] = M_PI + atan2(right_arm_forces[1], right_arm_forces[0]);
		}else{
			double wTb[4][4], ref_br[4], ref_wr[4];
			construct_wTb(roger->base_position, wTb);
			double rx, ry;
			fwd_arm_kinematics(roger, RIGHT, &rx, &ry);
			ry -= ARM_OFFSET;
			ref_br[0] = rx;
			ref_br[1] = ry;
			ref_br[2] = 0.0;
			ref_br[3] = 1.0;
			matrix_mult(4, 4, wTb, 1, ref_br, ref_wr);
			estimated_theta[2] = M_PI + atan2(pos[1]-ref_wr[1], pos[0]-ref_wr[0]);
		}
		// printf("%f %f %f\n", 180*estimated_theta[0]/M_PI, 180*estimated_theta[1]/M_PI, 180*estimated_theta[2]/M_PI);

		double j[2], del_theta[2], kappa, new_theta[2], l_pos[2], r_pos[2];
		kappa = 1;

		phi_w = 3.0 + 2.0*cos(estimated_theta[0] - estimated_theta[1])
				+ 2.0*cos(estimated_theta[0] - estimated_theta[2])
				+ 2.0*cos(estimated_theta[1] - estimated_theta[2]);
		j[0] = 2.0*sin(estimated_theta[0] - estimated_theta[1]) - 2.0*sin(estimated_theta[1] - estimated_theta[2]);
		j[1] = 2.0*sin(estimated_theta[0] - estimated_theta[2]) + 2.0*sin(estimated_theta[1] - estimated_theta[2]);
		double den = pow(j[0], 2) + pow(j[1],2);
		j[0] = j[0]/den;
		j[1] = j[1]/den;

		del_theta[0] = -1.0*kappa*j[0]*phi_w;
		del_theta[1] = -1.0*kappa*j[1]*phi_w;
		new_theta[0] = estimated_theta[1] + del_theta[0];
		new_theta[1] = estimated_theta[2] + del_theta[1];

		double R = 0.22;

		left[0] = pos[0] + R*cos(new_theta[0]);
		left[1] = pos[1] + R*sin(new_theta[0]);

		right[0] = pos[0] + R*cos(new_theta[1]);
		right[1] = pos[1] + R*sin(new_theta[1]);

		if ((fabs(left_arm_forces[0]) > 0 || fabs(left_arm_forces[1]) > 0)
		 && (fabs(right_arm_forces[0]) > 0 || fabs(right_arm_forces[1]) > 0 )
		 && (fabs(base_forces[0]) > 0 || fabs(base_forces[1]) > 0) && phi_w < 0.15){
			return_state = CONVERGED;
		}
	}else{
		return_state = NO_REFERENCE;
	}
	if (return_state == TRANSIENT){
		int l_in_range = inv_arm_kinematics(roger, LEFT, left[0], left[1]);
		int r_in_range = inv_arm_kinematics(roger, RIGHT, right[0], right[1]);
		if (l_in_range == FALSE || r_in_range == FALSE) return_state = NO_REFERENCE;
	}
  
  return return_state;
}


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// >>>>>>>>>>>>>>>> Complete this function <<<<<<<<
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// *********************************************************
// GRASP behavior
// Complete this function and implement a reinforcement learning process
// that constructs the grasping behavior
// **********************************************************
int GRASP(roger, time)
Robot* roger;
double time;
{
	// **************************************
	// MODIFY THE SIZE OF "internal_state[]" TO MATCH THE NUMBER OF YOUR STATES
	static int internal_state[7] = { NO_REFERENCE, NO_REFERENCE, NO_REFERENCE,
									   NO_REFERENCE, NO_REFERENCE, NO_REFERENCE, 
									   NO_REFERENCE};

	static int return_state = NO_REFERENCE;
	static int initialized = 0;

	static int previous_state = 0;
	static int previous_action = 0;
	static int counter = 0;

	static int last_saved_policy = 0;
	static double epsilon = 0.5;

	static int num_of_actions_picked = 0;
	int actions_max = 200;
	static double rewards_sum = 0;

  if (!initialized) {
    if (LOAD_POLICY_FROM_FILE) {
      LoadPolicy();
    }
    else {
      InitializePolicy();
    }
    InitializeQualityFunc();
    initialized = 1;
  }

	int selected_action = 0;

	BackupCommands(roger);

	// internal_state[0] = SEARCH(roger, time);
	// internal_state[1] = TRACK(roger, time);
	internal_state[0] = SEARCHTRACK(roger, time);
	// internal_state[2] = CHASE(roger, time);
	// internal_state[3] = TOUCH(roger, time);
	internal_state[1] = CHASETOUCH(roger, time);
	internal_state[2] = FCLOSURE(roger, time);

	int state = 0, i;
	for (i = 0; i < ACTION_NUM; i++){
		state += pow(3, i)*internal_state[i];
	}

	RestoreCommands(roger);
	// printf("Roger after restore: %f %f %f %f\n", roger->arm_setpoint[LEFT][0], roger->arm_setpoint[LEFT][1], roger->arm_setpoint[RIGHT][0], roger->arm_setpoint[RIGHT][1]);

	if (explore) {
	    if (counter % 200 == 0 && last_saved_policy != counter) {
	    	last_saved_policy = counter;
	       printf("Iteration num %d\n", counter);
	       GeneratePolicy(policy, Q_func);
	       SavePolicy(last_saved_policy/200);
	    }

	    // Terminate if ITERMAX is reached
	    if (counter > ITERMAX) {
	       printf("Generating Policy\n");

	     // Generate a greedy policy given the current Q table
	     // IMPLEMENT GeneratePolicy() 
	       GeneratePolicy(policy, Q_func);
	       // SavePolicy();
	       counter = 0;
	       explore = 0;
	       printf("Generated Policy!\n");
	       int i;
	       // for (i = 0; i < STATE_NUM; i++){
	       // 	printf("%d\n", policy[i]);
	       // }
	    }

	    if (previous_state != state){
			double total_reward = reward_func[previous_action];
			double gamma = 0.9, alpha = 0.07;
			if (internal_state[2] == TRANSIENT && previous_action == 2) {
				total_reward += 3.0/phi_w;
				// printf("Phi: %f\n", phi_w);
			}
			if(internal_state[0] != NO_REFERENCE && previous_action != 2){
				total_reward += 1.2/(0.1+track_error);
			}
			if(internal_state[1] != NO_REFERENCE && previous_action == 1){
				total_reward += 0.06/(chase_dist);
			}

			double maxQ = Q_func[state][0];
			for (i = 1; i < ACTION_NUM; i++){
				if (Q_func[state][i] > maxQ) maxQ = Q_func[state][i];
			}
			Q_func[previous_state][previous_action] += alpha*(total_reward + gamma*maxQ - Q_func[previous_state][previous_action]);
			// printf("State: %d, QMAX: %f, reward: %f\n", state, maxQ, total_reward);
			counter++;
	    }
	 	epsilon = 0.2 + 0.6*(counter/ITERMAX);
	 	double rand_num = (double)rand() / (double)RAND_MAX;
	 	if (rand_num > epsilon){
			selected_action = RandomInteger(ACTION_NUM - 1);
	 	} else{
	 		selected_action = 0;
	 		for (i = 1; i < ACTION_NUM; i++){
	 			if (Q_func[state][i] > Q_func[state][selected_action]) selected_action = i;
	 		}
	 	}
	}
	else {
		selected_action = policy[state];
		if (previous_state != state && num_of_actions_picked < actions_max){
			rewards_sum += reward_func[previous_action];
			if (internal_state[2] == TRANSIENT && previous_action == 2) rewards_sum += 3.0/phi_w;
			if(internal_state[0] != NO_REFERENCE && previous_action != 2) rewards_sum += 1.2/(0.1+track_error);
			if(internal_state[1] != NO_REFERENCE && previous_action == 1) rewards_sum += 0.06/(chase_dist);
			if (internal_state[2] == CONVERGED)	num_of_actions_picked = actions_max;
			num_of_actions_picked += 1;
	    }
		printf("selected_action: %d state: %d rewardSum: %f stop at 10: %d\n", selected_action, state, rewards_sum, num_of_actions_picked);
	}

	RunAction(roger, selected_action);
	previous_state = state;
	previous_action = selected_action;

	// Map internal_state to return_state. return_state should hold one of the
	// { NO_REFERENCE, TRANSIENT, CONVERGED } values. 

	return return_state;
}


void project9_control(roger, time)
Robot* roger;
double time;
{
  // SEARCHTRACK(roger, time);
  // FCLOSURE(roger, time);

  GRASP(roger, time);
	// BackupCommands(roger);
}

/************************************************************************/
void project9_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
// enter_params is currently set such that you can switch between 
// exploration and exploitation in the RL process
void project9_enter_params()
{
  printf("Current explore value is %d \n", explore);
  printf("Enter 1 to explore and 0 to exploit\n"); fflush(stdout);
  scanf("%d", &explore);
}

//function called when the 'visualize' button on the gui is pressed
void project9_visualize(roger)
Robot* roger;
{ }
