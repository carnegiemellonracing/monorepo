/**
 *  @file cvxgen_interface.h
 *  @author Carnegie Mellon Racing
 *  @brief uses the cvxgen solver to give desired torques
 */

#include <stm32h7xx_hal.h>

#include "solver.h"
#include "cvxgen_interface.h"
#include "motors.h"
#include "daq.h"
#include "math.h"
#include "safety_filter.h"

Vars vars;
Params params;
Workspace work;
Settings cvxgen_settings;


void cvxgen_init(int max_iters, unsigned int max_time_ms) {
	set_defaults();
	setup_indexing();
	cvxgen_settings.max_iters = max_iters;

	unsigned int max_time = HAL_RCC_GetHCLKFreq() * max_time_ms / 1000;

}

#define K_YAW 0.15f
#define K_LIN 80.0f
#define K_TIE 0.008f

/**
 * @brief load solve parameters here
 * @return true if converged, false otherwise
*/
bool load_data(
	float steering_angle_rad,
	float T_REQ_Nm,
	float M_REQ_Nm,
	float regen_torque_Nm,
	cmr_torqueDistributionNm_t * torque_max_Nm,
	cmr_torqueDistributionNm_t *result_torques_Nm
	) {
  
	//mostly copied sweep code here
	double cos_swangle = cos(steering_angle_rad);
	double sin_swangle = sin(steering_angle_rad);

	params.Q[0] = 2*K_LIN + 2*K_TIE + 3519.8941669047*K_YAW*cos_swangle*cos_swangle + 8393.5937826189*K_YAW*cos_swangle*sin_swangle + 5003.87321656127*K_YAW*sin_swangle*sin_swangle;
	params.Q[4] = 2*K_LIN - 3519.8941669047*K_YAW*cos_swangle*cos_swangle + 5003.87321656127*K_YAW*sin_swangle*sin_swangle;
	params.Q[5] = 2*K_LIN + 2*K_TIE + 3519.8941669047*K_YAW*cos_swangle*cos_swangle - 8393.5937826189*K_YAW*cos_swangle*sin_swangle + 5003.87321656127*K_YAW*sin_swangle*sin_swangle;
	params.Q[8] = 2*K_LIN + 3519.8941669047*K_YAW*cos_swangle + 4196.79689130945*K_YAW*sin_swangle;
	params.Q[9] = 2*K_LIN - 3519.8941669047*K_YAW*cos_swangle + 4196.79689130945*K_YAW*sin_swangle;
	params.Q[10] = 2*K_LIN + 2*K_TIE + 3519.8941669047*K_YAW;
	params.Q[12] = 2*K_LIN - 3519.8941669047*K_YAW*cos_swangle - 4196.79689130945*K_YAW*sin_swangle;     
	params.Q[13] = 2*K_LIN + 3519.8941669047*K_YAW*cos_swangle - 4196.79689130945*K_YAW*sin_swangle;     
	params.Q[14] = 2*K_LIN - 3519.8941669047*K_YAW;
	params.Q[15] = 2*K_LIN + 2*K_TIE + 3519.8941669047*K_YAW;

	// Mirrors 4
	params.Q[1] = 2*K_LIN - 3519.8941669047*K_YAW*cos_swangle*cos_swangle + 5003.87321656127*K_YAW*sin_swangle*sin_swangle;
	// Mirrors 8
	params.Q[2] = 2*K_LIN + 3519.8941669047*K_YAW*cos_swangle + 4196.79689130945*K_YAW*sin_swangle;      
	// Mirrors 12
	params.Q[3] = 2*K_LIN - 3519.8941669047*K_YAW*cos_swangle - 4196.79689130945*K_YAW*sin_swangle;
	// Mirrors 9
	params.Q[6] = 2*K_LIN - 3519.8941669047*K_YAW*cos_swangle + 4196.79689130945*K_YAW*sin_swangle;
	// Mirrors 13
	params.Q[7] = 2*K_LIN + 3519.8941669047*K_YAW*cos_swangle - 4196.79689130945*K_YAW*sin_swangle;
	// Mirrors 14
	params.Q[11] = 2*K_LIN - 3519.8941669047*K_YAW;

	params.q[0] = -2 * T_REQ_Nm * K_LIN - 83.9034464954176 * cos_swangle * K_YAW * M_REQ_Nm - 100.038724667613 * K_YAW * M_REQ_Nm * sin_swangle;
	params.q[1] = -2 * T_REQ_Nm * K_LIN + 83.9034464954176 * cos_swangle * K_YAW * M_REQ_Nm - 100.038724667613 * K_YAW * M_REQ_Nm * sin_swangle;
	params.q[2] = -2 * T_REQ_Nm * K_LIN - 83.9034464954176 * K_YAW * M_REQ_Nm;
	params.q[3] = -2 * T_REQ_Nm * K_LIN + 83.9034464954176 * K_YAW * M_REQ_Nm;

	params.tau_max[0] = torque_max_Nm->fl; // or tractive torque limit
	params.tau_max[1] = torque_max_Nm->fr;
	params.tau_max[2] = torque_max_Nm->rl;
	params.tau_max[3] = torque_max_Nm->rr;

	params.tau_min[0] = fmaxf(-torque_max_Nm->fl, regen_torque_Nm);
	params.tau_min[1] = fmaxf(-torque_max_Nm->fr, regen_torque_Nm);
	params.tau_min[2] = fmaxf(-torque_max_Nm->rl, regen_torque_Nm);
	params.tau_min[3] = fmaxf(-torque_max_Nm->rr, regen_torque_Nm);

	params.a[0] = getMotorSpeed_rpm(MOTOR_FL)/GEAR_RATIO; // angular velocity of wheel in rads/s
	params.a[1] = getMotorSpeed_rpm(MOTOR_FR)/GEAR_RATIO;
	params.a[2] = getMotorSpeed_rpm(MOTOR_RL)/GEAR_RATIO;
	params.a[3] = getMotorSpeed_rpm(MOTOR_RR)/GEAR_RATIO;

	params.b[0] = power_upper_limit_W; // Power limit kWh

    cvxgen_solve();

    if(work.converged) {
		result_torques_Nm->fl = vars.tau[0];
		result_torques_Nm->fr = vars.tau[1];
		result_torques_Nm->rl = vars.tau[2];
		result_torques_Nm->rr = vars.tau[3];
		return true;
    }else{ // added for testing purposes
		result_torques_Nm->fl = 0.0f;
		result_torques_Nm->fr = 0.0f;
		result_torques_Nm->rl = 0.0f;
		result_torques_Nm->rr = 0.0f;
		return false;
	}

//	result_torques_Nm->fl = 0.0f;
//	result_torques_Nm->fr = 0.0f;
//	result_torques_Nm->rl = 0.0f;
//	result_torques_Nm->rr = 0.0f;

}
