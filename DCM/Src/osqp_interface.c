/**
 *  @file osqp_interface.c
 *  @author Carnegie Mellon Racing
 *  @brief uses the osqp solver to give desired results
 */

#include <stm32h7xx_hal.h> // HAL interface

#include "osqp_interface.h"
#include "workspace.h"
#include "osqp.h"

#define ARM_MATH_CM4
#include "arm_math.h"

#define K_LIN 80.0
#define K_YAW 0.15
#define K_TIE 0.008

void osqp_guess(float FL, float FR, float RL, float RR)
{
	c_float new_x[4];

	new_x[0] = FL;
	new_x[1] = FR;
	new_x[2] = RL;
	new_x[3] = RR;

	c_float new_y[4];

	new_y[0] = 0;
	new_y[1] = 0;
	new_y[2] = 0;
	new_y[3] = 0;

	osqp_warm_start_x(&workspace, new_x);
	osqp_warm_start_y(&workspace, new_y);
}

void osqp_init(int max_iters)
{
	osqp_update_max_iter(&workspace, max_iters);
}

float solve_with_osqp(const double steering_angle,
           const double M_REQ,
           const double T_REQ,
           double *resFL,
           double *resFR,
           double *resRL,
           double *resRR
)
{
    uint32_t start_tick = HAL_GetTick();

	float32_t cos_swangle, sin_swangle;

	cos_swangle = cos(steering_angle);
	sin_swangle = sin(steering_angle);

//	arm_sin_cos_f32((float32_t)steering_angle, &sin_swangle, &cos_swangle);



    // UPDATE P

    c_float P_data[10];

    P_data[0] = 2.0 * (1759.94708345235 * cos_swangle * cos_swangle * K_YAW + 4196.79689130945 * cos_swangle * K_YAW * sin_swangle + K_LIN + K_TIE + 2501.93660828063 * K_YAW * sin_swangle * sin_swangle);
    P_data[1] = -3519.8941669047 * cos_swangle * cos_swangle * K_YAW + 2.0f * K_LIN + 5003.87321656127 * K_YAW * sin_swangle * sin_swangle;
    P_data[2] = 2.0 * (1759.94708345235 * cos_swangle * cos_swangle * K_YAW - 4196.79689130945 * cos_swangle * K_YAW * sin_swangle + K_LIN + K_TIE + 2501.93660828063 * K_YAW * sin_swangle * sin_swangle);
    P_data[3] = 3519.8941669047 * cos_swangle * K_YAW + 2.0 * K_LIN + 4196.79689130945 * K_YAW * sin_swangle;
    P_data[4] = -3519.8941669047 * cos_swangle * K_YAW + 2.0 * K_LIN + 4196.79689130945 * K_YAW * sin_swangle;
    P_data[5] = 2.0 * (K_LIN + K_TIE + 1759.94708345235 * K_YAW);
    P_data[6] = -3519.8941669047 * cos_swangle * K_YAW + 2.0 * K_LIN - 4196.79689130945 * K_YAW * sin_swangle;
    P_data[7] = 3519.8941669047 * cos_swangle * K_YAW + 2.0 * K_LIN - 4196.79689130945 * K_YAW * sin_swangle;
    P_data[8] = 2.0 * K_LIN - 3519.8941669047 * K_YAW;
    P_data[9] = 2.0 * (K_LIN + K_TIE + 1759.94708345235 * K_YAW);

    osqp_update_P(&workspace, P_data, NULL, 10);


    // UPDATE Q

    c_float q[4];

    q[0] = -2.0f * T_REQ * K_LIN - 83.9034464954176f * cos_swangle * K_YAW * M_REQ - 100.038724667613f * K_YAW * M_REQ * sin_swangle;
    q[1] = -2.0f * T_REQ * K_LIN + 83.9034464954176f * cos_swangle * K_YAW * M_REQ - 100.038724667613f * K_YAW * M_REQ * sin_swangle;
    q[2] = -2.0f * T_REQ * K_LIN - 83.9034464954176f * K_YAW * M_REQ;
    q[3] = -2.0f * T_REQ * K_LIN + 83.9034464954176f * K_YAW * M_REQ;


    osqp_update_lin_cost(&workspace, q);

    uint32_t end_tick = HAL_GetTick();
    // UPDATE L

    // UPDATE U

    // Solve Problem
    osqp_solve(&workspace);

    uint32_t solve_tick = HAL_GetTick();

    float solve_time = (float)(solve_tick - start_tick);

    *resFL = (&workspace)->solution->x[0] ;
    *resFR = (&workspace)->solution->x[1] ;
    *resRL = (&workspace)->solution->x[2] ;
    *resRR = (&workspace)->solution->x[3] ;
    return solve_time;
}
