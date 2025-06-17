/**
 * @file soc.c
 * @brief State of charge estimation.
 *
 * @author Carnegie Mellon Racing
 */

// ------------------------------------------------------------------------------------------------
// Includes

#include "soc.h"         // Interface to implement
#include "can.h"
#include <arm_math.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <limits.h>
#include <stdlib.h>
#include <CMR/tasks.h>
#include <CMR/can_types.h>  // CMR CAN types


static const uint32_t soc_priority = 5;
/** @brief SOC task period. */
static const TickType_t soc_period_ms = 100;
/** @brief SOC control task. */
static cmr_task_t soc_task;


// variables ----------------

float32_t timestep;
float32_t soc;
float32_t current;
float32_t voltage;
float32_t capacity = 46800.0f;
float32_t battery_voltage;
float32_t model_voltage;
float32_t error;
float32_t R = 10.0f;


// --------------------------

float32_t theta(float32_t t, float32_t tau) {
    if(t > (15 * tau)) {
        return 0;
    }
    return powf(t, tau);
}

float32_t coulombic_efficiency() {
    if(current > 0) {
        return 0.9;
    }
    else {
        return 1;
    }
}

// getting parameters based on SOC -------

float32_t r0() {
    if (current > 0) 
    {
        return 0.0026f; 
    } 
    else {
        return 0.0030f;
    } 
}

float32_t r1() {
    return 0.0015f;
}

float32_t r2() {
    return 0.00125f;
}

float32_t tau1() {
    return 0.027f;
}

float32_t tau2() {
    return 6.0f;
}

// --------------------------

// 2RC Kalman filter --------

float32_t posterior_state_data[3][1] = {{1.0f}, {0.0f}, {0.0f}};
float32_t prior_state_data[3][1] = {{1.0f}, {0.0f}, {0.0f}};

float32_t posterior_covar_data[3][3] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
float32_t prior_covar_data[3][3] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};

float32_t kalman_gain_data[3][1] = {{0.0f}, {0.0f}, {0.0f}};

arm_matrix_instance_f32 posterior_state = {3, 1, (float32_t*)(&posterior_state_data)};
arm_matrix_instance_f32 prior_state = {3, 1, (float32_t*)(&prior_state_data)};
arm_matrix_instance_f32 posterior_covar = {3, 3, (float32_t*)(&posterior_covar_data)};
arm_matrix_instance_f32 prior_covar = {3, 3, (float32_t*)(&prior_covar_data)};
arm_matrix_instance_f32 kalman_gain = {3, 1, (float32_t*)(&kalman_gain_data)};

void initState(float32_t *prevState, float32_t *newState) {
    *prevState = *newState;
}

float32_t getSOC() {
    return posterior_state_data[0][0];
}

float32_t theta_tau1() {
    return theta(timestep, tau1());
}

float32_t theta_tau2() {
    return theta(timestep, tau2());
}


float32_t A_data[3][3] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
float32_t B_data[3][1] = {{0.0f}, {0.0f}, {0.0f}};
float32_t C_data[1][3] = {{0.0f, 0.0f, 0.0f}};
float32_t D_data[1][1] = {{0.0f}};
float32_t Q_data[3][3] = {{0.1f, 0.0f, 0.0f}, {0.0f, 0.1f, 0.0f}, {0.0f, 0.0f, 0.1f}};

arm_matrix_instance_f32 A = {3, 3, (float32_t*)&A_data};
arm_matrix_instance_f32 B = {3, 1, (float32_t*)&B_data};
arm_matrix_instance_f32 C = {3, 1, (float32_t*)&C_data};
arm_matrix_instance_f32 D = {1, 1, (float32_t*)&D_data};
arm_matrix_instance_f32 Q = {3, 3, (float32_t*)&Q_data};

void updateA() {

    A_data[0][0] = 1;
    A_data[1][1] = theta_tau1();
    A_data[2][2] = theta_tau2();
}

void updateB() {

    B_data[0][0] = (timestep * (-1.0f)) / (capacity * coulombic_efficiency());
    B_data[1][0] = r1() * (1 - theta_tau1());
    B_data[2][0] = r2() * (1 - theta_tau2());
}

void updateC() {
    
    C_data[0][0] = ocv_soc();
    C_data[0][1] = -1;
    C_data[0][2] = -1;
}

void updateD() {

    D_data[0][0] = r0();
}

void error_covar_prior_estimation(){  

    float32_t A_trans_data[3][3];
    arm_matrix_instance_f32 A_trans = {3, 3, (float32_t*)&A_trans_data};
    arm_mat_trans_f32(&A, &A_trans);

    float32_t tmp1_data[3][3];
    float32_t tmp2_data[3][3];

    arm_matrix_instance_f32 tmp1 = {3, 3, (float32_t*)&tmp1_data};
    arm_matrix_instance_f32 tmp2 = {3, 3, (float32_t*)&tmp2_data};

    arm_mat_mult_f32(&A, &posterior_covar, &tmp1);
    arm_mat_mult_f32(&A_trans, &Q, &tmp2);
    arm_mat_mult_f32(&tmp1, &tmp2, &prior_covar);
}

void state_prior_estimate(){

    float32_t B_mult_data[3][1];
    arm_matrix_instance_f32 B_mult = {3, 1, (float32_t*)&B_mult_data};

    float32_t tmp1_data[3][3];
    arm_matrix_instance_f32 tmp1 = {3, 3, (float32_t*)&tmp1_data};

    arm_mat_mult_f32(&A, &posterior_state, &tmp1);
    arm_mat_scale_f32(&B, current, &B_mult);
    arm_mat_add_f32(&tmp1, &B_mult, &prior_state);
}
    
void kalman_gain_calculation(){

    float32_t C_trans_data[3][1];
    arm_matrix_instance_f32 C_trans = {3, 1, (float32_t*)&C_trans_data};
    arm_mat_trans_f32(&C, &C_trans);

    float32_t tmp1_data[3][1];
    float32_t tmp2_data[1][3];
    float32_t tmp3_data[1][1];

    arm_matrix_instance_f32 tmp1 = {3, 1, (float32_t*)&tmp1_data};
    arm_matrix_instance_f32 tmp2 = {1, 3, (float32_t*)&tmp2_data};
    arm_matrix_instance_f32 tmp3 = {1, 1, (float32_t*)&tmp3_data};

    arm_mat_mult_f32(&prior_covar, &C_trans, &tmp1);
    arm_mat_mult_f32(&C, &prior_covar, &tmp2);
    arm_mat_mult_f32(&tmp2, &C_trans, &tmp3);
    tmp3_data[0][0] += R;

    float32_t C_inv_data[1][1];
    arm_matrix_instance_f32 C_inv = {1, 1, (float32_t*)&C_inv_data};
    arm_mat_inverse_f32(&tmp3, &C_inv);

    arm_mat_mult_f32(&tmp1, &C_inv, &kalman_gain);
}

void battery_model(){

    float32_t D_mult_data[1][1];
    arm_matrix_instance_f32 D_mult = {1, 1, (float32_t*)&D_mult_data};
    arm_mat_scale_f32(&D, current, &D_mult);

    float32_t tmp1_data[1][1];
    float32_t tmp2_data[1][1];

    arm_matrix_instance_f32 tmp1 = {1, 1, (float32_t*)&tmp1_data};
    arm_matrix_instance_f32 tmp2 = {1, 1, (float32_t*)&tmp2_data};

    arm_mat_mult_f32(&C, &prior_state, &tmp1);
    arm_mat_mult_f32(&tmp1, &D_mult, &tmp2);
    model_voltage = tmp2_data[0][0];
}
    
void state_update(){

    error = voltage - model_voltage;

    float32_t tmp1_data[1][1];
    arm_matrix_instance_f32 tmp1 = {1, 1, (float32_t*)&tmp1_data};

    arm_mat_scale_f32(&kalman_gain, error, &tmp1);
    arm_mat_add_f32(&prior_state, &tmp1, &posterior_state);
}

void error_covar_update(){

    float32_t id[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    arm_matrix_instance_f32 identity_3 = {3, 3, (float32_t*)&id};

    float32_t tmp1_data[3][3];
    float32_t tmp2_data[3][3];
    arm_matrix_instance_f32 tmp1 = {3, 3, (float32_t*)&tmp1_data};    
    arm_matrix_instance_f32 tmp2 = {3, 3, (float32_t*)&tmp2_data}; 

    arm_mat_mult_f32(&kalman_gain, &C, &tmp1);
    arm_mat_sub_f32(&identity_3, &tmp1, &tmp2);
    arm_mat_mult_f32(&tmp2, &prior_covar, &posterior_covar);
}
    

// --------------------------

// running kalman filter ----

TickType_t prevTime;

float32_t run_iter(){

        updateA();
        updateB();
        updateC();
        updateD();

        error_covar_prior_estimation();
        state_prior_estimate();

        battery_model();
        kalman_gain_calculation();

        state_update();
        error_covar_update();

        return getSOC();
}   

// --------------------------

void calcSOC(void *pvParameters){
    (void) pvParameters;

    volatile cmr_canHVIHeartbeat_t *heartbeat_HVI = canVehicleGetPayload(CANRX_HVI_SENSE);

    while(1) {
        voltage = (heartbeat_HVI->packVoltage_cV) / 100.0f;
        current = (heartbeat_HVI->packCurrent_dA) / 10.0f;
        float32_t curTime = (float32_t)xTaskGetTickCount();

        timestep = (curTime - prevTime) / 1000.0f;
        prevTime = curTime;

        soc = run_iter();
    }
}

void socInit(){
    cmr_taskInit(
        &soc_task,
        "SOC",
        soc_priority,
        calcSOC,
        NULL
    );
}

