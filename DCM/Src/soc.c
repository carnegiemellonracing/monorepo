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
/** @brief Brake light control task. */
static cmr_task_t soc_task;



float32_t theta(float32_t t, float32_t tau) {
    if(t > (15 * tau)) {
        return 0;
    }
    return powf(t, tau);
}


// variables ----------------

float32_t timestep;
float32_t soc;
float32_t current;
float32_t voltage;
float32_t capacity;


// --------------------------

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

float32_t posterior_state[3] = {1.0f, 0.0f, 0.0f};
float32_t prior_state[3];

float32_t posterior_covar[3][3] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};

void initState(float32_t *prevState, float32_t *newState) {
    *prevState = *newState;
}

float32_t getSOC() {
    return posterior_state[0];
}

float32_t theta_tau1() {
    return theta(timestep, tau1());
}

float32_t theta_tau2() {
    return theta(timestep, tau2());
}


float32_t A_data[3][3] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
float32_t B_data[3] = {0.0f, 0.0f, 0.0f};
float32_t C_data[3] = {0.0f, 0.0f, 0.0f};
float32_t D_data[1] = {{0.0f}};

arm_matrix_instance_f32 A = {3, 3, &A_data};
arm_matrix_instance_f32 B = {3, 1, &B_data};
arm_matrix_instance_f32 C = {3, 1, &C_data};
arm_matrix_instance_f32 D = {1, 1, &D_data};

void updateA() {

    A_data[0][0] = 1;
    A_data[1][1] = theta_tau1();
    A_data[2][2] = theta_tau2();
    return A;
}

void updateB() {

    B_data[0] = (timestep * (-1.0f)) / (capacity() * coulombic_efficiency());
    B_data[1] = r1() * (1 - theta_tau1());
    B_data[2] = r2() * (1 - theta_tau2());
    return B;
}

void updateC() {
    
    C_data[0] = ocv_soc();
    C_data[1] = -1;
    C_data[2] = -1;
}

void updateD() {

    D_data[0] = r0();
}

void error_covar_prior_estimation(){  

    arm_matrix_instance_f32 A_trans;
    arm_mat_trans_f32(&A, &A_trans);
    arm_mat_mult_f32(&A, &posterior_covar, &prior_covar);
    arm_mat_mult_f32(&prior_covar, &A_trans, &prior_covar);
    arm_mat_mult_f32(&prior_covar, &Q, &prior_covar);
}

void state_prior_estimate(self){

    arm_matrix_instance_f32 B_mult;
    arm_mat_mult_f32(&A, &posterior_state, &prior_state);
    arm_mat_scale_f32(&B, current, &B_mult);
    arm_mat_add_f32(&prior_state, &B_mult, &prior_state);
}
    
void kalman_gain_calculation(self){

    arm_matrix_instance_f32 C_inv;
    arm_matrix_instance_f32 C_trans;
    arm_mat_trans_f32(&C, &C_trans);

    arm_mat_mult_f32(&prior_covar, &C, &kalman_gain);
    arm_mat_mult_f32(&kalman_gain, &C_trans, &kalman_gain);

    arm_mat_mult_f32(&C, &prior_covar, &C_inv);
    arm_mat_mult_f32(&C_inv, &C_trans, &C_inv);
    arm_mat_mult_f32(&C_inv, &R, &C_inv);
    arm_mat_inverse_f32(&C_inv, &C_inv);

    arm_mat_mult_f32(&kalman_gain, &C_inv, &kalman_gain);
}

void battery_model(self){

    arm_matrix_instance_f32 D_mult;
    arm_mat_scale_f32(&D, current, &D_mult);

    arm_mat_mult_f32(&C, &prior_state, &model_voltage);
    arm_mat_add_f32(&model_voltage, &D_mult, &model_voltage);
}
    
void state_update(self){

    arm_mat_sub_f32(&battery_voltage, &model_voltage, &error);
    
    arm_matrix_instance_f32 temp;
    arm_mat_mult_f32(&error, &kalman_gain, &temp);
    arm_mat_add_f32(&prior_state, &temp, &posterior_state);
}

void error_covar_update(){

    float32_t id[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    arm_matrix_instance_f32 identity_3 = {3, 3, &id};
    arm_matrix_instance_f32 temp;
    arm_mat_mult_f32(&kalman_gain, &C, &temp);
    arm_mat_sub_f32(&identity_3, &temp, &temp);
    arm_mat_mult_f32(&temp, &prior_covar, &posterior_covar);
}
    

// --------------------------

// running kalman filter ----

TickType_t prevTime;

float_32_t run_iter(self, battery_voltage, current){

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

        return soc;
}   

// --------------------------

void calcSOC(void *pvParameters){
    (void) pvParameters;

    volatile cmr_canHVIHeartbeat_t *heartbeat_HVI = canVehicleGetPayload(CANRX_HVI_SENSE);

    while(1) {
        voltage = heartbeat_HVI->packVoltage_cV / 100.0f;
        current = heartbeat_HVI->packCurrent_dA / 10.0f;
        float32_t curTime = (float32_t)xTaskGetTickCount();

        timestep = (curTime - prevTime) / 1000.0f;
        prevTime = curTime;

        run_iter(voltage, current);
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

