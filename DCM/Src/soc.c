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


static const uint32_t soc_priority = 6;
/** @brief SOC task period. */
static const TickType_t soc_period_ms = 50;
/** @brief SOC control task. */
static cmr_task_t soc_task;


// variables ----------------

float32_t timestep;
float32_t soc;
float32_t current;
float32_t voltage;
float32_t capacity = 46800.0f;
float32_t model_voltage;
float32_t error;
float32_t R = 10.0f;

// --------------------------

// lookup tables ------------

float32_t ocv_soc_lut[2][101] = {{3.219f, 3.2805f, 3.3412f, 3.4018f, 3.4316f, 3.4468f, 3.462f, 3.4718f, 3.4775f, 3.4832f, 3.49f, 3.4983f, 3.5066f, 3.5152f, 3.525f, 3.5348f, 3.5445f, 3.5519f, 3.5592f, 3.5666f, 3.5731f, 3.5796f, 3.5862f, 3.5918f, 3.5968f, 3.6018f, 3.6062f, 3.6092f, 3.6121f, 3.6148f, 3.6171f, 3.6212f, 3.6253f, 3.6293f, 3.6327f, 3.6354f, 3.638f, 3.6407f, 3.6436f, 3.6468f, 3.6501f, 3.6546f, 3.6596f, 3.6645f, 3.6693f, 3.6741f, 3.6791f, 3.684f, 3.6891f, 3.6945f, 3.7f, 3.7067f, 3.7158f, 3.7249f, 3.7339f, 3.7439f, 3.754f, 3.7641f, 3.774f, 3.7841f, 3.7943f, 3.8042f, 3.8139f, 3.8236f, 3.834f, 3.8442f, 3.8545f, 3.8648f, 3.8754f, 3.8859f, 3.8958f, 3.9051f, 3.9145f, 3.9245f, 3.9352f, 3.9457f, 3.956f, 3.966f, 3.9767f, 3.9875f, 3.9991f, 4.0106f, 4.0221f, 4.034f, 4.0459f, 4.058f, 4.0707f, 4.0833f, 4.0962f, 4.1084f, 4.1203f, 4.1325f, 4.1461f, 4.1599f, 4.1736f, 4.1873f, 4.2021f, 4.2182f, 4.2361f, 4.2544f, 4.2735f}, {0.0f, 0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f, 0.07f, 0.08f, 0.09f, 0.1f, 0.11f, 0.12f, 0.13f, 0.14f, 0.15f, 0.16f, 0.17f, 0.18f, 0.19f, 0.2f, 0.21f, 0.22f, 0.23f, 0.24f, 0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.3f, 0.31f, 0.32f, 0.33f, 0.34f, 0.35f, 0.36f, 0.37f, 0.38f, 0.39f, 0.4f, 0.41f, 0.42f, 0.43f, 0.44f, 0.45f, 0.46f, 0.47f, 0.48f, 0.49f, 0.5f, 0.51f, 0.52f, 0.53f, 0.54f, 0.55f, 0.56f, 0.57f, 0.58f, 0.59f, 0.6f, 0.61f, 0.62f, 0.63f, 0.64f, 0.65f, 0.66f, 0.67f, 0.68f, 0.69f, 0.7f, 0.71f, 0.72f, 0.73f, 0.74f, 0.75f, 0.76f, 0.77f, 0.78f, 0.79f, 0.8f, 0.81f, 0.82f, 0.83f, 0.84f, 0.85f, 0.86f, 0.87f, 0.88f, 0.89f, 0.9f, 0.91f, 0.92f, 0.93f, 0.94f, 0.95f, 0.96f, 0.97f, 0.98f, 0.99f, 1.0f}};
float32_t discharge_params[6][12] = {{0.977f, 0.879f, 0.787f, 0.690f, 0.597f, 0.491f, 0.402f, 0.307f, 0.208f, 0.124f, 0.045f, 0.003f}, {0.00277f, 0.00262f, 0.00288f, 0.00251f, 0.00253f, 0.00240f, 0.00268f, 0.00257f, 0.00247f, 0.00267f, 0.00254f, 0.00261f}, {0.00195f, 0.00188f, 0.00149f, 0.00177f, 0.00141f, 0.00132f, 0.00096f, 0.00100f, 0.00104f, 0.00082f, 0.00010f, 0.00200f}, {0.00119f, 0.00126f, 0.00133f, 0.00131f, 0.00152f, 0.00117f, 0.00097f, 0.00107f, 0.00105f, 0.00095f, 0.00010f, 0.00200f}, {0.0311f, 0.0266f, 0.0286f, 0.0265f, 0.0263f, 0.0257f, 0.0282f, 0.0274f, 0.0282f, 0.0497f, 1.0000f, 0.1000f}, {5.940f, 6.356f, 6.671f, 6.254f, 7.208f, 6.212f, 5.370f, 5.737f, 4.936f, 3.868f, 4.085f, 1.000f}};
float32_t charge_params[6][12] = {{0.980f, 0.882f, 0.790f, 0.692f, 0.597f, 0.495f, 0.406f, 0.314f, 0.215f, 0.131f, 0.052f}, {0.00300f, 0.00314f, 0.00325f, 0.00318f, 0.00317f, 0.00314f, 0.00310f, 0.00303f, 0.00303f, 0.00307f, 0.00314f}, {0.00210f, 0.00174f, 0.00152f, 0.00161f, 0.00134f, 0.00122f, 0.00114f, 0.00122f, 0.00139f, 0.00148f, 0.00293f}, {0.00129f, 0.00132f, 0.00143f, 0.00163f, 0.00166f, 0.00123f, 0.00107f, 0.00109f, 0.00133f, 0.00202f, 0.00676f}, {0.0293f, 0.0267f, 0.0280f, 0.0269f, 0.0280f, 0.0287f, 0.0260f, 0.0274f, 0.0286f, 0.0375f, 0.0689f}, {7.592f, 7.272f, 7.284f, 8.239f, 8.321f, 7.615f, 6.358f, 6.815f, 8.217f, 9.194f, 20.000f}};

// --------------------------

// lookup -------------------

float32_t interp(float32_t x, float32_t *xp, float32_t *fp, size_t n)
{
    if (x <= xp[0])
        return fp[0];
    if (x >= xp[n - 1])
        return fp[n - 1];

    for (size_t i = 1; i < n; i++)
    {
        if (x < xp[i])
        {
            float32_t x0 = xp[i - 1], x1 = xp[i];
            float32_t y0 = fp[i - 1], y1 = fp[i];
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
        }
    }
    return fp[n - 1];
}

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
        return interp(soc, charge_params[0], charge_params[1], 11); 
    } 
    return interp(soc, discharge_params[0], discharge_params[1], 12);
    
}

float32_t r1() {
    if (current > 0) 
    {
        return interp(soc, charge_params[0], charge_params[2], 11); 
    } 
    return interp(soc, discharge_params[0], discharge_params[2], 12);
}

float32_t r2() {
    if (current > 0) 
    {
        return interp(soc, charge_params[0], charge_params[3], 11); 
    }
    return interp(soc, discharge_params[0], discharge_params[3], 12);
}

float32_t tau1() {
    if (current > 0) 
    {
        return interp(soc, charge_params[0], charge_params[4], 11); 
    }
    return interp(soc, discharge_params[0], discharge_params[4], 12);
}

float32_t tau2() {
    if (current > 0) 
    {
        return interp(soc, charge_params[0], charge_params[5], 11); 
    }
    return interp(soc, discharge_params[0], discharge_params[5], 12);
}

float32_t ocv_soc() {
    return interp(voltage, ocv_soc_lut[0], ocv_soc_lut[1], 101);
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
arm_matrix_instance_f32 C = {1, 3, (float32_t*)&C_data};
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
    TickType_t lastWakeTime = xTaskGetTickCount();

    while(1) {
        voltage = (heartbeat_HVI->packVoltage_cV) / 100.0f;
        current = (heartbeat_HVI->packCurrent_dA) / 10.0f;
        float32_t curTime = (float32_t)xTaskGetTickCount();

        timestep = (curTime - prevTime) / 1000.0f;
        prevTime = curTime;

        soc = run_iter();
        vTaskDelayUntil(&lastWakeTime, soc_period_ms);
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

