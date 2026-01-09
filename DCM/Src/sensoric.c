#include "movella.h"
#include <CMR/can_types.h> 
#include <math.h>

#define TRANSFORM_RAW(message_index, raw_data) (scaling_constants[message_index] * (float) raw_data)
#define GYRO_FREQ_HZ (200.0f)

volatile movella_state_t movella_state = {
    .gyro = {
        .inited = false,
    },
};
volatile car_state_t car_state;

static inline float transform_raw(movella_message_t msg, float raw_data) {
    return scaling_constants[msg] * raw_data;
}

static void transform_velocity_D4B(volatile movella_state_t *movella_state, volatile car_state_t *car_state) {
    vector3 x_D_dot = { movella_state->global_velocity.x, movella_state->global_velocity.y, movella_state->global_velocity.z };
    matrix3x3 R_D;
    mat_copy(movella_state->R, R_D);
    matrix3x3 R_D_T;
    mat_transpose(R_D, R_D_T);
    vector3 x_D_D_dot;
    mat_vec_mult(R_D_T, x_D_dot, x_D_D_dot);

    movella_state->velocity.x = x_D_D_dot[0];
    movella_state->velocity.y = x_D_D_dot[1];
    movella_state->velocity.z = x_D_D_dot[2];

    float gyro_x = movella_state->gyro.x;
    float gyro_y = movella_state->gyro.y;
    float gyro_z = movella_state->gyro.z;
    matrix3x3 Omega_D = {
        {0.0f, -gyro_z, gyro_y},
        {gyro_z, 0.0, -gyro_x},
        {-gyro_y, gyro_x, 0.0},
    };
    
    matrix3x3 R_D_dot;
    mat_mult(R_D, Omega_D, R_D_dot);

    matrix3x3 R_DB;
    mat_transpose(R_BD, R_DB);
    
    matrix3x3 temp1;
    mat_mult(R_D_dot, R_DB, temp1);

    vector3 temp2;
    mat_vec_mult(temp1, x_BD, temp2);

    vector3 x_B_dot;
    vec_sub(x_D_dot, temp2, x_B_dot);
    
    matrix3x3 R_B;
    mat_mult(R_D, R_DB, R_B);
    
    matrix3x3 R_B_T;
    mat_transpose(R_B, R_B_T);
    
    vector3 x_B_B_dot;
    mat_vec_mult(R_B_T, x_B_dot, x_B_B_dot);

    mat_copy(R_B, car_state->R);
    car_state->velocity.x = x_B_B_dot[0];
    car_state->velocity.y = x_B_B_dot[1];
    car_state->velocity.z = x_B_B_dot[2];

    matrix3x3 Omega_B;
    mat_mult(R_BD, Omega_D, temp1);
    mat_mult(temp1, R_DB, Omega_B);
    car_state->gyro.x = Omega_B[2][1];
    car_state->gyro.y = Omega_B[0][2];
    car_state->gyro.z = Omega_B[1][0];
}

static void compute_slip(volatile movella_state_t *movella_state, volatile car_state_t *car_state) {
    car_state->slip_angle.body = atan2f(car_state->velocity.y, car_state->velocity.x);
    
    vector3 x_B_B_dot = { 
        car_state->velocity.x, 
        car_state->velocity.y, 
        car_state->velocity.z 
    };
    matrix3x3 Omega_B = {
        {0.0f, -car_state->gyro.z, car_state->gyro.y},
        {car_state->gyro.z, 0.0f, -car_state->gyro.x},
        {-car_state->gyro.y, car_state->gyro.x, 0.0f},
    };

    vector3 v_FL, v_FR, v_RL, v_RR;
    mat_vec_mult(Omega_B, x_FL, v_FL);
    mat_vec_mult(Omega_B, x_FR, v_FR);
    mat_vec_mult(Omega_B, x_RL, v_RL);
    mat_vec_mult(Omega_B, x_RR, v_RR);
    vec_add(x_B_B_dot, v_FL, v_FL);
    vec_add(x_B_B_dot, v_FR, v_FR);
    vec_add(x_B_B_dot, v_RL, v_RL);
    vec_add(x_B_B_dot, v_RR, v_RR);

    car_state->fl_velocity.x = v_FL[0];
    car_state->fl_velocity.y = v_FL[1];
    car_state->fr_velocity.x = v_FR[0];
    car_state->fr_velocity.y = v_FR[1];
    car_state->rl_velocity.x = v_RL[0];
    car_state->rl_velocity.y = v_RL[1];
    car_state->rr_velocity.x = v_RR[0];
    car_state->rr_velocity.y = v_RR[1];

    car_state->slip_angle.fl = atan2f(v_FL[1], v_FL[0]);
    car_state->slip_angle.fr = atan2f(v_FR[1], v_FR[0]);
    car_state->slip_angle.rl = atan2f(v_RL[1], v_RL[0]);
    car_state->slip_angle.rr = atan2f(v_RR[1], v_RR[0]);
}

static void compute_slip_device(volatile movella_state_t *movella_state, volatile car_state_t *car_state) {
    car_state->slip_angle.body = atan2f(car_state->velocity.y, car_state->velocity.x);
    
    vector3 x_D_D_dot = { 
        movella_state->velocity.x, 
        movella_state->velocity.y, 
        movella_state->velocity.z 
    };
    matrix3x3 Omega_D = {
        {0.0f, -movella_state->gyro.z, movella_state->gyro.y},
        {movella_state->gyro.z, 0.0f, -movella_state->gyro.x},
        {-movella_state->gyro.y, movella_state->gyro.x, 0.0f},
    };

    vector3 v_FL, v_FR, v_RL, v_RR;
    mat_vec_mult(Omega_D, x_D_FL, v_FL);
    mat_vec_mult(Omega_D, x_D_FR, v_FR);
    mat_vec_mult(Omega_D, x_D_RL, v_RL);
    mat_vec_mult(Omega_D, x_D_RR, v_RR);
    vec_add(x_D_D_dot, v_FL, v_FL);
    vec_add(x_D_D_dot, v_FR, v_FR);
    vec_add(x_D_D_dot, v_RL, v_RL);
    vec_add(x_D_D_dot, v_RR, v_RR);

    car_state->fl_velocity.x = v_FL[0];
    car_state->fl_velocity.y = v_FL[1];
    car_state->fr_velocity.x = v_FR[0];
    car_state->fr_velocity.y = v_FR[1];
    car_state->rl_velocity.x = v_RL[0];
    car_state->rl_velocity.y = v_RL[1];
    car_state->rr_velocity.x = v_RR[0];
    car_state->rr_velocity.y = v_RR[1];

    car_state->slip_angle.fl = atan2f(v_FL[1], v_FL[0]);
    car_state->slip_angle.fr = atan2f(v_FR[1], v_FR[0]);
    car_state->slip_angle.rl = atan2f(v_RL[1], v_RL[0]);
    car_state->slip_angle.rr = atan2f(v_RR[1], v_RR[0]);
}

void sensoric_parse(uint16_t canID, volatile void *payload) {
    
    volatile void* payload;

    canDaqRX_t sensoric_msg;
    bool msg_found = false;
    for(sensoric_msg = CANRX_DAQ_SENSORIC_VEL_ANG_POI; sensoric_msg <= CANRX_DAQ_SENSORIC_INFO; sensoric_msg++) {
        if(canID == canDaqRXMeta[sensoric_msg].canID) {
            msg_found = true;
            break;
        }
    }

    if(!msg_found) 
        return;

    switch (sensoric_msg)
    {
        // TODO: parse int stuff? what does parse_int do..

    case CANRX_DAQ_SENSORIC_VEL_ANG_POI:
        volatile cmr_canSensoricVelAngPoi_t *vel_ang_poi = payload;
        volatile int16_t vel_X_poi = &vel_ang_poi -> vel_X_poi;
        volatile int16_t vel_Y_poi = &vel_ang_poi -> vel_Y_poi;
        volatile int16_t vel_Z_poi = &vel_ang_poi -> vel_Z_poi;
        volatile int16_t ang_S_poi = &vel_ang_poi -> ang_S_poi;
        break;

    case CANRX_DAQ_SENSORIC_DIST_POI:
        volatile cmr_canSensoricDistPoi_t *dist_poi = payload;
        volatile int32_t dist_A_poi = &dist_poi -> dist_A_poi;
        volatile int16_t radius_poi = &dist_poi -> radius_poi;
        volatile int16_t acc_C_poi = &dist_poi -> acc_C_poi;
        break;

    case CANRX_DAQ_SENSORIC_PITCH_ROLL:
        volatile cmr_canSensoricPitchRoll_t *pitch_roll = payload;
        volatile int16_t pitch = &pitch_roll -> pitch;
        volatile int16_t roll = &pitch_roll -> roll;
        break;
    
    case CANRX_DAQ_SENSORIC_ACC_HOR:
        volatile cmr_canSensoricAccHor_t *acc_hor = payload;
        volatile int16_t acc_X_hor = &acc_hor -> acc_X_hor;
        volatile int16_t acc_Y_hor = &acc_hor -> acc_Y_hor;
        volatile int16_t acc_Z_hor = &acc_hor -> acc_Z_hor;
        break;

    case CANRX_DAQ_SENSORIC_RATE_HOR:
        volatile cmr_canSensoricRateHor_t *rate_hor = payload;
        volatile int16_t rate_X_hor = &rate_hor -> rate_X_hor;
        volatile int16_t rate_Y_hor = &rate_hor -> rate_Y_hor;
        volatile int16_t rate_Z_hor = &rate_hor -> rate_Z_hor;
        break;
    
    case CANRX_DAQ_SENSORIC_VEL_ANG:
        volatile cmr_canSensoricVelAng_t *vel_ang = payload;
        volatile int16_t vel_X = &vel_ang -> vel_X;
        volatile int16_t vel_Y = &vel_ang -> vel_Y; 
        volatile int16_t vel_A = &vel_ang -> vel_A;
        volatile int16_t ang_S = &vel_ang -> ang_S;
        break;
    
    case CANRX_DAQ_SENSORIC_DIST:
        volatile cmr_canSensoricDist_t *dist = payload;
        volatile int32_t dist_A = &dist -> dist_A;
        volatile int16_t radius = &dist -> radius;
        volatile int16_t acc_C = &dist -> acc_C;
        break;

    case CANRX_DAQ_SENSORIC_ACC:
        volatile cmr_canSensoricAcc_t *acc = payload;
        volatile int16_t acc_X = &acc -> acc_X;
        volatile int16_t acc_Y = &acc -> acc_Y;
        volatile int16_t acc_Z = &acc -> acc_Z;
        break;

    case CANRX_DAQ_SENSORIC_RATE:
        volatile cmr_canSensoricRate_t *rate = payload;
        volatile int16_t rate_X = &rate -> rate_X;
        volatile int16_t rate_Y = &rate -> rate_Y;
        volatile int16_t rate_Z = &rate -> rate_Z;
        break;

    case CANRX_DAQ_SENSORIC_VEL_ANG_SP:
        volatile cmr_canSensoricVelAngSp_t *vel_ang_sp = payload;
        volatile int16_t vel_A_sp = &vel_ang_sp -> vel_A_sp;
        volatile int16_t vel_S_sp = &vel_ang_sp -> vel_S_sp;
        volatile uint16_t quality_ch0 = &vel_ang_sp -> quality_ch0;
        volatile uint16_t quality_ch1 = &vel_ang_sp -> quality_ch1;
        break;

    case CANRX_DAQ_SENSORIC_DIST_VEL_SP:
        volatile cmr_canSensoricDistVelSp_t *dist_vel_sp = payload;
        volatile int32_t dist_A_sp = &dist_vel_sp -> dist_A_sp;
        volatile int16_t vel_X_sp = &dist_vel_sp -> vel_X_sp;
        volatile int16_t vel_Y_sp = &dist_vel_sp -> vel_Y_sp;
        break;
    
    default:
        break;
    }
}

void movella_test() {
    
    movella_state.global_velocity.x = 1.0;
    movella_state.global_velocity.y = 1.0;
    movella_state.global_velocity.z = 1.0;

    movella_state.gyro.x = 0.0;
    movella_state.gyro.y = 0.0;
    movella_state.gyro.z = 1.0;

    movella_state.R[0][0] = 1.0;
    movella_state.R[0][1] = 0.0;
    movella_state.R[0][2] = 0.0;
    movella_state.R[1][0] = 0.0;
    movella_state.R[1][1] = 1.0;
    movella_state.R[1][2] = 0.0;
    movella_state.R[2][0] = 0.0;
    movella_state.R[2][1] = 0.0;
    movella_state.R[2][2] = 1.0;

    transform_velocity_D4B(&movella_state, &car_state);
    compute_slip(&movella_state, &car_state);
    
    car_state_t temp;
    compute_slip_device(&movella_state, &temp);
}

void movella_random_test() {
    
    movella_state.global_velocity.x = 0.8;
    movella_state.global_velocity.y = 1.2;
    movella_state.global_velocity.z = -1.9;

    movella_state.gyro.x = 0.3;
    movella_state.gyro.y = -0.8;
    movella_state.gyro.z = 1.2;

    quaternion_t q = {
        .w = 0.9829670809135851,
        .x = 0.15280150797033248, 
        .y = 0.09174319450720117, 
        .z = 0.04483975093409589,
    };
    
    quaternion_to_R(&q, movella_state.R);

    car_state_t method_1;
    transform_velocity_D4B(&movella_state, &method_1);
    compute_slip(&movella_state, &method_1);
    
    car_state_t method_2;
    compute_slip_device(&movella_state, &method_2);
    compute_slip_device(&movella_state, &method_2);

}
