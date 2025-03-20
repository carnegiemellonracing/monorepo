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

typedef union {
    struct {
        uint8_t lsb;
        uint8_t msb;
    } data;
    int16_t parsed;
} int16_parser;

static int16_t parse_int16(volatile big_endian_16_t *big) {
    static int16_parser parser;
    parser.data.msb = big->msb;
    parser.data.lsb = big->lsb;
    return parser.parsed;
} 

static void quaternion_to_R(volatile quaternion_t *quaternion, volatile float R[3][3]) {
    float w = quaternion->w, x = quaternion->x, y = quaternion->y, z = quaternion->z;

    R[0][0] = 1 - 2 * y * y - 2 * z * z;
    R[0][1] = 2 * x * y - 2 * w * z;
    R[0][2] = 2 * x * z + 2 * w * y;

    R[1][0] = 2 * x * y + 2 * w * z;
    R[1][1] = 1 - 2 * x * x - 2 * z * z;
    R[1][2] = 2 * y * z - 2 * w * x;

    R[2][0] = 2 * x * z - 2 * w * y;
    R[2][1] = 2 * y * z + 2 * w * x;
    R[2][2] = 1 - 2 * x * x - 2 * y * y;
}

void mat_mult(const matrix3x3 A, const matrix3x3 B, matrix3x3 result) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void mat_vec_mult(const matrix3x3 A, const vector3 v, vector3 result) {
    for (int i = 0; i < 3; i++) {
        result[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            result[i] += A[i][j] * v[j];
        }
    }
}

void vec_add(const vector3 v1, const vector3 v2, vector3 result) {
    for (int i = 0; i < 3; i++) {
        result[i] = v1[i] + v2[i];
    }
}

void vec_sub(const vector3 v1, const vector3 v2, vector3 result) {
    for (int i = 0; i < 3; i++) {
        result[i] = v1[i] - v2[i];
    }
}

void mat_transpose(const matrix3x3 A, matrix3x3 result) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[j][i] = A[i][j];
        }
    }
}

void mat_copy(matrix3x3 A, matrix3x3 result) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = A[i][j];
        }
    }
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

void movella_parse(uint16_t canID, volatile void *payload) {
    
    // volatile void* payload;

    canDaqRX_t movella_msg;
    bool msg_found = false;
    for(movella_msg = CANRX_DAQ_MOVELLA_STATUS; movella_msg <= CANRX_DAQ_MOVELLA_VELOCITY; movella_msg++) {
        if(canID == canDaqRXMeta[movella_msg].canID) {
            msg_found = true;
            // payload = canDaqRXMeta[movella_msg].payload;
            break;
        }
    }

    if(!msg_found) 
        return;

    switch (movella_msg)
    {
    case CANRX_DAQ_MOVELLA_STATUS:
        
        break;

    case CANRX_DAQ_MOVELLA_QUATERNION:
        volatile cmr_canMovellaQuaternion_t *quaternion = payload;
        volatile int16_t w = parse_int16(&quaternion->q0);
        volatile int16_t x = parse_int16(&quaternion->q1);
        volatile int16_t y = parse_int16(&quaternion->q2);
        volatile int16_t z = parse_int16(&quaternion->q3);
        movella_state.quaternion.w = transform_raw(MOVELLA_QUATERNION, w);
        movella_state.quaternion.x = transform_raw(MOVELLA_QUATERNION, x);
        movella_state.quaternion.y = transform_raw(MOVELLA_QUATERNION, y);
        movella_state.quaternion.z = transform_raw(MOVELLA_QUATERNION, z);
        quaternion_to_R(&movella_state.quaternion, movella_state.R);
        break;

    case CANRX_DAQ_MOVELLA_IMU_EULER_ANGLES:
        volatile cmr_canMovellaEulerAngles_t *euler = payload;
        int16_t yaw = parse_int16(&euler->yaw);
        int16_t pitch = parse_int16(&euler->pitch);
        int16_t roll = parse_int16(&euler->roll);
        movella_state.euler_angles.yaw = transform_raw(MOVELLA_EULER_ANGLES, yaw);
        movella_state.euler_angles.pitch = transform_raw(MOVELLA_EULER_ANGLES, pitch);
        movella_state.euler_angles.roll = transform_raw(MOVELLA_EULER_ANGLES, roll);
        break;
    
    case CANRX_DAQ_MOVELLA_IMU_GYRO:
        volatile cmr_canMovellaIMUGyro_t *gyro = payload;
        volatile int16_t gyro_x = parse_int16(&gyro->gyro_x);
        volatile int16_t gyro_y = parse_int16(&gyro->gyro_y);
        volatile int16_t gyro_z = parse_int16(&gyro->gyro_z);
        movella_state.gyro.x_prev = movella_state.gyro.x;
        movella_state.gyro.y_prev = movella_state.gyro.y;
        movella_state.gyro.z_prev = movella_state.gyro.z;
        movella_state.gyro.x = transform_raw(MOVELLA_IMU_GYRO, gyro_x);
        movella_state.gyro.y = transform_raw(MOVELLA_IMU_GYRO, gyro_y);
        movella_state.gyro.z = transform_raw(MOVELLA_IMU_GYRO, gyro_z);
        if(movella_state.gyro.inited) {
            movella_state.alpha.x = (movella_state.gyro.x - movella_state.gyro.x_prev) * GYRO_FREQ_HZ;
            movella_state.alpha.y = (movella_state.gyro.y - movella_state.gyro.y_prev) * GYRO_FREQ_HZ;
            movella_state.alpha.z = (movella_state.gyro.z - movella_state.gyro.z_prev) * GYRO_FREQ_HZ;
        } else {
            movella_state.gyro.inited = true;
        }
        break;

    case CANRX_DAQ_MOVELLA_IMU_ACCEL:
        volatile cmr_canMovellaIMUAccel_t *accel = payload;
        int16_t accel_x = parse_int16(&accel->accel_x);
        int16_t accel_y = parse_int16(&accel->accel_y);
        int16_t accel_z = parse_int16(&accel->accel_z);
        movella_state.accel.x = transform_raw(MOVELLA_IMU_ACCEL, accel_x);
        movella_state.accel.y = transform_raw(MOVELLA_IMU_ACCEL, accel_y);
        movella_state.accel.z = transform_raw(MOVELLA_IMU_ACCEL, accel_z);
        break;
    
    case CANRX_DAQ_MOVELLA_VELOCITY:
        volatile cmr_canMovellaVelocity_t *velocity = payload;
        int16_t vel_x = parse_int16(&velocity->vel_x);
        int16_t vel_y = parse_int16(&velocity->vel_y);
        int16_t vel_z = parse_int16(&velocity->vel_z);
        movella_state.global_velocity.x = transform_raw(MOVELLA_VELOCITY, vel_x);
        movella_state.global_velocity.y = transform_raw(MOVELLA_VELOCITY, vel_y);
        movella_state.global_velocity.z = transform_raw(MOVELLA_VELOCITY, vel_z);
        transform_velocity_D4B(&movella_state, &car_state);
        compute_slip(&movella_state, &car_state);
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
