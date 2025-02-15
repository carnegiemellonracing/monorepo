#include "movella.h"
#include <CMR/can_types.h> 

#define TRANSFORM_RAW(message_index, raw_data) (scaling_constants[message_index] * (float) raw_data)

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
    parser.data.msb = big->lsb;
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

void movella_parse(canDaqRX_t movella_msg, volatile void* payload) {
    switch (movella_msg)
    {
    case CANRX_DAQ_MOVELLA_STATUS:
        
        break;

    case CANRX_DAQ_MOVELLA_QUATERNION:
        volatile cmr_canMovellaQuaternion_t *quaternion = payload;
        int16_t w = parse_int16(&quaternion->q0);
        int16_t x = parse_int16(&quaternion->q1);
        int16_t y = parse_int16(&quaternion->q2);
        int16_t z = parse_int16(&quaternion->q3);
        movella_state.quaternion.w = TRANSFORM_RAW(MOVELLA_QUATERNION, w);
        movella_state.quaternion.x = TRANSFORM_RAW(MOVELLA_QUATERNION, x);
        movella_state.quaternion.y = TRANSFORM_RAW(MOVELLA_QUATERNION, y);
        movella_state.quaternion.z = TRANSFORM_RAW(MOVELLA_QUATERNION, z);
        quaternion_to_R(&movella_state.quaternion, movella_state.rotation_matrix);
        break;

    case CANRX_DAQ_MOVELLA_IMU_EULER_ANGLES:
        volatile cmr_canMovellaEulerAngles_t *euler = payload;
        int16_t yaw = parse_int16(&euler->yaw);
        int16_t pitch = parse_int16(&euler->pitch);
        int16_t roll = parse_int16(&euler->roll);
        movella_state.euler_angles.yaw = TRANSFORM_RAW(MOVELLA_EULER_ANGLES, yaw);
        movella_state.euler_angles.pitch = TRANSFORM_RAW(MOVELLA_EULER_ANGLES, pitch);
        movella_state.euler_angles.roll = TRANSFORM_RAW(MOVELLA_EULER_ANGLES, roll);
        break;
    
    case CANRX_DAQ_MOVELLA_IMU_GYRO:
        volatile cmr_canMovellaIMUGyro_t *gyro = payload;
        int16_t gyro_x = parse_int16(&gyro->gyro_x);
        int16_t gyro_y = parse_int16(&gyro->gyro_y);
        int16_t gyro_z = parse_int16(&gyro->gyro_z);
        movella_state.gyro.x = TRANSFORM_RAW(gyro_x, MOVELLA_IMU_GYRO);
        movella_state.gyro.y = TRANSFORM_RAW(gyro_y, MOVELLA_IMU_GYRO);
        movella_state.gyro.z = TRANSFORM_RAW(gyro_z, MOVELLA_IMU_GYRO);
        break;

    case CANRX_DAQ_MOVELLA_IMU_ACCEL:
        volatile cmr_canMovellaIMUAccel_t *accel = payload;
        int16_t accel_x = parse_int16(&accel->accel_x);
        int16_t accel_y = parse_int16(&accel->accel_y);
        int16_t accel_z = parse_int16(&accel->accel_z);
        movella_state.accel.x = TRANSFORM_RAW(accel_x, MOVELLA_IMU_ACCEL);
        movella_state.accel.y = TRANSFORM_RAW(accel_y, MOVELLA_IMU_ACCEL);
        movella_state.accel.z = TRANSFORM_RAW(accel_z, MOVELLA_IMU_ACCEL);
        break;
    
    case CANRX_DAQ_MOVELLA_VELOCITY:
        volatile cmr_canMovellaVelocity_t *velocity = payload;
        int16_t vel_x = parse_int16(&velocity->vel_x);
        int16_t vel_y = parse_int16(&velocity->vel_y);
        int16_t vel_z = parse_int16(&velocity->vel_z);
        movella_state.velocity.x = TRANSFORM_RAW(vel_x, MOVELLA_VELOCITY);
        movella_state.velocity.y = TRANSFORM_RAW(vel_y, MOVELLA_VELOCITY);
        movella_state.velocity.z = TRANSFORM_RAW(vel_z, MOVELLA_VELOCITY);
        break;
    
    default:
        break;
    }
}
