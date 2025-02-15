#ifndef MOVELLA_H
#define MOVELLA_H

#include "can.h"

typedef enum {
    MOVELLA_QUATERNION = 0,
    MOVELLA_EULER_ANGLES,
    MOVELLA_IMU_ACCEL,
    MOVELLA_IMU_GYRO,
    MOVELLA_VELOCITY,
    MOVELLA_MESSAGE_NUM,
} movella_message_t;

static const float scaling_constants[MOVELLA_MESSAGE_NUM] = {
    [MOVELLA_QUATERNION] = 1 / (float) (1U << 15 - 1),
    [MOVELLA_EULER_ANGLES] = 1 / (float) (1U << 7),
    [MOVELLA_IMU_ACCEL] = 1 / (float) (1U << 8),
    [MOVELLA_IMU_GYRO] = 1 / (float) (1U << 9),
    [MOVELLA_VELOCITY] = 1 / (float) (1U << 6),
};

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

typedef struct {
    
    quaternion_t quaternion;

    struct {
        float yaw;
        float pitch;
        float roll;
    } euler_angles;

    float rotation_matrix[3][3];

    // Body-frame angular velocity.
    struct {
        float x;
        float y;
        float z;
    } gyro;
    
    // Body-frame acceleration.
    struct {
        float x;
        float y;
        float z;
    } accel;

    // Body-frame velocity.
    struct {
        float x;
        float y;
        float z;
    } velocity;

    struct {

    } status;

} movella_state_t;

static volatile movella_state_t movella_state;

void movella_parse(canDaqRX_t movella_msg, volatile void* payload);

#endif