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
    [MOVELLA_QUATERNION] = 1 / (float) (1U << 16 - 1),
    [MOVELLA_EULER_ANGLES] = 1 / (float) (1U << 7),
    [MOVELLA_IMU_ACCEL] = 1 / (float) (1U << 8),
    [MOVELLA_IMU_GYRO] = 1 / (float) (1U << 9),
    [MOVELLA_VELOCITY] = 1 / (float) (1U << 6),
};

typedef float matrix3x3[3][3];
typedef float vector3[3];

static const matrix3x3 R_BD = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

static const vector3 x_BD = { -0.5f, 0.0f, 0.2f };

// Car COG frame
static const vector3 x_FL = { 0.837f, 0.65f, -0.3f };
static const vector3 x_FR = { 0.837f, -0.65f, -0.3f };
static const vector3 x_RL = { -0.713f, 0.65f, -0.3f };
static const vector3 x_RR = { -0.713f, -0.65f, -0.3f };

static const vector3 x_D_FL = { x_FL[0] - x_BD[0], x_FL[1] - x_BD[1], x_FL[2] - x_BD[2] };
static const vector3 x_D_FR = { x_FR[0] - x_BD[0], x_FR[1] - x_BD[1], x_FR[2] - x_BD[2] };
static const vector3 x_D_RL = { x_RL[0] - x_BD[0], x_RL[1] - x_BD[1], x_RL[2] - x_BD[2] };
static const vector3 x_D_RR = { x_RR[0] - x_BD[0], x_RR[1] - x_BD[1], x_RR[2] - x_BD[2] };

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

    matrix3x3 R;

    // Movella-frame angular velocity.
    struct {
        bool inited;
        float x_prev;
        float y_prev;
        float z_prev;
        float x;
        float y;
        float z;
    } gyro;
    
    // Movella-frame angular acceleration.
    struct {
        float x;
        float y;
        float z;
    } alpha;
    
    // Movella-frame acceleration.
    struct {
        float x;
        float y;
        float z;
    } accel;

    // Movella-frame velocity.
    struct {
        float x;
        float y;
        float z;
    } velocity;
    
    struct {
        float x;
        float y;
        float z;
    } global_velocity;

    cmr_canMovellaStatus_t status;

} movella_state_t;

typedef struct {

    matrix3x3 R;

    // Car-frame angular velocity.
    struct {
        float x;
        float y;
        float z;
    } gyro;
    
    // Car-frame acceleration.
    struct {
        float x;
        float y;
        float z;
    } accel;

    // Car-frame velocity.
    struct {
        float x;
        float y;
        float z;
    } velocity;

    struct {
        float x;
        float y;
    } fl_velocity;

    struct {
        float x;
        float y;
    } fr_velocity;

    struct {
        float x;
        float y;
    } rl_velocity;

    struct {
        float x;
        float y;
    } rr_velocity;

    struct {
        float body;
        float fl;
        float fr;
        float rl;
        float rr;
    } slip_angle;

} car_state_t;

extern volatile movella_state_t movella_state;
extern volatile car_state_t car_state;

void movella_parse(uint16_t canID, volatile void *payload);
void movella_test();
void movella_random_test();

#endif