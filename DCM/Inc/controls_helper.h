#ifndef CONTROLS_HELPER_H
#define CONTROLS_HELPER_H

#include <stdint.h>
#include <stdbool.h>
#include <CMR/can_types.h>
#include <CMR/config_screen_helper.h>
#include <math.h>
#include "can.h"
#include "daq.h"
#include "motors.h"

// ------------------------------------------------------------------------------------------------
// Macro functions
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

// ------------------------------------------------------------------------------------------------
// constants

extern const float maxTorque_Nm;
extern const int16_t maxSpeed_rpm;

extern const float maxSlowTorque_Nm;
extern const int16_t maxSlowSpeed_rpm;

extern const float maxMediumTorque_Nm;
extern const int16_t maxMediumSpeed_rpm;

extern const float maxFastTorque_Nm;
extern const int16_t maxFastSpeed_rpm;

extern const float chassis_a;
extern const float chassis_b;
extern const float chassis_w_f;
extern const float chassis_w_r;

extern const float braking_threshold_psi;

extern const uint8_t brake_pressure_start;
extern const uint8_t paddle_pressure_start;

// ------------------------------------------------------------------------------------------------
// globals


// ------------------------------------------------------------------------------------------------
// helper functions

float Mz_calc(cmr_torqueDistributionNm_t *torque_req, float steering_angle_deg);
float getPackVoltage();
float getPackCurrent();
float getHVISensePackPower();
float getMaxCellVoltage();
float getMinCellVoltage();
float swAngleDegToSteeringAngleRad(int16_t swAngle_deg);
bool canTrustSBGVelocity(bool ignore_valid_bit);
bool setRegen(uint8_t *throttlePos_u8, uint8_t brakePressurePsi_u8, int32_t avgMotorSpeed_RPM);
float getRegenTorqueReq(uint8_t *throttlePos_u8, uint8_t brakePressurePsi_u8);
bool setPaddleRegen(uint8_t *throttlePos_u8, uint8_t brakePressurePsi_u8, int32_t avgMotorSpeed_RPM, uint8_t paddle_pressure, uint8_t paddle_regen_strength);
bool setParallelRegen(uint8_t throttlePos_u8, uint8_t brakePressurePsi_u8, int32_t avgMotorSpeed_RPM);

#endif /* CONTROL_HELPER_H */
