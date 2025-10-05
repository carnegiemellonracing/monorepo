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

extern const float braking_threshold_psi;

extern const uint16_t brake_pressure_start;
extern const uint8_t paddle_pressure_start;

// ------------------------------------------------------------------------------------------------
// globals


// ------------------------------------------------------------------------------------------------
// helper functions

float getPackVoltage();
float getPackCurrent();
float getHVISensePackPower();
float getMaxCellVoltage();
float getMinCellVoltage();
float swAngleMillidegToSteeringAngleRad(int32_t swAngle_deg);
bool canTrustSBGVelocity(bool ignore_valid_bit);
bool setRegen(uint8_t *throttlePos_u8, uint16_t brakePressurePsi_u8, int32_t avgMotorSpeed_RPM);
float getRegenTorqueReq(uint8_t *throttlePos_u8, uint16_t brakePressurePsi_u8);
bool setPaddleRegen(uint8_t *throttlePos_u8, uint16_t brakePressurePsi_u8, int32_t avgMotorSpeed_RPM, uint8_t paddle_pressure, uint8_t paddle_regen_strength);
bool setParallelRegen(uint8_t throttlePos_u8, uint16_t brakePressurePsi_u8, int32_t avgMotorSpeed_RPM);

#endif /* CONTROL_HELPER_H */
