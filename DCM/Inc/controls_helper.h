#ifndef CONTROLS_HELPER_H
#define CONTROLS_HELPER_H

#include <stdint.h>
#include <stdbool.h>
#include <CMR/can_types.h>
#include "../optimizer/optimizer.h"
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

#define PI 3.1415926535897932384626f
#define G 9.81f
#define DAQ_PUSHROD_ANGLE_FR 35.0f
#define DAQ_PUSHROD_ANGLE_RR 30.0f

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
float swAngleMillidegToSteeringAngleRad(int32_t swAngle_deg);

float get_load_cell_angle_rad(canDaqRX_t loadIndex);

float get_downforce(canDaqRX_t loadIndex, bool use_true_downforce);

void get_tractive_capabilities(float *cap_fl, float *cap_fr, float *cap_rl, float *cap_rr, bool use_true_downforce);

void compute_torque_limits(float cap_fl, float cap_fr, float cap_rl, float cap_rr,
  float *lim_fl, float *lim_fr, float *lim_rl, float *lim_rr);

void load_optimizer_state(optimizer_state_t *state, float normalized_throttle, int32_t swAngle_millideg_FL, int32_t swAngle_millideg_FR, 
  float lim_fl, float lim_fr, float lim_rl, float lim_rr, bool allow_regen);

bool canTrustSBGVelocity(bool ignore_valid_bit);
bool setRegen(uint8_t *throttlePos_u8, uint16_t brakePressurePsi_u8, int32_t avgMotorSpeed_RPM);
float getRegenTorqueReq(uint8_t *throttlePos_u8, uint16_t brakePressurePsi_u8);
bool setPaddleRegen(uint8_t *throttlePos_u8, uint16_t brakePressurePsi_u8, int32_t avgMotorSpeed_RPM, uint8_t paddle_pressure, uint8_t paddle_regen_strength);
bool setParallelRegen(uint8_t throttlePos_u8, uint16_t brakePressurePsi_u8, int32_t avgMotorSpeed_RPM);


#endif CONTROLS_HELPER_H
//#endif /* CONTROL_HELPER_H */
