/**
 * @file new_controls.h
 * @brief Vehicle control loops.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CONTROLS_H
#define CONTROLS_H

#define ASSUME_NO_TURN true
#define BUTTON_ACK 0b10000000
#define BUTTON_ACT 0b00000001
#define BUTTON_PTT 0b100000
#define BUTTON_SCRN 0b10000

// ------------------------------------------------------------------------------------------------
// Includes

#include <stdint.h>
#include <stdbool.h>
#include "controls_helper.h"

// ------------------------------------------------------------------------------------------------
// Gear functions forward declarations

void setSlowTorque(uint8_t throttlePos_u8, int16_t swAngle_deg);
void setFastTorque(uint8_t throttlePos_u8);
float getYawRateControlLeftRightBias(int16_t swAngle_millideg);
void setTractionControl(uint8_t throttlePos_u8, uint8_t brakePressurePsi_u8, int16_t swAngle_deg, float leftRightBias_Nm,
    bool assumeNoTurn, bool ignoreYawRate, bool allowRegen, float critical_speed_mps);
void setYawRateControl (
    uint8_t throttlePos_u8,
    uint8_t brakePressurePsi_u8,
    int16_t swAngle_deg,
    bool clampbyside
);
void setYawRateAndTractionControl(uint8_t throttlePos_u8, uint8_t brakePressurePsi_u8, int16_t swAngle_deg,
    bool assumeNoTurn, bool ignoreYawRate, bool allowRegen, float critical_speed_mps);
void setCruiseControlTorque(uint8_t throttlePos_u8, uint8_t brakePressurePsi_u8, int32_t avgMotorSpeed_RPM);
void setEnduranceTorque(int32_t avgMotorSpeed_RPM, uint8_t throttlePos_u8, uint8_t brakePos_u8, int16_t swAngle_deg,
    int32_t battVoltage_mV, int32_t battCurrent_mA, uint8_t brakePressurePsi_u8);
void setEnduranceTestTorque(
    int32_t avgMotorSpeed_RPM,
    uint8_t throttlePos_u8,
    uint8_t brakePos_u8,
    int16_t swAngle_deg,
    int32_t battVoltage_mV,
    int32_t battCurrent_mA,
    uint8_t brakePressurePsi_u8,
    bool clampbyside
);

// ------------------------------------------------------------------------------------------------
// Public functions

void initControls();
void integrateCurrent();
void runControls(cmr_canGear_t gear, uint8_t throttlePos_u8, uint8_t brakePos_u8, uint8_t brakePressurePsi_u8,
    int16_t swAngle_deg, int32_t battVoltage_mV, int32_t battCurrent_mA, bool blank_command);
void setControlsStatus(cmr_canGear_t gear);
const volatile cmr_canCDCControlsStatus_t *getControlsStatus();
void setFastTorqueWithParallelRegen(uint8_t brakePressurePsi_u8, uint8_t throttlePos_u8);

// ------------------------------------------------------------------------------------------------
// Global variables

extern const float braking_threshold_psi;

// CAN data structure for traction control
extern volatile cmr_can_controls_pid_debug_t yrcDebug; //YRC
extern volatile cmr_can_front_slip_ratio_data_t frontSlipRatios;
extern volatile cmr_can_rear_slip_ratio_data_t rearSlipRatios;
extern volatile cmr_can_front_whl_speed_setpoint_t frontWhlSetpoints;
extern volatile cmr_can_rear_whl_speed_setpoint_t rearWhlSetpoints;
extern volatile cmr_can_front_whl_velocity_t frontWhlVelocities;
extern volatile cmr_can_rear_whl_velocity_t rearWhlVelocities;
extern volatile cmr_can_solver_inputs_t solver_inputs;
extern volatile cmr_can_solver_aux_t solver_aux;
extern volatile cmr_canCDCWheelTorque_t solver_torques;
extern volatile cmr_can_solver_settings_t solver_settings;
extern volatile cmr_canCDCKiloCoulombs_t coulombCounting;

extern volatile bool use_true_downforce;

#endif /* CONTROLS_32E_H */
