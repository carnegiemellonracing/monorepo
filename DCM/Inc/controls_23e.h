/**
 * @file new_controls.h
 * @brief Vehicle control loops.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CONTROLS_32E_H
#define CONTROLS_32E_H

#define ASSUME_NO_TURN true
#define BUTTON_ACT 0b1000000
#define BUTTON_ACK 0b10000000
#define BUTTON_PTT 0b100000
#define BUTTON_SCRN 0b10000

// ------------------------------------------------------------------------------------------------
// Includes

#include <stdint.h>
#include <stdbool.h>
#include "controls_helper.h"

// ------------------------------------------------------------------------------------------------
// Gear functions forward declarations

void setSlowTorque(uint8_t throttlePos_u8);
void setFastTorque(uint8_t throttlePos_u8);
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
bool setCVXGENSolver(
    uint8_t throttlePos_u8,
    uint8_t brakePressurePsi_u8,
    int16_t swAngle_deg,
    bool enable_Regen,
    bool timing_Test,
	bool assumeNoTurn
);

// ------------------------------------------------------------------------------------------------
// Public functions

void initControls();
void runControls(cmr_canGear_t gear, uint8_t throttlePos_u8, uint8_t brakePos_u8, uint8_t brakePressurePsi_u8,
    int16_t swAngle_deg, int32_t battVoltage_mV, int32_t battCurrent_mA, bool blank_command);
void setControlsStatus(cmr_canGear_t gear);
const volatile cmr_canCDCControlsStatus_t *getControlsStatus(); 

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
extern volatile cmr_can_CVXGEN_info_t solverInfo;
extern volatile cmr_canCDCWheelTorque_t solverTorques;
extern volatile cmr_can_CVXGEN_counter_t nonConvergenceCounter;
extern volatile cmr_canCDCKiloCoulombs_t coulombCounting;

extern volatile bool use_true_downforce;

#endif /* CONTROLS_32E_H */
