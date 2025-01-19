/**
 * @file daq.h
 * @brief Data aquisition calculations.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef DAQ_H
#define DAQ_H

#include <CMR/can_types.h>
#include <CMR/can_ids.h>

/** @brief Gear ratio from motors to wheels. */
extern const float GEAR_RATIO;
extern const float EFFECTIVE_WHEEL_DIA_M; /** @brief effective wheel diameter */
extern const float EFFECTIVE_WHEEL_RAD_M; /** @brief effective wheel radius */

void daqWheelSpeedFeedback(cmr_canCDCWheelVelocity_t *speedFeedback);
void daqWheelTorqueFeedback(cmr_canCDCWheelTorque_t *torqueFeedback);
void daqWheelSpeedSetpoints(cmr_canCDCWheelVelocity_t *speedSetpoint);
void daqWheelTorqueSetpoints(cmr_canCDCWheelTorque_t *torqueSetpoint);

void daqPosePosition(cmr_canCDCPosePosition_t *posePos);
void daqPoseOrientation(cmr_canCDCPoseOrientation_t *poseOrient);
void daqPoseVelocity(cmr_canCDCPoseVelocity_t *poseVel);

float carVelocityToWheelRPM(float vel);
float carVelocityToMotorRPM(float vel);
float wheelRPMToCarVelocity(float wheelRPM);
float motorRPMToCarVelocity(float wheelRPM);
float estimateCarVelocityFromMotors();
cmr_can_controls_debug_global_t* getControlsDebugGlobal();
cmr_can_controls_debug_FR_t* getControlsDebugFr();
cmr_can_controls_debug_FL_t* getControlsDebugFl();
cmr_can_controls_debug_RR_t* getControlsDebugRr();
cmr_can_controls_debug_RL_t* getControlsDebugRl();
cmr_can_controls_pid_debug_t* getPidDebug();
#endif
