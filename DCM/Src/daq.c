/**
 * @file daq.c
 * @brief Data aquisition calculations.
 *
 * @par Does several calculations based on information from the Ellipse INS
 *      and inverters/motors to determine overall vehicle dynamic infomation
 *      that will be useful not just for control algorithms, but also 
 *      reconstriction from logs
 *
 * @author Carnegie Mellon Racing
 */

#include "daq.h"
#include "can.h"
#include "motors.h"
#include "constants.h"

#include <math.h>
#include <complex.h>

int32_t motorRPMtoWheelRPM10(int16_t rpm) {
    return (((int32_t) rpm) * 10 * gear_ratio_bot / (gear_ratio_top + gear_ratio_bot));
}

float motorCurrentToTorque10(int16_t current) {
    static const float UNIT_TO_NM = 0.001701171875;

    return ((float) current) * 10 * gear_ratio * UNIT_TO_NM;
}

float motorSetpointPercentToTorque10(int16_t sp) {
    static const float PCT10_TO_NM10 = 0.098;

    return ((float) sp) * PCT10_TO_NM10 * gear_ratio; 
}

int16_t getMotorTorqueRequest(motorLocation_t motor) {
    const cmr_canDTISetpoints_t *sp = getDTISetpoints(motor);

    return (sp->torqueLimPos > 0) ? sp->torqueLimPos : sp->torqueLimNeg;
}

void daqWheelSpeedFeedback(cmr_canCDCWheelVelocity_t *speedFeedback) {

    volatile cmr_canDTI_TX_Erpm_t *dtiERPM_FL = canTractiveGetPayload(CANRX_TRAC_FL_ERPM);
    volatile cmr_canDTI_TX_Erpm_t *dtiERPM_FR = canTractiveGetPayload(CANRX_TRAC_FR_ERPM);
    volatile cmr_canDTI_TX_Erpm_t *dtiERPM_RL = canTractiveGetPayload(CANRX_TRAC_RL_ERPM);
    volatile cmr_canDTI_TX_Erpm_t *dtiERPM_RR = canTractiveGetPayload(CANRX_TRAC_RR_ERPM);

    speedFeedback->frontLeft_rpm =  dtiERPM_FL->erpm / pole_pairs;
    speedFeedback->frontRight_rpm = dtiERPM_FR->erpm / pole_pairs;
    speedFeedback->rearLeft_rpm =   dtiERPM_RL->erpm / pole_pairs;
    speedFeedback->rearRight_rpm =  dtiERPM_RR->erpm / pole_pairs;
}

void daqWheelTorqueFeedback(cmr_canCDCWheelTorque_t *torqueFeedback) {
    volatile cmr_canDTI_TX_Current_t *dtiCurrentFL = canTractiveGetPayload(CANRX_TRAC_FL_CURRENT);
    volatile cmr_canDTI_TX_Current_t *dtiCurrentFR = canTractiveGetPayload(CANRX_TRAC_FR_CURRENT);
    volatile cmr_canDTI_TX_Current_t *dtiCurrentRL = canTractiveGetPayload(CANRX_TRAC_RL_CURRENT);
    volatile cmr_canDTI_TX_Current_t *dtiCurrentRR = canTractiveGetPayload(CANRX_TRAC_RR_CURRENT);

    torqueFeedback->frontLeft_Nm =  dtiCurrentFL->ac_current;
    torqueFeedback->frontRight_Nm = dtiCurrentFR->ac_current;
    torqueFeedback->rearLeft_Nm =   dtiCurrentRL->ac_current;
    torqueFeedback->rearRight_Nm =  dtiCurrentRR->ac_current;
}

void daqWheelSpeedSetpoints(cmr_canCDCWheelVelocity_t *speedSetpoint) {
    const cmr_canDTISetpoints_t *dtiSetpoint1FL = getDTISetpoints(MOTOR_FL);
    const cmr_canDTISetpoints_t *dtiSetpoint1FR = getDTISetpoints(MOTOR_FR);
    const cmr_canDTISetpoints_t *dtiSetpoint1RL = getDTISetpoints(MOTOR_RL);
    const cmr_canDTISetpoints_t *dtiSetpoint1RR = getDTISetpoints(MOTOR_RR);

    speedSetpoint->frontLeft_rpm =  dtiSetpoint1FL->velocity_rpm;
    speedSetpoint->frontRight_rpm = dtiSetpoint1FR->velocity_rpm;
    speedSetpoint->rearLeft_rpm =   dtiSetpoint1RL->velocity_rpm;
    speedSetpoint->rearRight_rpm =  dtiSetpoint1RR->velocity_rpm;
}

void daqWheelTorqueSetpoints(cmr_canCDCWheelTorque_t *torqueSetpoint) {
    torqueSetpoint->frontLeft_Nm =  getMotorTorqueRequest(MOTOR_FL);
    torqueSetpoint->frontRight_Nm = getMotorTorqueRequest(MOTOR_FR);
    torqueSetpoint->rearLeft_Nm =   getMotorTorqueRequest(MOTOR_RL);
    torqueSetpoint->rearRight_Nm =  getMotorTorqueRequest(MOTOR_RR);
}

void daqPosePosition(cmr_canCDCPosePosition_t *posePos) {
    volatile cmr_canSBGEKFPosition_t *sbgPos = canDAQGetPayload(CANRX_DAQ_SBG_POS);

    posePos->latitude_deg = ((float) sbgPos->latitude) / 10000000;
    posePos->longitude_deg = ((float) sbgPos->longitude) / 10000000;
}

float daqPoseOrientationRadToDeg(int16_t rad) {
    // 360 / (2*pi) / 10^4
    static const float RAD104_TO_DEG = 0.005729577;

    return ((float) rad) * RAD104_TO_DEG;
}

float daqPoseOrientationRad(int16_t rad) {
    // 360 / (2*pi) / 10^4
    static const float RAD_TO_DEG = 57.29577;

    return ((float) rad) * RAD_TO_DEG;
}

void daqPoseOrientation(cmr_canCDCPoseOrientation_t *poseOrient) {
    volatile cmr_canSBGEKFOrient_t *sbgOrient = canDAQGetPayload(CANRX_DAQ_SBG_ORIENT);

    poseOrient->roll_deg = (int16_t) (daqPoseOrientationRadToDeg(sbgOrient->roll) * 10);
    poseOrient->pitch_deg = (int16_t) (daqPoseOrientationRadToDeg(sbgOrient->pitch) * 10);
    poseOrient->yaw_deg = (int16_t) (daqPoseOrientationRadToDeg(sbgOrient->yaw) * 10);

    volatile cmr_canSBGEKFVelocity_t *sbgVel = canDAQGetPayload(CANRX_DAQ_SBG_VEL);
    // Perform transformations on car's velocity by using complex numbers as
    // a stand-in for a 2D vector. In the NED (North-East-Down) coordinate
    // frame, real->north, imag->east. Then in the car reference frame,
    // forward->real, right->imag.
    float vel_n_ned = ((float) sbgVel->velocity_n) / 100;
    float vel_e_ned = ((float) sbgVel->velocity_e) / 100;
    float complex velocity_ned = vel_n_ned + vel_e_ned * I;
    float slip_ang = cargf(velocity_ned);

    poseOrient->velocity_deg = (int16_t) (daqPoseOrientationRad(slip_ang) * 10);
}

void daqPoseVelocity(cmr_canCDCPoseVelocity_t *poseVel) {
    volatile cmr_canSBGBodyVelocity_t *sbgBodyVel = canDAQGetPayload(CANRX_DAQ_SBG_BODY_VEL);

    poseVel->longitudinalVel_mps = sbgBodyVel->velocity_forward;
    poseVel->lateralVel_mps = sbgBodyVel->velocity_right;
    poseVel->verticalVel_mps = sbgBodyVel->velocity_down;
}

float carVelocityToWheelRPM(float vel) {
    return (vel / (effective_wheel_dia_m * M_PI)) * 60.0f;
}

float carVelocityToMotorRPM(float vel) {
    return carVelocityToWheelRPM(vel) * gear_ratio;
}

float wheelRPMToCarVelocity(float wheelRPM) {
	return wheelRPM * (effective_wheel_dia_m * M_PI) / 60.0f;
}

float motorRPMToCarVelocity(float wheelRPM) {
    return wheelRPM * (effective_wheel_dia_m * M_PI) / 60.0 / gear_ratio;
}

float estimateCarVelocityFromMotors() {
    float speed = 0.0f;
    for (size_t i = 0; i < MOTOR_LEN; i++) {
    	speed += getMotorSpeed_rpm(i);
    	// speed += 5000;
    }
    speed /= MOTOR_LEN;   // Get average motor RPM
    speed /= gear_ratio;  // Convert to wheel RPM
    return wheelRPMToCarVelocity(speed);
}

// converts big endian to little endian
cmr_canIzzie_loadcell_calibrated_t getLoads(cmr_canIzzie_loadcell_raw_t raw_data){
    cmr_canIzzie_loadcell_calibrated_t to_return = { 0 };

    // TODO: FINISH
    return to_return;
}

volatile cmr_can_controls_debug_global_t controls_debug_struct_global;
volatile cmr_can_controls_debug_FR_t controls_debug_struct_fr;
volatile cmr_can_controls_debug_FL_t controls_debug_struct_fl;
volatile cmr_can_controls_debug_RR_t controls_debug_struct_rr;
volatile cmr_can_controls_debug_RL_t controls_debug_struct_rl;
volatile cmr_can_controls_pid_debug_t controls_pid_struct;

cmr_can_controls_debug_global_t* getControlsDebugGlobal(){
    return (cmr_can_controls_debug_global_t*) &controls_debug_struct_global;
} 
cmr_can_controls_debug_FR_t* getControlsDebugFr(){
    return (cmr_can_controls_debug_FR_t*) &controls_debug_struct_fr;
} 
cmr_can_controls_debug_FL_t* getControlsDebugFl(){
    return (cmr_can_controls_debug_FL_t*) &controls_debug_struct_fl;
} 
cmr_can_controls_debug_RR_t* getControlsDebugRr(){
    return (cmr_can_controls_debug_RR_t*) &controls_debug_struct_rr;
} 
cmr_can_controls_debug_RL_t* getControlsDebugRl(){
    return (cmr_can_controls_debug_RL_t*) &controls_debug_struct_rl;
} 
cmr_can_controls_pid_debug_t* getPidDebug(){
    return (cmr_can_controls_pid_debug_t*) &controls_pid_struct;
} 
