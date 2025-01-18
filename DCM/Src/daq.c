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

#include <math.h>
#include <complex.h>

// Gear ratio is defined as (TOP + BOTTOM) / (BOTTOM)
static const int32_t GEAR_RATIO_TOP = 8784;
static const int32_t GEAR_RATIO_BOT = 621;

const float GEAR_RATIO = 13.93; //updated for 24e
const float EFFECTIVE_WHEEL_DIA_M = 0.43; /** @brief effective wheel diameter */
const float EFFECTIVE_WHEEL_RAD_M = EFFECTIVE_WHEEL_DIA_M * 0.5f; /** @brief effective wheel radius */

int32_t motorRPMtoWheelRPM10(int16_t rpm) {
    return (((int32_t) rpm) * 10 * GEAR_RATIO_BOT / (GEAR_RATIO_TOP + GEAR_RATIO_BOT));
}

float motorCurrentToTorque10(int16_t current) {
    static const float UNIT_TO_NM = 0.001701171875;

    return ((float) current) * 10 * GEAR_RATIO * UNIT_TO_NM;
}

float motorSetpointPercentToTorque10(int16_t sp) {
    static const float PCT10_TO_NM10 = 0.098;

    return ((float) sp) * PCT10_TO_NM10 * GEAR_RATIO; 
}

int16_t getMotorTorqueRequest(motorLocation_t motor) {
    const cmr_canAMKSetpoints_t *sp = getAMKSetpoints(motor);

    return (sp->torqueLimPos_dpcnt > 0) ? sp->torqueLimPos_dpcnt : sp->torqueLimNeg_dpcnt;
}

void daqWheelSpeedFeedback(cmr_canCDCWheelVelocity_t *speedFeedback) {

    volatile cmr_canAMKActualValues1_t *amkAct1FL = canTractiveGetPayload(CANRX_TRAC_INV_FL_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1FR = canTractiveGetPayload(CANRX_TRAC_INV_FR_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1RL = canTractiveGetPayload(CANRX_TRAC_INV_RL_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1RR = canTractiveGetPayload(CANRX_TRAC_INV_RR_ACT1);

    speedFeedback->frontLeft_rpm =  (int16_t) motorRPMtoWheelRPM10(amkAct1FL->velocity_rpm);
    speedFeedback->frontRight_rpm = (int16_t) motorRPMtoWheelRPM10(amkAct1FR->velocity_rpm);
    speedFeedback->rearLeft_rpm =   (int16_t) motorRPMtoWheelRPM10(amkAct1RL->velocity_rpm);
    speedFeedback->rearRight_rpm =  (int16_t) motorRPMtoWheelRPM10(amkAct1RR->velocity_rpm);
}

void daqWheelTorqueFeedback(cmr_canCDCWheelTorque_t *torqueFeedback) {
    volatile cmr_canAMKActualValues1_t *amkAct1FL = canTractiveGetPayload(CANRX_TRAC_INV_FL_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1FR = canTractiveGetPayload(CANRX_TRAC_INV_FR_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1RL = canTractiveGetPayload(CANRX_TRAC_INV_RL_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1RR = canTractiveGetPayload(CANRX_TRAC_INV_RR_ACT1);

    torqueFeedback->frontLeft_Nm =  (int16_t) motorCurrentToTorque10(amkAct1FL->torqueCurrent_raw);
    torqueFeedback->frontRight_Nm = (int16_t) motorCurrentToTorque10(amkAct1FR->torqueCurrent_raw);
    torqueFeedback->rearLeft_Nm =   (int16_t) motorCurrentToTorque10(amkAct1RL->torqueCurrent_raw);
    torqueFeedback->rearRight_Nm =  (int16_t) motorCurrentToTorque10(amkAct1RR->torqueCurrent_raw);
}

void daqWheelSpeedSetpoints(cmr_canCDCWheelVelocity_t *speedSetpoint) {
    const cmr_canAMKSetpoints_t *amkSetpoint1FL = getAMKSetpoints(MOTOR_FL);
    const cmr_canAMKSetpoints_t *amkSetpoint1FR = getAMKSetpoints(MOTOR_FR);
    const cmr_canAMKSetpoints_t *amkSetpoint1RL = getAMKSetpoints(MOTOR_RL);
    const cmr_canAMKSetpoints_t *amkSetpoint1RR = getAMKSetpoints(MOTOR_RR);

    speedSetpoint->frontLeft_rpm =  (int16_t) motorRPMtoWheelRPM10(amkSetpoint1FL->velocity_rpm);
    speedSetpoint->frontRight_rpm = (int16_t) motorRPMtoWheelRPM10(amkSetpoint1FR->velocity_rpm);
    speedSetpoint->rearLeft_rpm =   (int16_t) motorRPMtoWheelRPM10(amkSetpoint1RL->velocity_rpm);
    speedSetpoint->rearRight_rpm =  (int16_t) motorRPMtoWheelRPM10(amkSetpoint1RR->velocity_rpm);
}

void daqWheelTorqueSetpoints(cmr_canCDCWheelTorque_t *torqueSetpoint) {
    torqueSetpoint->frontLeft_Nm =  (int16_t) motorSetpointPercentToTorque10(getMotorTorqueRequest(MOTOR_FL));
    torqueSetpoint->frontRight_Nm = (int16_t) motorSetpointPercentToTorque10(getMotorTorqueRequest(MOTOR_FR));
    torqueSetpoint->rearLeft_Nm =   (int16_t) motorSetpointPercentToTorque10(getMotorTorqueRequest(MOTOR_RL));
    torqueSetpoint->rearRight_Nm =  (int16_t) motorSetpointPercentToTorque10(getMotorTorqueRequest(MOTOR_RR));
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
    return (vel / (EFFECTIVE_WHEEL_DIA_M * M_PI)) * 60.0f;
}

float carVelocityToMotorRPM(float vel) {
    return carVelocityToWheelRPM(vel) * GEAR_RATIO;
}

float wheelRPMToCarVelocity(float wheelRPM) {
	return wheelRPM * (EFFECTIVE_WHEEL_DIA_M * M_PI) / 60.0f;
}

float motorRPMToCarVelocity(float wheelRPM) {
    return wheelRPM * (EFFECTIVE_WHEEL_DIA_M * M_PI) / 60.0 / GEAR_RATIO;
}

float estimateCarVelocityFromMotors() {
    float speed = 0.0f;
    for (size_t i = 0; i < MOTOR_LEN; i++) {
    	speed += getMotorSpeed_rpm(i);
    	// speed += 5000;
    }
    speed /= MOTOR_LEN;   // Get average motor RPM
    speed /= GEAR_RATIO;  // Convert to wheel RPM
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
