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
    const cmr_canAMKSetpoints_t *sp = getAMKSetpoints(motor);

    return (sp->torqueLimPos_dpcnt > 0) ? sp->torqueLimPos_dpcnt : sp->torqueLimNeg_dpcnt;
}

void daqWheelSpeedFeedback(cmr_canCDCWheelVelocity_t *speedFeedback) {

    volatile cmr_canAMKActualValues1_t *amkAct1FL = canTractiveGetPayload(CANRX_TRAC_INV_FL_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1FR = canTractiveGetPayload(CANRX_TRAC_INV_FR_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1RL = canTractiveGetPayload(CANRX_TRAC_INV_RL_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1RR = canTractiveGetPayload(CANRX_TRAC_INV_RR_ACT1);

    speedFeedback->frontLeft_rpm =  amkAct1FL->velocity_rpm;
    speedFeedback->frontRight_rpm = amkAct1FR->velocity_rpm;
    speedFeedback->rearLeft_rpm =   amkAct1RL->velocity_rpm;
    speedFeedback->rearRight_rpm =  amkAct1RR->velocity_rpm;
}

void daqWheelTorqueFeedback(cmr_canCDCWheelTorque_t *torqueFeedback) {
    volatile cmr_canAMKActualValues1_t *amkAct1FL = canTractiveGetPayload(CANRX_TRAC_INV_FL_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1FR = canTractiveGetPayload(CANRX_TRAC_INV_FR_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1RL = canTractiveGetPayload(CANRX_TRAC_INV_RL_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1RR = canTractiveGetPayload(CANRX_TRAC_INV_RR_ACT1);

    torqueFeedback->frontLeft_Nm =  amkAct1FL->torqueCurrent_raw;
    torqueFeedback->frontRight_Nm = amkAct1FR->torqueCurrent_raw;
    torqueFeedback->rearLeft_Nm =   amkAct1RL->torqueCurrent_raw;
    torqueFeedback->rearRight_Nm =  amkAct1RR->torqueCurrent_raw;
}

void daqWheelSpeedSetpoints(cmr_canCDCWheelVelocity_t *speedSetpoint) {
    const cmr_canAMKSetpoints_t *amkSetpoint1FL = getAMKSetpoints(MOTOR_FL);
    const cmr_canAMKSetpoints_t *amkSetpoint1FR = getAMKSetpoints(MOTOR_FR);
    const cmr_canAMKSetpoints_t *amkSetpoint1RL = getAMKSetpoints(MOTOR_RL);
    const cmr_canAMKSetpoints_t *amkSetpoint1RR = getAMKSetpoints(MOTOR_RR);

    speedSetpoint->frontLeft_rpm =  amkSetpoint1FL->velocity_rpm;
    speedSetpoint->frontRight_rpm = amkSetpoint1FR->velocity_rpm;
    speedSetpoint->rearLeft_rpm =   amkSetpoint1RL->velocity_rpm;
    speedSetpoint->rearRight_rpm =  amkSetpoint1RR->velocity_rpm;
}

void daqWheelTorqueSetpoints(cmr_canCDCWheelTorque_t *torqueSetpoint) {
    torqueSetpoint->frontLeft_Nm =  getMotorTorqueRequest(MOTOR_FL);
    torqueSetpoint->frontRight_Nm = getMotorTorqueRequest(MOTOR_FR);
    torqueSetpoint->rearLeft_Nm =   getMotorTorqueRequest(MOTOR_RL);
    torqueSetpoint->rearRight_Nm =  getMotorTorqueRequest(MOTOR_RR);
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
