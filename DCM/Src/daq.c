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



//TODO: broadcast more important constants 
// calculated wheel torques and speeds + front/rear bias 
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
