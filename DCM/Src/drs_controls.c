/**
 * @file drs_controls.c
 * @brief Vehicle control loops.
 *
 * @author Carnegie Mellon Racing
 */

// ------------------------------------------------------------------------------------------------
// Includes
#include <stdlib.h>     // abs()
//#include <stddef.h>
//#include <math.h>
#include "drs_controls.h"       //Interface to implement
#include "controls.h"



/* Threshold defaults */
#define SWANGLE_THRESHOLD 10 /* @brief angle at which corner is defined */
#define THROTTLE_THRESHOLD 10
#define BRAKE_THRESHOLD 35 /*@brief 20-30 when not pressed, 120 is high*/

/* Hysteresis margins */
#define SWANGLE_MARGIN 5
#define THROTTLE_MARGIN 5
#define BRAKE_MARGIN 1


// static drs_state_t drs_state = DRS_CLOSED;
static bool prev_swangle_high = false;
static bool prev_throttle_high = false;
static bool prev_brake_high = false;

/** @brief DRS states and debug info to be sent. */
cmr_canCDCDRSStates_t drs_state = {
    .state  = 0,
    .angle  = 0,
    .pwm_left = 0,
    .pwm_right = 0
};

/** 
 * Keep track of previous button press condition.
 * Set to false by default.
 * Used in runDrsControls
 */
static bool prev_button_pressed = false;

// Forward declaration
static void setAutoDrs(uint8_t throttle_pos,
                        uint16_t brake_psi, 
                        int16_t swAngle_deg);

/**
 * @brief Get a read-only pointer to the DRS state and debug values.
 * 
 * @return Const pointer to the current DRS state and debug values.
 */
const cmr_canCDCDRSStates_t *getDRSInfo() {
    return (const cmr_canCDCDRSStates_t*) &drs_state;
} 

/**
 * @brief Runs control loops and sets motor torque limits and velocity targets accordingly.
 *
 * @param gear Which gear the vehicle is in.
 * @param drsMode What drs mode is the vehicle in
 * @param throttlePos_u8 Throttle position, 0-255.
 * @param brakePressurePsi_u8 Brake position, 0-255.
 * @param swAngle_deg Steering wheel angle in degrees. Zero-centered, right turn positive.
 */
void runDrsControls(
    cmr_canGear_t gear,
    cmr_canDrsMode_t drsMode,
    uint8_t throttlePos_u8,
    uint16_t brakePressurePsi_u8, 
    int16_t swAngle_deg) {

    bool drs_button_pressed = (((volatile cmr_canDIMActions_t *)canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON))->buttons) & BUTTON_ACT;

    static cmr_canDrsMode_t prevDrsMode = CMR_CAN_DRSM_UNKNOWN;
    if (drsMode != prevDrsMode) {
        drs_state.state = CMR_CAN_DRS_STATE_CLOSED;
        setDrsPosition(DRS_CLOSED_DUTY_CYCLE);
    }
    prevDrsMode = drsMode;

    switch (drsMode) {
        case CMR_CAN_DRSM_QUIET:
            drs_state.state = CMR_CAN_DRS_STATE_CLOSED;
            setServoQuiet();
            break;
        case CMR_CAN_DRSM_OPEN:
            drs_state.state = CMR_CAN_DRS_STATE_OPEN;
            setDrsPosition(DRS_OPEN_DUTY_CYCLE);
            break;

        case CMR_CAN_DRSM_CLOSED:
            drs_state.state = CMR_CAN_DRS_STATE_CLOSED;
            setDrsPosition(DRS_CLOSED_DUTY_CYCLE);
            break;

        case CMR_CAN_DRSM_TOGGLE:
            if (prev_button_pressed && 
                !drs_button_pressed) {
                if (drs_state.state == CMR_CAN_DRS_STATE_CLOSED) {
                    drs_state.state = CMR_CAN_DRS_STATE_OPEN;
                    setDrsPosition(DRS_OPEN_DUTY_CYCLE);
                } else {
                    drs_state.state = CMR_CAN_DRS_STATE_CLOSED;
                    setDrsPosition(DRS_CLOSED_DUTY_CYCLE);
                }
            }
            break;

        case CMR_CAN_DRSM_HOLD:
            if (drs_button_pressed) {
                drs_state.state = CMR_CAN_DRS_STATE_OPEN;
                setDrsPosition(DRS_OPEN_DUTY_CYCLE);
            } else { 
                drs_state.state = CMR_CAN_DRS_STATE_CLOSED;
                setDrsPosition(DRS_CLOSED_DUTY_CYCLE);
            }
            break;

        case CMR_CAN_DRSM_AUTO:
            switch (gear) {
                case CMR_CAN_GEAR_FAST:
                    setAutoDrs(throttlePos_u8,
                               brakePressurePsi_u8,
                               swAngle_deg);
                    break;
                case CMR_CAN_GEAR_SLOW:
                    setAutoDrs(throttlePos_u8,
                               brakePressurePsi_u8,
                               swAngle_deg);
                    break;
                default:
                    setAutoDrs(throttlePos_u8,
                               brakePressurePsi_u8,
                               swAngle_deg);
                    break;
            }
            break;
        default: 
            drs_state.state = CMR_CAN_DRS_STATE_CLOSED;
            setDrsPosition(DRS_CLOSED_DUTY_CYCLE);
            break;
    }

    prev_button_pressed = drs_button_pressed;

}

static void setAutoDrs(uint8_t throttle_pos,
                    uint16_t brake_psi, 
                    int16_t swAngle_deg) {

    uint8_t swangle_threshold = SWANGLE_THRESHOLD;
    uint8_t throttle_threshold = THROTTLE_THRESHOLD;
    uint16_t brake_threshold = BRAKE_THRESHOLD; 
    getProcessedValue(&swangle_threshold, DRS_SWANGLE_THRESH_INDEX, unsigned_integer);
    getProcessedValue(&throttle_threshold, DRS_THROTTLE_THRESH_INDEX, unsigned_integer);
    //getProcessedValue(&brake_threshold, DRS_BRAKE_THRESH_INDEX, unsigned_integer);
    uint8_t swangle = (uint8_t) abs(swAngle_deg);

    bool swangle_high = (swangle > swangle_threshold + SWANGLE_MARGIN) ||
                        (prev_swangle_high && swangle > swangle_threshold - SWANGLE_MARGIN);
    bool throttle_high = (throttle_pos > throttle_threshold + THROTTLE_MARGIN) ||
                         (prev_throttle_high && throttle_pos > throttle_threshold - THROTTLE_MARGIN);
    bool brake_high = false;
    
    if (swangle_high) {
        drs_state.state = CMR_CAN_DRS_STATE_CLOSED;
        setDrsPosition(DRS_CLOSED_DUTY_CYCLE);
    } else {
        if (!throttle_high || brake_high) {
            drs_state.state = CMR_CAN_DRS_STATE_CLOSED;
            setDrsPosition(DRS_CLOSED_DUTY_CYCLE);
        } else {
            drs_state.state = CMR_CAN_DRS_STATE_OPEN;
            setDrsPosition(DRS_OPEN_DUTY_CYCLE);
        }
    }

    prev_swangle_high = swangle_high;
    prev_throttle_high = throttle_high;
    prev_brake_high = brake_high;
}



