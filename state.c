/**
 * @file state.c
 * @brief DIM state implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "state.h"  // interface to implement
#include "gpio.h"   // GPIO interface
#include "can.h"    // can interface
#include "stdlib.h"

/** @brief DIM state. */
static volatile struct {
    cmr_canState_t vsmReq;      /**< @brief Requested VSM state. */
    cmr_canGear_t gear;         /**< @brief Current gear. */
    cmr_canGear_t gearReq;      /**< @brief Requested gear. */
} state = {
    .vsmReq = CMR_CAN_GLV_ON,
    .gear = CMR_CAN_GEAR_FAST,
    .gearReq = CMR_CAN_GEAR_FAST
};

/**
 * @brief Gets the VSM state.
 *
 * @note VSM state is maintained in the received CAN heartbeat.
 *
 * @return The VSM state.
 */
cmr_canState_t stateGetVSM(void) {
    cmr_canRXMeta_t *heartbeatVSMMeta = canRXMeta + CANRX_HEARTBEAT_VSM;
    volatile cmr_canHeartbeat_t *heartbeatVSM =
        (void *) heartbeatVSMMeta->payload;

    return heartbeatVSM->state;
}

/**
 * @brief Gets the requested VSM state.
 *
 * @return The requested VSM state.
 */
cmr_canState_t stateGetVSMReq(void) {
    return state.vsmReq;
}

/**
 * @brief Gets the current gear.
 *
 * @return The current gear.
 */
cmr_canGear_t stateGetGear(void) {
    return state.gear;
}

/**
 * @brief Gets the requested gear.
 *
 * @return The requested gear.
 */
cmr_canGear_t stateGetGearReq(void) {
    return state.gearReq;
}

/**
 * @brief Gets the average wheel speed reported by the inverters.
 *
 * @param none
 *
 * @return average wheel speed
 */
int32_t getAverageWheelRPM(void) {
    /* Get CAN data */
    // Front Left
    cmr_canRXMeta_t *metaAMK_FL_Act1 = canRXMeta + CANRX_AMK_FL_ACT_1;
    volatile cmr_canAMKActualValues1_t *canAMK_FL_Act1 =
        (void *) metaAMK_FL_Act1->payload;
    // Front Right
    cmr_canRXMeta_t *metaAMK_FR_Act1 = canRXMeta + CANRX_AMK_FR_ACT_1;
    volatile cmr_canAMKActualValues1_t *canAMK_FR_Act1 =
        (void *) metaAMK_FR_Act1->payload;
    // Rear Left
    cmr_canRXMeta_t *metaAMK_RL_Act1 = canRXMeta + CANRX_AMK_RL_ACT_1;
    volatile cmr_canAMKActualValues1_t *canAMK_RL_Act1 =
        (void *) metaAMK_RL_Act1->payload;
    // Rear Right
    cmr_canRXMeta_t *metaAMK_RR_Act1 = canRXMeta + CANRX_AMK_RR_ACT_1;
    volatile cmr_canAMKActualValues1_t *canAMK_RR_Act1 =
        (void *) metaAMK_RR_Act1->payload;

    /* Extract wheel speeds */
    int32_t frontLeftRPM = -1*(canAMK_FL_Act1->velocity_rpm); // Motor direction reversed on left side
    int32_t frontRightRPM = canAMK_FR_Act1->velocity_rpm;
    int32_t rearLeftRPM = -1*(canAMK_RL_Act1->velocity_rpm); // Motor direction reversed on left side
    int32_t rearRightRPM = canAMK_RR_Act1->velocity_rpm;

    /* Compute average */
    int32_t average = (frontLeftRPM + frontRightRPM + rearLeftRPM + rearRightRPM)/4;

    return average;
}

/**
 * @brief Checks if vehicle is slow enough to request state down during RTD.
 *        This intends to stop accidental state down requests during driving.
 *
 * @param none
 *
 * @return If the car is slow enough to state down from rtd
 */
bool slowEnough(void) {
    int32_t avgWheelRPM = getAverageWheelRPM();

    uint16_t cutoff = 3; //mph

    /* Wheel Speed to Vehicle Speed Conversion
     *      (x rotations / 1min) * (18" * PI) * (1' / 12") * 
     *      (60min / 1hr) * (1 mi / 5280')
     *      = x * 0.0535                                   */
    uint16_t vehicleSpeed = avgWheelRPM * 0.0535;

    return abs(vehicleSpeed) < cutoff;
}

/**
 * @brief Checks if the requested VSM state is allowed.
 *
 * @param vsm The current VSM state.
 * @param vsmReq The requested VSM state.
 */
bool stateVSMReqIsValid(cmr_canState_t vsm, cmr_canState_t vsmReq) {
    switch (vsm) {
        case CMR_CAN_UNKNOWN:
            return (vsmReq == CMR_CAN_GLV_ON);
        case CMR_CAN_GLV_ON:
            return (vsmReq == CMR_CAN_GLV_ON) ||
                   (vsmReq == CMR_CAN_HV_EN);
        case CMR_CAN_HV_EN:
            return (vsmReq == CMR_CAN_GLV_ON) ||
                   (vsmReq == CMR_CAN_HV_EN) ||
                   (vsmReq == CMR_CAN_RTD);
        case CMR_CAN_RTD:
            return ((vsmReq == CMR_CAN_HV_EN) & slowEnough()) ||
                   (vsmReq == CMR_CAN_RTD);
        case CMR_CAN_ERROR:
            return (vsmReq == CMR_CAN_GLV_ON);
        case CMR_CAN_CLEAR_ERROR:
            return (vsmReq == CMR_CAN_GLV_ON);
        default:
            break;
    }

    return false;
}

/**
 * @brief Handles VSM state up button presses.
 *
 * @param pressed `true` if button is currently pressed.
 */
void stateVSMUpButton(bool pressed) {
    if (!pressed) {
        return;
    }

    cmr_canState_t vsmState = stateGetVSM();
    if (state.vsmReq < vsmState) {
        // Cancel state-down request.
        state.vsmReq = vsmState;
        return;
    }

    cmr_canState_t vsmReq = ((vsmState == CMR_CAN_UNKNOWN) || (vsmState == CMR_CAN_ERROR))
        ? (CMR_CAN_GLV_ON)  // Unknown state; request GLV_ON.
        : (vsmState + 1);        // Increment state.
    if (!stateVSMReqIsValid(vsmState, vsmReq)) {
        return;     // Invalid requested state.
    }

    state.vsmReq = vsmReq;
}

/**
 * @brief Handles VSM state down button presses.
 *
 * @param pressed `true` if button is currently pressed.
 */
void stateVSMDownButton(bool pressed) {
    if (!pressed) {
        return;
    }

    cmr_canState_t vsmState = stateGetVSM();
    if (state.vsmReq > vsmState) {
        // Cancel state-up request.
        state.vsmReq = vsmState;
        return;
    }

    if (
        state.vsmReq == CMR_CAN_RTD &&
        getAverageWheelRPM() > 5
    ) {
        // Only exit RTD when motor is basically stopped.
        return;
    }

    cmr_canState_t vsmReq = vsmState - 1;   // Decrement state.
    if (!stateVSMReqIsValid(vsmState, vsmReq)) {
        return;     // Invalid requested state.
    }

    state.vsmReq = vsmReq;
}

/**
 * @brief Handles gear change button presses.
 *
 * @param pressed `true` if button is currently pressed.
 */
void stateGearButton(bool pressed) {
    if (!pressed) {
        return;
    }

    if (stateGetVSM() != CMR_CAN_HV_EN) {
        return;     // Can only change gears in HV_EN.
    }

    cmr_canGear_t gear = state.gear;
    if (gear != state.gearReq) {
        return;     // Previous gear request not satisfied yet.
    }

    cmr_canGear_t gearReq = gear + 1;
    if (gearReq < CMR_CAN_GEAR_SLOW || gearReq >= CMR_CAN_GEAR_LEN) {
        gearReq = CMR_CAN_GEAR_SLOW;     // Wrap around; skip `GEAR_UNKNOWN` and `GEAR_REVERSE`.
    }

    state.gearReq = gearReq;
}

/**
 * @brief Updates state request to be consistent with VSM state.
 */
void updateReq(void) {
    state.vsmReq = stateGetVSM();
}

/**
 * @brief Updates the gear to be the requested gear.
 */
void stateGearUpdate(void) {
    state.gear = state.gearReq;
}

