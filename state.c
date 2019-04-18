/**
 * @file state.c
 * @brief DIM state implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "state.h"  // interface to implement

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
 * @brief Checks if the requested VSM state is allowed.
 *
 * @param vsm The current VSM state.
 * @param vsmReq The requested VSM state.
 */
static bool stateVSMReqIsValid(cmr_canState_t vsm, cmr_canState_t vsmReq) {
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
            return (vsmReq == CMR_CAN_HV_EN) ||
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

    cmr_canState_t vsm = stateGetVSM();
    if (state.vsmReq < vsm) {
        // Cancel state-down request.
        state.vsmReq = vsm;
        return;
    }

    cmr_canState_t vsmReq = (vsm == CMR_CAN_UNKNOWN)
        ? (CMR_CAN_GLV_ON)  // Unknown state; request GLV_ON.
        : (vsm + 1);        // Increment state.
    if (!stateVSMReqIsValid(vsm, vsmReq)) {
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

    cmr_canState_t vsm = stateGetVSM();
    if (state.vsmReq > vsm) {
        // Cancel state-up request.
        state.vsmReq = vsm;
        return;
    }

    cmr_canState_t vsmReq = vsm - 1;   // Decrement state.
    if (!stateVSMReqIsValid(vsm, vsmReq)) {
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
    if (gearReq >= CMR_CAN_GEAR_LEN) {
        gearReq = CMR_CAN_GEAR_REVERSE;     // Wrap around; skip `GEAR_UNKNOWN`.
    }

    state.gearReq = gearReq;
}

/**
 * @brief Updates the gear to be the requested gear.
 */
void stateGearUpdate(void) {
    state.gear = state.gearReq;
}

