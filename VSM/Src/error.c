/**
 * @file error.c
 * @brief Vehicle Safety Module error checking.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/gpio.h>   // cmr_gpioRead()
#include <CMR/panic.h>  // cmr_panic()

#include "error.h"      // Interface to implement

#include "can.h"        // cmr_canRXMeta_t, cmr_canState_t, cmr_canHVCHeartbeat, cmr_canHVCMode_t,
                        // canRX_t, canRXMeta[], getPayload(), getModuleState()
#include "gpio.h"       // gpio_t
#include "sensors.h"    // sensorChannel_t, sensors[]

/** @brief Required brake pressure to transition into RTD. */
const uint16_t brakePressureThreshold_PSI = 40;

/**
 * @brief Modules will be considered in the wrong state this many millisec
 * after the last state change.
 */
static const TickType_t badStateThres_ms = 50;

/**
 * @brief If first error has been detected, latch. Reset if in GLV_ON
 */
static bool detectedFirstError = false;

// Forward declarations
static int getBadModuleState(canRX_t module, cmr_canVSMState_t vsmState, TickType_t lastWakeTime);
static bool getASEmergency();

/**
 * @brief Checks for all errors and updates vsmStatus as needed.
 *
 * @param vsmStatus Pointer to VSM status struct to update.
 * @param lastWakeTime Pass in from any FreeRTOS task. Used to check for module timeouts.
 */
void updateCurrentErrors(volatile vsmStatus_t *vsmStatus, TickType_t lastWakeTime) {
    // Prevent preemption
    taskENTER_CRITICAL();

    // Initialize error bit vectors
    cmr_canError_t heartbeatErrors = CMR_CAN_ERROR_NONE;
    uint8_t moduleTimeoutMatrix = CMR_CAN_VSM_ERROR_SOURCE_NONE;
    uint8_t badStateMatrix = CMR_CAN_VSM_ERROR_SOURCE_NONE;
    uint8_t latchMatrix = CMR_CAN_VSM_LATCH_NONE;

    // Check for timeout errors
    for (canRX_t i = 0; i < CANRX_LEN; i++) {
        cmr_canRXMeta_t *rxMeta = &(canRXMeta[i]);

        if (cmr_canRXMetaTimeoutError(rxMeta, lastWakeTime) < 0
            && i != CANRX_RES) {
            heartbeatErrors |= rxMeta->errorFlag;
            moduleTimeoutMatrix |= vsmErrorSourceFlags[i];
            sendFirstError(detectedFirstError, __LINE__);
        }
    }


    // Check for out of range sensors
    cmr_sensorListGetFlags(&sensorList, NULL, &heartbeatErrors);

    // Check for improper states
    if (getBadModuleState(CANRX_HEARTBEAT_HVC, vsmStatus->canVSMStatus.internalState, lastWakeTime) < 0) {
        heartbeatErrors |= CMR_CAN_ERROR_VSM_MODULE_STATE;
        badStateMatrix |= CMR_CAN_VSM_ERROR_SOURCE_HVC;
        sendFirstError(detectedFirstError, __LINE__);
    }

    if (getBadModuleState(CANRX_HEARTBEAT_CDC, vsmStatus->canVSMStatus.internalState, lastWakeTime) < 0) {
        heartbeatErrors |= CMR_CAN_ERROR_VSM_MODULE_STATE;
        badStateMatrix |= CMR_CAN_VSM_ERROR_SOURCE_CDC;
        sendFirstError(detectedFirstError, __LINE__);
    }

    if (getBadModuleState(CANRX_HEARTBEAT_DIM, vsmStatus->canVSMStatus.internalState, lastWakeTime) < 0) {
        heartbeatErrors |= CMR_CAN_ERROR_VSM_MODULE_STATE;
        badStateMatrix |= CMR_CAN_VSM_ERROR_SOURCE_DIM;
        sendFirstError(detectedFirstError, __LINE__);
    }

    // Set software latch in the event of BMS voltage or temperature errors.
    // See rule EV 5.1.10.
    cmr_canHVCHeartbeat_t *hvcHeartbeat = getPayload(CANRX_HEARTBEAT_HVC);
    //cmr_gpioWrite(GPIO_OUT_SOFTWARE_ERR, 1);
    if (/*(cmr_canRXMetaTimeoutError(&(canRXMeta[CANRX_HEARTBEAT_HVC]), lastWakeTime) != 0) not commented out in 24a
     ||*/ (hvcHeartbeat->errorStatus & CMR_CAN_HVC_ERROR_PACK_OVERVOLT)
     || (hvcHeartbeat->errorStatus & CMR_CAN_HVC_ERROR_CELL_OVERVOLT)
     || (hvcHeartbeat->errorStatus & CMR_CAN_HVC_ERROR_CELL_OVERTEMP)) {

        cmr_gpioWrite(GPIO_OUT_SOFTWARE_ERR, 1);
    }
    else if (getASEmergency()){
        cmr_gpioWrite(GPIO_OUT_SOFTWARE_ERR, 1);
    }
    else {
        cmr_gpioWrite(GPIO_OUT_SOFTWARE_ERR, 0);
    }

    // Check all latches
    if (cmr_gpioRead(GPIO_IN_SOFTWARE_ERR) == 1) {
        heartbeatErrors |= CMR_CAN_ERROR_VSM_LATCHED_ERROR;
        latchMatrix |= CMR_CAN_VSM_LATCH_SOFTWARE;
        sendFirstError(detectedFirstError, __LINE__);
    }
    if (cmr_gpioRead(GPIO_IN_IMD_ERR) == 1) {
        heartbeatErrors |= CMR_CAN_ERROR_VSM_LATCHED_ERROR;
        latchMatrix |= CMR_CAN_VSM_LATCH_IMD;
        sendFirstError(detectedFirstError, __LINE__);
    }
    if (cmr_gpioRead(GPIO_IN_BSPD_ERR) == 1) {
        heartbeatErrors |= CMR_CAN_ERROR_VSM_LATCHED_ERROR;
        latchMatrix |= CMR_CAN_VSM_LATCH_BSPD;
        sendFirstError(detectedFirstError, __LINE__);
    }

    // Update vsmErrors struct
    vsmStatus->heartbeatErrors = heartbeatErrors;
    vsmStatus->canVSMStatus.moduleTimeoutMatrix = moduleTimeoutMatrix;
    vsmStatus->canVSMStatus.badStateMatrix = badStateMatrix;
    vsmStatus->canVSMStatus.latchMatrix = latchMatrix;

    // Update latched status.
    vsmStatus->canVSMLatchedStatus.moduleTimeoutMatrix |= moduleTimeoutMatrix;
    vsmStatus->canVSMLatchedStatus.badStateMatrix |= badStateMatrix;
    vsmStatus->canVSMLatchedStatus.latchMatrix |= latchMatrix;

    taskEXIT_CRITICAL();
}

/**
 * @brief Checks if the current DIM request is valid.
 *
 * If a request is rejected, it is latched in the `vsmStatus` struct. To clear
 * the latch, the DIM must stop requesting the rejected state by either timing
 * out the request or requesting a different state.
 *
 * @param vsmStatus Pointer to VSM status struct to update.
 * @param lastWakeTime Pass in from any FreeRTOS task. Used to check for module timeouts.
 *
 * @return `true` if the request is valid; `false` otherwise.
 */
static bool dimRequestIsValid(
    volatile vsmStatus_t *vsmStatus,
    TickType_t lastWakeTime
) {
    cmr_canDIMRequest_t *dimRequest = getPayload(CANRX_DIM_REQUEST);
    cmr_canState_t dimRequestedState = (cmr_canState_t)(dimRequest->requestedState);
    bool dimRequestTimeout = cmr_canRXMetaTimeoutError(
        &(canRXMeta[CANRX_DIM_REQUEST]), lastWakeTime
    );

    if (
        !dimRequestTimeout &&
        dimRequestedState == getCurrentStatus()->dimRequestReject
    ) {
        // Reject request, in case of continued DIM requests for a marked-rejected state
        return false;
    }

    cmr_canFSMData_t *fsmData = getPayload(CANRX_FSM_DATA);
    uint16_t throttlePosition = fsmData->throttlePosition;

    uint32_t brakePressureRear_PSI = cmr_sensorListGetValue(
        &sensorList, SENSOR_CH_BPRES_PSI
    );

    bool valid = true;
    switch (dimRequestedState) {
        case CMR_CAN_RTD:
            if (
                throttlePosition > 0 ||
                brakePressureRear_PSI < brakePressureThreshold_PSI
            ) {
                valid = false;
            }
            break;
        default:
            break;
    }

    vsmStatus->dimRequestReject = (valid)
        ? CMR_CAN_UNKNOWN       // Clear latch.
        : dimRequestedState;    // Latch rejected state.

    return valid;
}

/**
 * @brief Checks for all warnings and updates vsmStatus.heartbeatWarnings as needed.
 *
 * @param vsmStatus Pointer to VSM status struct to update.
 * @param lastWakeTime Pass in from any FreeRTOS task. Used to check for module timeouts.
 */
void updateCurrentWarnings(volatile vsmStatus_t *vsmStatus, TickType_t lastWakeTime) {
    // Prevent preemption
    taskENTER_CRITICAL();

    cmr_canWarn_t heartbeatWarnings = CMR_CAN_WARN_NONE;

    // Check for timeout warnings
    for (canRX_t i = 0; i < CANRX_LEN; i++) {
        cmr_canRXMeta_t *rxMeta = &(canRXMeta[i]);

        if (cmr_canRXMetaTimeoutWarn(rxMeta, lastWakeTime) < 0) {
            heartbeatWarnings |= rxMeta->warnFlag;
            sendFirstError(detectedFirstError, __LINE__);
        }
    }

    // Check for out of range sensors
    cmr_sensorListGetFlags(&sensorList, &heartbeatWarnings, NULL);

    // Check for invalid DIM requests.
    if (!dimRequestIsValid(vsmStatus, lastWakeTime)) {
        heartbeatWarnings |= CMR_CAN_WARN_VSM_DIM_REQ_NAK;
        sendFirstError(detectedFirstError, __LINE__);
    }

    vsmStatus->heartbeatWarnings = heartbeatWarnings;

    taskEXIT_CRITICAL();
}

/**
 * @brief Checks if a module is in the wrong state.
 *
 * @param module The module to check the state of. Must be a value of `CANRX_HEARTBEAT_XXX`
 * from can.h.
 * @param vsmState The current VSM internal state.
 * @param lastWakeTime pass in from updateCurrentErrors. Used to check for state change timeouts.
 *
 * @warning Using a non-heartbeat value of canRX_t will result in an undefined value.
 *
 * @return 0 when module state is correct, and -1 when incorrect. */
static int getBadModuleState(canRX_t module, cmr_canVSMState_t vsmState, TickType_t lastWakeTime) {
    bool wrongState = false;

    // Check HVC mode
    if (module == CANRX_HEARTBEAT_HVC) {
        cmr_canHVCHeartbeat_t *hvcHeartbeat = getPayload(CANRX_HEARTBEAT_HVC);
        cmr_canHVCMode_t hvcMode = hvcHeartbeat->hvcMode;

        switch (vsmState) {
            case CMR_CAN_VSM_STATE_ERROR:   // Fallthrough
            case CMR_CAN_VSM_STATE_CLEAR_ERROR: {
                if ((hvcMode != CMR_CAN_HVC_MODE_IDLE)
                 && (hvcMode != CMR_CAN_HVC_MODE_ERROR)) {
                    sendFirstError(detectedFirstError, __LINE__);
                    wrongState = true;
                }

                break;
            }

            case CMR_CAN_VSM_STATE_GLV_ON: {
                if (hvcMode != CMR_CAN_HVC_MODE_IDLE) {
                    sendFirstError(detectedFirstError, __LINE__);
                    wrongState = true;
                }

                break;
            }
            case CMR_CAN_VSM_STATE_REQ_PRECHARGE: {
                if ((hvcMode != CMR_CAN_HVC_MODE_IDLE)
                 && (hvcMode != CMR_CAN_HVC_MODE_START)) {
                    sendFirstError(detectedFirstError, __LINE__);
                    wrongState = true;
                }

                break;
            }

            case CMR_CAN_VSM_STATE_RUN_BMS: {
                if ((hvcMode != CMR_CAN_HVC_MODE_START)
                 && (hvcMode != CMR_CAN_HVC_MODE_RUN)) {
                    sendFirstError(detectedFirstError, __LINE__);
                    wrongState = true;
                }

                break;
            }

            case CMR_CAN_VSM_STATE_INVERTER_EN: //Fallthrough
            case CMR_CAN_VSM_STATE_HV_EN:
            case CMR_CAN_VSM_STATE_RTD:
            case CMR_CAN_VSM_STATE_AS_READY:
            case CMR_CAN_VSM_STATE_AS_DRIVING:
            case CMR_CAN_VSM_STATE_AS_FINISHED:
            case CMR_CAN_VSM_STATE_AS_EMERGENCY: {
                if (hvcMode != CMR_CAN_HVC_MODE_RUN) {
                    sendFirstError(detectedFirstError, __LINE__);
                    wrongState = true;
                }

                break;
            }

            default: {
                sendFirstError(detectedFirstError, __LINE__);
                cmr_panic("Invalid VSM state");
                break;
            }
        }
    }

    // Directly check state of any other module
    else {
        cmr_canState_t expectedState = vsmToCANState[vsmState];
        cmr_canState_t moduleState = getModuleState(module);

        if (moduleState != expectedState) {
            wrongState = true;
        }
    }

    // Module in wrong state and it has been more than 'badStateThres_ms'
    // millisec since last state change
    if ((lastWakeTime > lastStateChangeTime_ms + badStateThres_ms)
     && (wrongState)) {
        return -1;
    }

    return 0;
}

/**
 * @brief Check if Autonomous Systems are in emergency state*/

static bool getASEmergency(){
    return false;
}

/**
 * @brief Check all inverters if endurance mode. Else, check RR inverter*/

static bool checkInverters(){
    // TODO: change back before comp so that don't unnecessarily error out
    cmr_canState_t dimRequestedGear = (cmr_canState_t)(dimRequest->requestedGear);
    if (dimRequestedGear == CMR_CAN_GEAR_ENDURANCE){
        if (((amk1Actual->status_bv & CMR_CAN_AMK_STATUS_SYSTEM_READY) &&
            !cmr_canRXMetaTimeoutError(&(canRXMeta[CANRX_INVERTER_1]), lastWakeTime_ms)) ||
        ((amk2Actual->status_bv & CMR_CAN_AMK_STATUS_SYSTEM_READY) &&
            !cmr_canRXMetaTimeoutError(&(canRXMeta[CANRX_INVERTER_2]), lastWakeTime_ms)) ||
        ((amk3Actual->status_bv & CMR_CAN_AMK_STATUS_SYSTEM_READY) &&
            !cmr_canRXMetaTimeoutError(&(canRXMeta[CANRX_INVERTER_3]), lastWakeTime_ms)) ||
        ((amk4Actual->status_bv & CMR_CAN_AMK_STATUS_SYSTEM_READY) &&
            !cmr_canRXMetaTimeoutError(&(canRXMeta[CANRX_INVERTER_4]), lastWakeTime_ms))){
            return true;
        }
        else{
            sendFirstError(detectedFirstError, __LINE__);
            return false;
        }
    }
    else{
        if (((amk1Actual->status_bv & CMR_CAN_AMK_STATUS_SYSTEM_READY) &&
            !cmr_canRXMetaTimeoutError(&(canRXMeta[CANRX_INVERTER_1]), lastWakeTime_ms))){
                return true
        }
        else{
            sendFirstError(detectedFirstError, __LINE__);
            return false;
        }
    }
}