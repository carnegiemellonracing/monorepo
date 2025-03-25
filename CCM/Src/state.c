/**
 * @file state.c
 * @brief Charger Control Module state machine.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/tasks.h>      // Task interface, taskENTER_CRITICAL(), taskEXIT_CRITICAL()
#include <CMR/gpio.h>   // GPIO interface
#include <stdbool.h>

#include "charger.h"
#include "evse.h"
#include "gpio.h"
#include "sensors.h"
#include "state.h"

/** @brief State update priority. */
static const uint32_t stateUpdate_priority = 3;

/** @brief State update period. */
static const TickType_t stateUpdate_period_ms = 10;

/** @brief State update task. */
static cmr_task_t stateUpdate_task;

volatile cmr_CCMState_t state = CMR_CCM_STATE_CLEAR_ERROR;

static cmr_CCMCommand_t requestedCommand = CMR_CCM_COMMAND_NONE;

volatile cmr_canHVCMode_t hvcModeRequest = CMR_CAN_HVC_MODE_ERROR;

bool isChargerErrored(cmr_canRXMeta_t *metaChargerState) {
    if (cmr_canRXMetaTimeoutError(metaChargerState, xTaskGetTickCount()) < 0) {
        return true;
    }

    cmr_canDilongState_t *chargerState = (void *) metaChargerState->payload;

    return (chargerState->status_bv & (CMR_CAN_DILONG_STATUS_HARDWARE_FAILURE
                                     | CMR_CAN_DILONG_STATUS_CHARGER_OVERHEAT
                                     | CMR_CAN_DILONG_STATUS_INPUT_VOLTAGE
                                     | CMR_CAN_DILONG_STATUS_COMM_TIMEOUT));
}

uint16_t getChargerCurrent(cmr_canRXMeta_t *metaChargerState) {
    volatile cmr_canDilongState_t *chargerState = (void *) metaChargerState->payload;

    return (chargerState->outputCurrentHigh << 8) | (chargerState->outputCurrentLow);
}

/**
 * @brief Gets the next state based on the current state.
 *
 * @param lastWakeTime_ms Pass in from stateUpdate.
 * Used to update errors and warnings.
 *
 * @return Next CCM state.
 */
static cmr_CCMState_t getNextState(TickType_t lastWakeTime_ms) {
    // Nothing is wrong, default nextState to error and begin state transition logic
    cmr_CCMState_t nextState = CMR_CCM_STATE_ERROR;

    cmr_canRXMeta_t *metaChargerOneState = canChargerOneRXMeta + CANRX_CHARGER_ONE_STATE;
    cmr_canRXMeta_t *metaChargerTwoState = canChargerTwoRXMeta + CANRX_CHARGER_TWO_STATE;


    volatile cmr_canHVCHeartbeat_t *canHVCHeartbeat = canGetPayload(CANRX_HVC_HEARTBEAT);
    uint8_t hvcState = canHVCHeartbeat->hvcState;
    uint8_t hvcMode = canHVCHeartbeat->hvcMode;

    volatile cmr_canCCMCommand_t *canCCMCommand = canGetPayload(CANRX_CCM_COMMAND);

    taskENTER_CRITICAL();

    switch (state) {
        case CMR_CCM_STATE_ERROR: {
            if (requestedCommand == CMR_CCM_COMMAND_RESET || canCCMCommand->command == CMR_CAN_CCM_MODE_RESET) {
                // TODO: Check if any error states are still occurring
                nextState = CMR_CCM_STATE_CLEAR_ERROR;
            } else {
                nextState = CMR_CCM_STATE_ERROR;
            }

            break;
        }

        case CMR_CCM_STATE_CLEAR_ERROR: {
            // TODO: Conditionally clear error?
            nextState = CMR_CCM_STATE_CLEAR_HVC;

            break;
        }

        case CMR_CCM_STATE_CLEAR_HVC: {
            if (hvcState == CMR_CAN_HVC_STATE_CLEAR_ERROR) {
                nextState = CMR_CCM_STATE_IDLE_HVC;
            } else {
                nextState = CMR_CCM_STATE_CLEAR_HVC;
            }
            break;
        }

        case CMR_CCM_STATE_IDLE_HVC: {
            if (hvcState == CMR_CAN_HVC_STATE_ERROR){
                nextState = CMR_CCM_STATE_ERROR;
            } else if (hvcMode == CMR_CAN_HVC_MODE_IDLE &&
                hvcState == CMR_CAN_HVC_STATE_STANDBY) {
                nextState = CMR_CCM_STATE_STANDBY;
            } else {
                nextState = CMR_CCM_STATE_IDLE_HVC;
            }
            break;
        }

        case CMR_CCM_STATE_STANDBY: {
            if (hvcState == CMR_CAN_HVC_STATE_ERROR){
                nextState = CMR_CCM_STATE_ERROR;
            } else if (/*(getEvseState(pilotVoltage) == EVSE_READY) &&*/
                ((requestedCommand == CMR_CCM_COMMAND_SLOW) ||
                 (requestedCommand == CMR_CCM_COMMAND_FAST) ||
                 (canCCMCommand->command == CMR_CAN_CCM_MODE_RUN))) {
                nextState = CMR_CCM_STATE_CHARGE_REQ;
            } else {
                nextState = CMR_CCM_STATE_STANDBY;
            }
            break;
        }

        case CMR_CCM_STATE_CHARGE_REQ: {
            if (hvcState == CMR_CAN_HVC_STATE_ERROR){
                nextState = CMR_CCM_STATE_ERROR;
            } else if (hvcState == CMR_CAN_HVC_STATE_CHARGE_TRICKLE) {
                nextState = CMR_CCM_STATE_CHARGE;
            } else {
                nextState = CMR_CCM_STATE_CHARGE_REQ;
            }
            break;
        }

        case CMR_CCM_STATE_CHARGE: {
            if (hvcState == CMR_CAN_HVC_STATE_ERROR){
                nextState = CMR_CCM_STATE_ERROR;
            } else if ((requestedCommand == CMR_CCM_COMMAND_OFF) ||
                (canCCMCommand->command == CMR_CAN_CCM_MODE_IDLE)) {
                nextState = CMR_CCM_STATE_SHUTDOWN;
            } else if (hvcState == CMR_CAN_HVC_STATE_CHARGE_CONSTANT_VOLTAGE) {
                nextState = CMR_CCM_STATE_SLOW_CHARGE;
            } else {
                nextState = CMR_CCM_STATE_CHARGE;
            }
            break;
        }

        case CMR_CCM_STATE_SLOW_CHARGE: {
            if (hvcState == CMR_CAN_HVC_STATE_ERROR){
                nextState = CMR_CCM_STATE_ERROR;
            } else if (hvcState == CMR_CAN_HVC_STATE_DISCHARGE ||
                hvcState == CMR_CAN_HVC_STATE_STANDBY ||
                (requestedCommand == CMR_CCM_COMMAND_OFF) ||
                (canCCMCommand->command == CMR_CAN_CCM_MODE_IDLE)) {
                nextState = CMR_CCM_STATE_STANDBY ;
            }
            else {
                nextState = CMR_CCM_STATE_SLOW_CHARGE;
            }
            break;
        }

        case CMR_CCM_STATE_SHUTDOWN: {
            if (hvcState == CMR_CAN_HVC_STATE_ERROR){
                nextState = CMR_CCM_STATE_ERROR;
            } else if (getChargerCurrent(metaChargerOneState) == 0 && getChargerCurrent(metaChargerTwoState) == 0) {
                nextState = CMR_CCM_STATE_STANDBY;
            } else {
                nextState = CMR_CCM_STATE_SHUTDOWN;
            }

            break;
        }

        default: {
            nextState = CMR_CCM_STATE_ERROR;
            break;
        }
    }

    // Clear command
    requestedCommand = CMR_CCM_COMMAND_NONE;

    taskEXIT_CRITICAL();

    return nextState;
}

static void setStateOutputs() {
    updateChargerCommands(state);
    switch (state) {
    case CMR_CCM_STATE_CLEAR_HVC:
        hvcModeRequest = CMR_CAN_HVC_MODE_ERROR;
        break;
    case CMR_CCM_STATE_IDLE_HVC:
        hvcModeRequest = CMR_CAN_HVC_MODE_IDLE;
        break;
    case CMR_CCM_STATE_CHARGE_REQ:
        hvcModeRequest = CMR_CAN_HVC_MODE_CHARGE;
        break;
    case CMR_CCM_STATE_CHARGE:
        hvcModeRequest = CMR_CAN_HVC_MODE_CHARGE;
        break;
    case CMR_CCM_STATE_SHUTDOWN:
        hvcModeRequest = CMR_CAN_HVC_MODE_CHARGE;
        break;
    default:
        hvcModeRequest = CMR_CAN_HVC_MODE_IDLE;
    }
    if (state != CMR_CCM_STATE_ERROR) {
    	cmr_gpioWrite(GPIO_CHARGE_ENABLE, 1);
    } else {
    	cmr_gpioWrite(GPIO_CHARGE_ENABLE, 0);
    }
}

/**
 * @brief State update task.
 */
static void stateUpdate(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    cmr_canVSMState_t lastState = state;

    TickType_t lastWakeTime_ms = xTaskGetTickCount();
    while (1) {
        cmr_canVSMState_t nextState = getNextState(lastWakeTime_ms);

        if (nextState != lastState) {
            lastState = state;
        }

        state = nextState;

        setStateOutputs();

        vTaskDelayUntil(&lastWakeTime_ms, stateUpdate_period_ms);
    }
}

void setCommand(cmr_CCMCommand_t command) {
    if (command == CMR_CCM_COMMAND_RESET) {
        /*
         * Ignore requested command if we're not in an error state.
         */
        if (state != CMR_CCM_STATE_ERROR) {
            command = requestedCommand;
        }
    }

    requestedCommand = command;
}

/**
 * @brief Initializes the state machine.
 */
void stateInit(void) {
    cmr_taskInit(
        &stateUpdate_task,
        "State update",
        stateUpdate_priority,
        stateUpdate,
        NULL
    );
}
