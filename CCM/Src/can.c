/**
 * @file can.c
 * @brief Board-specific CAN implementation.
 *
 * Adding a new periodic message struct:
 *
 * 1. Add the corresponding index to the `canRX_t` enum in `can.h`.
 * 2. Add a configuration entry in `canRXMeta` at that index.
 * 3. Access the message using `canRXMeta[index]`.
 *
 * @author Carnegie Mellon Racing
 */

#include <string.h>     // memcpy()

#include <CMR/tasks.h>  // Task interface

#include "can.h"     // Interface to implement
#include "adc.h"     // adcVSense, adcISense
#include "charger.h" // getChargerCommand
#include "state.h"

// extern volatile cmr_CCMState_t state;

/**
 * @brief Vehicle CAN periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canVehicleRXMeta[] = {
    // XXX Edit this to include the appropriate periodic messages.
    [CANRX_HVC_HEARTBEAT] = {
        .canID = CMR_CANID_HEARTBEAT_HVC,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_HVC_PACK_VOLTAGE] = {
        .canID = CMR_CANID_HVC_PACK_VOLTAGE,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_HVC_CELL_TEMPS] = {
        .canID = CMR_CANID_HVC_MINMAX_CELL_TEMPS,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1500
    },
    [CANRX_HVC_CELL_VOLTAGE] = {
        .canID = CMR_CANID_HVC_MINMAX_CELL_VOLTAGE,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 50
    },
    [CANRX_CCM_COMMAND] = {
        .canID = CMR_CANID_CCM_COMMAND,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    }
};

/**
 * @brief Charger 1 periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canChargerOneRXMeta[] = {
    // XXX Edit this to include the appropriate periodic messages.
    [CANRX_CHARGER_ONE_STATE] = {
        .canID = CMR_CANID_DILONG_STATE,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1500
    }
};

/**
 * @brief Charger 2 periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canChargerTwoRXMeta[] = {
    // XXX Edit this to include the appropriate periodic messages.
    [CANRX_CHARGER_TWO_STATE] = {
        .canID = CMR_CANID_DILONG_STATE,
        .timeoutError_ms = 2000,
        .timeoutWarn_ms = 1500
    }
};

/** @brief Primary CAN interface. */
static cmr_can_t canVehicle;

/** @brief Charger One CAN interface. */
static cmr_can_t canChargerOne;

/** @brief Charger Two CAN interface. */
static cmr_can_t canChargerTwo;

/** @brief CAN 1 Hz TX priority. */
static const uint32_t canTX1Hz_priority = 3;

/** @brief CAN 1 Hz TX period (milliseconds). */
static const TickType_t canTX1Hz_period_ms = 1000;

/** @brief CAN 1 Hz TX task. */
static cmr_task_t canTX1Hz_task;

/**
 * @brief Task for sending CAN messages at 1 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX1Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();

    const cmr_canDilongCommand_t *chargerOneCommand = getChargerCommand(CHARGER_ONE);
    const cmr_canDilongCommand_t *chargerTwoCommand = getChargerCommand(CHARGER_TWO);

    while (1) {
        canChargerOneTX(CMR_CANID_DILONG_COMMAND, chargerOneCommand, sizeof(*chargerOneCommand), canTX1Hz_period_ms);
        canChargerTwoTX(CMR_CANID_DILONG_COMMAND, chargerTwoCommand, sizeof(*chargerTwoCommand), canTX1Hz_period_ms);

        cmr_canHeartbeat_t CCM_Heartbeat = {
            .state = state
        };
        canVehicleTX(CMR_CANID_HEARTBEAT_FSM, &CCM_Heartbeat, sizeof(CCM_Heartbeat), canTX1Hz_period_ms);
        // TODO: Consider sending CCM heartbeat, although there's nobody that even cares about
        //       CCM heartbeat :( so this may not be necessary
        vTaskDelayUntil(&lastWakeTime, canTX1Hz_period_ms);
    }
}

/** @brief CAN 100 Hz TX priority. */
static const uint32_t canTX100Hz_priority = 5;

/** @brief CAN 100 Hz TX period (milliseconds). */
static const TickType_t canTX100Hz_period_ms = 10;

/** @brief CAN 100 Hz TX task. */
static cmr_task_t canTX100Hz_task;

/**
 * @brief Task for sending CAN messages at 100 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX100Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1) {
        cmr_canHVCCommand_t hvcCommand = {
            .modeRequest = hvcModeRequest
        };

        canVehicleTX(CMR_CANID_HVC_COMMAND, &hvcCommand, sizeof(hvcCommand), canTX100Hz_period_ms);

        vTaskDelayUntil(&lastWakeTime, canTX100Hz_period_ms);
    }
}

/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // CAN1 - Charger One CAN initialization.
    cmr_canInit(
        &canChargerOne, CAN1,
        CMR_CAN_BITRATE_500K,
        canChargerOneRXMeta, sizeof(canChargerOneRXMeta) / sizeof(canChargerOneRXMeta[0]),
        NULL,
        GPIOB, GPIO_PIN_8,     // CAN1 RX port/pin.
        GPIOB, GPIO_PIN_9      // CAN1 TX port/pin.
    );

    // CAN2 - Charger Two CAN initialization.
    cmr_canInit(
        &canChargerTwo, CAN2,
        CMR_CAN_BITRATE_500K,
        canChargerTwoRXMeta, sizeof(canChargerTwoRXMeta) / sizeof(canChargerTwoRXMeta[0]),
        NULL,
        GPIOB, GPIO_PIN_12,     // CAN2 RX port/pin.
        GPIOB, GPIO_PIN_13      // CAN2 TX port/pin.
    );

    // CAN3 - Vehicle CAN initialization.
    cmr_canInit(
        &canVehicle, CAN3,
        CMR_CAN_BITRATE_500K,
        canVehicleRXMeta, sizeof(canVehicleRXMeta) / sizeof(canVehicleRXMeta[0]),
        NULL,
        GPIOA, GPIO_PIN_8,     // CAN3 RX port/pin.
        GPIOB, GPIO_PIN_4      // CAN3 TX port/pin.
    );

    // CAN1 filters.
    const cmr_canFilter_t canChargerOneFilters[] = {
        {
            .isMask = false,
            .isExtended = true,
            .rxFIFO = CAN_RX_FIFO1,
            .ids = {
                .extended = {
                    CMR_CANID_DILONG_STATE,
                    CMR_CANID_DILONG_STATE
                }
            }
        }
    };

    cmr_canFilter(
        &canChargerOne, canChargerOneFilters, sizeof(canChargerOneFilters) / sizeof(canChargerOneFilters[0])
    );



    // CAN2 filters.
    const cmr_canFilter_t canChargerTwoFilters[] = {
        {
            .isMask = false,
            .isExtended = true,
            .rxFIFO = CAN_RX_FIFO1,
            .ids = {
                .extended = {
                    CMR_CANID_DILONG_STATE,
                    CMR_CANID_DILONG_STATE
                }
            }
        }
    };

    cmr_canFilter(
        &canChargerTwo, canChargerTwoFilters, sizeof(canChargerTwoFilters) / sizeof(canChargerTwoFilters[0])
    );

    // CAN3 filters.
    // XXX Change these to whitelist the appropriate IDs.
    const cmr_canFilter_t canVehicleFilters[] = {
        {
            .isMask = false,
            .isExtended = false,
            .rxFIFO = CAN_RX_FIFO0,
            .ids = {
                .standard = {
                    CMR_CANID_HEARTBEAT_HVC,
                    CMR_CANID_HVC_PACK_VOLTAGE, // TODO: Should these be in the same one, etc.
                    CMR_CANID_HVC_MINMAX_CELL_TEMPS,
                    CMR_CANID_HEARTBEAT_HVC,
                }
            }
        },
        {
            .isMask = false,
            .isExtended = false,
            .rxFIFO = CAN_RX_FIFO0,
            .ids = {
                .standard = {
                    CMR_CANID_CCM_COMMAND,
                    CMR_CANID_HVC_MINMAX_CELL_VOLTAGE,
                    CMR_CANID_CCM_COMMAND,
                    CMR_CANID_CCM_COMMAND,
                }
            }
        }
    };

    cmr_canFilter(
        &canVehicle, canVehicleFilters, sizeof(canVehicleFilters) / sizeof(canVehicleFilters[0])
    );

    // Task initialization.
    cmr_taskInit(
        &canTX1Hz_task,
        "CAN TX 1Hz",
        canTX1Hz_priority,
        canTX1Hz,
        NULL
    );
    cmr_taskInit(
        &canTX100Hz_task,
        "CAN TX 100Hz",
        canTX100Hz_priority,
        canTX100Hz,
        NULL
    );
}

/**
 * @brief Sends a CAN message with the given ID over vehicle CAN.
 *
 * @param id The ID for the message.
 * @param data The data to send.
 * @param len The data's length, in bytes.
 * @param timeout The timeout, in ticks.
 *
 * @return 0 on success, or a negative error code on timeout.
 */
int canVehicleTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout) {
    return cmr_canTX(&canVehicle, id, false, data, len, timeout);
}

/**
 * @brief Sends a CAN message with the given ID over charger one CAN.
 *
 * @param id The ID for the message.
 * @param data The data to send.
 * @param len The data's length, in bytes.
 * @param timeout The timeout, in ticks.
 *
 * @return 0 on success, or a negative error code on timeout.
 */
int canChargerOneTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout) {
    return cmr_canTX(&canChargerOne, id, true, data, len, timeout);
}

/**
 * @brief Sends a CAN message with the given ID over vehicle CAN.
 *
 * @param id The ID for the message.
 * @param data The data to send.
 * @param len The data's length, in bytes.
 * @param timeout The timeout, in ticks.
 *
 * @return 0 on success, or a negative error code on timeout.
 */
int canChargerTwoTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout) {
    return cmr_canTX(&canChargerTwo, id, true, data, len, timeout);
}

