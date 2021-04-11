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

#include "can.h"    // Interface to implement
#include "adc.h"    // adcVSense, adcISense
#include "state.h"  // State interface
#include "tftDL.h"  // For RAM buffer indices

/**
 * @brief CAN periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canRXMeta[] = {
    [CANRX_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_HVC_PACK_VOLTAGE] = {
        .canID = CMR_CANID_HVC_PACK_VOLTAGE,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_AMK_FL_ACT_1] = {
        .canID = CMR_CANID_AMK_1_ACT_1,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_AMK_FR_ACT_1] = {
        .canID = CMR_CANID_AMK_2_ACT_1,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_AMK_RL_ACT_1] = {
        .canID = CMR_CANID_AMK_3_ACT_1,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_AMK_RR_ACT_1] = {
        .canID = CMR_CANID_AMK_4_ACT_1,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_HVC_PACK_TEMPS] = {
        .canID = CMR_CANID_HVC_MINMAX_CELL_TEMPS,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_VSM_STATUS] = {
        .canID = CMR_CANID_VSM_STATUS,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_CDC_MOTOR_TEMPS] = {
        .canID = CMR_CANID_CDC_MOTOR_TEMPS,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_PTCf_LOOP_A_TEMPS] = {
        .canID = CMR_CANID_PTCf_LOOP_TEMPS_A,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_PTCf_LOOP_B_TEMPS] = {
        .canID = CMR_CANID_PTCf_LOOP_TEMPS_B,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_PTCp_LOOP_A_TEMPS] = {
        .canID = CMR_CANID_PTCp_LOOP_TEMPS_A,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_PTCp_LOOP_B_TEMPS] = {
        .canID = CMR_CANID_PTCp_LOOP_TEMPS_B,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_HVC_HEARTBEAT] = {
        .canID = CMR_CANID_HEARTBEAT_HVC,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_CDC_MOTOR_FAULTS] = {
        .canID = CMR_CANID_CDC_MOTOR_FAULTS,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_CDL_BROADCAST] = {
        .canID = CMR_CANID_CDL_BROADCAST,
        .timeoutError_ms = 4000,
        .timeoutWarn_ms = 2000
    },
    [CANRX_SBG_STATUS_3] = {
        .canID = CMR_CANID_SBG_STATUS_3,
        .timeoutError_ms = 800,
        .timeoutWarn_ms = 400
    }
};

/** @brief Primary CAN interface. */
static cmr_can_t can;

/** @brief CAN 10 Hz TX priority. */
static const uint32_t canTX10Hz_priority = 3;

/** @brief CAN 10 Hz TX period (milliseconds). */
static const TickType_t canTX10Hz_period_ms = 100;

/** @brief CAN 10 Hz TX task. */
static cmr_task_t canTX10Hz_task;

/**
 * @brief Task for sending CAN messages at 10 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX10Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_canDIMPowerDiagnostics_t powerDiagnostics = {
            .busVoltage_mV = adcRead(ADC_VSENSE) * 8 * 11 / 10, // TODO: figure out where 8, 10 come from
            .busCurrent_mA = adcRead(ADC_ISENSE) * 8 / 20 / 10 // TODO: figure out where 8, 10 come from
        };

        canTX(
            CMR_CANID_DIM_POWER_DIAGNOSTICS,
            &powerDiagnostics, sizeof(powerDiagnostics),
            canTX10Hz_period_ms
        );

        cmr_canState_t stateVSM = stateGetVSM();
        cmr_canState_t stateVSMReq = stateGetVSMReq();
        cmr_canGear_t gear = stateGetGear();
        cmr_canGear_t gearReq = stateGetGearReq();

        /* if DIM is requesting a state/gear change
         * send this request to VSM */
        if (
            (stateVSM != stateVSMReq) ||
            (gear != gearReq)
        ) {
            cmr_canDIMRequest_t request = {
                .requestedState = stateVSMReq,
                .requestedGear = gearReq
            };
            canTX(
                CMR_CANID_DIM_REQUEST,
                &request, sizeof(request),
                canTX10Hz_period_ms
            );

            stateGearUpdate();
        }

        vTaskDelayUntil(&lastWakeTime, canTX10Hz_period_ms);
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

    cmr_canRXMeta_t *heartbeatVSMMeta = canRXMeta + CANRX_HEARTBEAT_VSM;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_canState_t vsmState = stateGetVSM();
        cmr_canHeartbeat_t heartbeat = {
            .state = vsmState
        };

        uint16_t error = CMR_CAN_ERROR_NONE;
        if (cmr_canRXMetaTimeoutError(heartbeatVSMMeta, lastWakeTime) < 0) {
            error |= CMR_CAN_ERROR_VSM_TIMEOUT;
        }
        memcpy(&heartbeat.error, &error, sizeof(error));

        uint16_t warning = CMR_CAN_WARN_NONE;
        if (cmr_canRXMetaTimeoutWarn(heartbeatVSMMeta, lastWakeTime) < 0) {
            warning |= CMR_CAN_WARN_VSM_TIMEOUT;
        }
        memcpy(&heartbeat.warning, &warning, sizeof(warning));

        canTX(
            CMR_CANID_HEARTBEAT_DIM,
            &heartbeat,
            sizeof(heartbeat),
            canTX100Hz_period_ms
        );

        vTaskDelayUntil(&lastWakeTime, canTX100Hz_period_ms);
    }
}

/**
 * @brief Callback for the RAM messages to the DIM to write to the screen
 * 
 * For displaying from the RAM buffer, character indices 0-20 inclusive are for the
 * message at the top. This corresponds to text->address 0, 1, 2, 3, and 4.
 * Character indices 40-52 inclusive are for the first note on the right side of the screen.
 * This corresponds to text->address 0x0A, 0x0B, and 0x0C.
 * Character indices 60-72 inclusive are for the second note on the right side of the screen.
 * This corresponds to text->address 0x0F, 0x10, and 0x11.
 * Character indices 80-92 inclusive are for the third note on the right side of the screen.
 * This corresponds to text->address 0x14, 0x15, and 0x16.
 */
void ramRxCallback (cmr_can_t *can, uint16_t canID, const void *data, size_t dataLen) {
    if (canID == CMR_CANID_DIM_TEXT_WRITE) {
        cmr_canDIMTextWrite_t *text = (cmr_canDIMTextWrite_t *)data;
        if (dataLen == sizeof(cmr_canDIMTextWrite_t)) {
            uint16_t index = ((uint16_t)text->address) << 2;
            if (index < RAMBUFLEN) {
                memcpy(RAMBUF + index, &(text->data), 4);
                
                /* 
                 * Replace all null terminators with spaces - otherwise screen
                 * doesn't display any characters after a null terminator is found
                 * clearing is so that if found null terminator within in section of text,
                 * all characters after null terminator will also be cleared
                 */
                bool clearing = false;
                for(uint16_t i = 0; i < RAMBUFLEN; i++) {
                    if (i == NOTE1_INDEX || i == NOTE2_INDEX || i == NOTE3_INDEX) {
                        clearing = false;
                    }
                    if (RAMBUF[i] == '\0' || clearing) {
                        clearing = true;
                        RAMBUF[i] = ' ';
                    }
                }
            }
        }
    }
}

/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // CAN2 initialization.
    cmr_canInit(
        &can, CAN2, CMR_CAN_BITRATE_500K,
        canRXMeta, sizeof(canRXMeta) / sizeof(canRXMeta[0]),
        &ramRxCallback,
        GPIOB, GPIO_PIN_12,     // CAN2 RX port/pin.
        GPIOB, GPIO_PIN_13      // CAN2 TX port/pin.
    );

    // Clear RAM Buf - Set all to Spaces
    for(uint16_t i = 0; i < RAMBUFLEN; i++) {
        RAMBUF[i] = ' ';
    }

    // CAN2 filters.
    const cmr_canFilter_t canFilters[] = {
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_HEARTBEAT_VSM,
                CMR_CANID_HVC_PACK_VOLTAGE,
                CMR_CANID_CDC_WHEEL_SPEEDS,
                CMR_CANID_CDC_MOTOR_DATA
            }
        },
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO1,
            .ids = {
                CMR_CANID_SBG_STATUS_3,
                CMR_CANID_DIM_TEXT_WRITE,
                CMR_CANID_PTCf_LOOP_TEMPS_B,
                CMR_CANID_PTCp_LOOP_TEMPS_A
            }
        },
        {
            .isMask = false,
            .rxFIFO= CAN_RX_FIFO1,
            .ids = {
                CMR_CANID_AFC1_DRIVER_TEMPS,
                CMR_CANID_HVC_MINMAX_CELL_TEMPS,
                CMR_CANID_VSM_STATUS,
                CMR_CANID_CDL_BROADCAST
            }
        },
        {
            .isMask = false,
            .rxFIFO= CAN_RX_FIFO1,
            .ids = {
                CMR_CANID_CDC_MOTOR_TEMPS,
                CMR_CANID_PTCf_LOOP_TEMPS_A,
                CMR_CANID_HEARTBEAT_HVC,
                CMR_CANID_CDC_MOTOR_FAULTS
            }
        },
        {
            .isMask = false,
            .rxFIFO= CAN_RX_FIFO1,
            .ids = {
                CMR_CANID_PTCp_LOOP_TEMPS_B,
                CMR_CANID_PTCp_LOOP_TEMPS_B,
                CMR_CANID_PTCp_LOOP_TEMPS_B,
                CMR_CANID_PTCp_LOOP_TEMPS_B
            }
        }
    };
    cmr_canFilter(
        &can, canFilters, sizeof(canFilters) / sizeof(canFilters[0])
    );

    // Task initialization.
    cmr_taskInit(
        &canTX10Hz_task,
        "CAN TX 10Hz",
        canTX10Hz_priority,
        canTX10Hz,
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
 * @brief Sends a CAN message with the given ID.
 *
 * @param id The ID for the message.
 * @param data The data to send.
 * @param len The data's length, in bytes.
 * @param timeout The timeout, in ticks.
 *
 * @return 0 on success, or a negative error code on timeout.
 */
int canTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout) {
    return cmr_canTX(&can, id, data, len, timeout);
}
