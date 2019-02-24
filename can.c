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

#include <FreeRTOS.h>   // FreeRTOS interface
#include <task.h>       // xTaskCreate()

#include "can.h"    // Interface to implement
#include "adc.h"    // adcVSense, adcISense

/**
 * @brief CAN periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canRXMeta[] = {
    // XXX Edit this to include the appropriate periodic messages.
    [CANRX_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    },
    [CANRX_FSM_DATA] = {
        .canID = CMR_CANID_FSM_DATA,
        .timeoutError_ms = 1000,
        .timeoutWarn_ms = 500
    }
};

/** @brief Primary CAN interface. */
static cmr_can_t can;

/** @brief CAN 10 Hz TX priority. */
static const uint32_t canTX10HzPriority = 3;

/** @brief CAN 10 Hz TX period (milliseconds). */
static const TickType_t canTX10HzPeriod_ms = 100;

static cmr_canFanState_t fanStatePTC = CMR_CAN_FAN_OFF;
static uint8_t pumpStatePTC = 0;

/**
 * @brief Task for sending CAN messages at 10 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX10HzTask(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_canPTCCoolingStatus_t coolMsg = {
            .fanState = fanStatePTC,
            .pumpState = pumpStatePTC,
            .preRadiatorTemp_C = adcChannels[ADC_RAD_THERM_1].value,
            .postRadiatorTemp_C = adcChannels[ADC_RAD_THERM_2].value
        };

        cmr_canPTCVoltageDiagnostics_t voltMsg = {
            .logicVoltage_mV = adcChannels[ADC_LOGIC_VSENSE].value,
            .loadVoltage_mV = adcChannels[ADC_POWER_VSENSE].value
        };

        cmr_canPTCCurrentDiagnostics_t ampMsg = {
            .logicCurrent_mA = adcChannels[ADC_LOGIC_ISENSE].value,
            .loadCurrent_mA = adcChannels[ADC_POWER_ISENSE].value,
            .fanCurrent_mA = adcChannels[ADC_FAN_ISENSE].value
        };

        canTX(CMR_CANID_PTC_COOLING_STATUS, &coolMsg, sizeof(coolMsg));
        canTX(CMR_CANID_PTC_VOLTAGE_DIAGNOSTICS, &voltMsg, sizeof(voltMsg));
        canTX(CMR_CANID_PTC_CURRENT_DIAGNOSTICS, &ampMsg, sizeof(ampMsg));

        vTaskDelayUntil(&lastWakeTime, canTX10HzPeriod_ms);
    }
}

/** @brief CAN 100 Hz TX priority. */
static const uint32_t canTX100HzPriority = 5;

/** @brief CAN 100 Hz TX period (milliseconds). */
static const TickType_t canTX100HzPeriod_ms = 10;

/**
 * @brief Task for sending CAN messages at 100 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX100HzTask(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    cmr_canRXMeta_t *heartbeatVSMMeta = canRXMeta + CANRX_HEARTBEAT_VSM;
    volatile cmr_canHeartbeat_t *heartbeatVSM = (void *) heartbeatVSMMeta->payload;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_canHeartbeat_t heartbeat = {
            .state = heartbeatVSM->state
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

        canTX(CMR_CANID_HEARTBEAT_PTC, &heartbeat, sizeof(heartbeat));

        vTaskDelayUntil(&lastWakeTime, canTX100HzPeriod_ms);
    }
}

/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // CAN2 initialization.
    cmr_canInit(
        &can, CAN2,
        canRXMeta, sizeof(canRXMeta) / sizeof(canRXMeta[0]),
        NULL, "canRX",
        GPIOB, GPIO_PIN_12,     // CAN2 RX port/pin.
        GPIOB, GPIO_PIN_13      // CAN2 TX port/pin.
    );

    // CAN2 filters.
    // XXX Change these to whitelist the appropriate IDs.
    const cmr_canFilter_t canFilters[] = {
        {
            .rxFIFO = CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_HEARTBEAT_VSM,
                CMR_CANID_HEARTBEAT_VSM,
                CMR_CANID_HEARTBEAT_VSM,
                CMR_CANID_FSM_DATA
            }
        }
    };
    cmr_canFilter(
        &can, canFilters, sizeof(canFilters) / sizeof(canFilters[0])
    );

    // Task creation.
    xTaskCreate(
        canTX10HzTask, "canTX10Hz",
        configMINIMAL_STACK_SIZE, NULL, canTX10HzPriority, NULL
    );
    xTaskCreate(
        canTX100HzTask, "canTX100Hz",
        configMINIMAL_STACK_SIZE, NULL, canTX100HzPriority, NULL
    );
}

/**
 * @brief Sends a CAN message with the given ID.
 *
 * @param id The ID for the message.
 * @param data The data to send.
 * @param len The data's length, in bytes.
 */
void canTX(cmr_canID_t id, const void *data, size_t len) {
    cmr_canTX(&can, id, data, len);
}

