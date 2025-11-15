/**
 * @file can.h
 * @brief CAN implemenation
 * 
 * @author Jasmine Li
 */

#include "can.h"    // Interface to implement
#include "gpio.h"
#include "adc.h"

/** @brief Primary CAN interface. */
static cmr_can_t can;

/** @brief CAN 200 Hz TX priority. */
static const uint32_t canTX200Hz_priority = 1; // important for controls

/** @brief CAN 200 Hz TX task. */
static cmr_task_t canTX200Hz_task;

/** @brief CAN 200 Hz TX period (milliseconds). */
static const TickType_t canTX200Hz_period_ms = 5;

cmr_canRXMeta_t canRXMeta[] = {
    [CANRX_HEARTBEAT_VSM] = { 
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25 },
    [CANRX_VSM_STATUS] = { 
        .canID = CMR_CANID_VSM_STATUS,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25 }
};
// Forward declarations 
static void sendCIAData();

/**
 * @brief Task for sending CAN messages at 200 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX200Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1) {
      // has all the things you want to send over CAN
        sendCIAData();
        vTaskDelayUntil(&lastWakeTime, canTX200Hz_period_ms);
    }
}

/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // CAN2 initialization.
    cmr_canInit(
        &can, CAN1, CMR_CAN_BITRATE_500K,
        canRXMeta, sizeof(canRXMeta) / sizeof(canRXMeta[0]),
        NULL,
        GPIOA, GPIO_PIN_11,  // RX port/pin.  UPDATE LATER
        GPIOA, GPIO_PIN_12   // CAN1 TX port/pin.
    );

    // CAN2 filters.
    const cmr_canFilter_t canFilters[] = {
        { .isMask = false,
          .rxFIFO = CAN_RX_FIFO0,
          .ids = { // always want 4*n IDs
              CMR_CANID_HEARTBEAT_VSM,
              CMR_CANID_HEARTBEAT_VSM,
              CMR_CANID_HEARTBEAT_VSM,
              CMR_CANID_HEARTBEAT_VSM } },
    };

    cmr_canFilter(
        &can, canFilters, sizeof(canFilters) / sizeof(canFilters[0]));

    // Task initialization.
    cmr_taskInit(
        &canTX200Hz_task,
        "CAN TX 200Hz",
        canTX200Hz_priority,
        canTX200Hz,
        NULL);
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

/**
 * @brief Gets a pointer to a CAN payload.
 *
 * @param msg The desired CAN message.
 *
 * @return Pointer to desired payload.
 */
volatile void *canGetPayload(canRX_t msg) {
    return &(canRXMeta[msg].payload);
}

/**
 * @brief Sets up CIA_Heartbeat, which mirrors VSM Heartbeat, then sends it (everything is through CAN).
 *
 * @param lastWakeTime Pass in from canTX100Hz. Used to update lastStateChangeTime and errors/warnings.
 */
static void sendCIAData() {
	volatile cmr_canCIAData_t CIA_data;

    CIA_data.data[0] = adcRead(ADC_AMP_1);
    CIA_data.data[1] = adcRead(ADC_AMP_2);
    CIA_data.data[2] = adcRead(ADC_AMP_3);
    CIA_data.data[3] = adcRead(ADC_AMP_4);
    CIA_data.data[4] = adcRead(ADC_AMP_5);
    CIA_data.data[5] = adcRead(ADC_THERM);

	canTX(
		CMR_CANID_CIA_DATA,
		&CIA_data,
		sizeof(cmr_canCIAData_t),
		canTX200Hz_period_ms
	);
}
