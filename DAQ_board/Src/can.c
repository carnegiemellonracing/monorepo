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

#include <CMR/tasks.h>  // Task interface

#include "can.h"        // Interface to implement
#include "adc.h"        // adcRead()
#include "mlx90640.h"   // mlx90640GetRow()

/** @brief Thermal image row forwarded onto CAN. */
#define TIRETEMP_ROW    12
/** @brief CAN ID for the forwarded row. */
#define TIRETEMP_CANID  CMR_CANID_DAQ_0_TIRETEMP

/**
 * @brief CAN periodic message receive metadata
 *
 * @note Indexed by `canRX_t`.
 */
cmr_canRXMeta_t canRXMeta[] = {
    [CANRX_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 100,
        .timeoutWarn_ms = 25,
    },
    [CANRX_IZZE_LOADCELL] = {
        .canID = CMR_IZZIE_LOADCELL,
        .timeoutError_ms = 500,
        .timeoutWarn_ms = 250,
    },
};

/** @brief CAN 10 Hz TX priority. */
static const uint32_t canTX10Hz_priority = 3;
/** @brief CAN 10 Hz TX period (milliseconds). */
static const TickType_t canTX10Hz_period_ms = 100;

/** @brief CAN 100 Hz TX priority. */
static const uint32_t canTX100Hz_priority = 5;
/** @brief CAN 100 Hz TX period (milliseconds). */
static const TickType_t canTX100Hz_period_ms = 10;

/** @brief CAN 10 Hz TX task. */
static cmr_task_t canTX10Hz_task;
/** @brief CAN 100 Hz TX task. */
static cmr_task_t canTX100Hz_task;

/** @brief Primary CAN interface. */
static cmr_can_t can;

// Forward declarations
static void sendHeartbeat(void);
static void sendThermistors(void);
static void sendTireTemp(void);

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
        sendThermistors();
        sendTireTemp();

        vTaskDelayUntil(&lastWakeTime, canTX10Hz_period_ms);
    }
}

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
        sendHeartbeat();

        vTaskDelayUntil(&lastWakeTime, canTX100Hz_period_ms);
    }
}

/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // CAN1 initialization.
    cmr_canInit(
        &can, CAN1,
        CMR_CAN_BITRATE_500K,
        canRXMeta, sizeof(canRXMeta) / sizeof(canRXMeta[0]),
        NULL,
        GPIOA, GPIO_PIN_11,     // CAN1 RX port/pin.
        GPIOA, GPIO_PIN_12      // CAN1 TX port/pin.
    );

    const cmr_canFilter_t canFilters[] = {
        {
            .isMask = false,
            .rxFIFO = CAN_RX_FIFO0,
            .ids = {
                CMR_CANID_HEARTBEAT_VSM,
                CMR_IZZIE_LOADCELL,
            }
        },
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

/**
 * @brief Gets a pointer to the payload of a received CAN message.
 *
 * @param rxMsg The message to get the payload of.
 *
 * @return Pointer to payload, or NULL if rxMsg is invalid.
 */
void *getPayload(canRX_t rxMsg) {
    configASSERT((uint16_t) rxMsg < (uint16_t) CANRX_LEN);

    cmr_canRXMeta_t *rxMeta = &(canRXMeta[rxMsg]);

    return (void *)(&rxMeta->payload);
}

/**
 * @brief Converts a raw ADC reading to NTC thermistor resistance (ohms).
 *
 * @param counts Raw 12-bit ADC value.
 *
 * @return Thermistor resistance in ohms.
 */
static uint32_t resistor_to_temp(uint32_t counts) {
    // TODO
    return counts;
}

/**
 * @brief Sets up VSM heartbeat and sends it.
 */
static void sendHeartbeat(void) {
    cmr_canHeartbeat_t heartbeat = {
        .state = CMR_CAN_GLV_ON,
    };

    // Periodic telemetry: a drop on a full mailbox is acceptable (resent next
    // cycle), so the return is intentionally ignored.
    (void) canTX(CMR_CANID_DAQ_VSM_HEARTBEAT, &heartbeat, sizeof(heartbeat), canTX100Hz_period_ms);
}

/**
 * @brief Reflect thermistor values onto the bus.
 */
static void sendThermistors(void) {
    cmr_canDAQTherm_t msg = {
        .therm_1 = resistor_to_temp(adcRead(ADC_THERM_1)),
        .therm_2 = resistor_to_temp(adcRead(ADC_THERM_2)),
    };

    // Periodic telemetry: drop-and-resend on a full mailbox, return ignored.
    (void) canTX(CMR_CANID_DAQ_0_THERMISTOR, &msg, sizeof(msg), canTX10Hz_period_ms);
}

/**
 * @brief Forwards one thermal image row onto the bus in 4-pixel chunks.
 *
 * The 32-column row is split across 8 CAN frames of four signed deci-degree-C
 * values each (8 bytes, no padding), sent on consecutive IDs
 * CMR_CANID_DAQ_0_TIRETEMP .. CMR_CANID_DAQ_7_TIRETEMP (0x65c .. 0x663).
 */
static void sendTireTemp(void) {
    int16_t row[MLX_COLS];
    mlx90640GetRow(TIRETEMP_ROW, row);

    struct {
        int16_t temp_dC[4];
    } msg;

    for (int j = 0; j < MLX_COLS; j += 4) {
        for (int k = 0; k < 4; k++) {
            int col = j + k;
            msg.temp_dC[k] = (col < MLX_COLS) ? row[col] : 0;
        }
        // Stop the burst if a mailbox is unavailable, bounding worst-case task
        // blocking; the full row is retransmitted on the next cycle anyway.
        if (canTX(TIRETEMP_CANID + j / 4, &msg, sizeof(msg), canTX10Hz_period_ms) != 0) {
            return;
        }
    }
}