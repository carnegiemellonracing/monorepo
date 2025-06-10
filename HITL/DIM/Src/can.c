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
#include "can.h"      // Interface to implement

/** @brief Primary CAN interface. */
static cmr_can_t veh_can;
static cmr_can_t daq_can;
static cmr_can_t trac_can;


// -------------- DEFINE STATIC VARIABLES TO BE CHANGED (WITH GETTERS AND SETTERS) HERE -------------- //


// --------------------------------------------------------------------------------------------------- //



cmr_canRXMeta_t canRXMeta_VEH[] = {
    [CANRX_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    }, // and more (refer to VSM repo CAN file) i.e. vsm status, sensors, etc.
};

cmr_canRXMeta_t canRXMeta_DAQ[] = {
    [CANRX_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    }, // and more (refer to VSM repo CAN file) i.e. vsm status, sensors, etc.
};

cmr_canRXMeta_t canRXMeta_TRAC[] = {
    [CANRX_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    }, // and more (refer to VSM repo CAN file) i.e. vsm status, sensors, etc.
};

int canTX_VEH(cmr_canID_t id, const void *data, size_t len, TickType_t timeout) {
    return cmr_canTX(&veh_can, id, data, len, timeout);
}

int canTX_DAQ(cmr_canID_t id, const void *data, size_t len, TickType_t timeout) {
    return cmr_canTX(&daq_can, id, data, len, timeout);
}

int canTX_TRAC(cmr_canID_t id, const void *data, size_t len, TickType_t timeout) {
    return cmr_canTX(&trac_can, id, data, len, timeout);
}


// -------------- DEFINE CAN MESSAGE CONSTRUCTION FUNCTIONS HERE -------------- //


// ---------------------------------------------------------------------------- //




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
    while (1) {

        // insert 1Hz messages

        vTaskDelayUntil(&lastWakeTime, canTX1Hz_period_ms);
    }
}


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

        // insert 10Hz messages

        vTaskDelayUntil(&lastWakeTime, canTX10Hz_period_ms);
    }
}


/** @brief CAN 100 Hz TX priority. */
static const uint32_t canTX100Hz_priority = 5;

/** @brief CAN 100 Hz TX period (milliseconds). */
static const TickType_t canTX100Hz_period_ms = 10;

/** @brief CAN 100 Hz TX task. */
static cmr_task_t canTX100Hz_task;

static void canTX100Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        
        // insert 100Hz messages
        // send VSM heartbeat
        vTaskDelayUntil(&lastWakeTime, canTX100Hz_period_ms);
    }
}


/** @brief CAN 200 Hz TX priority. */
static const uint32_t canTX200Hz_priority = 5;

/** @brief CAN 200 Hz TX period (milliseconds). */
static const TickType_t canTX200Hz_period_ms = 5;

/** @brief CAN 200 Hz TX task. */
static cmr_task_t canTX200Hz_task;

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

        // insert 200Hz messages

        vTaskDelayUntil(&lastWakeTime, canTX200Hz_period_ms);
    }
}

void *getPayloadDAQ(canRX_DAQ_t rxMsg) {
    if (rxMsg >= CANRX_LEN_DAQ) {
        return NULL; // TODO switch to configassert
    }

    cmr_canRXMeta_t *rxMeta = &(canRXMeta_DAQ[rxMsg]);

    return (void *)(&rxMeta->payload);
}

void *getPayloadVEH(canRX_VEH_t rxMsg) {
    if (rxMsg >= CANRX_LEN_VEH) {
        return NULL; // TODO switch to configassert
    }

    cmr_canRXMeta_t *rxMeta = &(canRXMeta_VEH[rxMsg]);

    return (void *)(&rxMeta->payload);
}

void *getPayloadTRAC(canRX_TRAC_t rxMsg) {
    if (rxMsg >= CANRX_LEN_TRAC) {
        return NULL; // TODO switch to configassert
    }

    cmr_canRXMeta_t *rxMeta = &(canRXMeta_TRAC[rxMsg]);

    return (void *)(&rxMeta->payload);
}


/**
 * @brief Initializes the CAN interface.
 * @note No CAN filtering since we want to observe all messages produced by the test board
 */
void canInit(void) {
    // CAN2 initialization.
    cmr_canInit(
        &daq_can, CAN1,
		CMR_CAN_BITRATE_500K,
        canRXMeta_DAQ, sizeof(canRXMeta_VEH) / sizeof(canRXMeta_VEH[0]),
        NULL,
        GPIOA, GPIO_PIN_11,     // CAN2 RX port/pin.
        GPIOA, GPIO_PIN_12      // CAN2 TX port/pin.
    );
    cmr_canInit(
        &veh_can, CAN2,
		CMR_CAN_BITRATE_500K,
        canRXMeta_VEH, sizeof(canRXMeta_VEH) / sizeof(canRXMeta_VEH[0]),
        NULL,
        GPIOB, GPIO_PIN_12,     // CAN2 RX port/pin.
        GPIOB, GPIO_PIN_13      // CAN2 TX port/pin.
    );
    cmr_canInit(
        &trac_can, CAN3,
		CMR_CAN_BITRATE_500K,
        canRXMeta_TRAC, sizeof(canRXMeta_VEH) / sizeof(canRXMeta_VEH[0]),
        NULL,
        GPIOA, GPIO_PIN_8,     // CAN2 RX port/pin.
        GPIOB, GPIO_PIN_4      // CAN2 TX port/pin.
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
    cmr_taskInit(
        &canTX200Hz_task,
        "CAN TX 200Hz",
        canTX200Hz_priority,
        canTX200Hz,
        NULL
    );
}
