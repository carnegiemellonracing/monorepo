/**
 * @file can.h
 * @brief Controller Area Network interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_CAN_H
#define CMR_CAN_H

#include <stm32f4xx_hal.h>  // HAL_CAN_MODULE_ENABLED,
                            // CAN_HandleTypeDef, CAN_TypeDef

#ifdef HAL_CAN_MODULE_ENABLED

#include <FreeRTOS.h>   // FreeRTOS interface
#include <semphr.h>     // Semaphore interface

#include <stdint.h>

/** @brief Issue severity levels. */
typedef enum {
    CMR_CAN_SEVERITY_NONE = 0,  /**< @brief No issues. */
    CMR_CAN_SEVERITY_LOW,       /**< @brief Low-severity issue. */
    CMR_CAN_SEVERITY_MEDIUM,    /**< @brief Medium-severity issue. */
    CMR_CAN_SEVERITY_HIGH,      /**< @brief High-severity issue. */
    CMR_CAN_SEVERITY_LEN        /**< @brief Total number of severity levels. */
} cmr_canSeverity_t;

/** @brief Periodic message reception metadata. */
typedef struct {
    const uint16_t canID;       /**< @brief Associated CAN ID. */

    /** @brief Payload reference output location. */
    volatile void **const payloadRef;

    /** @brief Timeout severity. */
    const cmr_canSeverity_t timeoutSeverity;

    /** @brief Threshold period for timeout warning, in milliseconds. */
    const TickType_t timeoutWarn_ms;

    /** @brief Threshold period for timeout error, in milliseconds. */
    const TickType_t timeoutError_ms;

    /** @brief Last receive timestamp, in milliseconds. */
    volatile TickType_t lastReceived_ms;

    /** @brief Raw message payload. */
    volatile uint8_t payload[8];
} cmr_canRXMeta_t;

int cmr_canRXMetaTimeoutWarn(const cmr_canRXMeta_t *meta, TickType_t now_ms);
int cmr_canRXMetaTimeoutError(const cmr_canRXMeta_t *meta, TickType_t now_ms);

/**
 * @brief Represents a CAN interface.
 *
 * @note The contents of this struct are opaque to the library consumer.
 */
typedef struct cmr_can cmr_can_t;

typedef void (*cmr_canRXCallback_t)(
    cmr_can_t *can, uint16_t canID, const void *data, size_t dataLen
);

struct cmr_can {
    CAN_HandleTypeDef handle;   /**< @brief HAL CAN handle. */
    SemaphoreHandle_t txMutex;  /**< @brief Transmit mutex. */

    /** @brief Metadata for periodic messages to receive. */
    cmr_canRXMeta_t *rxMeta;

    /** @brief Number of periodic receive messages. */
    size_t rxMetaLen;

    /** @brief Callback for other messages received, or `NULL` to ignore. */
    cmr_canRXCallback_t rxCallback;
};

void cmr_canInit(
    cmr_can_t *can, CAN_TypeDef *instance,
    cmr_canRXMeta_t *rxMeta, size_t rxMetaLen,
    cmr_canRXCallback_t rxCallback, const char *rxTaskName,
    GPIO_TypeDef *rxPort, uint16_t rxPin,
    GPIO_TypeDef *txPort, uint16_t txPin
);

int cmr_canRXTimeoutErrors(
    cmr_can_t *can, cmr_canSeverity_t minTimeoutSeverity, TickType_t now_ms
);

/**
 * @brief Represents a CAN filter's configuration.
 */
typedef struct {
    /**
     * @brief The CAN receive FIFO to configure (one of `CAN_RX_FIFOx` from
     * `stm32f4xx_hal_can.h`).
     */
    uint32_t rxFIFO;
    uint16_t ids[4];    /**< @brief The IDs to whitelist. */
} cmr_canFilter_t;

void cmr_canFilter(
    cmr_can_t *can, const cmr_canFilter_t *filters, size_t filtersLen
);

int cmr_canTX(
    cmr_can_t *can, uint16_t id, const void *data, size_t len
);

void cmr_canFieldEnable(uint8_t *field, const void *value, size_t len);
void cmr_canFieldDisable(uint8_t *field, const void *value, size_t len);

#endif /* HAL_CAN_MODULE_ENABLED */

#endif /* CMR_CAN_H */

