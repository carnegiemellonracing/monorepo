/**
 * @file can.h
 * @brief Controller Area Network interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_CAN_H
#define CMR_CAN_H

#include "platform.h"  // HAL_CAN_MODULE_ENABLED,
                       // CAN_HandleTypeDef, CAN_TypeDef

#ifdef HAL_CAN_MODULE_ENABLED

#include <stdbool.h>
#include <stdint.h>

#include <FreeRTOS.h>   // FreeRTOS interface
#include <semphr.h>     // Semaphore interface

#include "tasks.h"      // Task interface

#include <CMR/can_types.h>  // cmr_canError_t, cmr_canWarn_t

/** @brief Number of CAN filter banks allocated for each interface. */
#define CMR_CAN_FILTERBANKS 14

/** @brief CAN bit rate enumeration. */
typedef enum {
    CMR_CAN_BITRATE_250K = 0,   /**< @brief 250 kbit/s. */
    CMR_CAN_BITRATE_500K,       /**< @brief 500 kbit/s. */
    CMR_CAN_BITRATE_1M          /**< @brief 1 Mbit/s. */
} cmr_canBitRate_t;

/** @brief Periodic message reception metadata. */
typedef struct {
    const uint16_t canID;       /**< @brief Associated CAN ID. */

    /** @brief Threshold period for timeout warning, in milliseconds. */
    const TickType_t timeoutWarn_ms;

    /** @brief Heartbeat flag to set upon exceeding timeout warning threshold. */
    const cmr_canWarn_t warnFlag;

    /** @brief Threshold period for timeout error, in milliseconds. */
    const TickType_t timeoutError_ms;

    /** @brief Heartbeat flag to set upon exceeding timeout error threshold. */
    const cmr_canError_t errorFlag;

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
    CAN_HandleTypeDef handle;       /**< @brief HAL CAN handle. */
    SemaphoreHandle_t txSem;        /**< @brief Transmit counting semaphore. */
    StaticSemaphore_t txSemBuf;     /**< @brief Transmit semaphore buffer. */

    /** @brief Metadata for periodic messages to receive. */
    cmr_canRXMeta_t *rxMeta;

    /** @brief Number of periodic receive messages. */
    size_t rxMetaLen;

    /** @brief Callback for other messages received, or `NULL` to ignore. */
    cmr_canRXCallback_t rxCallback;
};

void cmr_canInit(
    cmr_can_t *can, CAN_TypeDef *instance,
    cmr_canBitRate_t bitRate,
    cmr_canRXMeta_t *rxMeta, size_t rxMetaLen,
    cmr_canRXCallback_t rxCallback,
    GPIO_TypeDef *rxPort, uint16_t rxPin,
    GPIO_TypeDef *txPort, uint16_t txPin
);

/**
 * @brief Represents a CAN filter's configuration.
 */
typedef struct {
    bool isMask;        /**< @brief `true` to mask; `false` to whitelist. */

    /**
     * @brief The CAN receive FIFO to configure (one of `CAN_RX_FIFOx` from
     * `stm32f4xx_hal_can.h`).
     */
    uint32_t rxFIFO;

    /**
     * @brief The associated IDs.
     *
     * When `isMask` is `false`, this is a list of IDs to whitelist. A message
     * with an ID equal to at least one of the IDs will be accepted into the
     * specified FIFO; otherwise, it is ignored.
     *
     * Otherwise, when `isMask` is `true`, the first half of this array are IDs,
     * and the second half selects bits from the ID to match (set means "match";
     * clear means "don't care").
     *
     * Thus, an incoming message with ID "msg_id" is checked against each mask:
     *
     *      (msg_id & ids[2] == ids[0] & ids[2]) ||
     *      (msg_id & ids[3] == ids[1] & ids[3])
     *
     * If the above condition is true, the message is accepted into the
     * specified FIFO; otherwise, it is ignored.
     */
    uint16_t ids[4];
} cmr_canFilter_t;

void cmr_canFilter(
    cmr_can_t *can, const cmr_canFilter_t *filters, size_t filtersLen
);

int cmr_canTX(
    cmr_can_t *can,
    uint16_t id, const void *data, size_t len,
    TickType_t timeout
);

void cmr_canFieldEnable(uint8_t *field, const void *value, size_t len);
void cmr_canFieldDisable(uint8_t *field, const void *value, size_t len);
uint32_t cmr_canGPIOAF(CAN_TypeDef *instance, GPIO_TypeDef *port);
void cmr_canFilter(cmr_can_t *can, const cmr_canFilter_t *filters, size_t filtersLen);

/* Platform-specific external dependencies (don't expose to application code) */
extern uint32_t _platform_canGPIOAF(CAN_TypeDef *instance, GPIO_TypeDef *port);
extern void _platform_canFilter(cmr_can_t *can, const cmr_canFilter_t *filters, size_t filtersLen);
extern void _platform_canInit(
    cmr_can_t *can, CAN_TypeDef *instance,
    cmr_canBitRate_t bitRate,
    cmr_canRXMeta_t *rxMeta, size_t rxMetaLen,
    cmr_canRXCallback_t rxCallback,
    GPIO_TypeDef *rxPort, uint16_t rxPin,
    GPIO_TypeDef *txPort, uint16_t txPin
);

#endif /* HAL_CAN_MODULE_ENABLED */

#endif /* CMR_CAN_H */

