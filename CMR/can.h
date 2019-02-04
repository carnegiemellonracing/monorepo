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

/**
 * @brief Represents a CAN interface.
 *
 * @note The contents of this struct are opaque to the library consumer.
 */
typedef struct {
    /**< @brief HAL CAN handle. */
    CAN_HandleTypeDef handle;

    /**< @brief Transmit mutex. */
    SemaphoreHandle_t txMutex;
} cmr_can_t;

typedef enum {
	CAN_PAYLOAD_STATUS_OK,
    CAN_PAYLOAD_STATUS_TIMEOUT,
    CAN_PAYLOAD_STATUS_NO_TIMEOUT,
    CAN_PAYLOAD_STATUS_NO_PAYLOAD,
	CAN_PAYLOAD_STATUS_LEN
} cmr_canPayloadStatus_t;

typedef enum {
	TIMEOUT_SEVERITY_LOW,
	TIMEOUT_SEVERITY_MEDIUM,
	TIMEOUT_SEVERITY_HIGH,
	TIMEOUT_SEVERITY_LEN
} cmr_timeoutSeverity_t;

// Reception payload and metadata
typedef struct {
    uint16_t canID;
    void *payloadStruct;
    size_t payloadSize;
    cmr_timeoutSeverity_t timeoutSeverity;
    TickType_t lastReceived_ms;
    TickType_t timeoutWarnThreshold_ms;
    TickType_t timeoutErrorThreshold_ms;
} cmr_rxMetadata_t;

void cmr_canInit(
    cmr_can_t *can, CAN_TypeDef *instance,
    const char *rxTaskName,
	const cmr_rxMetadata_t *rxMetadataArray,
	size_t rxMetadataArrayLength,
    GPIO_TypeDef *rxPort, uint16_t rxPin,
    GPIO_TypeDef *txPort, uint16_t txPin
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

void *cmr_getPayloadStructAddress(size_t payloadIndex);
size_t cmr_getPayloadStructSize(size_t payloadIndex);
cmr_canPayloadStatus_t cmr_resetTimeout(size_t payloadIndex);
cmr_canPayloadStatus_t cmr_checkMessageTimeoutWarn(size_t payloadIndex);
cmr_canPayloadStatus_t cmr_checkMessageTimeoutError(size_t payloadIndex);
cmr_canPayloadStatus_t cmr_checkSeverityTimeouts(uint8_t minTimeoutSeverity);
void cmr_setTimeoutSeverity(size_t payloadIndex, cmr_timeoutSeverity_t timeoutSeverity);
void cmr_setTimeoutWarnThreshold(size_t payloadIndex, TickType_t warnThreshold_ms);
void cmr_setTimeoutErrorThreshold(size_t payloadIndex, TickType_t errorThreshold_ms);

void cmr_canFieldEnable(uint8_t *field, const void *value, size_t len);
void cmr_canFieldDisable(uint8_t *field, const void *value, size_t len);

#endif /* HAL_CAN_MODULE_ENABLED */

#endif /* CMR_CAN_H */

