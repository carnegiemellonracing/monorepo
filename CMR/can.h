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

#include <stdint.h>

/**
 * @brief Callback for received messages.
 *
 * Called with the received message's CAN ID, the received data, and its length
 * in bytes.
 */
typedef void (*cmr_canRXCallback_t)(
    uint16_t id, const void *data, size_t len
);

/**
 * @brief Represents a CAN interface.
 *
 * @note The contents of this struct are opaque to the library consumer.
 */
typedef struct {
    CAN_HandleTypeDef handle;   /**< @brief HAL CAN handle. */

    /** @brief Callback for received messages. */
    cmr_canRXCallback_t rxCallback;
} cmr_can_t;

void cmr_canInit(
    cmr_can_t *can, CAN_TypeDef *instance,
    cmr_canRXCallback_t rxCallback, const char *rxTaskName,
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

void cmr_canFieldEnable(uint8_t *field, const void *value, size_t len);
void cmr_canFieldDisable(uint8_t *field, const void *value, size_t len);

#endif /* HAL_CAN_MODULE_ENABLED */

#endif /* CMR_CAN_H */

