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
 * @brief Represents a CAN interface.
 *
 * @note The contents of this struct are opaque to the library consumer.
 */
typedef struct {
    CAN_HandleTypeDef handle;   /**< @brief HAL CAN handle. */
} cmr_can_t;

void cmr_canInit(cmr_can_t *can, CAN_TypeDef *instance,
                 GPIO_TypeDef *rxPort, uint16_t rxPin,
                 GPIO_TypeDef *txPort, uint16_t txPin);

void cmr_canFilter(cmr_can_t *can, uint32_t filterBank, uint32_t rxFIFO,
                   uint16_t canID1, uint16_t canID2,
                   uint16_t canID3, uint16_t canID4);

#endif /* HAL_CAN_MODULE_ENABLED */

#endif /* CMR_CAN_H */

