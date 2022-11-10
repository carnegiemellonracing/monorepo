/**
 * @file i2c.h
 * @brief I2C interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_I2C_H
#define CMR_I2C_H

#include "platform.h"

#ifdef HAL_I2C_MODULE_ENABLED

#include <FreeRTOS.h>
#include <semphr.h> // semaphore interface
#include <stdint.h>

extern const uint32_t I2C_CLOCK_LOW;
extern const uint32_t I2C_CLOCK_HI;

typedef struct {
    I2C_HandleTypeDef handle;      /**< @brief HAL I2C Handle.*/
    DMA_HandleTypeDef dmatx_handle; /**< @brief HAL DMA TX Handle.*/
    DMA_HandleTypeDef dmarx_handle; /**< @brief HAL DMA RX Handle.*/

    SemaphoreHandle_t txDone; /**< @brief Semaphore for TX.*/
    StaticSemaphore_t txDoneBuf; /**< @brief Static buffer for TX semaphore.*/
    SemaphoreHandle_t rxDone; /**< @brief Semaphore for RX.*/
    StaticSemaphore_t rxDoneBuf; /**< @brief Static buffer for RX semaphore.*/
} cmr_i2c_t;

int cmr_i2cTX(cmr_i2c_t *i2c, uint16_t devAddr, uint8_t *data,
              size_t dataLength, uint32_t timeout_ms);

int cmr_i2cRX(cmr_i2c_t *i2c, uint16_t devAddr, uint8_t *data,
              size_t dataLength, uint32_t timeout_ms);

void cmr_i2cInit(
    cmr_i2c_t *i2c, I2C_TypeDef *instance,
    uint32_t clockSpeed, uint32_t ownAddr,
    GPIO_TypeDef *i2cClkPort, uint32_t i2cClkPin,
    GPIO_TypeDef *i2cDataPort, uint32_t i2cDataPin
);

int cmr_i2cDmaTX(cmr_i2c_t *i2c, uint16_t devAddr, uint8_t *data,
              size_t dataLength, uint32_t timeout_ms);

int cmr_i2cDmaRX(cmr_i2c_t *i2c, uint16_t devAddr, uint8_t *data,
              size_t dataLength, uint32_t timeout_ms);

void cmr_i2cDmaInit(
    cmr_i2c_t *i2c, I2C_TypeDef *instance,
    DMA_Stream_TypeDef *txDmaStream, DMA_Stream_TypeDef *rxDmaStream,
    uint32_t clockSpeed, uint32_t ownAddr,
    GPIO_TypeDef *i2cClkPort, uint32_t i2cClkPin,
    GPIO_TypeDef *i2cDataPort, uint32_t i2cDataPin
);

#endif /* HAL_I2C_MODULE_ENABLED */

#endif /* CMR_I2C_H */

