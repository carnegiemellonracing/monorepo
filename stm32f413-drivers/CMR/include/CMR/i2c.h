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

    GPIO_TypeDef *i2cClkPort;
    uint32_t i2cClkPin;
    GPIO_TypeDef *i2cDataPort;
    uint32_t i2cDataPin;

    SemaphoreHandle_t doneSem; /**< @brief Semaphore for TX.*/
    StaticSemaphore_t doneSemBuf; /**< @brief Static buffer for TX semaphore.*/
    uint32_t error; /**< @brief Error code when DMA receives an error.*/

    uint8_t *slaveTxBuff; /**< @brief i2c slave TX message buffer.*/
    uint16_t slaveTxLen; /**< @brief i2c slave TX message length.*/
    uint8_t *slaveRxBuff; /**< @brief i2c slave RX message buffer.*/
    uint16_t slaveRxLen; /**< @brief i2c slave RX message length.*/
    
} cmr_i2c_t;

int cmr_i2cTX(cmr_i2c_t *i2c, uint16_t devAddr, uint8_t *data,
              size_t dataLength, uint32_t timeout_ms);

int cmr_i2cRX(cmr_i2c_t *i2c, uint16_t devAddr, uint8_t *data,
              size_t dataLength, uint32_t timeout_ms);
int cmr_i2cMemRX(cmr_i2c_t *i2c, uint16_t devAddr, uint16_t memaddress, uint8_t memsize, uint8_t *data,
              size_t dataLength, uint32_t timeout_ms);

void cmr_i2cInit(
    cmr_i2c_t *i2c, I2C_TypeDef *instance,
    uint32_t clockSpeed, uint32_t ownAddr,
    GPIO_TypeDef *i2cClkPort, uint32_t i2cClkPin,
    GPIO_TypeDef *i2cDataPort, uint32_t i2cDataPin
);

// BEGINNING of functions for mcu acting as i2c slave.
void cmr_i2cSlaveInit(
    cmr_i2c_t *i2c, I2C_TypeDef *instance
);

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);

int cmr_i2cSlaveTx(cmr_i2c_t *i2c, uint8_t *txData, size_t len);

int cmr_i2cSlaveRx(cmr_i2c_t *i2c, uint8_t *rxData, size_t len);

// END of functions for mcu acting as i2c slave.

extern void _platform_i2cInit(cmr_i2c_t *i2c, I2C_TypeDef *instance, uint32_t clockSpeed, uint32_t ownAddr);

#ifdef F413
int cmr_i2cDmaTX(cmr_i2c_t *i2c, uint16_t devAddr, uint8_t *data,
              size_t dataLength, uint32_t timeout_ms);

int cmr_i2cDmaRX(cmr_i2c_t *i2c, uint16_t devAddr, uint8_t *data,
              size_t dataLength, uint32_t timeout_ms);

void cmr_i2cDmaInit(
    cmr_i2c_t *i2c, I2C_TypeDef *instance,
    DMA_Stream_TypeDef *txDmaStream, uint32_t txDmaChannel,
    DMA_Stream_TypeDef *rxDmaStream, uint32_t rxDmaChannel,
    uint32_t clockSpeed, uint32_t ownAddr,
    GPIO_TypeDef *i2cClkPort, uint32_t i2cClkPin,
    GPIO_TypeDef *i2cDataPort, uint32_t i2cDataPin
);
#endif /* F413 */

#endif /* HAL_I2C_MODULE_ENABLED */

#endif /* CMR_I2C_H */

