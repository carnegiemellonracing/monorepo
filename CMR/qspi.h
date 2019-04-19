/**
 * @file qspi.h
 * @brief QuadSPI interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_QSPI_H
#define CMR_QSPI_H

#include <stm32f4xx_hal.h>  // HAL_QSPI_MODULE_ENABLED,
                            // QSPI_HandleTypeDef, QUADSPI_TypeDef,
                            // QSPI_InitTypeDef, DMA_HandleTypeDef

#ifdef HAL_QSPI_MODULE_ENABLED

#include <FreeRTOS.h>   // FreeRTOS interface
#include <semphr.h>     // Semaphore interface

/** @brief Represents a single QuadSPI pin. */
typedef struct {
    /** @brief Pin's GPIO port (`GPIOx` from `stm32f413xx.h`). */
    GPIO_TypeDef *port;

    /** @brief GPIO pin (`GPIO_PIN_x` from `stm32f4xx_hal_gpio.h`). */
    uint16_t pin;
} cmr_qspiPin_t;

/** @brief Represents a QuadSPI port's pins. */
typedef struct {
    cmr_qspiPin_t io[4];    /**< @brief I/O pins. */
    cmr_qspiPin_t sck;      /**< @brief QuadSPI clock. */
    cmr_qspiPin_t nss;      /**< @brief Slave select (active low). */
} cmr_qspiPinConfig_t;

/**
 * @brief Represents a QuadSPI port.
 *
 * @note The contents of this struct are opaque to the library consumer.
 */
typedef struct {
    QSPI_HandleTypeDef handle;      /**< @brief HAL SPI handle. */
    DMA_HandleTypeDef dma;          /**< @brief HAL DMA handle. */

    SemaphoreHandle_t doneSem;      /**< @brief Done binary semaphore. */
    StaticSemaphore_t doneSemBuf;   /**< @brief Done semaphore storage. */
} cmr_qspi_t;

void cmr_qspiInit(
    cmr_qspi_t *qspi, QUADSPI_TypeDef *instance, const QSPI_InitTypeDef *init,
    const cmr_qspiPinConfig_t *pins,
    DMA_Stream_TypeDef *dma, uint32_t dmaChannel
);

void cmr_qspiCmd(cmr_qspi_t *qspi, const QSPI_CommandTypeDef *cmd);
void cmr_qspiTX(
    cmr_qspi_t *qspi, const QSPI_CommandTypeDef *cmd, const void *data
);
void cmr_qspiRX(
    cmr_qspi_t *qspi, const QSPI_CommandTypeDef *cmd, void *data
);

#endif /* HAL_QSPI_MODULE_ENABLED */

#endif /* CMR_QSPI_H */

