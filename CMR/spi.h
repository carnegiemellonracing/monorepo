/**
 * @file spi.h
 * @brief Serial Peripheral Interface port.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_SPI_H
#define CMR_SPI_H

#include "platform.h"       // HAL_SPI_MODULE_ENABLED, HAL_DMA_MODULE_ENABLED
                            // SPI_HandleTypeDef, SPI_TypeDef, SPI_InitTypeDef,
                            // DMA_HandleTypeDef, DMA_Stream_TypeDef

#ifdef HAL_SPI_MODULE_ENABLED
#ifdef HAL_DMA_MODULE_ENABLED

#include <FreeRTOS.h>   // FreeRTOS interface
#include <semphr.h>     // Semaphore interface

/** @brief Represents a single SPI pin. */
typedef struct {
    /** @brief Pin's GPIO port (`GPIOx` from `stm32f413xx.h`). */
    GPIO_TypeDef *port;

    /** @brief Channel's GPIO pin (`GPIO_PIN_x` from `stm32f4xx_hal_gpio.h`). */
    uint16_t pin;
} cmr_spiPin_t;

/** @brief Represents a SPI port's pins. */
typedef struct {
    cmr_spiPin_t mosi;  /**< @brief Master out, slave in. */
    cmr_spiPin_t miso;  /**< @brief Master in, slave out. */
    cmr_spiPin_t sck;   /**< @brief SPI clock. */
    cmr_spiPin_t nss;   /**< @brief Slave select (active low). */
} cmr_spiPinConfig_t;

/**
 * @brief Represents a SPI port.
 *
 * @note The contents of this struct are opaque to the library consumer.
 */
typedef struct cmr_spi cmr_spi_t;

typedef void (*cmr_spiCallback_t)(cmr_spi_t *spi);

struct cmr_spi {
    SPI_HandleTypeDef handle;   /**< @brief HAL SPI handle. */
    DMA_HandleTypeDef rxDMA;    /**< @brief Receiving HAL DMA handle. */
    DMA_HandleTypeDef txDMA;    /**< @brief Transmitting HAL DMA handle. */
    cmr_spiPin_t nssPin;   /**< @brief Slave select pin. */

    SemaphoreHandle_t doneSem;     /**< @brief Done binary semaphore. */
    StaticSemaphore_t doneSemBuf;  /**< @brief Done semaphore storage. */
};

void cmr_spiInit(
    cmr_spi_t *spi, SPI_TypeDef *instance, const SPI_InitTypeDef *init,
    const cmr_spiPinConfig_t *pins,
    DMA_Stream_TypeDef *rxDMA, uint32_t rxDMAChannel,
    DMA_Stream_TypeDef *txDMA, uint32_t txDMAChannel
);

int cmr_spiTXRX(
    cmr_spi_t *spi, const void *txData, void *rxData, size_t len
);

#endif /* HAL_DMA_MODULE_ENABLED */
#endif /* HAL_SPI_MODULE_ENABLED */

#endif /* CMR_SPI_H */

