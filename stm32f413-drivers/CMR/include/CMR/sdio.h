/**
 * @file sdio.h
 * @brief Secure Digital Input Output.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_SD_H
#define CMR_SD_H

#ifdef HAL_SD_MODULE_ENABLED
#ifdef HAL_DMA_MODULE_ENABLED

#include <FreeRTOS.h>  // FreeRTOS interface
#include <semphr.h>    // Semaphore interface
#include <stdbool.h>

/** @brief Represents a single SDIO pin. */
typedef struct {
    /** @brief Pin's GPIO port (`GPIOx` from `stm32f413xx.h`). */
    GPIO_TypeDef *port;

    /** @brief Channel's GPIO pin (`GPIO_PIN_x` from `stm32f4xx_hal_gpio.h`). */
    uint16_t pin;
} cmr_sdioPin_t;

/** @brief Represents a SD port's pins. */
typedef struct {
    cmr_sdioPin_t d0;  /**< @brief DAT0 Pin **/
    cmr_sdioPin_t d1;  /**< @brief DAT1 Pin. For 4-bit mode. **/
    cmr_sdioPin_t d2;  /**< @brief DAT2 Pin. For 4-bit mode. **/
    cmr_sdioPin_t d3;  /**< @brief DAT3 Pin. For 4-bit mode. **/
    cmr_sdioPin_t clk; /**< @brief SDIO Clock Pin **/
    cmr_sdioPin_t cmd; /**< @brief SDIO Command Pin **/
    cmr_sdioPin_t det; /**< @brief SDIO Detect Pin **/
} cmr_sdioPinConfig_t;

/**
 * @brief Represents a SDIO inteface.
 *
 * @note The contents of this struct are opaque to the library consumer.
 */
typedef struct {
    SD_HandleTypeDef handle; /**< @brief HAL SDIO/SDMMC handle. */
    DMA_HandleTypeDef txDMA; /**< @brief DMA handle for SDIO Tx. */

    // cmr_sdioPinConfig_t
    //     pins; /**< @brief SDIO pin mapping (D0..D3, CLK, CMD, DET). */

    FATFS fs; /**< @brief FatFS object. */
} cmr_sdio_t;

void cmr_sdioInit(cmr_sdio_t *sdio, SD_TypeDef *instance,
                  const SD_InitTypeDef *init, const cmr_sdioPinConfig_t *pins,
                  DMA_HandleTypeDef *txDMA);

HAL_StatusTypeDef cmr_sdioWriteBlocks(cmr_sdio_t *sdio, const uint8_t *buffer,
                                      uint32_t blockAddr, uint32_t numBlocks,
                                      uint32_t timeout);

#endif /* HAL_DMA_MODULE_ENABLED */
#endif /* HAL_SD_MODULE_ENABLED */

#endif /* CMR_SD_H */
