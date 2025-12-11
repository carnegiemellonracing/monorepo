/**
 * @file sdio.c
 * @brief Secure Digital Input Output wrapper implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/sdio.h>  // Interface to implement

#include <CMR/platform.h>  // HAL_SD_MODULE_ENABLED, HAL_DMA_MODULE_ENABLED
                       // SPI_HandleTypeDef, SPI_TypeDef, SPI_InitTypeDef,
                       // DMA_HandleTypeDef, DMA_Stream_TypeDef


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
#ifdef HAL_SD_MODULE_ENABLED
#ifdef HAL_DMA_MODULE_ENABLED

static SD_HandleTypeDef hsd;

void cmr_sdioInit(cmr_sdio_t *sdio, SD_TypeDef *instance,
                  const SD_InitTypeDef *init, const cmr_sdioPinConfig_t *pins,
                  DMA_Stream_TypeDef *txDMA, uint32_t txDMAChannel) {
    *sdio = (cmr_sdio_t) {
        .handle =
            {
                .Instance = instance,
                .init = *init,
            },
        .txDMA = {.Instance = txDMA,
                  .Channel = txDMAChannel,
                  .Direction = DMA_MEMORY_TO_PERIPH,
                  .PeriphInc = DMA_PINC_DISABLE,
                  .MemInc = DMA_MINC_ENABLE,
                  .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
                  .MemDataAlignment = DMA_MDATAALIGN_BYTE,
                  .Mode = DMA_NORMAL,
                  .Priority = DMA_PRIORITY_LOW,
                  .FIFOMode = DMA_FIFOMODE_DISABLE},
        .fatfs = {0},  // TO EDIT
    }
}

HAL_StatusTypeDef cmr_sdioWriteBlocks(cmr_sdio_t *sdio, const uint8_t *buffer,
                                      uint32_t blockAddr, uint32_t numBlocks,
                                      uint32_t timeout) {}

#endif /* HAL_DMA_MODULE_ENABLED */
#endif /* HAL_SD_MODULE_ENABLED */
