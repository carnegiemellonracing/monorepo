/**
 * @file sdio.c
 * @brief Secure Digital Input Output wrapper implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/sdio.h>  // Interface to implement

#include "platform.h"  // HAL_SD_MODULE_ENABLED, HAL_DMA_MODULE_ENABLED
                       // SPI_HandleTypeDef, SPI_TypeDef, SPI_InitTypeDef,
                       // DMA_HandleTypeDef, DMA_Stream_TypeDef

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
