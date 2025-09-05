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

#endif /* HAL_DMA_MODULE_ENABLED */
#endif /* HAL_SD_MODULE_ENABLED */
