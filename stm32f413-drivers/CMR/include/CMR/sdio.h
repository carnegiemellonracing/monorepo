/**
 * @file sdio.h
 * @brief simple wrapper for SDIO that lets you write to an FATFS SD Card 
 * using an SDIO DMA. Note we choose not to implement reading from SD card 
 * as we are unsure of desired interface of future users.
 *
 * @author Ayush Garg
 */

#include <stdint.h>
#include "CMR/gpio.h"    // pin defintions
#include "fatfs.h"  // File Writing Middleware

#pragma once

#ifdef HAL_SD_MODULE_ENABLED
#ifdef HAL_DMA_MODULE_ENABLED
// Code should be reasonable easy to port (and might just work) for H7 but 
// untested so we have this include guard to draw attention to it
#ifdef F413 

typedef struct {
    cmr_gpioPin_t clockPin;
    cmr_gpioPin_t cmdPin;
    cmr_gpioPin_t* dataPins;
    uint8_t dataPinLength;
} cmr_sdioPinConfig_t;

void cmr_sdioInit(cmr_sdioPinConfig_t *pins);
uint8_t cmr_SDIO_mount();
uint32_t cmr_SDIO_remainingSpace();
uint8_t cmr_SDIO_openFile(FIL* filObj, char* path);
uint8_t cmr_SDIO_write(FIL* filObj, void* data, uint16_t dataLen);
uint8_t cmr_SDIO_closeFile(FIL* fileObj);
uint8_t cmr_SDIO_unmount();

#endif /* F413 */
#endif /* HAL_DMA_MODULE_ENABLED */
#endif /* HAL_SD_MODULE_ENABLED */