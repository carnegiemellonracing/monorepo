/**
 * @file dma.h
 * @brief Direct memory access management.
 *
 * This interface is intended for use by drivers that use DMA; however, it can
 * also be used by a library consumer to configure a new DMA stream.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_DMA_H
#define CMR_DMA_H

#include "platform.h"       // HAL_DMA_MODULE_ENABLED, DMA_HandleTypeDef

#ifdef HAL_DMA_MODULE_ENABLED

void cmr_dmaInit(DMA_HandleTypeDef *handle);

#endif /* HAL_DMA_MODULE_ENABLED */

#endif /* CMR_DMA_INTERRUPT_H */

