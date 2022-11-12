
#ifndef _CMR_BOOTLOADER_H
#define _CMR_BOOTLOADER_H

#include "stm32f4xx_hal.h"
#include "can.h"

void cmr_bootloaderInit(void);

void cmr_bootloaderReceiveCallback(CAN_RxHeaderTypeDef *msg, uint8_t *rxData);

#endif
