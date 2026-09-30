/**
 * @file memorator.h
 * @brief memorator interface
 * @author Ayush Garg
 */

#pragma once

#include <stdint.h>     /* integer types */
#include <stm32f4xx_hal.h>  // HAL interface
#include <CMR/rtc.h>   // Real-Time Clock
#include <CMR/can_types.h>  // cmr_canRAMError_t


void memoratorWrite(    uint16_t ID, 
                        RTC_TimeTypeDef timeStamp,
                        uint8_t dataLength, 
                        uint8_t* data);

void memoratorInit(void);
cmr_canRAMError_t memoratorGetErrors(void);
void HAL_SD_MspInit(SD_HandleTypeDef* hsd);

