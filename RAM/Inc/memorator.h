/**
 * @file memorator.h
 * @brief memorator interface
 * @author Ayush Garg
 */

#pragma once

#include <stdint.h>     /* integer types */
#include <stm32f4xx_hal.h>  // HAL interface


void memoratorWrite(    uint16_t ID, 
                        uint32_t timeStamp,
                        uint8_t dataLength, 
                        uint8_t* data);

void memoratorInit(void);
void HAL_SD_MspInit(SD_HandleTypeDef* hsd);

