/**
 * @file memorator.h
 * @brief memorator interface
 * @author Ayush Garg
 */

#pragma once

#include <stdint.h>     /* integer types */

void memoratorWrite(    uint16_t ID, 
                        uint32_t timeStamp,
                        uint8_t dataLength, 
                        uint8_t* data);

void memoratorInit(void);
