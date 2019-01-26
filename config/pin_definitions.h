#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

// ------------------------------------------------------------------------------------------------
// Includes

#include "stm32f4xx_hal.h"

// ------------------------------------------------------------------------------------------------
// Ports

extern GPIO_TypeDef *const vSensePort;
extern GPIO_TypeDef *const iSensePort;

extern GPIO_TypeDef *const mcuStatusLEDPort;

extern GPIO_TypeDef *const can2RXPort;
extern GPIO_TypeDef *const can2TXPort;

// ------------------------------------------------------------------------------------------------
// Pins

extern const uint16_t vSensePin;
extern const uint16_t iSensePin;

extern const uint16_t mcuStatusLEDPin;

extern const uint16_t can2RXPin;
extern const uint16_t can2TXPin;

#endif /* PIN_DEFINITIONS_H */
