// ------------------------------------------------------------------------------------------------
// Includes

#include "pin_definitions.h"

// ------------------------------------------------------------------------------------------------
// Ports

GPIO_TypeDef *const vSensePort       = GPIOA;
GPIO_TypeDef *const iSensePort       = GPIOA;

GPIO_TypeDef *const mcuStatusLEDPort = GPIOB;

GPIO_TypeDef *const can2RXPort       = GPIOB;
GPIO_TypeDef *const can2TXPort       = GPIOB;

// ------------------------------------------------------------------------------------------------
// Pins

const uint16_t vSensePin             = GPIO_PIN_0;
const uint16_t iSensePin             = GPIO_PIN_1;

const uint16_t mcuStatusLEDPin       = GPIO_PIN_6;

const uint16_t can2RXPin             = GPIO_PIN_12;
const uint16_t can2TXPin             = GPIO_PIN_13;
