/**
 * @file can.c
 * @brief Board-specific CAN implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "can.h"      // Interface to implement

 // static config settings
CAN_TypeDef *instance = CAN1;
GPIO_TypeDef *rxPort = GPIOB;
uint16_t rxPin = GPIO_PIN_8;
GPIO_TypeDef *txPort = GPIOB;
uint16_t txPin = GPIO_PIN_9;

/**
 * @brief Initializes the CAN interface.
 */
void canInit(void) {
    // required clocks
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();

    // Configure CAN RX pin.
    GPIO_InitTypeDef pinConfig = {
        .Pin = rxPin,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF8_CAN1,
    };
    HAL_GPIO_Init(rxPort, &pinConfig);

    // Configure CAN TX pin.
    pinConfig.Pin = txPin;
    pinConfig.Alternate = GPIO_AF8_CAN1;
    HAL_GPIO_Init(txPort, &pinConfig);
}

void canDeinit(void) {
    HAL_GPIO_DeInit(rxPort, rxPin);
    HAL_GPIO_DeInit(txPort, txPin);

    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_CAN1_CLK_DISABLE();
    
}