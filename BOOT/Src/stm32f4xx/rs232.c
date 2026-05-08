/**
 * @file gpio.c
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface

#include "rs232.h"   // Interface to implement

/**
 * @brief Initializes the RS232 interface.
 */
void rs232Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Enable the USART3 peripheral clock */
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_USART6_CLK_ENABLE();

    /* UART TX and RX GPIO pin configuration. */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}

/**
 * @brief Deinitializes the RS232 interface.
 */
void rs232Deinit(void) {
    // RS232 deinitialization code
    /* Deinit used GPIOs. */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_9);
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14);
    /* UART clock disable. */
    __HAL_RCC_USART6_CLK_DISABLE();
    __HAL_RCC_GPIOG_CLK_DISABLE();
}