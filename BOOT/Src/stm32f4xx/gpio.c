/**
 * @file gpio.c
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface

#include "gpio.h"   // Interface to implement

GPIO_TypeDef *led_port = GPIOB;
uint16_t led_pin = GPIO_PIN_0;

GPIO_TypeDef *push_button_port = GPIOC;
uint16_t push_button_pin = GPIO_PIN_13;

const int toggle_time_ms = 500;

/**
 * @brief Initializes the GPIO interface.
 */
void gpioInit(void) {
    // LED toggle
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef pinConfig = {
        .Pin = led_pin,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };
    HAL_GPIO_Init(led_port, &pinConfig);
    
    pinConfig.Pin = push_button_pin;
    pinConfig.Mode = GPIO_MODE_INPUT;
    pinConfig.Pull = GPIO_PULLDOWN;
    pinConfig.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(push_button_port, &pinConfig);
}


/**
 * @brief Deinitializes the GPIO interface.
 */
void gpioDeinit(void) {
    __HAL_RCC_GPIOB_CLK_DISABLE();
    HAL_GPIO_DeInit(led_port, led_pin);
    HAL_GPIO_DeInit(push_button_port, push_button_pin);
}


/************************************************************************************//**
** \brief     Task function for blinking the LED as a fixed timer interval.
** \return    none.
**
****************************************************************************************/
void timedLedToggle(void)
{
  static int32_t nextBlinkEvent = 0;

  /* check for blink event */
  if (TimerGet() >= nextBlinkEvent)
  {
    HAL_GPIO_TogglePin(led_port, led_pin);

    /* schedule the next blink event */
    nextBlinkEvent = TimerGet() + toggle_time_ms;
  }
}


bool getPushButton(void) {
    return HAL_GPIO_ReadPin(push_button_port, push_button_pin) == GPIO_PIN_SET;
}