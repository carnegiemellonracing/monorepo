/**
 * @file gpio.c
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface

#include "gpio.h"   // Interface to implement

GPIO_TypeDef *port = GPIOD;
uint16_t pin = GPIO_PIN_2;

const int toggle_time_ms = 500;

/**
 * @brief Initializes the GPIO interface.
 */
void gpioInit(void) {
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // Configure CAN RX pin.
    GPIO_InitTypeDef pinConfig = {
        .Pin = pin,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };
    HAL_GPIO_Init(port, &pinConfig);
}


/**
 * @brief Deinitializes the GPIO interface.
 */
void gpioDeinit(void) {
    __HAL_RCC_GPIOD_CLK_DISABLE();
    HAL_GPIO_DeInit(port, pin);
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
    
    /* schedule the next blink event */
    nextBlinkEvent = TimerGet() + toggle_time_ms;
  }
}
