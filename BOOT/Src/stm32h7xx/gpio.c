/**
 * @file gpio.c
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#include "gpio.h"   // Interface to implement

/**
 * To be implemented
static const cmr_gpioPinConfig_t gpioPinConfigs[GPIO_LEN] = {
    [GPIO_LED_STATUS] = { //same as IND BLUE on schematic (no change)
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_0,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_PUSH_BUTTON] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_13,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_PULLDOWN,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    }
};
 */

/**
 * @brief Initializes the GPIO interface.
 */
void gpioInit(void) {
    /* to be implemented*/
}


/**
 * @brief Deinitializes the GPIO interface.
 */
void gpioDeinit(void) {
  /* to be implemented*/
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
    cmr_gpioToggle(GPIO_LED_STATUS);

    /* schedule the next blink event */
    nextBlinkEvent = TimerGet() + LED_TOGGLE_TIME_MS;
  }
}


bool getPushButton(void) {
    return cmr_gpioRead(GPIO_PUSH_BUTTON) == 1;
}