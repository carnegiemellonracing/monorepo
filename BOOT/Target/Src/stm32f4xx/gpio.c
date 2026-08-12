/**
 * @file gpio.c
 * @brief Board-specific GPIO interface.
 *
 */

#include "gpio.h"   // Interface to implement
#include <CMR/board_info.h>   // global board info
// BLT timer stuff
#include "types.h"
#include "timer.h"

/**
 * @brief Initializes the GPIO interface.
 */
void gpioInit(void) {
    cmr_gpioPin_t status_gpio = cmr_getBootloaderStatusLedPin();
    const cmr_gpioPinConfig_t ledConfig = { 
        .port = status_gpio.port,
        .init = {
            .Pin = status_gpio.pin,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    };
    cmr_gpioPinInit(
        &ledConfig, 1
    );
}


/**
 * @brief Deinitializes the GPIO interface.
 */
void gpioDeinit(void) {
    cmr_gpioPin_t status_gpio = cmr_getBootloaderStatusLedPin();
    const cmr_gpioPinConfig_t ledConfig = { 
        .port = status_gpio.port,
        .init = {
            .Pin = status_gpio.pin,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    };
    cmr_gpioPinDeInit(
        &ledConfig, 1
    );
}



/**
 * @brief Toggles the LED at a timed interval.
 */
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