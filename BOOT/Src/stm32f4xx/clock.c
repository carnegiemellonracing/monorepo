#include <CMR/rcc.h>                            /* RCC interface                      */
#include <CMR/panic.h>                          /* panic interface                    */
#include <stm32f4xx_hal.h>                      /* HAL interface                      */
#include "clock.h"                                 /* clock interface                     */

/**
 * Initialize the system clock
 */
void clockInit(void)
{
    cmr_rccSystemInternalClockEnable();
}

/**
 * Deinitialize the system clock
 */
void clockDeinit(void)
{
    cmr_rccResetSystemClock();

}