#include "clock.h"                                 /* clock interface                     */
#include <CMR/rcc.h>                            /* RCC interface                      */

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