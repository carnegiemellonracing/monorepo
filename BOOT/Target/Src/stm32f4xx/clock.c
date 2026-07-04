/**
 * @file clock.c
 * @brief Board-specific clock implementation for STM32F4 targets.
 *
 * Provides minimal system clock initialization and deinitialization used by
 * the BOOT application. The implementation uses the CMR RCC helper API.
 *
 */

#include "clock.h"                                 /* clock interface                     */
#include <CMR/rcc.h>                                /* RCC interface                       */

/**
 * @brief Initialize the system clock.
 *
 */
void clockInit(void)
{
    cmr_rccSystemInternalClockEnable();
}

/**
 * @brief Deinitialize the system clock.
 *
 */
void clockDeinit(void)
{
    cmr_rccResetSystemClock();

}