/**
 * @file clock.c
 * @brief Board-specific clock implementation for STM32H7 targets.
 *
 * Provides initialization/deinitialization stubs for the H7 platform. The
 * implementations are intentionally minimal here and should call the CMR
 * platform RCC helpers where available.
 *
 * @author Carnegie Mellon Racing
 */

#include "clock.h"                                 /* clock interface                     */
#include <CMR/rcc.h>                                /* RCC interface                       */

/**
 * @brief Initialize the system clock for STM32H7.
 *
 */
void clockInit(void)
{
    /* to be implemented*/
}

/**
 * @brief Deinitialize the system clock for STM32H7.
 *
 */
void clockDeinit(void)
{
    /* to be implemented*/

}