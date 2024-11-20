/**
 * @file platform.h
 * @brief Platform-dependent shims (HALALAL)
 *
 * The core driver suite is largely platform independent, as the basic API
 * HAL provides us is (mostly) the same across the suite of MCU's ST offers us.
 * However, fundamental hardware differences across the product line make
 * some interactions with the HAL target-specific by nature, such as
 * peripheral initialization, where the the core peripheral may be duplicated
 * over the product line, but the number of instances and specific configuration needed for
 * each instance of a peripheral may vary between targets.
 *
 * The necessary declarations of such target-specific driver functions
 * are found in e.g. f413.h, where the file name corresponds to the target suffix,
 * and the relevant definitions can be switched on at compile time
 * by settings in parent repositories (by passing -DF413).
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_PLATFORM_H
#define CMR_PLATFORM_H


#ifdef F413
#include <stm32f4xx_hal.h>
#endif /* F413 */

#ifdef H735
#include <stm32h7xx_hal.h>
#endif /* H735 */


#endif /* CMR_PLATFORM_H */
