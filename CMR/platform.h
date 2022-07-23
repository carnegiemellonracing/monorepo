#ifndef PLATFORM_H
#define PLATFORM_H

#define F413_COMPARE    0
#define L431_COMPARE    1
#define H735_COMPARE    2

#define CONCAT_PROCESSOR(platform)    (platform ## _COMPARE)
#define COMPARE_PROCESSOR(platform)    CONCAT_PROCESSOR(platform)

#if defined(PLATFORM) && (COMPARE_PROCESSOR(PLATFORM)==F413_COMPARE)
#include <stm32f4xx_hal.h>
#elif defined(PLATFORM) && (COMPARE_PROCESSOR(PLATFORM)==L431_COMPARE)
#include <stm32l4xx_hal.h>
#elif defined(PLATFORM) && (COMPARE_PROCESSOR(PLATFORM)==H735_COMPARE)
#include <stm32h7xx_hal.h>
#endif

#undef CONCAT_PROCESSOR
#undef COMPARE_PROCESSOR

#endif /* PLATFORM_H */
