#ifndef PLATFORM_H
#define PLATFORM_H

#define F413_COMPARE    0
#define L431_COMPARE    1
#define H735_COMPARE    2

#define COMPARE_PROCESSOR(processor)    (processor ## _COMPARE)

#if defined(PROCESSOR) && (COMPARE_PROCESSOR(PROCESSOR)==F413_COMPARE)
#include <stm32f4xx_hal.h>
#elif defined(PROCESSOR) && (COMPARE_PROCESSOR(PROCESSOR)==L431_COMPARE)
#include <stm32l4xx_hal.h>
#elif defined(PROCESSOR) && (COMPARE_PROCESSOR(PROCESSOR)==H735_COMPARE)
#include <stm32h7xx_hal.h>
#endif

#endif /* PLATFORM_H */