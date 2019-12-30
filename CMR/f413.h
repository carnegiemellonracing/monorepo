#ifndef CMR_F413_H
#define CMR_F413_H

#include "can.h"      // can types
#include <string.h>   // memcpy()
#include <stdbool.h>  // bool

#include "rcc.h"      // cmr_rccCANClockEnable(), cmr_rccGPIOClockEnable()
#include "panic.h"    // cmr_panic()

#ifdef HAL_CAN_MODULE_ENABLED

uint32_t _platform_canGPIOAF(CAN_TypeDef *instance, GPIO_TypeDef *port);
void _platform_canFilter(cmr_can_t *can, const cmr_canFilter_t *filters, size_t filtersLen);
void _platform_canInit(
    cmr_can_t *can, CAN_TypeDef *instance,
    cmr_canRXMeta_t *rxMeta, size_t rxMetaLen,
    cmr_canRXCallback_t rxCallback,
    GPIO_TypeDef *rxPort, uint16_t rxPin,
    GPIO_TypeDef *txPort, uint16_t txPin
);

#endif /* HAL_CAN_MODULE_ENABLED */

#ifdef HAL_RCC_MODULE_ENABLED

void _platform_rccSystemClockEnable(void);
void _platform_rccSystemInternalClockEnable(void);

#ifdef HAL_GPIO_MODULE_ENABLED
void _platform_rccGPIOClockEnable(GPIO_TypeDef *port);
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
void _platform_rccADCClockEnable(ADC_TypeDef *instance);
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_CAN_MODULE_ENABLED
void _platform_rccCANClockEnable(CAN_TypeDef *instance);
#endif /* HAL_CAN_MODULE_ENABLED */

#endif /* HAL_RCC_MODULE_ENABLED */

#endif /* CMR_F413_H */
