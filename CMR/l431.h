#ifndef CMR_L431_H
#define CMR_L431_H

#ifdef L431

#include <string.h>   // memcpy()
#include <stdbool.h>  // bool

#include "can.h"      // can types
#include "adc.h"      // adc types
#include "rcc.h"      // cmr_rccCANClockEnable(), cmr_rccGPIOClockEnable()
#include "config.h"   // config types
#include "panic.h"    // cmr_panic()

#ifdef HAL_CAN_MODULE_ENABLED

uint32_t _platform_canGPIOAF(CAN_TypeDef *instance, GPIO_TypeDef *port);
void _platform_canFilter(cmr_can_t *can, const cmr_canFilter_t *filters, size_t filtersLen);
void _platform_canInit(
    cmr_can_t *can, CAN_TypeDef *instance,
    cmr_canBitRate_t bitRate,
    cmr_canRXMeta_t *rxMeta, size_t rxMetaLen,
    cmr_canRXCallback_t rxCallback,
    GPIO_TypeDef *rxPort, uint16_t rxPin,
    GPIO_TypeDef *txPort, uint16_t txPin
);

#endif /* HAL_CAN_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED

void _platform_adcInit(cmr_adc_t *adc, ADC_TypeDef *instance, cmr_adcChannel_t *channels, const size_t channelsLen);
ADC_ChannelConfTypeDef _platform_adcChannelConfig(const cmr_adcChannel_t *channel, uint32_t rank);
GPIO_InitTypeDef _platform_adcPinConfig(const cmr_adcChannel_t *channel);

#endif /* HAL_ADC_MODULE_ENABLED */

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

#ifdef HAL_FLASH_MODULE_ENABLED
void _platform_configInit(cmr_config_t *config, volatile uint32_t *cache, size_t cacheLen, uint32_t sector);
void _platform_configCommit(cmr_config_t *config);
#endif /* HAL_FLASH_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
void _platform_rccTIMClockEnable(TIM_TypeDef *instance);
#endif /* HAL_TIM_MODULE_ENABLED */

#endif /* L431 */

#endif /* CMR_L431_H */
