#ifndef CMR_H725_H
#define CMR_H725_H

#ifdef H725

#include <string.h>   // memcpy()
#include <stdbool.h>  // bool

#include "fdcan.h"      // can types
#include "adc.h"      // adc types
#include "rcc.h"      // cmr_rccCANClockEnable(), cmr_rccGPIOClockEnable()
#include "config.h"   // config types
#include "panic.h"    // cmr_panic()
#include "i2c.h"

#ifdef HAL_FDCAN_MODULE_ENABLED

uint32_t _platform_FDcanGPIOAF(FDCAN_GlobalTypeDef *instance, GPIO_TypeDef *port);
void _platform_rccFDCanClockEnable();
void _platform_canFilter(cmr_can_t *can, const cmr_canFilter_t *filters, size_t filtersLen);
void _platform_FDCANInit(
    cmr_can_t *can, FDCAN_GlobalTypeDef *instance,
    cmr_canRXMeta_t *rxMeta, size_t rxMetaLen,
    cmr_canRXCallback_t rxCallback
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

#ifdef HAL_I2C_MODULE_ENABLED
void _platform_i2cInit(cmr_i2c_t *i2c, I2C_TypeDef *instance, uint32_t clockSpeed, uint32_t ownAddr);
void _platform_i2cClockInit(I2C_TypeDef *instance);
#endif /* HAL_I2C_MODULE_ENABLED */

#endif /* H725 */

#endif /* CMR_H725_H */