/**
 * @file rcc.h
 * @brief Reset and clock controller interface.
 *
 * @note Unless otherwise noted, if a device depends on clocks controlled by the
 * RCC, they will be automatically enabled.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_RCC_H
#define CMR_RCC_H

#include <stm32f4xx_hal.h>  // HAL_RCC_MODULE_ENABLED, HAL_GPIO_MODULE_ENABLED,
                            // HAL_ADC_MODULE_ENABLED, HAL_CAN_MODULE_ENABLED,
                            // HAL_SPI_MODULE_ENABLED,
                            // GPIO_TypeDef, ADC_TypeDef, CAN_TypeDef,
                            // SPI_TypeDef, QUADSPI_TypeDef

#ifdef HAL_RCC_MODULE_ENABLED

void cmr_rccSystemClockEnable(void);

#ifdef HAL_GPIO_MODULE_ENABLED
void cmr_rccGPIOClockEnable(GPIO_TypeDef *port);
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
void cmr_rccADCClockEnable(ADC_TypeDef *instance);
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_CAN_MODULE_ENABLED
void cmr_rccCANClockEnable(CAN_TypeDef *instance);
#endif /* HAL_CAN_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
void cmr_rccI2CClockEnable(I2C_TypeDef *instance);
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_SPI_MODULE_ENABLED
void cmr_rccSPIClockEnable(SPI_TypeDef *instance);
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_QSPI_MODULE_ENABLED
void cmr_rccQSPIClockEnable(QUADSPI_TypeDef *instance);
#endif /* HAL_QSPI_MODULE_ENABLED */

#ifdef HAL_USART_MODULE_ENABLED
void cmr_rccUSARTClockEnable(USART_TypeDef *instance);
#endif /* HAL_USART_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
void cmr_rccTIMClockEnable(TIM_TypeDef *instance);
#endif /* HAL_TIM_MODULE_ENABLED */

#endif /* HAL_RCC_MODULE_ENABLED */

#endif /* CMR_RCC_H */

