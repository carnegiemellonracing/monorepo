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

#include <stm32f4xx_hal.h>  // GPIO_TypeDef, ADC_TypeDef

void cmr_rccSystemClockEnable(void);

void cmr_rccGPIOClockEnable(GPIO_TypeDef *port);
void cmr_rccADCClockEnable(ADC_TypeDef *instance);
void cmr_rccCANClockEnable(CAN_TypeDef *instance);

#endif /* CMR_RCC_H */

