/**
 * @brief bsrr.h
 * @file GPIO "bit set/reset register" interface.
 *
 * See STM32F413 RM0430 7.4.7 for more implementation details.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_BSRR_H
#define CMR_BSRR_H

#include <stm32f4xx_hal.h>  // HAL_GPIO_MODULE_ENABLED,
                            // GPIO_TypeDef *, GPIO_PIN_x

#ifdef HAL_GPIO_MODULE_ENABLED

#include <stdint.h>     // uint16_t

void cmr_bsrr(GPIO_TypeDef *port, uint16_t set, uint16_t reset);

#endif /* HAL_GPIO_MODULE_ENABLED */

#endif /* CMR_BSRR_H */

