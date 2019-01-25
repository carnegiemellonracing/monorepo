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

#include <stdint.h>     // uint16_t
#include <stm32f4xx_hal.h>  // GPIO_TypeDef *, GPIO_PIN_x

void cmr_bsrr(GPIO_TypeDef *port, uint16_t set, uint16_t reset);

#endif /* CMR_BSRR_H */

