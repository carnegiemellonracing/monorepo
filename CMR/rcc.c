/**
 * @file rcc.c
 * @brief Reset and clock controller wrapper implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "rcc.h"    // Interface to implement

/**
 * @brief Enables the specified GPIO port's clock.
 *
 * @param port The GPIO port.
 */
void cmr_rccGPIOClockEnable(GPIO_TypeDef *port) {
    switch (port) {
        case GPIOA:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            break;
        case GPIOB:
            __HAL_RCC_GPIOB_CLK_ENABLE();
            break;
        case GPIOC:
            __HAL_RCC_GPIOC_CLK_ENABLE();
            break;
        case GPIOD:
            __HAL_RCC_GPIOD_CLK_ENABLE();
            break;
        case GPIOE:
            __HAL_RCC_GPIOE_CLK_ENABLE();
            break;
        case GPIOF:
            __HAL_RCC_GPIOF_CLK_ENABLE();
            break;
        case GPIOG:
            __HAL_RCC_GPIOG_CLK_ENABLE();
            break;
        case GPIOH:
            __HAL_RCC_GPIOH_CLK_ENABLE();
            break;
    }
}

/**
 * @brief Enables the specified ADC's clock.
 *
 * @param instance The HAL ADC instance.
 */
void cmr_rccADCClockEnable(ADC_TypeDef *instance) {
    switch (instance) {
        case ADC1:
            __HAL_RCC_ADC1_CLK_ENABLE();
            break;
    }
}

/**
 * @brief Enables the specified CAN interface's clock.
 *
 * @param instance The HAL CAN instance.
 */
void cmr_rccCANClockEnable(CAN_TypeDef *instance) {
    switch (instance) {
        case CAN3:
            __HAL_RCC_CAN3_CLK_ENABLE();
            break;
        case CAN2:
            __HAL_RCC_CAN2_CLK_ENABLE();
            // Fallthrough; CAN2 also requires CAN1 clock.
        case CAN1:
            __HAL_RCC_CAN1_CLK_ENABLE();
            break;
    }
}

