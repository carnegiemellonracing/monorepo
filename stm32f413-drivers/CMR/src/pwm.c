/**
 * @file pwm.c
 * @brief Pulse Width Modulation port wrapper implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/pwm.h>    // Interface to implement
#include <CMR/panic.h>  // cmr_panic()

#ifdef HAL_TIM_MODULE_ENABLED

#include <FreeRTOS.h>
#include "CMR/rcc.h"

static uint32_t cmr_timerToAltFunc(TIM_TypeDef *timer) {
    switch ((uintptr_t) timer) {
#ifdef F413
        case TIM1_BASE:
            return GPIO_AF1_TIM1;
        case TIM2_BASE:
            return GPIO_AF1_TIM2;
        case TIM3_BASE:
            return GPIO_AF2_TIM3;
        case TIM4_BASE:
            return GPIO_AF2_TIM4;
        case TIM5_BASE:
            return GPIO_AF2_TIM5;
        case TIM8_BASE:
            return GPIO_AF3_TIM8;
        case TIM9_BASE:
            return GPIO_AF3_TIM9;
        case TIM10_BASE:
            return GPIO_AF3_TIM10;
        case TIM11_BASE:
            return GPIO_AF3_TIM11;
        case TIM12_BASE:
            return GPIO_AF9_TIM12;
        case TIM13_BASE:
            return GPIO_AF9_TIM13;
        case TIM14_BASE:
            return GPIO_AF9_TIM14;
#endif
    }

    return (uint32_t) -1;
}

/**
 * @brief Initializes PWM for the specified pin.
 *
 * @param pwmChannel A PWM channel struct to use.
 * @param pwmPinConfig The pin to initialize.
 */
void cmr_pwmInit(cmr_pwm_t *pwmChannel,
                 const cmr_pwmPinConfig_t *pwmPinConfig) {

    configASSERT(pwmPinConfig != NULL);
    configASSERT(pwmPinConfig->timer != NULL);

    configASSERT(pwmPinConfig->presc > 0 && pwmPinConfig->presc <= UINT16_MAX + 1);
    configASSERT(pwmPinConfig->period_ticks > 0 && pwmPinConfig->period_ticks <= UINT16_MAX + 1);

    cmr_rccGPIOClockEnable(pwmPinConfig->port);

    *pwmChannel = (cmr_pwm_t) {
        .handle = {
            .Instance = pwmPinConfig->timer,
            .Init = {
                .Prescaler = pwmPinConfig->presc - 1,
                .CounterMode = TIM_COUNTERMODE_UP,
                .Period = pwmPinConfig->period_ticks - 1,
                .ClockDivision = TIM_CLOCKDIVISION_DIV1,
                .RepetitionCounter = 0,
                .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE
            }
        },
        .channel = pwmPinConfig->channel
    };

    cmr_rccTIMClockEnable(pwmPinConfig->timer);

    if (HAL_TIM_Base_Init(&pwmChannel->handle) != HAL_OK) {
        cmr_panic("pwmInit HAL_TIM_Base_Init failed!");
        return;
    }

    TIM_ClockConfigTypeDef clockSrcConfig = {
        .ClockSource = TIM_CLOCKSOURCE_INTERNAL
    };
    if (HAL_TIM_ConfigClockSource(&pwmChannel->handle, &clockSrcConfig) != HAL_OK) {
        cmr_panic("pwmInit HAL_TIM_ConfigClockSource failed!");
        return;
    }

    if (HAL_TIM_PWM_Init(&pwmChannel->handle) != HAL_OK) {
        cmr_panic("pwmInit HAL_TIM_PWM_Init failed!");
        return;
    }

    // Disable fancy master/slave stuff if applicable since we won't use it.
    // Based on Cube-generated code.
    if (IS_TIM_MASTER_INSTANCE(pwmPinConfig->timer)) {
        TIM_MasterConfigTypeDef masterConfig = {
            .MasterOutputTrigger = TIM_TRGO_RESET,
            .MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE
        };

        if (HAL_TIMEx_MasterConfigSynchronization(&pwmChannel->handle, &masterConfig) != HAL_OK) {
            cmr_panic("pwmInit HAL_TIMEx_MasterConfigSynchronization failed!");
            return;
        }
    }

    // Begin with 0% duty cycle
    // Do not use cmr_pwmSetDutyCycle() because that would prematurely call
    // HAL_TIM_PWM_Start()
    TIM_OC_InitTypeDef outputCompareConfig = {
        .OCMode = TIM_OCMODE_PWM1,
        .Pulse = 16000, // TODO remove this 50% hardcoding
        .OCPolarity = TIM_OCPOLARITY_HIGH,
        .OCNPolarity = TIM_OCNPOLARITY_HIGH,
        .OCFastMode = TIM_OCFAST_DISABLE,
        .OCIdleState = TIM_OCIDLESTATE_RESET,
        .OCNIdleState = TIM_OCNIDLESTATE_RESET
    };

    if (HAL_TIM_PWM_ConfigChannel(&pwmChannel->handle, &outputCompareConfig, pwmChannel->channel) != HAL_OK) {
        cmr_panic("pwmInit cmr_pwmSetDutyCycle failed!");
    }

    // Disable fancy break/dead time stuff if applicable since we won't use it.
    // Based on Cube-generated code.
    if (IS_TIM_BREAK_INSTANCE(pwmPinConfig->timer)) {
        TIM_BreakDeadTimeConfigTypeDef breakDeadConfig = {
            .OffStateRunMode = TIM_OSSR_DISABLE,
            .OffStateIDLEMode = TIM_OSSI_DISABLE,
            .LockLevel = TIM_LOCKLEVEL_OFF,
            .DeadTime = 0,
            .BreakState = TIM_BREAK_DISABLE,
            .BreakPolarity = TIM_BREAKPOLARITY_HIGH,
            .AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE
        };

        if (HAL_TIMEx_ConfigBreakDeadTime(&pwmChannel->handle, &breakDeadConfig) != HAL_OK) {
            cmr_panic("pwmInit HAL_TIMEx_ConfigBreakDeadTime failed!");
            return;
        }
    }

    GPIO_InitTypeDef pinConfig = {
        .Pin = pwmPinConfig->pin,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = cmr_timerToAltFunc(pwmPinConfig->timer)
    };
    HAL_GPIO_Init(pwmPinConfig->port, &pinConfig);

    if (HAL_TIM_PWM_Start(&pwmChannel->handle, pwmChannel->channel) != HAL_OK) {
        cmr_panic("pwmInit HAL_TIM_PWM_Start failed!");
    }
}

/**
 * @brief Sets the duty cycle of a PWM channel.
 *
 * @param pwmChannel The PWM channel to set the duty cycle of.
 * @param dutyCycle_pcnt The duty cycle, in percent no greater than 100, to set pwmChannel to.
 */
void cmr_pwmSetDutyCycle(cmr_pwm_t *pwmChannel, uint32_t dutyCycle_pcnt) {
    configASSERT(pwmChannel != NULL);
    configASSERT(dutyCycle_pcnt <= 100);

    static TIM_OC_InitTypeDef outputCompareConfig = {
        .OCMode = TIM_OCMODE_PWM1,
        .Pulse = 0,
        .OCPolarity = TIM_OCPOLARITY_HIGH,
        .OCNPolarity = TIM_OCNPOLARITY_HIGH,
        .OCFastMode = TIM_OCFAST_DISABLE,
        .OCIdleState = TIM_OCIDLESTATE_RESET,
        .OCNIdleState = TIM_OCNIDLESTATE_RESET
    };

    uint32_t period_ticks = pwmChannel->handle.Init.Period + 1;

    // Prevent overflow (should never occur)
    if (UINT32_MAX / period_ticks < dutyCycle_pcnt) {
        outputCompareConfig.Pulse = period_ticks / 100 * dutyCycle_pcnt;
    } else {
        outputCompareConfig.Pulse = period_ticks * dutyCycle_pcnt / 100;
    }

    if (HAL_TIM_PWM_Stop(&pwmChannel->handle, pwmChannel->channel) != HAL_OK) {
        cmr_panic("pwmSetPeriod HAL_TIM_PWM_Stop failed!");
    }

    if (HAL_TIM_PWM_ConfigChannel(&pwmChannel->handle, &outputCompareConfig, pwmChannel->channel) != HAL_OK) {
        cmr_panic("pwmSetDutyCycle cmr_pwmSetDutyCycle failed!");
    }

    if (HAL_TIM_PWM_Start(&pwmChannel->handle, pwmChannel->channel) != HAL_OK) {
        cmr_panic("pwmSetPeriod HAL_TIM_PWM_Start failed!");
    }
}

#endif /* HAL_TIM_MODULE_ENABLED */
