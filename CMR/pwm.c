/**
 * @file pwm.c
 * @brief Pulse Width Modulation port wrapper implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "pwm.h"    // Interface to implement
#include "panic.h"  // cmr_panic()

#ifdef HAL_TIM_MODULE_ENABLED

#include <FreeRTOS.h>
#include "rcc.h"

static uint32_t timerToAltFunc(TIM_TypeDef *timer) {
    if (timer == TIM1) {
        return GPIO_AF1_TIM1;
    }
    if (timer == TIM2) {
        return GPIO_AF1_TIM2;
    }
    if (timer == TIM3) {
        return GPIO_AF2_TIM3;
    }
    if (timer == TIM4) {
        return GPIO_AF2_TIM4;
    }
    if (timer == TIM5) {
        return GPIO_AF2_TIM5;
    }
    if (timer == TIM8) {
        return GPIO_AF3_TIM8;
    }
    if (timer == TIM9) {
        return GPIO_AF3_TIM9;
    }
    if (timer == TIM10) {
        return GPIO_AF3_TIM10;
    }
    if (timer == TIM11) {
        return GPIO_AF3_TIM11;
    }
    if (timer == TIM12) {
        return GPIO_AF9_TIM12;
    }
    if (timer == TIM13) {
        return GPIO_AF9_TIM13;
    }
    if (timer == TIM14) {
        return GPIO_AF9_TIM14;
    }

    return (uint32_t) -1;
}

/**
 * @brief Initializes PWM for the specified pin.
 *
 * @param pwmPin The pin to initialize.
 * @param pwmChannel A PWM channel struct to use.
 *
 * @note pwmFreq_Hz = 96 MHz / (pwmPinConfig->presc * pwmPinConfig->period_ticks)
 */
void cmr_pwmInit(const cmr_pwmPinConfig_t *pwmPinConfig,
                 cmr_pwmChannel_t *pwmChannel) {

    configASSERT(pwmPinConfig != NULL);
    configASSERT(pwmPinConfig->timer != NULL);
    configASSERT(pwmPinConfig->channel == TIM_CHANNEL_1 ||
                 pwmPinConfig->channel == TIM_CHANNEL_2 ||
                 pwmPinConfig->channel == TIM_CHANNEL_3 ||
                 pwmPinConfig->channel == TIM_CHANNEL_4);

    configASSERT(pwmPinConfig->presc <= UINT16_MAX + 1);
    configASSERT(pwmPinConfig->period_ticks <= UINT16_MAX + 1);

    cmr_rccGPIOClockEnable(pwmPinConfig->port);

    *pwmChannel = (cmr_pwmChannel_t) {
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

    // Disable fancy master/slave stuff if applicable
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

    // Disable fancy break/dead time stuff if applicable
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

    uint32_t altFunc = timerToAltFunc(pwmPinConfig->timer);
    GPIO_InitTypeDef pinConfig = {
        .Pin = pwmPinConfig->pin,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = altFunc
    };
    HAL_GPIO_Init(pwmPinConfig->port, &pinConfig);

    if (HAL_TIM_PWM_Start(&pwmChannel->handle, pwmChannel->channel) != HAL_OK) {
        cmr_panic("pwmInit HAL_TIM_PWM_Start failed!");
    }
}

/**
 * @brief Sets the period of a PWM channel.
 *
 * @param pwmChannel The PWM channel to set the duty cycle of.
 * @param period_ticks How much to divide the timer's peripheral clock by. Maximum 65536 (2^16).
 * @param presc PWM period in timer ticks (after clock division). Maximum 65536 (2^16).
 *
 * @note pwmFreq_Hz = 96 MHz / (presc * period_ticks)
 */
void cmr_pwmSetPeriod(cmr_pwmChannel_t *pwmChannel, uint32_t presc, uint32_t period_ticks) {
    configASSERT(pwmChannel != NULL);
    configASSERT(presc <= UINT16_MAX + 1);
    configASSERT(period_ticks <= UINT16_MAX + 1);

    // Find current duty cycle
    uint32_t ccrVal_ticks = 0;
    TIM_TypeDef *timer = pwmChannel->handle.Instance;
    switch (pwmChannel->channel) {
        case TIM_CHANNEL_1:
            ccrVal_ticks = timer->CCR1;
            break;
        case TIM_CHANNEL_2:
            ccrVal_ticks = timer->CCR2;
            break;
        case TIM_CHANNEL_3:
            ccrVal_ticks = timer->CCR3;
            break;
        case TIM_CHANNEL_4:
            ccrVal_ticks = timer->CCR4;
            break;
    }

    uint32_t currentPeriod_ticks = pwmChannel->handle.Init.Period + 1;
    uint32_t dutyCycle_pcnt = ccrVal_ticks / currentPeriod_ticks;

    pwmChannel->handle.Init.Prescaler = presc - 1;
    pwmChannel->handle.Init.Period = period_ticks - 1;

    if (HAL_TIM_PWM_Stop(&pwmChannel->handle, pwmChannel->channel) != HAL_OK) {
        cmr_panic("pwmSetPeriod HAL_TIM_PWM_Stop failed!");
    }

    // Maintain duty cycle with new period
    cmr_pwmSetDutyCycle(pwmChannel, dutyCycle_pcnt);

    // Commit new period to timer
    if (HAL_TIM_PWM_Init(&pwmChannel->handle) != HAL_OK) {
        cmr_panic("pwmSetPeriod cmr_pwmSetPeriod failed!");
    }

    if (HAL_TIM_PWM_Start(&pwmChannel->handle, pwmChannel->channel) != HAL_OK) {
        cmr_panic("pwmSetPeriod HAL_TIM_PWM_Start failed!");
    }
}

/**
 * @brief Sets the duty cycle of a PWM channel.
 *
 * @param pwmChannel The PWM channel to set the duty cycle of.
 * @param dutyCycle_pcnt The duty cycle, in percent no greater than 100, to set pwmChannel to.
 */
void cmr_pwmSetDutyCycle(cmr_pwmChannel_t *pwmChannel, uint32_t dutyCycle_pcnt) {
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
