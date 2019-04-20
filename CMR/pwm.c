/**
 * @file pwm.c
 * @brief Pulse Width Modulation port wrapper implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "pwm.h"    // Interface to implement

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

    cmr_rccTIMClockEnable(pwmPinConfig->timer);

    uint32_t altFunc = timerToAltFunc(pwmPinConfig->timer);

    GPIO_InitTypeDef pinConfig = {
        .Pin = pwmPinConfig->pin,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = altFunc
    };
    cmr_rccGPIOClockEnable(pwmPinConfig->port);
    HAL_GPIO_Init(pwmPinConfig->port, &pinConfig);

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

    // if (HAL_TIM_Base_Init(&pwmChannel->handle) != HAL_OK) {
    //     return;
    // }

    TIM_ClockConfigTypeDef clockSrcConfig = {
        .ClockSource = TIM_CLOCKSOURCE_INTERNAL
    };
    if (HAL_TIM_ConfigClockSource(&pwmChannel->handle, &clockSrcConfig) != HAL_OK) {
        return;
    }

    if (HAL_TIM_PWM_Init(&pwmChannel->handle) != HAL_OK) {
        return;
    }

    // Disable fancy master/slave stuff if applicable
    if (IS_TIM_MASTER_INSTANCE(pwmPinConfig->timer)) {
        TIM_MasterConfigTypeDef masterConfig = {
            .MasterOutputTrigger = TIM_TRGO_RESET,
            .MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE
        };

        if (HAL_TIMEx_MasterConfigSynchronization(&pwmChannel->handle, &masterConfig) != HAL_OK) {
            return;
        }
    }

    // Begin with 0% duty cycle
    cmr_pwmSetDutyCycle(pwmChannel, 0);

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
            return;
        }
    }

    HAL_TIM_PWM_Start(&pwmChannel->handle, pwmPinConfig->channel);
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
void cmr_pwmSetPeriod(cmr_pwmChannel_t *pwmChannel, uint32_t period_ticks, uint32_t presc) {
    configASSERT(pwmChannel != NULL);
    configASSERT(presc <= UINT16_MAX + 1);
    configASSERT(period_ticks <= UINT16_MAX + 1);

    pwmChannel->handle.Init.Prescaler = presc - 1;
    pwmChannel->handle.Init.Period = period_ticks - 1;

    HAL_TIM_PWM_Init(&pwmChannel->handle);
}

/**
 * @brief Sets the duty cycle of a PWM channel.
 *
 * @param pwmChannel The PWM channel to set the duty cycle of.
 * @param dutyCycle_pcnt The duty cycle, in percent no greater than 100, to set pwmChannel to.
 */
void cmr_pwmSetDutyCycle(cmr_pwmChannel_t *pwmChannel, uint32_t dutyCycle_pcnt) {
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

    if (UINT32_MAX / period_ticks < dutyCycle_pcnt) {
        outputCompareConfig.Pulse = period_ticks / 100 * dutyCycle_pcnt;
    } else {
        outputCompareConfig.Pulse = period_ticks * dutyCycle_pcnt / 100;
    }

    HAL_TIM_PWM_ConfigChannel(&pwmChannel->handle, &outputCompareConfig, pwmChannel->channel);
}

#endif /* HAL_TIM_MODULE_ENABLED */
