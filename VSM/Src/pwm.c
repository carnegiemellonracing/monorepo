/**
 * @file pwm.c
 * @brief Board-specific PWM implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "pwm.h"    // Interface to implement

/**
 * @brief Board-specific PWM pin configuration.
 *
 * Replace/add more PWM pin configurations here as appropriate. Each
 * enumeration value of `pwm_t` should get a configuration.
 *
 * @see `CMR/pwm.h` for various initialization values.
 */

static cmr_pwmPin_t pwmPinConfigs[PWM_LEN] = {
    [PWM_GREEN] = {
        .pwmPinConfig = {
            .port = GPIOA,
            .pin = GPIO_PIN_9,
            .channel = TIM_CHANNEL_2,
            .presc = 24,
            .period_ticks = 1000,
            .timer = TIM1
        }
    },
    [PWM_RED] = {
        .pwmPinConfig = {
            .port = GPIOA,
            .pin = GPIO_PIN_10,
            .channel = TIM_CHANNEL_3,
            .presc = 24,
            .period_ticks = 1000,
            .timer = TIM1
        }
    },
    [PWM_YELLOW] = {
        .pwmPinConfig = {
            .port = GPIOA,
            .pin = GPIO_PIN_11,
            .channel = TIM_CHANNEL_4,
            .presc = 10000,
            .period_ticks = 3200,
            .timer = TIM1
        }
    },
    [PWM_BLUE] = {
        .pwmPinConfig = {
            .port = GPIOB,
            .pin = GPIO_PIN_10,
            .channel = TIM_CHANNEL_3,
            .presc = 10000,
            .period_ticks = 3200,
            .timer = TIM2
        }
    }
};

/**
 * @brief Initializes the ADC interface.
 */
void pwmInit(void) {
    cmr_pwmPinInit(
        pwmPinConfigs, 
        sizeof(pwmPinConfigs) / sizeof(pwmPinConfigs[0])
    );
}

/**
 * @brief Sets the duty cycle of the given PWM pin.
 *
 * @param pin The pin.
 * @param dutyCycle_pcnt The duty cycle percentage
 *
 */
void pwmSetDutyCycle(pwm_t pin, uint32_t dutyCycle_pcnt) {
    cmr_pwmSetDutyCycle(&(pwmPinConfigs[pin].pwmChannel), dutyCycle_pcnt);
}

