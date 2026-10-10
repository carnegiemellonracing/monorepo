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
    
    //PWM_PUMP_LEFT
    [PWM_PUMP_1] = {
        .pwmPinConfig = {
            .port = GPIOB,
            .pin = GPIO_PIN_14,
            .channel = TIM_CHANNEL_1,
            .presc = 24,
            .period_ticks = 40000,
            .timer = TIM12
        }
    },
    //PWM_PUMP_RIGHT
    [PWM_PUMP_2] = {
        .pwmPinConfig = {
            .port = GPIOA,
            .pin = GPIO_PIN_5,
            .channel = TIM_CHANNEL_1,
            .presc = 24,
            .period_ticks = 40000,
            .timer = TIM2
        }
    },

    //PWM_FAN_LEFT
    [PWM_FAN_1] = {
        .pwmPinConfig = {
            .port = GPIOA,
            .pin = GPIO_PIN_11,
            .channel = TIM_CHANNEL_4,
            .presc = 24,
            .period_ticks = 40000,
            .timer = TIM1
        }
    },

    //PWM_FAN_RIGHT
    [PWM_FAN_2] = {
        .pwmPinConfig = {
            .port = GPIOB,
            .pin = GPIO_PIN_1,
            .channel = TIM_CHANNEL_4,
            .presc = 24,
            .period_ticks = 40000,
            .timer = TIM3
        }
    },

    [PWM_FAN_CMP] = {
        .pwmPinConfig = {
            .port = GPIOC,
            .pin = GPIO_PIN_8,
            .channel = TIM_CHANNEL_3,
            .presc = 24,
            .period_ticks = 40000,
            .timer = TIM8
        }
    },
    
    [PWM_GREEN] = {
        .pwmPinConfig = {
            .port = GPIOA,
            .pin = GPIO_PIN_2,
            .channel = TIM_CHANNEL_1,
            .presc = 10000,
            .period_ticks = 3200,
            .timer = TIM9
        }
    },
    [PWM_RED] = {
        .pwmPinConfig = {
            .port = GPIOA,
            .pin = GPIO_PIN_0,
            .channel = TIM_CHANNEL_1,
            .presc = 10000,
            .period_ticks = 3200,
            .timer = TIM5
        }
    },
    [PWM_YELLOW] = {
        .pwmPinConfig = {
            .port = GPIOA,
            .pin = GPIO_PIN_6,
            .channel = TIM_CHANNEL_1,
            .presc = 10000,
            .period_ticks = 3200,
            .timer = TIM13
        }
    },
    [PWM_BLUE] = {
        .pwmPinConfig = {
            .port = GPIOA,
            .pin = GPIO_PIN_7,
            .channel = TIM_CHANNEL_1,
            .presc = 10000,
            .period_ticks = 3200,
            .timer = TIM14
        }
    }
};

/**
 * @brief Initializes the PWM interface.
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

