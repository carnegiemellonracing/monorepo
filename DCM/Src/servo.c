/**
 * @file servo.c
 * @brief Control DRS Servos
 *
 * @author Carnegie Mellon Racing
 */

#include "servo.h"        // Board-specific CAN interface
#include "gpio.h"       // Board-specific GPIO interface
#include <CMR/pwm.h>        // PWM interface
#include <CMR/gpio.h>       // GPIO interface
#include <CMR/can_types.h>  // CMR CAN types

/** @brief PWM driver state. */
static cmr_pwm_t servo_left_PWM;
static cmr_pwm_t servo_right_PWM;

extern cmr_canCDCDRSStates_t drs_state;


static uint32_t dutyCycleToLeftDutyCycle (uint32_t duty_cycle) {
    return (duty_cycle == 0) ? DRS_MAX_DUTY_CYCLE: DRS_MAX_DUTY_CYCLE - duty_cycle;
}

static uint32_t dutyCycleToRightDutyCycle (uint32_t duty_cycle) {
    // Help with servo alignment in housing

	return (duty_cycle == 0) ? DRS_MIN_DUTY_CYCLE - 3 : DRS_MIN_DUTY_CYCLE + duty_cycle - 4;
}

void setServoQuiet() {
	cmr_pwmSetDutyCycle(&servo_right_PWM, 0);
	cmr_pwmSetDutyCycle(&servo_left_PWM, 0);
}

void setDrsPosition(uint32_t duty_cycle) {
    uint32_t right_duty_cycle = dutyCycleToRightDutyCycle(duty_cycle);
    uint32_t left_duty_cycle = dutyCycleToLeftDutyCycle(duty_cycle);
    drs_state.angle = (uint8_t) duty_cycle;
    drs_state.pwm_left = (uint8_t) left_duty_cycle;
    drs_state.pwm_right = (uint8_t) right_duty_cycle;
    cmr_pwmSetDutyCycle(&servo_right_PWM, right_duty_cycle);
    cmr_pwmSetDutyCycle(&servo_left_PWM, left_duty_cycle);
}

/**
 * @brief Task for initialising the servos.
 *
 * @return Does not return.
 */
void servoInit() {
    const cmr_pwmPinConfig_t pwmPinConfigLeft = { // GPIO A0
        .port = GPIOA, 
        .pin = GPIO_PIN_1,
        .channel = TIM_CHANNEL_1,
        .presc = 107,
        .period_ticks = 1000,
        .timer = TIM2
    };
    const cmr_pwmPinConfig_t pwmPinConfigRight = { // GPIO A1
        .port = GPIOA,
        .pin = GPIO_PIN_2,
        .channel = TIM_CHANNEL_2,
        .presc = 107,
        .period_ticks = 1000,
        .timer = TIM2
    };

    cmr_pwmInit(&servo_left_PWM, &pwmPinConfigLeft);
    cmr_pwmInit(&servo_right_PWM, &pwmPinConfigRight);
    setDrsPosition(DRS_CLOSED_DUTY_CYCLE);
}
