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
// static cmr_pwm_t servo_left_PWM;
// static cmr_pwm_t servo_right_PWM;

static cmr_pwm_t servo_pwm;

#define DRS_CLOSED_ANGLE 80
#define DRS_OPENED_ANGLE 145

extern cmr_canCDCDRSStates_t drs_state;

void setServoQuiet() {
	cmr_pwmSetDutyCycle(&servo_pwm, 0);
    
    // set DCM DRS GPIO pins low
    cmr_gpioWrite(GPIO_DRS_ENABLE_1, 0);
    cmr_gpioWrite(GPIO_DRS_ENABLE_2, 0);
}

// process duty cycle (ratio of pwm signal)
static uint32_t angleToDutyCycle (int angle) {
    // map angle to percentage duty cycle
    // between five and ten percent?
    int percentDutyCycle = (angle / 270) * 100;
    return percentDutyCycle;
}


void setDRS(bool open) {

    // OR THE TARGET ANGLE ALSO COMES FROM SWANGLE
    int target_angle = open ? DRS_OPENED_ANGLE : DRS_CLOSED_ANGLE;
    float duty_percent = angletoDutyCycle(target_angle);

    cmr_pwmSetDutyCycle(&servo_pwm, duty_percent);

    cmr_gpioWrite(GPIO_DRS_ENABLE_1, 1);
    cmr_gpioWrite(GPIO_DRS_ENABLE_2, 1);
}

// static uint32_t dutyCycleToLeftDutyCycle (uint32_t duty_cycle) {
//     return (duty_cycle == 0) ? DRS_MAX_DUTY_CYCLE: DRS_MAX_DUTY_CYCLE - duty_cycle;
// }

// static uint32_t dutyCycleToRightDutyCycle (uint32_t duty_cycle) {
//     // Help with servo alignment in housing

// 	return (duty_cycle == 0) ? DRS_MIN_DUTY_CYCLE - 3 : DRS_MIN_DUTY_CYCLE + duty_cycle - 4;
// }

// void setDrsPosition(uint32_t duty_cycle) {
//     uint32_t right_duty_cycle = dutyCycleToRightDutyCycle(duty_cycle);
//     uint32_t left_duty_cycle = dutyCycleToLeftDutyCycle(duty_cycle);
//     drs_state.angle = (uint8_t) duty_cycle;
//     drs_state.pwm_left = (uint8_t) left_duty_cycle;
//     drs_state.pwm_right = (uint8_t) right_duty_cycle;
//     cmr_pwmSetDutyCycle(&servo_right_PWM, right_duty_cycle);
//     cmr_pwmSetDutyCycle(&servo_left_PWM, left_duty_cycle);

//     // servo write angle limits are 80 to 145

//     // set DCM DRS GPIO pins high
//     cmr_gpioWrite(GPIO_DRS_ENABLE_1, 1);
//     cmr_gpioWrite(GPIO_DRS_ENABLE_2, 1);
// }

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

    cmr_pwmInit(&servo_pwm, &pwmPinConfigLeft);
    cmr_pwmInit(&servo_pwm, &pwmPinConfigRight);
    setDRS(false);

    // cmr_pwmInit(&servo_left_PWM, &pwmPinConfigLeft);
    // cmr_pwmInit(&servo_right_PWM, &pwmPinConfigRight);
    // NEED TO UPDATE THIS BASED ON NEW DRS SET POSITION
    //setDrsPosition(DRS_CLOSED_DUTY_CYCLE);
}
