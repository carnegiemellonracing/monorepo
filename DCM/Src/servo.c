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
#include "constants.h"
#include <math.h>

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
// when lift > drag, we want it to be OPEN
// skidpad always closed
// we want to check swangle but also velocity 
// we want swangle and velocity to find the lateral g on the car
// exceeds threshold of lateral g, then we open it and closed
// braking - we want it CLOSED
// power limited - OPEN
// traction limited - CLOSED
//    -- thresholding here: t limited at beg of accel, and during skidpad


static uint32_t angleToDutyCycle (int angle) {
    // map angle to percentage duty cycle
    // between five and ten percent?
    int percentDutyCycle = (angle / 270) * 100;
    return percentDutyCycle;
}



// math for relating swangle and velocity
float calculate_latg(int16_t swAngle_millideg, float velocity_mps) {
    
    if (swAngle_millideg == 0) {
        return 0.0f;
    }

    // accel = v^2 / r and the r comes from angle
    float swangle_rad = (swAngle_millideg / 1000.0f) * (M_PI / 180.0f); // millideg to rad

    // avoid div by 0 
    if (fabsf(tanf(swangle_rad)) < 1e-4) {
        return 0.0f;
    }

    // turning radius
    float radius = wheelbase_m  / tanf(swangle_rad);

    // lateral g = v^2 / r / 9.81
    float lat_g = (velocity_mps * velocity_mps) / (radius * 9.81f);

    return lat_g;
}





// take the swangle, and proportional to speed
void setDRS(bool open) {

    // OR THE TARGET ANGLE ALSO COMES FROM SWANGLE
    int target_angle = open ? DRS_OPENED_ANGLE : DRS_CLOSED_ANGLE;
    float duty_percent = angletoDutyCycle(target_angle);

    cmr_pwmSetDutyCycle(&servo_pwm, duty_percent);

    cmr_gpioWrite(GPIO_DRS_ENABLE_1, 1);
    cmr_gpioWrite(GPIO_DRS_ENABLE_2, 1);
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

    cmr_pwmInit(&servo_pwm, &pwmPinConfigLeft);
    cmr_pwmInit(&servo_pwm, &pwmPinConfigRight);
    setDRS(false);
}
