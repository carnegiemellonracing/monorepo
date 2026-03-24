 /**
 * @file drs.c
 * @brief Control DRS Servos
 *
 * @author Carnegie Mellon Racing
 */

#include "drs.h"        // Board-specific CAN interface
#include "gpio.h"       // Board-specific GPIO interface
#include <CMR/pwm.h>        // PWM interface
#include <CMR/gpio.h>       // GPIO interface
#include <CMR/can_types.h>  // CMR CAN types
#include "constants.h"
#include <math.h>
#include "movella.h"
#include "safety_filter.h"
#include "controls.h"

/** @brief PWM driver state. */
// static cmr_pwm_t servo_left_PWM;
// static cmr_pwm_t servo_right_PWM;

static cmr_pwm_t servo_pwm;
float timeSinceStraightLine;
float timeSinceBraking;


// might need to change these depending on servo config
#define DRS_CLOSED_ANGLE 80
#define DRS_OPENED_ANGLE 145

#define LAT_G_UPPER_THRESH 1.2f
#define LAT_G_LOWER_THRESH 0.8f

#define TIME_THRESHOLD 0.5f
#define TIME_BRAKING_THRESHOLD 1.0f

extern cmr_canCDCDRSStates_t drs_state;

// TODO: add steering wheel button for DRS


void setServoQuiet() {
	cmr_pwmSetDutyCycle(&servo_pwm, 0);
    
    // set DCM DRS GPIO pins low
    cmr_gpioWrite(GPIO_DRS_ENABLE_1, 0);
    //cmr_gpioWrite(GPIO_DRS_ENABLE_2, 0);
}

uint32_t angleToDutyCycle (int angle) {
    // map angle to percentage duty cycle
    // between five and ten percent?
    int percentDutyCycle = (angle / 270) * 100;
    return percentDutyCycle;
}

// math for relating swangle and velocity, slightly redundant but we'll keep it cuz why not - still scales swangle
float calculate_latg(int16_t swAngle_millideg) {
    if (swAngle_millideg == 0) {
        return 0.0f;
    }
    float lat_g = (calculate_yaw_rate_setpoint_radps(swAngle_millideg) * movella_get_velocity());
    return lat_g;
}


// main DRS control function

void processDRSControl(int16_t swAngle_millideg, bool braking,
                       bool skidpad, bool accel) {
    
    float lat_g = calculate_latg(swAngle_millideg);

    timeSinceStraightLine += 0.005f;
    if (lat_g > LAT_G_UPPER_THRESH || skidpad) {
        timeSinceStraightLine = 0.0f;
    }
    timeSinceStraightLine = fminf(timeSinceStraightLine, 10.0f);

    if (braking) {
        timeSinceBraking = 0.0f;
    } else {
        timeSinceBraking += 0.005f;
    }

    bool cornering_hard = (lat_g > LAT_G_UPPER_THRESH) || (timeSinceStraightLine < TIME_THRESHOLD);
    bool recently_braked = (timeSinceBraking < TIME_BRAKING_THRESHOLD);

    bool opened = false;

    if (accel) {
        opened = !recently_braked && !skidpad;
    } else if (!cornering_hard && !recently_braked && !skidpad) {
        opened = true;
    }

    setDRS(opened);
}



void setDRS(bool open) {

    int target_angle = open ? DRS_OPENED_ANGLE : DRS_CLOSED_ANGLE;
    float duty_percent = angleToDutyCycle(target_angle);

    cmr_pwmSetDutyCycle(&servo_pwm, duty_percent);

    cmr_gpioWrite(GPIO_DRS_ENABLE_1, 1);
    // cmr_gpioWrite(GPIO_DRS_ENABLE_2, 1); // only one pin? 
}

/**
 * @brief Task for initialising the servos.
 *
 * @return Does not return.
 */
void servoInit() { //do we need to change this?
    const cmr_pwmPinConfig_t pwmPinConfigServo = { // GPIO A0
        .port = GPIOA, 
        .pin = GPIO_PIN_1,
        .channel = TIM_CHANNEL_1,
        .presc = 107,
        .period_ticks = 1000,
        .timer = TIM2
    };

    cmr_pwmInit(&servo_pwm, &pwmPinConfigServo);
    setDRS(false);
}
