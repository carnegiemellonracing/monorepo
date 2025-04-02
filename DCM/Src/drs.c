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

#define DRS_CLOSED_ANGLE 80
#define DRS_OPENED_ANGLE 145

#define LAT_G_UPPER_THRESH 1.2
#define LAT_G_LOWER_THRESH 0.8

extern cmr_canCDCDRSStates_t drs_state;

void setServoQuiet() {
	cmr_pwmSetDutyCycle(&servo_pwm, 0);
    
    // set DCM DRS GPIO pins low
    cmr_gpioWrite(GPIO_DRS_ENABLE_1, 0);
    cmr_gpioWrite(GPIO_DRS_ENABLE_2, 0);
}


uint32_t angleToDutyCycle (int angle) {
    // map angle to percentage duty cycle
    // between five and ten percent?
    int percentDutyCycle = (angle / 270) * 100;
    return percentDutyCycle;
}

// math for relating swangle and velocity
float calculate_latg(int16_t swAngle_millideg) {
    if (swAngle_millideg == 0) {
        return 0.0f;
    }
    float lat_g = (calculate_yaw_rate_setpoint_radps(swAngle_millideg) * movella_get_velocity());
    return lat_g;
}

bool is_power_limited()
{
    const float pack_voltage_V = getPackVoltage();
    const float pack_current_A = getPackCurrent();
    const float pack_power_W = pack_voltage_V * pack_current_A;

    // exceeds power limit threshold 
    if (pack_power_W > (0.8 * getPowerLimit_W())) {
        return true;
    }
    return false;

}

void processDRSControl(int16_t swAngle_millideg, bool braking, 
                       bool traction_limited, bool skidpad) 
    {
        float lat_g = calculate_latg(swAngle_millideg);

        
        // DRS IS OPENED WHEN:
        // power limited 
        // G is below threshold
        bool opened = true;

        // DRS IS CLOSED WHEN:
        // braking, traction limited, skidpad, or G is past threshold
        if (braking || skidpad) {
            opened = false;
        }

        else if (!(is_power_limited()) && lat_g > LAT_G_UPPER_THRESH) {
            opened = false;
        }


        setDRS(opened);
    }


void setDRS(bool open) {

    int target_angle = open ? DRS_OPENED_ANGLE : DRS_CLOSED_ANGLE;
    float duty_percent = angleToDutyCycle(target_angle);

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
