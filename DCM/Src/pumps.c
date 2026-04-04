/**
 * @file pumps.c
 * @brief Fan Control implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "can.h"        // Board-specific CAN interface
#include "gpio.h"       // Board-specific GPIO interface
#include "fans.h"        // Interface to implement
#include "pwm.h"
#include <CMR/pwm.h>        // PWM interface
#include <CMR/gpio.h>       // GPIO interface
#include <CMR/sensors.h>       // Sensors interface
#include "sensors.h"

/** @brief PWM driver state. */
static cmr_pwm_t pump_left_PWM;
static cmr_pwm_t pump_right_PWM;

cmr_pwmPinConfig_t pump_left = {
    .port = GPIOB,
    .pin = GPIO_PIN_6,
    .channel = TIM_CHANNEL_1,
    .presc = 24,
    .period_ticks = 40000,
    .timer = TIM4
};

cmr_pwmPinConfig_t pump_right = {
    .port = GPIOG,
    .pin = GPIO_PIN_13,
    .channel = TIM_CHANNEL_2,
    .presc = 24,
    .period_ticks = 40000,
    .timer = TIM23
};

/** @brief Pump control task priority. */
//static const uint32_t pumpControl_priority = 4;
/** @brief Pump control task period. */
//static const TickType_t pumpControl_period_ms = 50;
/** @brief Pump control task. */
//static cmr_task_t pumpControl_task;


// TODO: Tune these constants
#define PUMP_MOTOR_TEMP_LOW_dC 530
#define PUMP_MOTOR_TEMP_HIGH_dC 560
#define PUMP_MOTOR_STATE_LOW 30
#define PUMP_MOTOR_STATE_HIGH 100
#define PUMP_INVERTER_TEMP_LOW_dC 440
#define PUMP_INVERTER_TEMP_HIGH_dC 480
#define PUMP_INVERTER_STATE_LOW 30
#define PUMP_INVERTER_STATE_HIGH 100

#define MAX(a, b) ((a) > (b) ? (a) : (b))


//extern cmr_sensor_t *sensors;

static uint16_t accum_temp;
static uint16_t inverter_temp;

void pumpsOn();
void pumpsOff();

/**
 * @brief Task for controlling the pumps.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */

void pumpsOn() {

    cmr_gpioWrite(GPIO_PUMP_LEFT, 0);
    cmr_gpioWrite(GPIO_PUMP_RIGHT, 0);
    cmr_gpioWrite(GPIO_FAN_1, 0);
    cmr_gpioWrite(GPIO_FAN_2, 0);
    cmr_gpioWrite(GPIO_FAN_ON, 0);
    // cmr_gpioWrite(GPIO_PUMP_LEFT, 1);
    // cmr_gpioWrite(GPIO_PUMP_RIGHT, 1);
    // cmr_gpioWrite(GPIO_FAN_1, 1);
    // cmr_gpioWrite(GPIO_FAN_2, 1);
    // cmr_gpioWrite(GPIO_FAN_ON, 1);
    return;

    //int accum_temp = (cmr_sensorListGetValue(sensors, SENSOR_CH_THERM_1) + cmr_sensorListGetValue(sensors, SENSOR_CH_THERM_2)) / 2;
    // Below: revision - using CAN to get data from HVC
    //Next line's citation: from what nsaizan wrote in this file above
    // Use the temperature of the hottest cell as the AC's temperature
    // if accumtemp < 56 remain at low speed
    // if accumtemp > 58 remain at high speed
    // linear in between

    // Get igbt temperatures for each inverter.
    cmr_canDTI_TX_TempFault_t *inv1_temps = (cmr_canDTI_TX_TempFault_t *) canTractiveGetPayload(CANRX_TRAC_FL_TEMPFAULT);
    int16_t inv1MotorTemp_dC = inv1_temps->motor_temp;
    cmr_canDTI_TX_TempFault_t *inv2_temps = (cmr_canDTI_TX_TempFault_t *) canTractiveGetPayload(CANRX_TRAC_FR_TEMPFAULT);
    int16_t inv2MotorTemp_dC = inv2_temps->motor_temp;
    cmr_canDTI_TX_TempFault_t *inv3_temps = (cmr_canDTI_TX_TempFault_t *) canTractiveGetPayload(CANRX_TRAC_RL_TEMPFAULT);
    int16_t inv3MotorTemp_dC = inv3_temps->motor_temp;
    cmr_canDTI_TX_TempFault_t *inv4_temps = (cmr_canDTI_TX_TempFault_t *) canTractiveGetPayload(CANRX_TRAC_RR_TEMPFAULT);
    int16_t inv4MotorTemp_dC = inv4_temps->motor_temp;

    int16_t motor_temp_avg = (inv1MotorTemp_dC + inv2MotorTemp_dC + inv3MotorTemp_dC + inv4MotorTemp_dC) / 4;

    if (motor_temp_avg < PUMP_MOTOR_TEMP_LOW_dC)
        pump_Left_State = PUMP_MOTOR_STATE_LOW;
    else if (motor_temp_avg > PUMP_MOTOR_TEMP_HIGH_dC)
        pump_Left_State = PUMP_MOTOR_STATE_HIGH;
    else {
        pump_Left_State = ((PUMP_MOTOR_STATE_HIGH - PUMP_MOTOR_STATE_LOW) * (motor_temp_avg - PUMP_MOTOR_TEMP_LOW_dC)) / (PUMP_MOTOR_TEMP_HIGH_dC - PUMP_MOTOR_TEMP_LOW_dC) + PUMP_MOTOR_STATE_LOW;
    }
    pump_Left_State = (pump_Left_State < 100) ? pump_Left_State : 100;

    // Get igbt temperatures for each inverter.
    int16_t inv1IgbtTemp_dC = inv1_temps->ctlr_temp;
    int16_t inv2IgbtTemp_dC = inv2_temps->ctlr_temp;
    int16_t inv3IgbtTemp_dC = inv3_temps->ctlr_temp;
    int16_t inv4IgbtTemp_dC = inv4_temps->ctlr_temp;

    // Use average igbt temperature - derates at 50C
    int16_t inverter_temp = (inv1IgbtTemp_dC + inv2IgbtTemp_dC + inv3IgbtTemp_dC + inv4IgbtTemp_dC) / 4;
    // if inverter_temp < 44 remain at low speed
    // if inverter_temp > 48 remain at high speed
    // linear in between                

    if (inverter_temp < PUMP_INVERTER_TEMP_LOW_dC)
        pump_Right_State = PUMP_INVERTER_STATE_LOW;
    else if (inverter_temp > PUMP_INVERTER_TEMP_HIGH_dC)
        pump_Right_State = PUMP_INVERTER_STATE_HIGH;
    else {
        pump_Right_State = ((PUMP_INVERTER_STATE_HIGH - PUMP_INVERTER_STATE_LOW) * (inverter_temp - PUMP_INVERTER_TEMP_LOW_dC) / (PUMP_INVERTER_TEMP_HIGH_dC - PUMP_INVERTER_TEMP_LOW_dC)) + PUMP_INVERTER_STATE_LOW;
    }
    pump_Right_State = (pump_Right_State < 100) ? pump_Right_State : 100;

    // duty cycle is inverted because of MOSFETS
    pwmSetDutyCycle(PWM_PUMP_LEFT, (uint32_t) 100-pump_Left_State);
    pwmSetDutyCycle(PWM_PUMP_RIGHT, (uint32_t) 100-pump_Right_State);

    // if(MAX(inv1IgbtTemp_dC, MAX(inv2IgbtTemp_dC, MAX(inv3IgbtTemp_dC, inv4IgbtTemp_dC))) > 400) {
    //      cmr_gpioWrite(GPIO_PUMP_LEFT, 0);
    //  }
    //  else {
    //      cmr_gpioWrite(GPIO_PUMP_LEFT, 1);
    //  }
    // if(MAX(inv1MotorTemp_dC, MAX(inv2MotorTemp_dC, MAX(inv3MotorTemp_dC, inv4MotorTemp_dC))) > 750) {
    //      cmr_gpioWrite(GPIO_PUMP_RIGHT, 0);
    //  }
    //  else {
    //      cmr_gpioWrite(GPIO_PUMP_RIGHT, 1);
    //  }
    // cmr_gpioWrite(GPIO_PUMP_ON, 0);
    
    // if (pump_Left_State >= 50 || pump_Right_State >= 50) {
    //     cmr_gpioWrite(GPIO_PUMP_ON, 1);
    // } else {
    //     cmr_gpioWrite(GPIO_PUMP_ON, 0);
    // }
}

void pumpsOff() {
    pump_Left_State = 0;
    pump_Right_State = 0;
    // duty cycle is inverted because of MOSFETS
    // // we want them to be off when without AC
    // cmr_pwmSetDutyCycle(PWM_PUMP_LEFT, 100);
    // cmr_pwmSetDutyCycle(PWM_PUMP_RIGHT, 100);
    // cmr_gpioWrite(GPIO_PUMP_ON, 1);
    cmr_gpioWrite(GPIO_PUMP_LEFT, 1);
    cmr_gpioWrite(GPIO_PUMP_RIGHT, 1);
    cmr_gpioWrite(GPIO_FAN_1, 1);
    cmr_gpioWrite(GPIO_FAN_2, 1);
    cmr_gpioWrite(GPIO_FAN_ON, 1);
    // cmr_gpioWrite(GPIO_PUMP_ON, 0);
}

