/**
 * @file pumps.c
 * @brief Fan Control implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "can.h"        // Board-specific CAN interface
#include "gpio.h"       // Board-specific GPIO interface
#include "fans.h"        // Interface to implement
#include <CMR/pwm.h>        // PWM interface
#include <CMR/gpio.h>       // GPIO interface
#include <CMR/sensors.h>       // Sensors interface
#include "sensors.h"

/** @brief PWM driver state. */
static cmr_pwm_t pump_1_PWM;
static cmr_pwm_t pump_2_PWM;

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
    //int accum_temp = (cmr_sensorListGetValue(sensors, SENSOR_CH_THERM_1) + cmr_sensorListGetValue(sensors, SENSOR_CH_THERM_2)) / 2;
    // Below: revision - using CAN to get data from HVC
    //Next line's citation: from what nsaizan wrote in this file above
    // Use the temperature of the hottest cell as the AC's temperature
    // if accumtemp < 56 remain at low speed
    // if accumtemp > 58 remain at high speed
    // linear in between

    // Get igbt temperatures for each inverter.
    cmr_canAMKActualValues2_t *inv1_temps = (cmr_canAMKActualValues2_t *) canGetPayload(CANRX_INV1_STATUS);
    int16_t inv1MotorTemp_dC = inv1_temps->motorTemp_dC;
    cmr_canAMKActualValues2_t *inv2_temps = (cmr_canAMKActualValues2_t *) canGetPayload(CANRX_INV2_STATUS);
    int16_t inv2MotorTemp_dC = inv2_temps->motorTemp_dC;
    cmr_canAMKActualValues2_t *inv3_temps = (cmr_canAMKActualValues2_t *) canGetPayload(CANRX_INV3_STATUS);
    int16_t inv3MotorTemp_dC = inv3_temps->motorTemp_dC;
    cmr_canAMKActualValues2_t *inv4_temps = (cmr_canAMKActualValues2_t *) canGetPayload(CANRX_INV4_STATUS);
    int16_t inv4MotorTemp_dC = inv4_temps->motorTemp_dC;

    int16_t motor_temp_avg = (inv1MotorTemp_dC + inv2MotorTemp_dC + inv3MotorTemp_dC + inv4MotorTemp_dC) / 4;

    if (motor_temp_avg < PUMP_MOTOR_TEMP_LOW_dC)
        pump_1_State = PUMP_MOTOR_STATE_LOW;
    else if (motor_temp_avg > PUMP_MOTOR_TEMP_HIGH_dC)
        pump_1_State = PUMP_MOTOR_STATE_HIGH;
    else {
        pump_1_State = ((PUMP_MOTOR_STATE_HIGH - PUMP_MOTOR_STATE_LOW) * (motor_temp_avg - PUMP_MOTOR_TEMP_LOW_dC)) / (PUMP_MOTOR_TEMP_HIGH_dC - PUMP_MOTOR_TEMP_LOW_dC) + PUMP_MOTOR_STATE_LOW;
    }
    pump_1_State = (pump_1_State < 100) ? pump_1_State : 100;

    // Get igbt temperatures for each inverter.
    int16_t inv1IgbtTemp_dC = inv1_temps->igbtTemp_dC;
    int16_t inv2IgbtTemp_dC = inv2_temps->igbtTemp_dC;
    int16_t inv3IgbtTemp_dC = inv3_temps->igbtTemp_dC;
    int16_t inv4IgbtTemp_dC = inv4_temps->igbtTemp_dC;

    // Use average igbt temperature - derates at 50C
    int16_t inverter_temp = (inv1IgbtTemp_dC + inv2IgbtTemp_dC + inv3IgbtTemp_dC + inv4IgbtTemp_dC) / 4;
    // if inverter_temp < 44 remain at low speed
    // if inverter_temp > 48 remain at high speed
    // linear in between                

    if (inverter_temp < PUMP_INVERTER_TEMP_LOW_dC)
        pump_2_State = PUMP_INVERTER_STATE_LOW;
    else if (inverter_temp > PUMP_INVERTER_TEMP_HIGH_dC)
        pump_2_State = PUMP_INVERTER_STATE_HIGH;
    else {
        pump_2_State = ((PUMP_INVERTER_STATE_HIGH - PUMP_INVERTER_STATE_LOW) * (inverter_temp - PUMP_INVERTER_TEMP_LOW_dC) / (PUMP_INVERTER_TEMP_HIGH_dC - PUMP_INVERTER_TEMP_LOW_dC)) + PUMP_INVERTER_STATE_LOW;
    }
    pump_2_State = (pump_2_State < 100) ? pump_2_State : 100;

    // duty cycle is inverted because of MOSFETS
    // cmr_pwmSetDutyCycle(&pump_1_PWM, (uint32_t) 100-pump_1_State);
    // cmr_pwmSetDutyCycle(&pump_2_PWM, (uint32_t) 100-pump_2_State);

    cmr_gpioWrite(GPIO_PUMP_LEFT, 0);
    cmr_gpioWrite(GPIO_PUMP_RIGHT, 0);
    cmr_gpioWrite(GPIO_PUMP_ON, 0);
    
    // if (pump_1_State >= 50 || pump_2_State >= 50) {
    //     cmr_gpioWrite(GPIO_PUMP_ON, 1);
    // } else {
    //     cmr_gpioWrite(GPIO_PUMP_ON, 0);
    // }
}

void pumpsOff() {
    pump_1_State = 0;
    pump_2_State = 0;
    // duty cycle is inverted because of MOSFETS
    // // we want them to be off when without AC
    // cmr_pwmSetDutyCycle(&pump_1_PWM, 100-pump_1_State);
    // cmr_pwmSetDutyCycle(&pump_2_PWM, 100-pump_2_State);
    cmr_gpioWrite(GPIO_PUMP_LEFT, 1);
    cmr_gpioWrite(GPIO_PUMP_RIGHT, 1);
    cmr_gpioWrite(GPIO_PUMP_ON, 1);
}

