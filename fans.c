/**
 * @file fans.c
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
static cmr_pwm_t fan_1_PWM;
static cmr_pwm_t fan_2_PWM;

/** @brief Fan control task priority. */
static const uint32_t fanControl_priority = 4;
/** @brief Fan control task period. */
static const TickType_t fanControl_period_ms = 50;
/** @brief Fan control task. */
static cmr_task_t fanControl_task;


#define FAN_AC_TEMP_LOW_dC 560
#define FAN_AC_TEMP_HIGH_dC 580
#define FAN_AC_STATE_LOW 30
#define FAN_AC_STATE_HIGH 100
#define FAN_INVERTER_TEMP_LOW_dC 560
#define FAN_INVERTER_TEMP_HIGH_dC 580
#define FAN_INVERTER_STATE_LOW 30
#define FAN_INVERTER_STATE_HIGH 100

//extern cmr_sensor_t *sensors;

uint16_t accum_temp;
uint16_t inverter_temp;



//TODO: 
/*

    // power errors(shunt resistor), water over heating errors, oil overheatin errors
    // no oil overheating errors cuz going into uprights
    // temperature 
    // pump always on 35 c  
    // pump turn on at 53 start turning on and 56 turning at 100
    // fan turn on at 56 starting 58 turn it to max
    change the values from looking at the loop
    2 loops -> 1 for motor inverter pump fan radiat, 1 for accumulator
    There are two loops
        one for motor inverter  -> radiator, pump, fan
        one accumulator -> radiator, pump, fan
    2 thermistors surrounding each radiator
    2 thermistors surrounding the inverter

    // pump turn on at 53 start turning on and 56 turning at 100
    // fan turn on at 56 starting 58 turn it to max

    First, we should try to read the temperature data from the amk's for inverter and accumulator to turn on the pumps and the fans. 
    If it doesn't work, then just use the avg the temperatures surrounding the inverter and accumulator loops. 
    Don't send errors. Send warnings. 

    1. fix the temperature readings, from amk to turn on the pumps and fans
    2. add warning checking 
    3. fix the 53, 56 values

    tune values of linear function?
    check implmentation of thermistor read values to celsius? do we hve to worry about that?
    can ?? hasn't been decided on
*/

/**
 * @brief Task for controlling the fans.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void fanControl(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    // TODO: Fix based on Datasheet
    /* Initialize PWM channels to 25kHz for fan control lines */
    /* 96Mhz / (24 * 160) = 25kHz */
    const cmr_pwmPinConfig_t pwmPinConfig1 = {
        .port = GPIOB,
        .pin = GPIO_PIN_4,
        .channel = TIM_CHANNEL_1,
        .presc = 24,
        .period_ticks = 160,
        .timer = TIM3
    };
    const cmr_pwmPinConfig_t pwmPinConfig2 = {
        .port = GPIOB,
        .pin = GPIO_PIN_5,
        .channel = TIM_CHANNEL_2,
        .presc = 24,
        .period_ticks = 160,
        .timer = TIM3
    };

    cmr_pwmInit(&fan_1_PWM, &pwmPinConfig1);
    cmr_pwmInit(&fan_2_PWM, &pwmPinConfig2);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {

        switch (heartbeat.state) {
            case CMR_CAN_HV_EN: // hv pump enable same as rtd pump enable
            case CMR_CAN_RTD:
            {
            	//int accum_temp = (cmr_sensorListGetValue(sensors, SENSOR_CH_THERM_1) + cmr_sensorListGetValue(sensors, SENSOR_CH_THERM_2)) / 2;
                // Below: revision - using CAN to get data from HVC
                //Next line's citation: from what nsaizan wrote in this file above
                cmr_canHVCPackMinMaxCellTemps_t *minMaxTemps = (cmr_canHVCPackMinMaxCellTemps_t *) canGetPayload(CANRX_HVC_MINMAX_TEMPS);
                // Use the temperature of the hottest cell as the AC's temperature
                uint16_t accum_temp = minMaxTemps->maxCellTemp_dC;
                // if accumtemp < 56 remain at low speed
                // if accumtemp > 58 remain at high speed
                // linear in between                

                if (accum_temp < FAN_AC_TEMP_LOW_dC) 
                    fan_1_State = 30;
                else if (accum_temp > FAN_AC_TEMP_HIGH_dC)
                    fan_1_State = 100;
                else {
                    fan_1_State = (FAN_AC_STATE_HIGH - FAN_AC_STATE_LOW) * (accum_temp - FAN_AC_TEMP_LOW_dC) / (FAN_AC_TEMP_HIGH_dC - FAN_AC_TEMP_LOW_dC) + FAN_AC_STATE_LOW;
                }
                fan_1_State = (fan_1_State > 100) ? 100 : fan_1_State;

                // Get igbt temperatures for each inverter.
                cmr_canAMKActualValues2_t *inv1_temps = (cmr_canAMKActualValues2_t *) canGetPayload(CANRX_INV1_STATUS);
                int16_t inv1IgbtTemp_dC = inv1_temps->igbtTemp_dC;
                cmr_canAMKActualValues2_t *inv2_temps = (cmr_canAMKActualValues2_t *) canGetPayload(CANRX_INV2_STATUS);
                int16_t inv2IgbtTemp_dC = inv2_temps->igbtTemp_dC;
                cmr_canAMKActualValues2_t *inv3_temps = (cmr_canAMKActualValues2_t *) canGetPayload(CANRX_INV3_STATUS);
                int16_t inv3IgbtTemp_dC = inv3_temps->igbtTemp_dC;
                cmr_canAMKActualValues2_t *inv4_temps = (cmr_canAMKActualValues2_t *) canGetPayload(CANRX_INV4_STATUS);
                int16_t inv4IgbtTemp_dC = inv4_temps->igbtTemp_dC;

                // Use average igbt temperature
                int16_t inverter_temp = (inv1IgbtTemp_dC + inv2IgbtTemp_dC + inv3IgbtTemp_dC + inv4IgbtTemp_dC) / 4;
                // if inverter_temp < 56 remain at low speed
                // if inverter_temp > 58 remain at high speed
                // linear in between                

                if (inverter_temp < FAN_INVERTER_TEMP_LOW_dC) 
                    fan_2_State = 30;
                else if (inverter_temp > FAN_INVERTER_TEMP_HIGH_dC)
                    fan_2_State = 100;
                else {
                    fan_2_State = (FAN_INVERTER_STATE_HIGH - FAN_INVERTER_STATE_LOW) * (inverter_temp - FAN_INVERTER_TEMP_LOW_dC) / (FAN_INVERTER_TEMP_HIGH_dC - FAN_INVERTER_TEMP_LOW_dC) + FAN_INVERTER_STATE_LOW;
                }
                fan_2_State = (fan_2_State > 100) ? 100 : fan_2_State;

                cmr_pwmSetDutyCycle(&fan_1_PWM, (uint32_t) fan_1_State);
                cmr_pwmSetDutyCycle(&fan_2_PWM, (uint32_t) fan_2_State);
                
                if (fan_1_State >= 50 || fan_2_State >= 50) {
                    cmr_gpioWrite(GPIO_FAN_ON, 1);
                } else {
                    cmr_gpioWrite(GPIO_FAN_ON, 0);
                }
                break;
        	}
            default:
                fan_1_State = 0;
                fan_2_State = 0;
                cmr_pwmSetDutyCycle(&fan_1_PWM, fan_1_State);
                cmr_pwmSetDutyCycle(&fan_2_PWM, fan_2_State);
                cmr_gpioWrite(GPIO_FAN_ON, 0);
                break;
        }

        vTaskDelayUntil(&lastWakeTime, fanControl_period_ms);
    }
}

/**
 * @brief Initialization of fan control task.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
void fanInit() {
    cmr_taskInit(
        &fanControl_task,
        "fanControl",
        fanControl_priority,
        fanControl,
        NULL
    );
}
