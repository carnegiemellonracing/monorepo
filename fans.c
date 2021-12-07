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

#define INVERTER_SCALE  (3.5)
#define INVERTER_OFFSET (-57.5)
//#define ACCUM_SCALE     (2.3) //Doesn't get to 100 fan speed. Tune this value during testing. TODO
#define ACCUM_OFFSET    (-1930) // And this one. TODO

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

    /* Enable the half bridges so that output isn't floating */
    cmr_gpioWrite(GPIO_FAN_1_ENABLE, 1);
    cmr_gpioWrite(GPIO_FAN_2_ENABLE, 1);
   

    /* Get reference to VSM Heartbeat and accumulator min/max cell temperatures */
    // volatile cmr_canHeartbeat_t *vsmHeartbeat = canGetPayload(CANRX_HEARTBEAT_VSM);
    // volatile cmr_canHVCPackMinMaxCellTemps_t *minMaxTemps = canGetPayload(CANRX_HVC_MINMAX_TEMPS);

    //cmr_canHeartbeat_t *heartbeat = &heartbeat;

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
                cmr_canHVCPackMinMaxCellTemps_t *minMaxTemps = canGetPayload(CANRX_HVC_MINMAX_TEMPS);
                // Use the temperature of the hottest cell as the AC's temperature
                uint16_t accum_temp = minMaxTemps->maxCellTemp_dC;


                //55C assumed high temperature, 25C assumed low temperature
                // ^ in dC: 550 dC and 250 dC
                //30 min speed, 100 max speed
                //a(accum_temp) + b = fan_speed
                fan_1_State = (accum_temp * 7 / 2) + ACCUM_OFFSET;
               
                int inverter_temp = (cmr_sensorListGetValue(&sensorList, SENSOR_CH_THERM_3) + cmr_sensorListGetValue(&sensorList, SENSOR_CH_THERM_4)) / 2;
                // 45C assumed high temperature (50C inverter starts to derate), 25C assumed low temperature
                // 30 min speed, 100 max speed 
                // a(inverter_temp) + b = fan speed 
                fan_2_State = (inverter_temp*INVERTER_SCALE) + INVERTER_OFFSET;

                cmr_pwmSetDutyCycle(&fan_1_PWM, (uint32_t) fan_1_State);
                cmr_pwmSetDutyCycle(&fan_2_PWM, (uint32_t) fan_2_State);
                cmr_gpioWrite(GPIO_FAN_1_ENABLE, 1);
                cmr_gpioWrite(GPIO_FAN_2_ENABLE, 1);
                break;

            // case CMR_CAN_HV_EN:
            //     //we need to change this also 

            //     fan_1_State = 50;
            //     fan_2_State = 50;
            //     cmr_pwmSetDutyCycle(&fan_1_PWM, fan_1_State);
            //     cmr_pwmSetDutyCycle(&fan_2_PWM, fan_2_State);
            //     cmr_gpioWrite(GPIO_FAN_1_ENABLE, 1);
            //     cmr_gpioWrite(GPIO_FAN_2_ENABLE, 1);
            //     break;
        	}
            default:
                fan_1_State = 0;
                fan_2_State = 0;
                cmr_pwmSetDutyCycle(&fan_1_PWM, fan_1_State);
                cmr_pwmSetDutyCycle(&fan_2_PWM, fan_2_State);
                cmr_gpioWrite(GPIO_FAN_1_ENABLE, 1);
                cmr_gpioWrite(GPIO_FAN_2_ENABLE, 1);
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
