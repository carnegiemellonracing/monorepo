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
static cmr_pwm_t pump_1_PWM;
static cmr_pwm_t pump_2_PWM;

/** @brief Pump control task priority. */
static const uint32_t pumpControl_priority = 4;
/** @brief Pump control task period. */
static const TickType_t pumpControl_period_ms = 50;
/** @brief Pump control task. */
static cmr_task_t pumpControl_task;

#define INVERTER_SCALE  (3.5)
#define INVERTER_OFFSET (-57.5)
//#define ACCUM_SCALE     (2.3) //Doesn't get to 100 fan speed. Tune this value during testing. TODO
#define ACCUM_OFFSET    (-28.3) // And this one. TODO

//extern cmr_sensor_t *sensors;

uint16_t accum_temp;
uint16_t inverter_temp;

/**
 * @brief Task for controlling the pumps.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void pumpControl(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    /* Enable the half bridges so that output isn't floating */
        cmr_gpioWrite(GPIO_PUMP_1_ENABLE, 1);
        cmr_gpioWrite(GPIO_PUMP_2_ENABLE, 1);

        /* Get reference to VSM Heartbeat */
        // volatile cmr_canHeartbeat_t *vsmHeartbeat = canGetPayload(CANRX_HEARTBEAT_VSM);

        //cmr_canHeartbeat_t *heartbeat = &heartbeat;

        /* Initialize PWM channels to 25kHz for fan control lines */
        /* 96Mhz / (24 * 40000) = 100Hz */
        /*
        const cmr_pwmPinConfig_t pwmPinConfig1 = {
            .port = GPIOB,
            .pin = GPIO_PIN_5,
            .channel = TIM_CHANNEL_2,
            .presc = 24,
            .period_ticks = 40000,
            .timer = TIM3
        };
        */
        const cmr_pwmPinConfig_t pwmPinConfig1 = {
            .port = GPIOB,
            .pin = GPIO_PIN_7,
            .channel = TIM_CHANNEL_2, //TODO
            .presc = 24,
            .period_ticks = 40000,
            .timer = TIM4
        };
        const cmr_pwmPinConfig_t pwmPinConfig2 = {
            .port = GPIOB,
            .pin = GPIO_PIN_8,
            .channel = TIM_CHANNEL_3, //TODO
            .presc = 24,
            .period_ticks = 40000,
            .timer = TIM4
        };
        cmr_pwmInit(&pump_1_PWM, &pwmPinConfig1);
        cmr_pwmInit(&pump_2_PWM, &pwmPinConfig2);
        // cmr_pwmInit(&channel_3_PWM, &pwmPinConfig3);

        TickType_t lastWakeTime = xTaskGetTickCount();
        while (1) {

            switch (heartbeat.state) {
                case CMR_CAN_HV_EN: // hv pump enable same as rtd pump enable
                case CMR_CAN_RTD:
                {
                    //int accum_temp = (cmr_sensorListGetValue(sensors, SENSOR_CH_THERM_5) + cmr_sensorListGetValue(sensors, SENSOR_CH_THERM_6)) / 2; //TODO: how to get value
                    // Below: revision - using CAN to get data from HVC
                    //Next line's citation: from what nsaizan wrote in this file above
                    cmr_canHVCPackMinMaxCellTemps_t *minMaxTemps = canGetPayload(CANRX_HVC_MINMAX_TEMPS);
                    // Use the temperature of the hottest cell as the AC's temperature
                    uint16_t accum_temp = minMaxTemps->maxCellTemp_dC;
                    
                    
                    //55C assumed high temperature, 25C assumed low temperature
                    //30 min speed, 100 max speed
                    //a(accum_temp) + b = fan_speed
                    pump_1_State = (accum_temp * 7 /3) - (3620 / 3);

                    uint16_t inverter_temp = (cmr_sensorListGetValue(&sensorList, SENSOR_CH_THERM_7) + cmr_sensorListGetValue(&sensorList, SENSOR_CH_THERM_8)) / 2;
                    // 45C assumed high temperature (50C inverter starts to derate), 25C assumed low temperature
                    // 30 min speed, 100 max speed 
                    // a(inverter_temp) + b = fan speed 
                    pump_2_State = (inverter_temp*INVERTER_SCALE) + INVERTER_OFFSET;

                    cmr_pwmSetDutyCycle(&pump_1_PWM, (uint32_t) pump_1_State);
                    cmr_pwmSetDutyCycle(&pump_2_PWM, (uint32_t) pump_2_State);
                    
                    cmr_gpioWrite(GPIO_PUMP_1_ENABLE, 1);
                    cmr_gpioWrite(GPIO_PUMP_2_ENABLE, 1);
                    break;
                // case CMR_CAN_HV_EN:
                //     pump_1_State = 100;
                //     pump_2_State = 100;
                //     cmr_pwmSetDutyCycle(&pump_1_PWM, pump_1_State);
                //     cmr_pwmSetDutyCycle(&pump_2_PWM, pump_2_State);
                //     cmr_gpioWrite(GPIO_PUMP_1_ENABLE, 1);
                //     cmr_gpioWrite(GPIO_PUMP_2_ENABLE, 1);
                //     break;
                }
                default:
                    pump_1_State = 0;
                    pump_2_State = 0;
                    cmr_pwmSetDutyCycle(&pump_1_PWM, pump_1_State);
                    cmr_pwmSetDutyCycle(&pump_2_PWM, pump_2_State);
                    cmr_gpioWrite(GPIO_PUMP_1_ENABLE, 1);
                    cmr_gpioWrite(GPIO_PUMP_2_ENABLE, 1);
                    break;
            }

            vTaskDelayUntil(&lastWakeTime, pumpControl_period_ms);
        }
}

/**
 * @brief Initialization of pump control task.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
void pumpInit() {
    cmr_taskInit(
        &pumpControl_task,
        "pumpControl",
        pumpControl_priority,
        pumpControl,
        NULL
    );
}
