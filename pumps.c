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


#define PUMP_TEMP_LOW_dC 530
#define PUMP_TEMP_HIGH_dC 560
#define PUMP_STATE_LOW 30
#define PUMP_STATE_HIGH 100


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
                        //int accum_temp = (cmr_sensorListGetValue(sensors, SENSOR_CH_THERM_1) + cmr_sensorListGetValue(sensors, SENSOR_CH_THERM_2)) / 2;
                    // Below: revision - using CAN to get data from HVC
                    //Next line's citation: from what nsaizan wrote in this file above
                    cmr_canHVCPackMinMaxCellTemps_t *minMaxTemps = (cmr_canHVCPackMinMaxCellTemps_t *) canGetPayload(CANRX_HVC_MINMAX_TEMPS);
                    // Use the temperature of the hottest cell as the AC's temperature
                    uint16_t accum_temp = minMaxTemps->maxCellTemp_dC;
                    // if accumtemp < 56 remain at low speed
                    // if accumtemp > 58 remain at high speed
                    // linear in between                

                    if (accum_temp < PUMP_TEMP_LOW_dC) 
                        pump_1_State = 30;
                    else if (accum_temp > PUMP_TEMP_HIGH_dC)
                        pump_1_State = 100;
                    else {
                        int16_t a = ((PUMP_STATE_HIGH - PUMP_STATE_LOW) / (PUMP_TEMP_HIGH_dC - PUMP_TEMP_LOW_dC)); // 58-56 = 2 / 100-30 = 70
                        int16_t b = (PUMP_STATE_LOW) - a * (PUMP_TEMP_LOW_dC); //y = ax + b b = y - ax
                        pump_1_State = a * accum_temp + b;
                    }

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

                    if (inverter_temp < PUMP_TEMP_LOW_dC) 
                        pump_2_State = 30;
                    else if (inverter_temp > PUMP_TEMP_HIGH_dC)
                        pump_2_State = 100;
                    else {
                        int16_t a = ((PUMP_STATE_HIGH - PUMP_STATE_LOW) / (PUMP_TEMP_HIGH_dC - PUMP_TEMP_LOW_dC));  // 58-56 = 2 / 100-30 = 70
                        int16_t b = (PUMP_STATE_LOW) - a * (PUMP_TEMP_LOW_dC); //y = ax + b b = y - ax
                        pump_2_State = a * inverter_temp + b;
                    }

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
