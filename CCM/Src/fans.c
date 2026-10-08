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
#include "state.h"


/** @brief PWM driver state. */
static cmr_pwm_t fan_1_PWM;

/** @brief Fan control task priority. */
static const uint32_t fanControl_priority = 4;
/** @brief Fan control task period. */
static const TickType_t fanControl_period_ms = 50;
/** @brief Fan control task. */
static cmr_task_t fanControl_task;

#define FAN_CCM_TEMP_LOW_dC 350
#define FAN_CCM_TEMP_HIGH_dC 380
#define FAN_CCM_STATE_LOW 30
#define FAN_CCM_STATE_HIGH 100


extern cmr_sensor_t *sensors;

/**
 * @brief Task for controlling the fans.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void fanControl(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    /* Initialize PWM channels to 25kHz for fan control lines */
    /* 96Mhz / (24 * 160) = 25kHz */
    const cmr_pwmPinConfig_t pwmPinConfig1 = {
        .port = GPIOC,
        .pin = GPIO_PIN_7,
        .channel = TIM_CHANNEL_2,
        .presc = 24,
        .period_ticks = 160,
        .timer = TIM8
    };

    cmr_pwmInit(&fan_1_PWM, &pwmPinConfig1);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        //Keep air temp inside charger under 40 degrees C
        
    	uint8_t fan_1_State;
        switch (state) {
            case CMR_CCM_STATE_CHARGE_REQ: // hv pump enable same as rtd pump enable
            case CMR_CCM_STATE_CHARGE:
            {
            	//int accum_temp = (cmr_sensorListGetValue(sensors, SENSOR_CH_THERM_1) + cmr_sensorListGetValue(sensors, SENSOR_CH_THERM_2)) / 2;
                // Below: revision - using CAN to get data from HVC
                //Next line's citation: from what nsaizan wrote in this file above
                // cmr_canHVCPackMinMaxCellTemps_t *minMaxTemps = (cmr_canHVCPackMinMaxCellTemps_t *) canGetPayload(CANRX_HVC_MINMAX_TEMPS);
                
                // Use the temperature of the hottest cell as the AC's temperature
                uint16_t ccm_temp = (cmr_sensorListGetValue(&sensorList, SENSOR_CH_THERM_1) + cmr_sensorListGetValue(&sensorList, SENSOR_CH_THERM_2)) / 2;
                // if accumtemp < 35 remain at low speed
                // if accumtemp > 38 remain at high speed
                // linear in between                

                if (ccm_temp < FAN_CCM_TEMP_LOW_dC) 
                    fan_1_State = 30;
                else if (ccm_temp > FAN_CCM_TEMP_HIGH_dC)
                    fan_1_State = 100;
                else {
                    fan_1_State = (FAN_CCM_STATE_HIGH - FAN_CCM_STATE_LOW) * (ccm_temp - FAN_CCM_TEMP_LOW_dC) / (FAN_CCM_TEMP_HIGH_dC - FAN_CCM_TEMP_LOW_dC) + FAN_CCM_STATE_LOW;
                }
                fan_1_State = (fan_1_State > 100) ? 100 : fan_1_State;

                // duty cycle is inverted because of MOSFETS
                cmr_pwmSetDutyCycle(&fan_1_PWM, (uint32_t) 100-fan_1_State);
                break;
        	}
            default:
                fan_1_State = 0;
                // duty cycle is inverted because of MOSFETS
                // should be off until RTD
                cmr_pwmSetDutyCycle(&fan_1_PWM, 100-0);
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
