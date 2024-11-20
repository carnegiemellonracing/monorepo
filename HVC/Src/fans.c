/*
 * fans.c
 *
 *  Created on: Aug 15, 2023
 *      Author: sidsr
 */
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


/** @brief PWM driver state. */
static cmr_pwm_t fan_PWM;

uint16_t fanState;


/** @brief Fan control task priority. */
static const uint32_t fanControl_priority = 4;
/** @brief Fan control task period. */
static const TickType_t fanControl_period_ms = 50;
/** @brief Fan control task. */
static cmr_task_t fanControl_task;


#define FAN_AC_TEMP_LOW_dC 530
#define FAN_AC_TEMP_HIGH_dC 580
#define FAN_AC_STATE_LOW 0
#define FAN_AC_STATE_HIGH 100


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
        .port = GPIOA,
        .pin = GPIO_PIN_6,
        .channel = TIM_CHANNEL_1,
        .presc = 24,
        .period_ticks = 160,
        .timer = TIM3
    };

    cmr_pwmInit(&fan_PWM, &pwmPinConfig1);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
    	cmr_canHVCState_t state = getState();
        switch (state) {
            case CMR_CAN_HVC_STATE_CHARGE_CONSTANT_VOLTAGE:
            case CMR_CAN_HVC_STATE_CHARGE_CONSTANT_CURRENT:
            case CMR_CAN_HVC_MODE_RUN:
            {

                uint16_t accum_temp = getPackMaxCellTemp();
                // Use the temperature of the hottest cell as the AC's temperature
                // if accumtemp < 56 remain at low speed
                // if accumtemp > 58 remain at high speed
                // linear in between

                if (accum_temp < FAN_AC_TEMP_LOW_dC)
                    fanState = FAN_AC_STATE_LOW;
                else if (accum_temp > FAN_AC_TEMP_HIGH_dC)
                    fanState = FAN_AC_STATE_HIGH;
                else {
                    fanState = (FAN_AC_STATE_HIGH - FAN_AC_STATE_LOW) * (accum_temp - FAN_AC_TEMP_LOW_dC) / (FAN_AC_TEMP_HIGH_dC - FAN_AC_TEMP_LOW_dC) + FAN_AC_STATE_LOW;
                }
                fanState = (fanState > 100) ? 100 : fanState;


                cmr_pwmSetDutyCycle(&fan_PWM, (uint32_t) fanState);

                break;
            }

            default:
                fanState = 0;
                cmr_pwmSetDutyCycle(&fan_PWM, 0);
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


