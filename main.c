/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/can.h>    // CAN interface
#include <CMR/adc.h>    // ADC interface
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/tasks.h>  // Task interface
#include <CMR/pwm.h>    // PWM interface
#include <CMR/can_types.h>  // CMR CAN types

#include "gpio.h"   // Board-specific GPIO interface
#include "can.h"    // Board-specific CAN interface
#include "adc.h"    // Board-specific ADC interface
#include "sensors.h"    // Board-specific sensors interface

static cmr_pwmChannel_t pwmChannel;

/** @brief Status LED task priority. */
static const uint32_t statusLED_priority = 2;
/** @brief Status LED task period. */
static const TickType_t statusLED_period_ms = 250;
/** @brief Status LED task. */
static cmr_task_t statusLED_task;

/** @brief Cooling control task priority. */
static const uint32_t coolingControl_priority = 4;
/** @brief Cooling control task period. */
static const TickType_t coolingControl_period_ms = 50;
/** @brief Cooling control task. */
static cmr_task_t coolingControl_task;

/** @brief Brake light control task priority. */
static const uint32_t brakelight_priority = 4;
/** @brief Brake light control task period. */
static const TickType_t brakelight_period_ms = 50;
/** @brief Brake light control task. */
static cmr_task_t brakelight_task;

/** @brief DCDC 5V input preapplication time. */
static const uint32_t dcdc_preapply_period_ms = 500;

/** @brief Brake disconnection solenoid task priority. */
static const uint32_t brakeDisconnect_priority = 5;
/** @brief Brake disconnection solenoid task period. */
static const TickType_t brakeDisconnect_period_ms = 10;
/** @brief Brake disconnection solenoid task. */
static cmr_task_t brakeDisconnect_task;

/**
 * @brief Task for toggling the status LED.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void statusLED(void *pvParameters) {
    (void) pvParameters;

    cmr_gpioWrite(GPIO_LED_STATUS, 0);
    cmr_gpioWrite(GPIO_FAN_ENABLE, 1); // Turn the fan driver on

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_gpioToggle(GPIO_LED_STATUS);

        if (lastWakeTime > 10000) {
//            cmr_pwmSetPeriod(&pwmChannel, 40000, 48);
        }
        else if (lastWakeTime > 5000) {
            cmr_pwmSetDutyCycle(&pwmChannel, 100);
        }
        else {
            cmr_pwmSetDutyCycle(&pwmChannel, 0);
        }


        vTaskDelayUntil(&lastWakeTime, statusLED_period_ms);
    }
}

/**
 * @brief Task for controlling the radiator fan and pump.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void coolingControl(void *pvParameters) {
    (void) pvParameters;

    cmr_gpioWrite(GPIO_FAN_ENABLE, 1); // Turn the fan driver on

    // Get reference to VSM Heartbeat
    cmr_canRXMeta_t *vsmHeartbeatMeta = &(canRXMeta[CANRX_HEARTBEAT_VSM]);
    volatile cmr_canHeartbeat_t *vsmHeartbeat = (void *)(&vsmHeartbeatMeta->payload);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        switch (vsmHeartbeat->state) {
            case CMR_CAN_RTD:
                fanState = CMR_CAN_FAN_HIGH;
                pumpState = CMR_CAN_PTC_PUMP_STATE_ON;
                cmr_gpioWrite(GPIO_FAN, 1);     // Fan full on
                cmr_gpioWrite(GPIO_PUMP, 1);    // Pump on

                break;
            case CMR_CAN_HV_EN:
                fanState = CMR_CAN_FAN_LOW;
                pumpState = CMR_CAN_PTC_PUMP_STATE_ON;
                cmr_gpioToggle(GPIO_FAN);       // 50% duty cycle :)
                cmr_gpioWrite(GPIO_PUMP, 1);    // Pump on

                break;
            default:
                fanState = CMR_CAN_FAN_OFF;
                pumpState = CMR_CAN_PTC_PUMP_STATE_OFF;
                cmr_gpioWrite(GPIO_FAN, 0);     // Fan off
                cmr_gpioWrite(GPIO_PUMP, 0);    // Pump off

                break;
        }

        vTaskDelayUntil(&lastWakeTime, coolingControl_period_ms);
    }
}

/**
 * @brief Task for controlling the brakelight.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void brakelight(void *pvParameters) {
    (void) pvParameters;

    // Get reference to VSM Heartbeat
    cmr_canRXMeta_t *fsmDataMeta = &(canRXMeta[CANRX_FSM_DATA]);
    volatile cmr_canFSMData_t *fsmData = (void *) fsmDataMeta->payload;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        if (fsmData->brakePressureFront_PSI > 0) {
            cmr_gpioWrite(GPIO_BRAKELIGHT, 1);
        } else {
            cmr_gpioWrite(GPIO_BRAKELIGHT, 0);
        }

        vTaskDelayUntil(&lastWakeTime, brakelight_period_ms);
    }
}

/**
 * @brief Task for closing the dcdc output relay in response to VSM internal state
 * changes. Task is not renamed because it uses existing brake solenoid infrastructure
 * and will be reverted later
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void brakeDisconnect(void *pvParameters) {
    (void) pvParameters;
    // Currently used to switch dcdc relay by observing VSM internal state
    cmr_canRXMeta_t *VSM_statusMeta = canRXMeta + CANRX_VSM_STATUS;
    volatile cmr_canVSMStatus_t *VSM_status = (void *) VSM_statusMeta->payload;
    TickType_t lastWakeTime = xTaskGetTickCount();

    bool enable = false;
    TickType_t enableTime = 0;

    while (1) {
        switch (VSM_status->internalState) {
        	case CMR_CAN_VSM_STATE_DCDC_EN:
        	case CMR_CAN_VSM_STATE_HV_EN:
        	case CMR_CAN_VSM_STATE_RTD:
                if (!enable) {
                    enable = true;
                    enableTime = lastWakeTime;
                }
        		break;
        	// dcdc relay should stay open with respect to any other observed VSM states
            default:
                enable = false;
                break;
        }

        if (enable && (lastWakeTime - enableTime > dcdc_preapply_period_ms)) {
            cmr_gpioWrite(GPIO_BRAKE_DISCON, 1);
        } else {
            cmr_gpioWrite(GPIO_BRAKE_DISCON, 0);
        }

        vTaskDelayUntil(&lastWakeTime, brakeDisconnect_period_ms);
    }
}

/**
 * @brief Firmware entry point.
 *
 * Device configuration and task initialization should be performed here.
 *
 * @return Does not return.
 */
int main(void) {
    // System initialization.
    HAL_Init();
    cmr_rccSystemClockEnable();

    // Peripheral configuration.
    gpioInit();
    canInit();
    adcInit();
    sensorsInit();

    const cmr_pwmPinConfig_t pwmPinConfig = {
        .port = GPIOA,
        .pin = GPIO_PIN_8,
        .channel = TIM_CHANNEL_1,
        .presc = 24,
        .period_ticks = 40000,  // 96 MHz / (24 * 40000) = 100 Hz
        .timer = TIM1
    };

    cmr_pwmInit(&pwmPinConfig, &pwmChannel);

    cmr_taskInit(
        &statusLED_task,
        "statusLED",
        statusLED_priority,
        statusLED,
        NULL
    );
    cmr_taskInit(
        &brakelight_task,
        "brakelight",
        brakelight_priority,
        brakelight,
        NULL
    );
    cmr_taskInit(
        &brakeDisconnect_task,
        "brakeDisconnect",
        brakeDisconnect_priority,
        brakeDisconnect,
        NULL
    );
    cmr_taskInit(
        &coolingControl_task,
        "coolingControl",
        coolingControl_priority,
        coolingControl,
        NULL
    );

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}

