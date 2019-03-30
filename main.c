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
#include <CMR/can_types.h>  // CMR CAN types

#include "gpio.h"   // Board-specific GPIO interface
#include "can.h"    // Board-specific CAN interface
#include "adc.h"    // Board-specific ADC interface
#include "sensors.h"    // Board-specific sensors interface

/** Task priorities and periods. */
static const uint32_t statusLED_priority = 2;
static const TickType_t statusLED_period_ms = 250;
static cmr_task_t statusLED_task;

static const uint32_t coolingControl_priority = 4;
static const TickType_t coolingControl_period_ms = 50;
static cmr_task_t coolingControl_task;

static const uint32_t brakelight_priority = 4;
static const TickType_t brakelight_period_ms = 50;
static cmr_task_t brakelight_task;

static const uint32_t brakeDisconnect_priority = 5;
static const TickType_t brakeDisconnect_period_ms = 10;
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

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_gpioToggle(GPIO_LED_STATUS);

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

	cmr_gpioToggle(GPIO_FAN_ENABLE); // Turn the fan driver on

    // Get reference to VSM Heartbeat
    cmr_canRXMeta_t *heartbeatVSMMeta = canRXMeta + CANRX_HEARTBEAT_VSM;
    volatile cmr_canHeartbeat_t *heartbeatVSM = (void *) heartbeatVSMMeta->payload;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_canHeartbeat_t heartbeat = {
            .state = heartbeatVSM->state
        };

        switch (heartbeat.state){
            case CMR_CAN_HV_EN:
                fanStatePTC = CMR_CAN_FAN_LOW;
                pumpStatePTC = 1;
                // Fan Low
                cmr_gpioToggle(GPIO_FAN); // 50% duty cycle :)
                // Pump on
                cmr_gpioWrite(GPIO_PUMP, 1);
                break;
            case CMR_CAN_RTD:
                fanStatePTC = CMR_CAN_FAN_HIGH;
                pumpStatePTC = 1;
                // Fan Full
                cmr_gpioWrite(GPIO_FAN, 1);
                // Pump on
                cmr_gpioWrite(GPIO_PUMP, 1);
                break;
            default:
                fanStatePTC = CMR_CAN_FAN_OFF;
                pumpStatePTC = 0;
                // Fan Off
                cmr_gpioWrite(GPIO_FAN, 0);
                // Pump off
                cmr_gpioWrite(GPIO_PUMP, 0);
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
    cmr_canRXMeta_t *dataFSMMeta = canRXMeta + CANRX_FSM_DATA;
    volatile cmr_canFSMData_t *dataFSM = (void *) dataFSMMeta->payload;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        if (dataFSM->brakePressureFront_PSI > 0) {
            cmr_gpioWrite(GPIO_BRAKELIGHT, 1);
        } else {
            cmr_gpioWrite(GPIO_BRAKELIGHT, 0);
        }

        vTaskDelayUntil(&lastWakeTime, brakelight_period_ms);
    }
}

/**
 * @brief Task for controlling the brake solenoid, which
 * disconnects the rear brakes from the brake pedal. Simultaneously,
 * the CDC should begin using the brake pressure to command
 * regen torque on the rears.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void brakeDisconnect(void *pvParameters) {
    (void) pvParameters;

    // Get reference to VSM Heartbeat
    cmr_canRXMeta_t *heartbeatVSMMeta = canRXMeta + CANRX_HEARTBEAT_VSM;
    volatile cmr_canHeartbeat_t *heartbeatVSM = (void *) heartbeatVSMMeta->payload;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        switch (heartbeatVSM->state){
            case CMR_CAN_RTD:
                // Check that the dash is requesting this mode of operation
                if (0) { // TODO Dash/TOM should publish a driver setting for enabling software braking
                    cmr_gpioWrite(GPIO_BRAKE_DISCON, 1);
                } else {
                    cmr_gpioWrite(GPIO_BRAKE_DISCON, 0);
                }
                break;
            default:
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
    sensorInit();

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

