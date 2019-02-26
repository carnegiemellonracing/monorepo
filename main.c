/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface

#include <FreeRTOS.h>       // FreeRTOS interface
#include <task.h>           // xTaskCreate(), vTaskStartScheduler()

#include <CMR/panic.h>      // cmr_panic()
#include <CMR/rcc.h>        // RCC interface
#include <CMR/can.h>        // CAN interface
#include <CMR/adc.h>        // ADC interface
#include <CMR/gpio.h>       // cmr_gpioToggle
#include <CMR/can_types.h>  // CMR can

#include "gpio.h"   // Board-specific GPIO interface
#include "can.h"    // Board-specific CAN interface
#include "adc.h"    // Board-specific ADC interface

/** Task priorities and periods. */
static const uint32_t statusLEDPriority = 2;
static const TickType_t statusLEDPeriod_ms = 250;
static const uint32_t coolingControlPriority = 4;
static const TickType_t coolingControlPeriod_ms = 50;
static const uint32_t brakelightPriority = 4;
static const TickType_t brakelightPeriod_ms = 50;
static const uint32_t brakeDisconnectPriority = 5;
static const TickType_t brakeDisconnectPeriod_ms = 10;

/**
 * @brief Task for toggling the status LED.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void statusLEDTask(void *pvParameters) {
    cmr_gpioWrite(GPIO_LED_STATUS, 0);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_gpioToggle(GPIO_LED_STATUS);

        vTaskDelayUntil(&lastWakeTime, statusLEDPeriod_ms);
    }
}

/**
 * @brief Task for controlling the radiator fan and pump.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void coolingControlTask(void *pvParameters) {
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

        vTaskDelayUntil(&lastWakeTime, coolingControlPeriod_ms);
    }
}

/**
 * @brief Task for controlling the brakelight.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void brakelightTask(void *pvParameters) {
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

        vTaskDelayUntil(&lastWakeTime, brakelightPeriod_ms);
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
static void brakeDisconnectTask(void *pvParameters) {
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

        vTaskDelayUntil(&lastWakeTime, statusLEDPeriod_ms);
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

    // Node tasks.
    xTaskCreate(
        statusLEDTask, "statusLED",
        configMINIMAL_STACK_SIZE, NULL,
        statusLEDPriority, NULL
    );
    xTaskCreate(
        brakelightTask, "brakelight",
        configMINIMAL_STACK_SIZE, NULL,
        brakelightPriority, NULL
    );
    xTaskCreate(
        brakeDisconnectTask, "brakeDisconnect",
        configMINIMAL_STACK_SIZE, NULL,
        brakeDisconnectPriority, NULL
    );
    xTaskCreate(
        coolingControlTask, "coolingControl",
        configMINIMAL_STACK_SIZE, NULL,
        coolingControlPriority, NULL
    );

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}

