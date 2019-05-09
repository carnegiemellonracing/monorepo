/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface

#include <CMR/panic.h>      // cmr_panic()
#include <CMR/rcc.h>        // RCC interface
#include <CMR/can.h>        // CAN interface
#include <CMR/adc.h>        // ADC interface
#include <CMR/gpio.h>       // GPIO interface
#include <CMR/tasks.h>      // Task interface
#include <CMR/pwm.h>        // PWM interface
#include <CMR/can_types.h>  // CMR CAN types

#include "gpio.h"       // Board-specific GPIO interface
#include "can.h"        // Board-specific CAN interface
#include "adc.h"        // Board-specific ADC interface
#include "sensors.h"    // Board-specific sensors interface

static cmr_pwm_t fanPWM;

/** @brief Cell temperature at which PTC begins commanding AFC max cooling. */
static const uint32_t maxCoolingEnableTemp_C = 55;
/** @brief Cell temperature at which PTC stops commanding AFC max cooling. */
static const uint32_t minCoolingDisableTemp_C = 52;

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

/** @brief Brake light threshold (pounds-per-square-inch). */
static const uint16_t brakeLightThreshold_PSI = 20;

/** @brief Brake light control task priority. */
static const uint32_t brakelight_priority = 4;
/** @brief Brake light control task period. */
static const TickType_t brakelight_period_ms = 50;
/** @brief Brake light control task. */
static cmr_task_t brakelight_task;

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
            cmr_pwmSetDutyCycle(&fanPWM, 100);
        }
        else {
            cmr_pwmSetDutyCycle(&fanPWM, 0);
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

    // Get reference to accumulator min/max cell temperatures
    cmr_canRXMeta_t *minMaxTempsMeta = canRXMeta + CANRX_HVC_MINMAX_TEMPS;
    volatile cmr_canHVCPackMinMaxCellTemps_t *minMaxTemps = (void *) minMaxTempsMeta->payload;

    // Initialize fan PWM
    const cmr_pwmPinConfig_t pwmPinConfig = {
        .port = GPIOA,
        .pin = GPIO_PIN_8,
        .channel = TIM_CHANNEL_1,
        .presc = 24,
        .period_ticks = 40000,  // 96 MHz / (24 * 40000) = 100 Hz
        .timer = TIM1
    };
    cmr_pwmInit(&fanPWM, &pwmPinConfig);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        // Simple hysteresis for accumulator cooling
        uint32_t maxCellTemp_C = minMaxTemps->maxCellTemp_dC / 10;
        if (maxCellTemp_C >= maxCoolingEnableTemp_C) {
            afcMaxCoolingEnabled = true;
        }
        else if (maxCellTemp_C <= minCoolingDisableTemp_C) {
            afcMaxCoolingEnabled = false;
        }

        switch (vsmHeartbeat->state) {
            case CMR_CAN_RTD:
                fanState = CMR_CAN_FAN_HIGH;
                pumpState = CMR_CAN_PTC_PUMP_STATE_ON;
                cmr_pwmSetDutyCycle(&fanPWM, 100);  // Fan full on
                cmr_gpioWrite(GPIO_PUMP, 1);        // Pump on

                break;
            case CMR_CAN_HV_EN:
                fanState = CMR_CAN_FAN_LOW;
                pumpState = CMR_CAN_PTC_PUMP_STATE_ON;
                cmr_pwmSetDutyCycle(&fanPWM, 50);   // 50% duty cycle
                cmr_gpioWrite(GPIO_PUMP, 1);        // Pump on

                break;
            default:
                fanState = CMR_CAN_FAN_OFF;
                pumpState = CMR_CAN_PTC_PUMP_STATE_OFF;
                cmr_pwmSetDutyCycle(&fanPWM, 0);    // Fan off
                cmr_gpioWrite(GPIO_PUMP, 0);        // Pump off

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

    cmr_canRXMeta_t *vsmSensorsMeta = canRXMeta + CANRX_VSM_SENSORS;
    volatile cmr_canVSMSensors_t *vsmSensors = (void *) vsmSensorsMeta->payload;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        // TODO Switch to front brake pressure.
        if (vsmSensors->brakePressureRear_PSI > brakeLightThreshold_PSI) {
            cmr_gpioWrite(GPIO_BRAKELIGHT, 1);
        } else {
            cmr_gpioWrite(GPIO_BRAKELIGHT, 0);
        }

        vTaskDelayUntil(&lastWakeTime, brakelight_period_ms);
    }
}

/**
 * @brief Task for controlling the brake solenoid, which disconnects the rear
 * brakes from the brake pedal. Simultaneously, the CDC should begin using the
 * brake pressure to command regen torque on the rears.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void brakeDisconnect(void *pvParameters) {
    (void) pvParameters;

    cmr_canRXMeta_t *cdcSolenoidPTCMeta = canRXMeta + CANRX_CDC_SOLENOID_PTC;
    volatile cmr_canCDCSolenoidPTC_t *cdcSolenoidPTC = (void *) cdcSolenoidPTCMeta->payload;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_gpioWrite(GPIO_BRAKE_DISCON, cdcSolenoidPTC->solenoidEnable);

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

