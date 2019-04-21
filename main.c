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

#include "gpio.h"   // Board-specific GPIO interface
#include "can.h"    // Board-specific CAN interface
#include "adc.h"    // Board-specific ADC interface

static cmr_pwmChannel_t pwmChannel;

/** @brief Status LED priority. */
static const uint32_t statusLED_priority = 2;

/** @brief Status LED period (milliseconds). */
static const TickType_t statusLED_period_ms = 250;

/** @brief Status LED task. */
static cmr_task_t statusLED_task;

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

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}

