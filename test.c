#include <stm32f4xx_hal.h>  // HAL interface
#include <FreeRTOS.h>       // FreeRTOS API
#include <task.h>           // Task interface
#include <stdbool.h>
#include <stdio.h>

#include <CMR/tasks.h>      // CMR task drivers 
#include <CMR/gpio.h>

#include "test.h"
#include "gpio.h"
#include "adc.h"

void testGPIOWrite()
{
    cmr_gpioToggle(GPIO_LED_STATUS);
    cmr_gpioToggle(GPIO_LED_IMD);
    cmr_gpioToggle(GPIO_LED_AMS);
    cmr_gpioToggle(GPIO_LED_BSPD);
    cmr_gpioToggle(GPIO_PD_N);
}

void testGPIORead()
{
    volatile int read;
    for (size_t pin = GPIO_SS_MODULE; pin <= GPIO_SS_BOTS; pin++)
    {
        read = cmr_gpioRead(pin);
        printf("GPIO Pin %u: %d\n", pin, read);
    }
}

void testADCRead()
{
    volatile uint32_t read;
    for (size_t ch = ADC_VSENSE; ch < ADC_LEN; ch++)
    {
        read = adcRead(ADC_VSENSE);
        printf("ADC Channel %u: %ld\n", ch, read);
    }
}

/** @brief Test task priority. */
static const uint32_t test_priority = 3;

/** @brief Test task period (milliseconds). */
static const TickType_t test_period_ms = 250;

/** @brief Test task. */
static cmr_task_t test_task;

static void test(void *pvParameters) {
    (void) pvParameters;

    for (
        TickType_t lastWakeTime = xTaskGetTickCount();
        1;
        vTaskDelayUntil(&lastWakeTime, test_period_ms)
    ) {
        testGPIOWrite();
        testGPIORead();
        testADCRead();
    }
}

void testInit()
{
    cmr_taskInit(
        &test_task,
        "test",
        test_priority,
        test,
        NULL
    );
}
