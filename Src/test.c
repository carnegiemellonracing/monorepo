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
#include "expanders.h"

void testGPIOWrite()
{
    cmr_gpioToggle(GPIO_LED_IMD);
    cmr_gpioToggle(GPIO_LED_AMS);
    cmr_gpioToggle(GPIO_LED_BSPD);
}

void testGPIORead()
{
    volatile int read;
    for (size_t pin = GPIO_LED_0; pin < GPIO_LEN; pin++)
    {
        read = cmr_gpioRead(pin);
        printf("GPIO Pin %u: %d\n", pin, read);
    }
}

//void testExpanderWrite()
//{
//    static bool toggle = false;
//    expanderSetLED(EXP_LED_1, toggle);
//    expanderSetLED(EXP_LED_2, toggle);
//    toggle = !toggle;
//}

void testADCRead()
{
//    volatile uint32_t read;
//    for (size_t ch = ADC_VSENSE; ch < ADC_LEN; ch++)
//    {
//        read = adcRead(ADC_VSENSE);
//        printf("ADC Channel %u: %ld\n", ch, read);
//    }
}

/** @brief Test task priority. */
static const uint32_t test_priority = 3;

/** @brief Test task period (milliseconds). */
static const TickType_t test_period_ms = 250;

/** @brief Test task. */
static cmr_task_t test_task;

static void test(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        vTaskDelayUntil(&lastWakeTime, test_period_ms);
    //    testGPIOWrite();
        // testGPIORead();
        // testADCRead();
        testExpanderWrite();
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
