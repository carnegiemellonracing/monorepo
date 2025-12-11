/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/tasks.h>  // Task interface

#include "gpio.h"   // Board-specific GPIO interface
#include "can.h"    // Board-specific CAN interface
#include "adc.h"    // Board-specific ADC interface
#include "bms_error.h"
#include "watchdog.h"   // Board-specific Watchdog interface
#include "fans.h" // Board-specific Fan interface
#include "bq_interface.h"

/** @brief Status LED priority. */
static const uint32_t statusLED_priority = 2;

/** @brief BMB Sample Task priority. */
static const uint32_t bmbSample_priority = 3;

/** @brief BMB Sample Task priority. */
static const uint32_t setState_priority = 4;

/** @brief Status LED period (milliseconds). */
static const TickType_t statusLED_period_ms = 250;

/** @brief Status LED task. */
static cmr_task_t statusLED_task;

/** @brief BMB Sample Task */
static cmr_task_t bmbSample_task;

/** @brief Set State Task */
static cmr_task_t setState_task;

/**
 * @brief Task for toggling the status LED.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void statusLED(void *pvParameters) {
    (void) pvParameters;

    cmr_gpioWrite(GPIO_MCU_LED, 0);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_gpioToggle(GPIO_MCU_LED);

        vTaskDelayUntil(&lastWakeTime, statusLED_period_ms);
    }
}

uint32_t DWT_Delay_Init(void)
{
    /* Disable TRC */
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
    /* Enable TRC */
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;
 
    /* Disable clock cycle counter */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
    /* Enable  clock cycle counter */
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;
 
    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;
 
    /* 3 NO OPERATION instructions */
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
 
    /* Check if clock cycle counter has started */
    if(DWT->CYCCNT)
    {
       return 0; /*clock cycle counter started*/
    }
    else
    {
      return 1; /*clock cycle counter not started*/
    }
}

/* Debug Exception and Monitor Control Register base address */
#define DEMCR                 *((volatile uint32_t*) 0xE000EDFCu)

/* ITM register addresses */
#define ITM_STIMULUS_PORT0    *((volatile uint32_t*) 0xE0000000u)
#define ITM_TRACE_EN          *((volatile uint32_t*) 0xE0000E00u)

/**
 * @brief Firmware entry point.
 *
 * Device configuration and task initialization should be performed here.
 *
 * @return Does not return.
 */
int main(void) {
	// Enable TRCENA
//	DEMCR |= ( 1 << 24);
//	// Enable stimulus port 0
//	ITM_TRACE_EN |= ( 1 << 0);

//    // System initialization.
//    uint32_t *ACTLR = (uint32_t *)0xE000E008;
//
//
//    *ACTLR |= 2; // disable write buffering

    // System initialization.

	HAL_Init();
    cmr_rccSystemClockEnable();
    // cmr_rccSystemInternalClockEnable();
    DWT_Delay_Init();

    // Peripheral configuration.
    gpioInit();
    BMBInit();
    adcInit();
    sensorsInit();
    canInit();

    //init fan task
    //fanInit();
    //wwdgInit();

    cmr_taskInit(
        &statusLED_task,
        "statusLED",
        statusLED_priority,
        statusLED,
        NULL
    );
//BMB_task
    cmr_taskInit(
        &bmbSample_task,
        "BMB Sample Task",
        bmbSample_priority,
        vBMBSampleTask,
        NULL
    );

    // State Task
    cmr_taskInit(
        &setState_task,
        "Set State Task",
        setState_priority,
        vSetStateTask,
        NULL
    );


    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}

