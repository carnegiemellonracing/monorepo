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

#include "adc.h"        // Board-specific ADC interface
#include "can.h"        // Board-specific CAN interface
#include "gpio.h"       // Board-specific GPIO interface
#include "statusLED.h"  // Board-specific statusLED interface
#include "watchDog.h"   // Board-specific watchDog interface


/** @brief Status LED priority. */
static const uint32_t statusLED_priority = 2;

/** @brief BMB Sample Task priority. */
static const uint32_t bmbSample_priority = 3;

/** @brief BMB Sample Task priority. */
static const uint32_t setState_priority = 4;

/** @brief BMB Sample Task */
static cmr_task_t bmbSample_task;

/** @brief Set State Task */
static cmr_task_t setState_task;

static void stateInit(){
    cmr_taskInit(
        &setState_task,
        "Set State Task",
        setState_priority,
        vSetStateTask,
        NULL
    );
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

    // Peripheral configuration.
    gpioInit();
    adcInit();
    sensorsInit();
    canInit();
    
    statusLEDInit(); 
    watchDogInit();

    // State Task
    stateInit();

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}

