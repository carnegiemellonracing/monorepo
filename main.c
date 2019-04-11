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
#include <CMR/i2c.h>  // Segment interface
#include <string.h>
#include <stdio.h>

#include "gpio.h"   // Board-specific GPIO interface
#include "can.h"    // Board-specific CAN interface
#include "adc.h"    // Board-specific ADC interface
#include "main.h"   // DIM-specific definitions
#include "segments.h"

/**======================================**/
/**              STATUS LED              **/
/**======================================**/
/** @brief Status LED priority. */
static const uint32_t statusLED_priority = 2;

/** @brief Status LED period (milliseconds). */
static const TickType_t statusLED_period_ms = 250;

static const uint32_t display_len = 8;

/** @brief Status LED task. */
static cmr_task_t statusLED_task;

static cmr_i2c_t i2c1;

volatile cmr_canState_t VSM_state = CMR_CAN_UNKNOWN;
volatile cmr_canState_t DIM_requested_state = CMR_CAN_GLV_ON;
volatile cmr_canGear_t DIM_gear = GEAR_FAST;
volatile cmr_canGear_t DIM_newGear = GEAR_FAST;
volatile cmr_canGear_t DIM_oldGear = GEAR_FAST;

volatile int32_t HVC_pack_voltage = 0;

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

    //static const uint32_t pack_voltage  = 447;

    uint8_t *text[] = {
    		"?",
    		"GLV ON   ",
			"HV %c %.3u",
			"RD %c %.3u",
			"ERROR",
			"C_ERROR",
    };

/*
    GEAR_UNKNOWN = 0,   /**< @brief Unknown Gear State
    GEAR_REVERSE,       /**< @brief Reverse mode
    GEAR_SLOW,          /**< @brief Slow mode
    GEAR_FAST,          /**< @brief Fast simple mode
    GEAR_ENDURANCE,     /**< @brief Endurance-event mode
    GEAR_AUTOX,         /**< @brief Autocross-event mode
    GEAR_SKIDPAD,       /**< @brief Skidpad-event mode
    GEAR_ACCEL,         /**< @brief Acceleration-event mode
    GEAR_TEST,          /**< @brief Test mode (for experimentation)
*/

    while (1) {
        uint8_t gear;
            switch(DIM_gear) {
                case GEAR_UNKNOWN :
                    gear = '-';
                    break;
                case GEAR_REVERSE :
                    gear = 'R';
                    break;
                case GEAR_SLOW :
                    gear = 'S';
                    break;
                case GEAR_FAST :
                    gear = 'F';
                    break;
                case GEAR_ENDURANCE :
                    gear = 'E';
                    break;
                case GEAR_AUTOX :
                    gear = 'X';
                    break;
                case GEAR_SKIDPAD :
                    gear = '8';
                    break;
                case GEAR_ACCEL :
                    gear = 'A';
                    break;
                case GEAR_TEST :
                    gear = 'T';
                    break;
                default :
                    gear = '-';
            }

    	uint8_t display_str[display_len];
    	uint32_t ret_len = 0;
    	switch(VSM_state) {
    		case CMR_CAN_UNKNOWN :
    		case CMR_CAN_GLV_ON :
    		case CMR_CAN_HV_EN :
    		case CMR_CAN_RTD :
    		case CMR_CAN_ERROR :
    		case CMR_CAN_CLEAR_ERROR :
    			ret_len = snprintf(display_str, display_len, text[VSM_state], gear, HVC_pack_voltage);
    			break;
    		default:
    			ret_len = snprintf(display_str, display_len, text[0], HVC_pack_voltage);
    			break;
    	}
    	writeSegmentText(display_str, ret_len);
        cmr_gpioToggle(GPIO_LED_STATUS);
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
    cmr_i2cInit(&i2c1, I2C1, 100000, 0, GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_7);
    initSegmentDisplays(&i2c1);
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
