/*
 * BMB_task.c
 *
 *  Created on: Jun 8, 2020
 *      Author: vamsi
 */

#include "slave_uart.h"
#include "BMB_task.h"
#include "gpio.h"
#include "state_task.h"


// Max valid thermistor temp, beyond which it is considered a short
static const int16_t THERM_MAX_TEMP = 850;
// Min valid thermistor temp, beyond which it is considered open
static const int16_t THERM_MIN_TEMP = -10;

static int16_t linearTemp(uint16_t ADC);
static int16_t lutTemp(uint16_t ADC);


//Fill in data to this array
static BMB_Data_t BMBData[NUM_BMBS];

void vBMBSampleTask(void *pvParameters) {

    // Index of the BMB we're currently sampling
    uint8_t BMBIndex = 0;
    // Whether or not slect on the analog mux is asserted
    bool BMBActivityLEDEnable = false;

    // Previous wake time pointer
    portTickType xLastWakeTime = xTaskGetTickCount();

    // Period
    const portTickType xPeriod = 1000 / BMB_SAMPLE_TASK_RATE;		// In ticks (ms)

    
}

