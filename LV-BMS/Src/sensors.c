/*
 * sensors.c
 *
 *  Created on: Jun 15, 2020
 *      Author: vamsi
 */

#include <stdlib.h>
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/tasks.h>  // Task interface
#include <math.h>       // tanh()

#include "sensors.h"
#include "adc.h"
#include "can.h"   // Board-specific CAN interface
#include "gpio.h"  // Board-specific GPIO interface

/** @brief Slope for current sense transfer function */
#define I_TRANS_M 12.2
/** @brief Intercept for current sense transfer function */
#define I_TRANS_B -37878


/**
 * @brief Mapping of sensor channels to ADC channels.
 */
static const adcChannel_t sensorsADCCHANNELS[SENSOR_CH_LEN] = {
	[SENSOR_CH_HALL_EFFECT] = ADC_HALL_EFFECT
};

/** @brief forward declaration */
static cmr_sensor_t sensors[];

/** @brief The list of sensors sampled by the driver. */
cmr_sensorList_t sensorList;

/**
 * @brief Gets a new value from an ADC sensor.
 *
 * This function simply copies data from the ADC - it does not make any requests
 * to ADC hardware.
 *
 * @param sensor The ADC sensor to sample.
 *
 * @return The latest sampled value from the ADC.
 */
static uint32_t sampleADCSensor(const cmr_sensor_t *sensor) {
    sensorChannel_t sensorChannel = sensor - sensors;
    configASSERT(sensorChannel < SENSOR_CH_LEN);
    return adcRead(sensorsADCCHANNELS[sensorChannel]); //* ADCChannelPolarity[ch]; Figure adc polarity
}

/**
 * @brief Converts a raw ADC value into a current.
 *
 * @param sensor The sensor to read.
 *
 * @param reading The ADC value to convert.
 *
 * @return Current in dA.
 */
static int32_t adcToCurrent(const cmr_sensor_t *sensor, uint32_t reading) {
    (void)sensor;

    int32_t current = I_TRANS_M * reading + I_TRANS_B;
    return current;
}

static cmr_sensor_t sensors[SENSOR_CH_LEN] = {
	[SENSOR_CH_HALL_EFFECT] = {
		.conv = adcToCurrent,
		.sample = sampleADCSensor,
		.readingMin = 0,
		.readingMax = 4096,
		.outOfRange_pcnt = 10,
	}
};

/** @brief Sensors update priority. */
static const uint32_t sensorsUpdate_priority = 1;

/** @brief Sensors update period (milliseconds). */
static const TickType_t sensorsUpdate_period_ms = 5;

/** @brief Sensors update task. */
static cmr_task_t sensorsUpdate_task;

/**
 * @brief Task for updating sensor values.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void sensorsUpdate(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_sensorListUpdate(&sensorList);
        vTaskDelayUntil(&lastWakeTime, sensorsUpdate_period_ms);
    }
}

/**
 * @brief Initializes the sensor interface.
 */
void sensorsInit(void) {
    cmr_sensorListInit(
        &sensorList,
        sensors, sizeof(sensors) / sizeof(sensors[0])
    );

    // Task creation.
    cmr_taskInit(
        &sensorsUpdate_task,
        "sensors update",
        sensorsUpdate_priority,
        sensorsUpdate,
        NULL
    );
}

// Accessor functions used in the state machine. These casts should be safe because all the feasible values
// for any of these variables should be less than INT_MAX, so the value will be preserved on the cast.


int32_t getLVmilliamps(){
    return ((int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_HALL_EFFECT));
}