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


/** @brief Number of samples for current measurement rolling average. */
#define BUS_CURRENT_SAMPLES 10

/** @brief Slope for voltage sense transfer function */
#define V_TRANS_M 19.506
/** @brief Intercept for voltage sense transfer function */
#define V_TRANS_B -8313.3

/** @brief 90 degree sw left lock adc value. */
#define SWANGLE_90DEG_LEFT 1345
/** @brief -90 degree sw RIGHT lock adc value. */
#define SWANGLE_90DEG_RIGHT 3155


/**
 * @brief Mapping of sensor channels to ADC channels.
 */
static const adcChannels_t sensorsADCCHANNELS[SENSOR_CH_LEN] = {
    [SENSOR_CH_V24V]       = ADC_V24V,
	//[SENSOR_CH_VREF] = ADC_V24V, //keep, from hvi 
	[SENSOR_CH_AIR_POWER]  = ADC_AIR_POWER,
	[SENSOR_CH_SAFETY]     = ADC_SAFETY,
	[SENSOR_CH_HV] = ADC_VSENSE, 
	[SENSOR_CH_CURRENT] = ADC_ISENSE
};

/** @brief forward declaration */
static cmr_sensor_t sensors[SENSOR_CH_LEN];

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
 * @brief Converts a raw ADC value into a low voltage value.
 *
 * @param sensor The sensor to read.
 *
 * @param reading The ADC value to convert.
 *
 * @return Voltage in mV.
 */
// 24v voltage divider is factor of 1.13/14.43
static int32_t ADCtoMV_24v(const cmr_sensor_t *sensor, uint32_t reading) {
    (void) sensor;

    return ((int32_t) reading) * 7.39;

}

/**
 * @brief Converts a raw ADC value into HV voltage
 *
 * @param sensor The sensor to read.
 *
 * @param reading The ADC value to convert.
 *
 * @return Voltage in V.
 */
// was determined experimentally
static int32_t ADCtoMV_HV(const cmr_sensor_t *sensor, uint32_t reading) {
    (void) sensor;

	return (((int32_t) reading) * 268 - 426400);
}


/**
 * 
 * HVI conversion functions
 * 
 */


/**
 * @brief Converts a raw ADC value to voltage in centivolts.
 *
 * @param sensor The sensor to read.
 *
 * @param reading The ADC value to convert.
 *
 * @return Voltage in centivolts.
 */
static int32_t adcToVoltage(const cmr_sensor_t *sensor, uint32_t reading) {
    (void)sensor;

    int32_t voltage = reading;
    // uint32_t voltage = V_TRANS_M * reading + V_TRANS_B;

    return voltage;
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

    int32_t current = (int32_t) (((0.143) * (double)reading - 294.0) * 10.0);
    return current;
}



static cmr_sensor_t sensors[SENSOR_CH_LEN] = {
	[SENSOR_CH_V24V] = { //hvc 
		.conv = ADCtoMV_24v,
		.sample = sampleADCSensor,
		.readingMin = 0,
		.readingMax = 24000,
		// TODO change to unoverted values
		// TODO check adc bits
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	}, 
	[SENSOR_CH_AIR_POWER] = { //hvc 
		.conv = ADCtoMV_24v,
		.sample = sampleADCSensor,
		.readingMin = 0,
		//.readingMax = MaxPackVolatge,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
	[SENSOR_CH_SAFETY] = { //hvc 
		.conv = ADCtoMV_24v,
		.sample = sampleADCSensor,
		//.readingMin = ?,
		//.readingMax = ?,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
	[SENSOR_CH_HV] = { //hvi 
		.conv = adcToVoltage,
		.sample = sampleADCSensor,
		.readingMin = 0,
		.readingMax = 4096,
		.outOfRange_pcnt = 10, 
		.warnFlag = 0 }, 
	[SENSOR_CH_CURRENT] = { //hvi 
        .conv = adcToCurrent,
        .sample = sampleADCSensor,
        .readingMin = 0,
        .readingMax = 4096,
        .outOfRange_pcnt = 10,
        .warnFlag = 0 
    },
};

/** @brief Sensors update priority. */
static const uint32_t sensorsUpdate_priority = 1;

/** @brief Sensors update period (milliseconds). */
static const TickType_t sensorsUpdate_period_ms = 50;

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
        vTaskDelayUntil(&lastWakeTime, sensorsUpdate_period_ms);

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
int32_t getLVmillivolts(){
    return (int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_V24V);
}

int32_t getAIRmillivolts(){
    return ((int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_AIR_POWER));
}

int32_t getSafetymillivolts(){
    return ((int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_SAFETY));
}

int32_t getHVmillivolts(){
    return ((int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_VSENSE));
}

int32_t getHVmilliamps(){
    return ((int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_ISENSE));
}
