/*
 * sensors.c
 *
 *  Created on: Jun 15, 2020
 *      Author: vamsi
 */

#include <stdlib.h>

#include "sensors.h"
#include "adc.h"

// ADC reference voltage in millivolts, should be measured from ADCVREFP and ADCVREFN
static const int16_t mvVRef = 2600;

// Current shunt calibrations from measurement
static const int16_t shuntOffsetVoltage = 0;
static const int32_t shuntOffsetCurrent = 0;

static const int16_t senseOffsetVoltage = 1100;

/**
 * @brief Mapping of sensor channels to ADC channels.
 */
static const adcChannels_t sensorsADCCHANNELS[SENSOR_CH_LEN] = {
    [SENSOR_CH_V24V]       = ADC_V24V,     
	[SENSOR_CH_HV_PLUS]    = ADC_HV_PLUS,
	[SENSOR_CH_HV_MINUS]   = ADC_HV_MINUS,
	[SENSOR_CH_BATT_PLUS]  = ADC_BATT_PLUS,
    [SENSOR_CH_BATT_MINUS] = ADC_HV_MINUS,
	[SENSOR_CH_SHUNT_LV]   = ADC_SHUNT_LV,
	[SENSOR_CH_AIR_POWER]  = ADC_AIR_POWER,
	[SENSOR_CH_DCDC_CURR]  = ADC_DCDC_CURR,
	[SENSOR_CH_LV_CURR]    = ADC_LV_CURR
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
    return adcRead(sensorsADCCHANNELS[sensorChannel]);
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
static uint32_t adcToMV_24v(const cmr_sensor_t *sensor, uint32_t reading) {
	(void) sensor;

	return ((((uint32_t) reading)*mvVRef)/160);
}

/**
 * @brief Converts a raw ADC value into a High voltage value.
 *
 * @param sensor The sensor to read.
 *
 * @param reading The ADC value to convert.
 *
 * @return Voltage in mV.
 */
static int32_t adcToMV_HVSense(const cmr_sensor_t *sensor, uint32_t reading) {
	(void) sensor;

	int32_t mvSigLV = (((int32_t)reading)*mvVRef)/2048; // ADC transform to mV
    return (((mvSigLV-senseOffsetVoltage)*8844)/44) + senseOffsetVoltage; // LV mV to HV mV
}

/**
 * @brief Converts a raw ADC value into a shunt current value.
 *
 * @param sensor The sensor to read.
 *
 * @param reading The ADC value to convert.
 *
 * @return Current in mA.
 */
static int32_t adcToMA_shunt(const cmr_sensor_t *sensor, uint32_t reading) {
	(void) sensor;

	int32_t mvShuntLV = (((int32_t)reading)*mvVRef)/2048; // ADC transform to mV
    return (mvShuntLV - shuntOffsetVoltage)*244 + shuntOffsetCurrent; // ShuntLV mV to pack current mA
}

/**
 * @brief Converts a raw ADC value into a low voltage current value.
 *
 * @param sensor The sensor to read.
 *
 * @param reading The ADC value to convert.
 *
 * @return Current in mA.
 */
static uint32_t adcToMA_24v(const cmr_sensor_t *sensor, uint32_t reading) {
	(void) sensor;
	
	return ((((uint32_t) reading)*mvVRef)/5120);
}

/**
 * @brief Converts a raw ADC value into a DCDC current value.
 *
 * @param sensor The sensor to read.
 *
 * @param reading The ADC value to convert.
 *
 * @return Current in mA.
 */
static uint32_t adcToMA_DCDC(const cmr_sensor_t *sensor, uint32_t reading) {
	(void) sensor;
	//TODO: Implement
	return 69;
}

static cmr_sensor_t sensors[SENSOR_CH_LEN] = {
	[SENSOR_CH_V24V] = {
		.conv = adcToMV_24v,
		.sample = sampleADCSensor,
		.readingMin = 0,
		.readingMax = 24000,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
	[SENSOR_CH_HV_PLUS] = {
		.conv = adcToMV_HVSense,
		.sample = sampleADCSensor,
		.readingMin = 0,
		//.readingMax = MaxPackVolatge,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
	[SENSOR_CH_HV_MINUS] = {
		.conv = adcToMV_HVSense,
		.sample = sampleADCSensor,
		//.readingMin = ?,
		//.readingMax = ?,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
	[SENSOR_CH_BATT_PLUS] = {
		.conv = adcToMV_HVSense,
		.sample = sampleADCSensor,
		//.readingMin = 0,
		//.readingMax = 24000,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
	[SENSOR_CH_BATT_MINUS] = {
		.conv = adcToMV_HVSense,
		.sample = sampleADCSensor,
		//.readingMin = 0,
		//.readingMax = 24000,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
	[SENSOR_CH_SHUNT_LV] = {
		.conv = adcToMA_shunt,
		.sample = sampleADCSensor,
		//.readingMin = 0,
		//.readingMax = 24000,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
	[SENSOR_CH_AIR_POWER] = {
		.conv = adcToMV_24v,
		.sample = sampleADCSensor,
		//.readingMin = 0,
		//.readingMax = 24000,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
	[SENSOR_CH_DCDC_CURR] = {
		.conv = adcToMA_DCDC,
		.sample = sampleADCSensor,
		//.readingMin = 0,
		//.readingMax = 24000,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
	[SENSOR_CH_LV_CURR] = {
		.conv = adcToMA_24v,
		.sample = sampleADCSensor,
		//.readingMin = 0,
		//.readingMax = 24000,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	}
};

/** @brief Sensors update priority. */
static const uint32_t sensorsUpdate_priority = 5;

/** @brief Sensors update period (milliseconds). */
static const TickType_t sensorsUpdate_period_ms = 10;

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
