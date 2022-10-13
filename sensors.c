/*
 * sensors.c
 *
 *  Created on: Jun 15, 2020
 *      Author: vamsi
 */

#include <stdlib.h>

#include "sensors.h"
#include "adc.h"

/**
 * @brief Mapping of sensor channels to ADC channels.
 */
static const adcChannels_t sensorsADCCHANNELS[SENSOR_CH_LEN] = {
    [SENSOR_CH_V24V]       = ADC_V24V,     
	[SENSOR_CH_AIR_POWER]  = ADC_AIR_POWER,
	[SENSOR_CH_SAFETY]     = ADC_SAFETY,
	[SENSOR_CH_IBATT_FILTERED] = ADC_IBATT_FILTERED,
	[SENSOR_CH_VSENSE]     = ADC_VSENSE,
	[SENSOR_CH_ISENSE]     = ADC_ISENSE
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
 * @brief Converts a raw ADC value into a low voltage value.
 *
 * @param sensor The sensor to read.
 *
 * @param reading The ADC value to convert.
 *
 * @return Voltage in mV.
 */
// 24v voltage divider is factor of 1.13/14.43
// Vref = 1.65v
// div = 1 / [(Rtop + Rbottom) / Rbottom] = 14.43 / 1.13 = 12.76
// adc_scale = 3.3 / 4096
// final scale volts = [1 / (((1.13 + 13.3) / 1.13) * (3.3 / 2^12))] = 97.198 (adc counts / output volt)
// final scale millivolts = 97.198 * (1 volt / 1000 mv) = 0.097198 (adc counts / output mv)
// Scale up by 2^12 then divide by (0.097198 * 2^12)
// V = div * ((ADC/2048) * Vref)
static int32_t ADCtoMV_24v(const cmr_sensor_t *sensor, uint32_t reading) {
    (void) sensor;
	
	return (((int32_t) reading) << 12) / 398;
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
// HV voltage divider is 806 Ohm differential with 2M pull down to gnd
// Vref = 1.65v
// div = 1 / [(Rtop + Rbottom) / Rbottom] = 14.43 / 1.13 = 12.76
// adc_scale = 3.3 / 4096
// final scale volts = [1 / (((1.13 + 13.3) / 1.13) * (3.3 / 2^12))] = 97.198 (adc counts / output volt)
// final scale millivolts = 97.198 * (1 volt / 1000 mv) = 0.097198 (adc counts / output mv)
// Scale up by 2^12 then divide by (0.097198 * 2^12)
// V = div * ((ADC/2048) * Vref)
static int32_t ADCtoMV_24v(const cmr_sensor_t *sensor, uint32_t reading) {
    (void) sensor;
	
	return (((int32_t) reading) << 12) / 398;
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
// 24v current shunt amplifier has a transfer function of: 
// (2^12 adc counts / 3.3 adc volts) * (1 adc volt / 20 real volts) * (1 real volt / 0.62 shunt ohms) * (1 amp / 1000 ma) = 0.1000098 adc counts / ma
// Scale up by 2^11 then divide by (0.1000098 * 2^11)
// (20 real volts / 1 adc volts) * (1 real volt / 0.62 ohms) = 
static int32_t adcToMA_24v(const cmr_sensor_t *sensor, uint32_t reading) {
	(void) sensor;
	
	return (((int32_t) reading) << 11) / 205;
}

static cmr_sensor_t sensors[SENSOR_CH_LEN] = {
	[SENSOR_CH_V24V] = {
		.conv = ADCtoMV_24v,
		.sample = sampleADCSensor,
		.readingMin = 0,
		.readingMax = 24000,
		// TODO change to unoverted values
		// TODO check adc bits
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
	[SENSOR_CH_AIR_POWER] = {
		.conv = ADCtoMV_24v,
		.sample = sampleADCSensor,
		.readingMin = 0,
		//.readingMax = MaxPackVolatge,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
	[SENSOR_CH_SAFETY] = {
		.conv = ADCtoMV_24v,
		.sample = sampleADCSensor,
		//.readingMin = ?,
		//.readingMax = ?,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
    [SENSOR_CH_IBATT_FILTERED] = {
		.conv = adcToMA_24v,
		.sample = sampleADCSensor,
		//.readingMin = ?,
		//.readingMax = ?,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
    [SENSOR_CH_VSENSE] = {
		.conv = adcToMA_24v,
		.sample = sampleADCSensor,
		//.readingMin = ?,
		//.readingMax = ?,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
    [SENSOR_CH_ISENSE] = {
		.conv = adcToMA_24v,
		.sample = sampleADCSensor,
		//.readingMin = ?,
		//.readingMax = ?,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
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

int32_t getLVmilliamps(){
	return ((int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_IBATT_FILTERED));
}

int32_t getAIRmillivolts(){
    return ((int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_SAFETY));
}
