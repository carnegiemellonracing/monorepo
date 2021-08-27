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

static const int16_t ADC_HALF = 2048;

// Current average sample count = rate(Hz) * time(s)
static const int16_t numSamplesInstant = 20;//100 * 2;
static const int16_t numSamplesAverage = 3000;//100 * 30;
#define NUM_SAMPLES_AVERAGE 3000

static volatile int32_t currentSingleSample = 0;
static volatile int32_t currentAvg = 0;
static volatile int32_t currentInstant = 0;

/**
 * @brief Mapping of sensor channels to ADC channels.
 */
static const adcChannels_t sensorsADCCHANNELS[SENSOR_CH_LEN] = {
    [SENSOR_CH_V24V]       = ADC_V24V,     
	[SENSOR_CH_AIR_POWER]  = ADC_AIR_POWER,
	[SENSOR_CH_SAFETY]     = ADC_SAFETY,
	[SENSOR_CH_HV]         = ADC_HV,
    [SENSOR_CH_BATT]       = ADC_BATT,
	[SENSOR_CH_SHUNT_P]    = ADC_SHUNT_P,
	[SENSOR_CH_SHUNT_N]    = ADC_SHUNT_N,
	[SENSOR_CH_LV_CURR]    = ADC_LV_CURR,
	[SENSOR_CH_REF1V65]    = ADC_REF1V65
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
    return adcRead(sensorsADCCHANNELS[sensorChannel]) * ADCChannelPolarity[ch];
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
static inline int32_t ADCtoMV_24v(const cmr_sensor_t *sensor, int16_t reading) {
    (void) sensor;
	
	return (((int32_t) reading) << 12) / 398;
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
// Voltage output on shunt_n will be between 0.415 - 2.465 which corresponds to 500 A to -500 A.
// ADC measurement on shunt_n measures does differential with 1.65V as P and shunt_n as N
//     This is inverted, so the adc measuement reports shunt_n - 1.65V. We're doing this the
//         the hacky way though, so we don't want that. Uninvert here.
//     At resting, shunt_n is 1.44V. The ADC therefore measures 0.21v at zero shunt current.
//         Subtract that 0.21v offset
//     Shunt_n goes more negative with more current. Therefore, the diff measurement goes more positive.
// (3.3 V / 4096 adc counts) * (500 sensor mV / 2.05 V) *  (2000 mA / 1 sensor mV) = 393.007 mA / adc counts
// Shunt_n is centered at 1.44V -> add [(1.65V - 1.44V) * (2048 adc counts / 1.65V)] = 260.6545 adc counts to measurement
// Perform (adc_input * (393.007mA / adc count)) + (260.6545 counts * (393.007mA / adc count))
// Measured offset when shunt voltages are equal. Add this offset.
static const int32_t SHUNT_MA_OFFSET = 6250;
static inline int32_t ADCtoMA_shunt(const cmr_sensor_t *sensor, int16_t reading){
    (void) sensor;

	adc_input *= -1;
    return (((int32_t) reading) * 393) - 102439 - SHUNT_MA_OFFSET;
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
static uint32_t adcToMA_24v(const cmr_sensor_t *sensor, int16_t reading) {
	(void) sensor;
	
	return (((int32_t) reading) << 11) / 205;
}

/**
 * @brief Converts a raw ADC value into an HV value.
 *
 * @param sensor The sensor to read.
 *
 * @param reading The ADC value to convert.
 *
 * @return Voltage in V.
 */
// Voltage divider (28.7k / (28.7k + 6M)
// (3.3 V / 4096 adc count) * (60287 real volts /287 adc volts) * (1000 mV / V)  = 169.237 mV / adc count
// This value is measured differentially in the ADC compared to a 1.65V reference. 
//     The measurement code also inverts this, so the input to this function represents (Vsense - 1.65V)
//     Further, the Vsense amp is refrenced to 0.3V, so Vsense will be shifted up by 0.3V, therefore
//         we need to shift vsense up by 1.35V to get the true measurement
//     This translates to an offset of (1350 adc mv * (60287 real volts /287 adc volts) * 256 = 72,576,000)
// to get better resolution multiply by 256 
// (169.237 * 256) = 43324.7
// divide back by 256
static const int32_t HV_V_OFFSET_MV_x256 = 51200;
static inline int32_t ADCtoMV_HVSense(const cmr_sensor_t *sensor, int16_t reading){
    int32_t volt = ((((int32_t) adc_input) * 43324 + 72576000 + HV_V_OFFSET_MV_x256) >> 8);
    return volt;
}

static cmr_sensor_t sensors[SENSOR_CH_LEN] = {
	[SENSOR_CH_V24V] = {
		.conv = ADCtoMV_24v,
		.sample = sampleADCSensor,
		.readingMin = 0,
		.readingMax = 24000,
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
		.conv = ,
		.sample = sampleADCSensor,
		//.readingMin = ?,
		//.readingMax = ?,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
	[SENSOR_CH_HV] = {
		.conv = ADCtoMV_HVSense,
		.sample = sampleADCSensor,
		//.readingMin = 0,
		//.readingMax = 24000,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
	[SENSOR_CH_BATT] = {
		.conv = adcToMV_HVSense,
		.sample = sampleADCSensor,
		//.readingMin = 0,
		//.readingMax = 24000,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
	[SENSOR_CH_SHUNT_P] = {
		.conv = ADCtoMA_shunt,
		.sample = sampleADCSensor,
		//.readingMin = 0,
		//.readingMax = 24000,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
	[SENSOR_CH_SHUNT_N] = {
		.conv = ADCtoMA_shunt,
		.sample = sampleADCSensor,
		//.readingMin = 0,
		//.readingMax = 24000,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
	[SENSOR_CH_LV_CURR] = {
		.conv = ADCtoMA_24v,
		.sample = sampleADCSensor,
		//.readingMin = 0,
		//.readingMax = 24000,
		.outOfRange_pcnt = 10,
		//.warnFlag = What errors to use?
	},
	[SENSOR_CH_REF1V65] = {
		.conv = ,
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

		currentSingleSample = (int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_SHUNT_N);

		// Rolling average
        // A single sample is too noisy for an "instant" measurement so do a small average
        currentInstant = (currentInstant*(numSamplesInstant-1) + currentSingleSample) / numSamplesInstant;
        currentAvg = (currentAvg*(numSamplesAverage-1) + currentSingleSample) / numSamplesAverage;

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
(int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_BATT_PLUS)

// Accessor functions used in the state machine. These casts should be safe because all the feasible values
// for any of these variables should be less than INT_MAX, so the value will be preserved on the cast.
int32_t getLVmillivolts(){
    return (int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_V24V);
}

int32_t getLVmilliamps(){
	return (int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_LV_CURR);
}

int32_t getAIRmillivolts(){
    return (int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_AIR_POWER);
}

int32_t getHVmillivolts(){
    return (int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_HV);
}

int32_t getBattMillivolts(){
    return (int32_t) cmr_sensorListGetValue(&sensorList, SENSOR_CH_BATT);
}

int32_t getCurrentInstant(){
    return currentInstant;
}

int32_t getCurrentAverage(){
    return currentAvg;
}
