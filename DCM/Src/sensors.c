/**
 * @file sensors.c
 * @brief Board-specific sensors implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <stdlib.h>     // abs()

#include <CMR/tasks.h>  // Task interface

#include "sensors.h"    // Interface to implement
#include "adc.h"        // Board-specific ADC interface
#include "can.h"        // Board-specific CAN interface

/** @brief Number of samples for current measurement rolling average. */
#define BUS_CURRENT_SAMPLES 10

/**
 * @brief Mapping of sensor channels to ADC channels.
 */
const adcChannel_t sensorsADCChannels[SENSOR_CH_LEN] = {
    [SENSOR_CH_VOLTAGE_CV]  = ADC_VSENSE,
    [SENSOR_CH_AVG_CURRENT_DA]  = ADC_ISENSE
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
    return adcRead(sensorsADCChannels[sensorChannel]);
}

/**
 * @brief Converts a raw ADC value into a low-voltage bus voltage.
 *
 * @param sensor The sensor to read.
 *
 * @param reading The ADC value to convert.
 *
 * @return Voltage in mV.
 */
static int32_t adcToBusVoltage_cV(const cmr_sensor_t *sensor, uint32_t reading) {
    (void) sensor;  // Placate compiler.
//    float voltage = 0.01163 * reading - 78.4429;
    return (int32_t) (reading); //* 100.0f);
}

/**
 * @brief Converts a raw ADC value into a low-voltage bus current.
 *
 * @param sensor The sensor to read.
 *
 * @param reading The ADC value to convert.
 *
 * @return Current in mA.
 */
static int32_t adcToAvgBusCurrent_cA(const cmr_sensor_t *sensor, uint32_t reading) {
	float current = 0.006965525 * reading - 145.88875;

    return (int32_t) (reading); //* 100.0f);
}

static cmr_sensor_t sensors[SENSOR_CH_LEN] = {
    [SENSOR_CH_VOLTAGE_CV] = {
        .conv = adcToBusVoltage_cV,
        .sample = sampleADCSensor,
        .readingMin = 0, // 20 Volts
        .readingMax = 65536, // 26 Volts
        .outOfRange_pcnt = 10,
        .warnFlag = CMR_CAN_WARN_BUS_VOLTAGE,
    },
    [SENSOR_CH_AVG_CURRENT_DA] = {
        .conv = adcToAvgBusCurrent_cA,
        .sample = sampleADCSensor,
        .readingMin = 0,  // 10 mA
        .readingMax = 65536, // 100 mA
        .outOfRange_pcnt = 10,
        .warnFlag = CMR_CAN_WARN_BUS_CURRENT
    }
};

int32_t getVoltage(void) {
	return cmr_sensorListGetValue(&sensorList, SENSOR_CH_VOLTAGE_CV);
}

int32_t getCurrent(void) {
	return cmr_sensorListGetValue(&sensorList, SENSOR_CH_AVG_CURRENT_DA);
}

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

        vTaskDelayUntil (&lastWakeTime, sensorsUpdate_period_ms);
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
        "sensor update",
        sensorsUpdate_priority,
        sensorsUpdate,
        NULL
    );
}
