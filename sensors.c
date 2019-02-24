/**
 * @file sensors.c
 * @brief Board-specific sensors implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <FreeRTOS.h>   // FreeRTOS interface
#include <task.h>       // xTaskCreate()

#include "sensors.h"    // Interface to implement
#include "adc.h"        // adcChannels_t, adcChannels
#include <CMR/adc.h>    // ADC_MAX

// Forward declarations
static void sensorUpdate_task(void *pvParameters);
static adcChannels_t sensorToADCChannel(cmr_sensor_t *sensor);
static int32_t adcToBusVoltage_mV(cmr_sensor_t *sensor);
static int32_t adcToBusCurrent_mA(cmr_sensor_t *sensor);
static void checkSensor(cmr_sensor_t *sensor);

// TODO calibrate all of these min/max values
cmr_sensor_t sensors[SENSOR_CH_LEN] = {
    [SENSOR_CH_LOGIC_VOLTAGE_MV] = {
        .adcToValue = &adcToBusVoltage_mV,
        .minADC = 2256, // 20 Volts
        .maxADC = 2933, // 26 Volts
        .warnThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 24000
    },
    [SENSOR_CH_LOGIC_CURRENT_MA] = {
        .adcToValue = &adcToBusCurrent_mA,
        .minADC = 250,  // 10 mA
        .maxADC = 2500, // 100 mA
        .warnThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 20
    }
    [SENSOR_CH_LOAD_VOLTAGE_MV] = {
        .adcToValue = &adcToBusVoltage_mV,
        .minADC = 2256, // 20 Volts
        .maxADC = 2933, // 26 Volts
        .warnThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 24000
    },
    [SENSOR_CH_LOAD_CURRENT_MA] = {
        .adcToValue = &adcToBusCurrent_mA,
        .minADC = 250,  // 10 mA
        .maxADC = 2500, // 100 mA
        .warnThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 20
    }
};

// TODO Set channels correctly based on wiring
adcChannels_t sensorADCChannels[SENSOR_CH_LEN] = {
    [SENSOR_CH_LOGIC_VOLTAGE_MV]  = ADC_LOGIC_VSENSE,
    [SENSOR_CH_LOGIC_CURRENT_MA]  = ADC_LOGIC_ISENSE,
    [SENSOR_CH_LOAD_VOLTAGE_MV]  = ADC_POWER_VSENSE,
    [SENSOR_CH_LOAD_CURRENT_MA]  = ADC_POWER_ISENSE
};

/** @brief Sensor update priority. */
static const uint32_t sensorUpdate_priority = 5;

/** @brief Sensor update period (milliseconds). */
static const TickType_t sensorUpdate_period_ms = 10;

static void sensorUpdate_task(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        for (size_t i = 0; i < SENSOR_CH_LEN; i++) {
            cmr_sensor_t *sensor = &(sensors[i]);

            sensor->value = (*sensor->adcToValue)(sensor);
        }

        vTaskDelayUntil(&lastWakeTime, sensorUpdate_period_ms);
    }
}

/**
 * @brief Gets the ADC channel associated with a sensor.
 *
 * @param sensor The sensor whose corresponding ADC channel to locate.
 *
 * @return The ADC channel associated with the given sensor. ADC_LEN if
 * none were found.
 */
static adcChannels_t sensorToADCChannel(cmr_sensor_t *sensor) {
    for (sensorChannel_t ch = 0; ch < SENSOR_CH_LEN; ch++) {
        if (&(sensors[ch]) == sensor) {
            return sensorADCChannels[ch];
        }
    }

    return ADC_LEN;
}

/**
 * @brief Converts a raw sensor ADC value to a percentage.
 *
 * @param sensor The sensor to read.
 *
 * @return Percentage value of sensor (0 to 100, inclusive). Sets channel.error field
 * and returns 0 in case of error.
 */
static int32_t adcToPcnt(cmr_sensor_t *sensor) {
    adcChannels_t adcChannel = sensorToADCChannel(sensor);
    uint32_t adcVal = adcChannels[adcChannel].value;

    checkSensor(sensor);
    if (sensor->error != SENSOR_ERR_NONE) {
        return 0;
    }

    // Scale to a percentage
    int32_t sensorPcnt = 0;
    if (adcVal > sensor->maxADC) {
        sensorPcnt = 100;
    }
    else if (adcVal < sensor->minADC) {
        sensorPcnt = 0;
    }
    else {
        uint32_t sensorRange = sensor->maxADC - sensor->minADC;
        sensorPcnt = (adcVal - sensor->minADC) * 100 / sensorRange;
    }

    return sensorPcnt;
}

/**
 * @brief Converts a raw ADC value into a low-voltage bus voltage.
 *
 * @param sensor The sensor to read.
 *
 * @return Voltage in mV. Sets channel.error field in case of error.
 */
static int32_t adcToBusVoltage_mV(cmr_sensor_t *sensor) {
    adcChannels_t adcChannel = sensorToADCChannel(sensor);
    uint32_t adcVal = adcChannels[adcChannel].value;

    checkSensor(sensor);

    // value * 0.8 (mV per bit) * 11 (1:11 voltage divider)
    uint32_t busVoltage_mV = adcVal * 8 * 11 / 10;

    return (int32_t) busVoltage_mV;
}

/**
 * @brief Converts a raw ADC value into a low-voltage bus current.
 *
 * @param sensor The sensor to read.
 *
 * @return Current in mA. Sets channel.error field in case of error.
 */
static int32_t adcToBusCurrent_mA(cmr_sensor_t *sensor) {
    adcChannels_t adcChannel = sensorToADCChannel(sensor);
    uint32_t adcVal = adcChannels[adcChannel].value;

    checkSensor(sensor);

    /* value * 0.8 (mV per bit) / 20 (gain of current shunt monitor)
     * http://www.ti.com/lit/ds/symlink/ina196.pdf
     * page 3 section 5 for INA196 */
    int32_t busCurrent_mA = adcVal * 8 / 10 / 20;

    return (int32_t) busCurrent_mA;
}

/**
 * @brief Sets sensor error field if sensor is out of range by the configured percentage.
 *
 * @param sensor The sensor to read.
 */
static void checkSensor(cmr_sensor_t *sensor) {
    adcChannels_t adcChannel = sensorToADCChannel(sensor);
    uint32_t adcVal = adcChannels[adcChannel].value;
    uint32_t adcRange = sensor->maxADC - sensor->minADC;

    // Return if sensor is too far above expected range
    // accounting for overflow
    uint32_t upperThres = sensor->maxADC + (adcRange * sensor->warnThres_pcnt / 100);
    if (upperThres >= sensor->maxADC && adcVal > upperThres) {
        sensor->error = SENSOR_ERR_OUT_OF_RANGE;
    }

    // Return if sensor is too far below expected range
    // accounting for underflow
    uint32_t lowerThres = sensor->minADC - (adcRange * sensor->warnThres_pcnt / 100);
    if (lowerThres <= sensor->minADC && adcVal < lowerThres) {
        sensor->error = SENSOR_ERR_OUT_OF_RANGE;
    }

    sensor->error = SENSOR_ERR_NONE;
}


/**
 * @brief Initializes the sensor interface.
 */
void sensorInit(void) {
    // Task creation.
    xTaskCreate(
        sensorUpdate_task, "sensorUpdate",
        configMINIMAL_STACK_SIZE, NULL, sensorUpdate_priority, NULL
    );
}
