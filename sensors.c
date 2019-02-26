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
static void checkSensor(cmr_sensor_t *sensor);
static int32_t adcConv_LogicVoltageMV(cmr_sensor_t *s);
static int32_t adcConv_LogicCurrentMA(cmr_sensor_t *s);
static int32_t adcConv_LoadVoltageMV(cmr_sensor_t *s);
static int32_t adcConv_LoadCurrentMA(cmr_sensor_t *s);
static int32_t adcConv_FanCurrentMA(cmr_sensor_t *s);
static int32_t adcConv_BoardTherm(cmr_sensor_t *s);
static int32_t adcConv_RadTherm(cmr_sensor_t *s);
static int32_t adcFractionalConvert(cmr_sensor_t *sensor, int32_t numerator, int32_t divisor, int32_t offset);

// TODO calibrate all of these min/max values
cmr_sensor_t sensors[SENSOR_CH_LEN] = {
    [SENSOR_CH_LOGIC_VOLTAGE_MV] = {
        .adcToValue = &adcConv_LogicVoltageMV,
        .minADC = 2256, // 20 Volts
        .maxADC = 2933, // 26 Volts
        .warnThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 24000
    },
    [SENSOR_CH_LOGIC_CURRENT_MA] = {
        .adcToValue = &adcConv_LogicCurrentMA,
        .minADC = 250,  // 10 mA
        .maxADC = 2500, // 100 mA
        .warnThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 20
    },
    [SENSOR_CH_LOAD_VOLTAGE_MV] = {
        .adcToValue = &adcConv_LoadVoltageMV,
        .minADC = 2256, // 20 Volts
        .maxADC = 2933, // 26 Volts
        .warnThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 24000
    },
    [SENSOR_CH_LOAD_CURRENT_MA] = {
        .adcToValue = &adcConv_LoadCurrentMA,
        .minADC = 250,  // 10 mA
        .maxADC = 2500, // 100 mA
        .warnThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 20
    },
    [SENSOR_CH_FAN_CURRENT_MA] = {
        .adcToValue = &adcConv_FanCurrentMA,
        .minADC = 250,  // 10 mA
        .maxADC = 2500, // 100 mA
        .warnThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 20
    },
    [SENSOR_CH_BOARD_THERM_1] = {
        .adcToValue = &adcConv_BoardTherm,
        .minADC = 250,  // 10 mA
        .maxADC = 2500, // 100 mA
        .warnThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 20
    },
    [SENSOR_CH_BOARD_THERM_2] = {
        .adcToValue = &adcConv_BoardTherm,
        .minADC = 250,  // 10 mA
        .maxADC = 2500, // 100 mA
        .warnThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 20
    },
    [SENSOR_CH_PRE_RAD_THERM] = {
        .adcToValue = &adcConv_RadTherm,
        .minADC = 250,  // 10 mA
        .maxADC = 2500, // 100 mA
        .warnThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 20
    },
    [SENSOR_CH_POST_RAD_THERM] = {
        .adcToValue = &adcConv_RadTherm,
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
    [SENSOR_CH_LOAD_CURRENT_MA]  = ADC_POWER_ISENSE,
    [SENSOR_CH_FAN_CURRENT_MA]  = ADC_FAN_ISENSE,
    [SENSOR_CH_BOARD_THERM_1]  = ADC_BOARD_THERM_1,
    [SENSOR_CH_BOARD_THERM_2]  = ADC_BOARD_THERM_2,
    [SENSOR_CH_PRE_RAD_THERM]  = ADC_RAD_THERM_1,
    [SENSOR_CH_POST_RAD_THERM]  = ADC_RAD_THERM_2
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
 * @brief Conversion functions for ADC measurements
 */
// TODO Calibrate!
static int32_t adcConv_LogicVoltageMV(cmr_sensor_t *s){
    return adcFractionalConvert(s, 88, 10, 0);
}

static int32_t adcConv_LogicCurrentMA(cmr_sensor_t *s){
    return adcFractionalConvert(s, 8, 200, 0);
}

static int32_t adcConv_LoadVoltageMV(cmr_sensor_t *s){
    return adcFractionalConvert(s, 88, 10, 0);
}

static int32_t adcConv_LoadCurrentMA(cmr_sensor_t *s){
    return adcFractionalConvert(s, 8, 200, 0);
}

static int32_t adcConv_FanCurrentMA(cmr_sensor_t *s){
    return adcFractionalConvert(s, 8, 200, 0);
}

static int32_t adcConv_BoardTherm(cmr_sensor_t *s){
    return adcFractionalConvert(s, 1, 1, 0);
}

static int32_t adcConv_RadTherm(cmr_sensor_t *s){
    return adcFractionalConvert(s, 1, 1, 0);
}

/**
 * @brief Reads the value of the sensor channel.
 *
 * @param ch The channel to be read.
 *
 * @return Sensor value.
 */
int32_t sensorRead(sensorChannel_t ch){
    return sensors[ch].value;
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
 * @brief Scales a raw ADC value from a sensor
 *
 * @param sensor The sensor to read.
 *
 * @return Scaled value. Sets channel.error field in case of error.
 */
static int32_t adcFractionalConvert(cmr_sensor_t *sensor, int32_t numerator, int32_t divisor, int32_t offset){
    adcChannels_t adcChannel = sensorToADCChannel(sensor);
    uint32_t adcVal = adcChannels[adcChannel].value;

    checkSensor(sensor);

    uint32_t conv = (adcVal + offset) * numerator / divisor;

    return (int32_t) conv;
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
