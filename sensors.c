/**
 * @file sensors.c
 * @brief Board-specific sensors implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/tasks.h>  // Task interface

#include "sensors.h"    // Interface to implement
#include "adc.h"        // adcChannels_t, adcChannels
#include <CMR/adc.h>    // ADC_MAX

// Forward declarations
static void sensorUpdate(void *pvParameters);
static adcChannels_t sensorToADCChannel(cmr_sensor_t *sensor);
static void checkSensor(cmr_sensor_t *sensor);
static int32_t adcConv_LogicVoltage_mV(cmr_sensor_t *s);
static int32_t adcConv_LogicCurrent_mA(cmr_sensor_t *s);
static int32_t adcConv_LoadVoltage_mV(cmr_sensor_t *s);
static int32_t adcConv_LoadCurrent_mA(cmr_sensor_t *s);
static int32_t adcConv_FanCurrent_mA(cmr_sensor_t *s);
static int32_t adcConv_SwitchTemp_C(cmr_sensor_t *s);
static int32_t adcConv_RadTherm(cmr_sensor_t *s);
static int32_t adcFractionalConvert(cmr_sensor_t *sensor, int32_t numerator, int32_t divisor, int32_t offset);

// TODO calibrate all of these min/max values
cmr_sensor_t sensors[SENSOR_CH_LEN] = {
    [SENSOR_CH_LOGIC_VOLTAGE_MV] = {
        .adcToValue = &adcConv_LogicVoltage_mV,
        .minADC = 2256, // 20 Volts
        .maxADC = 2933, // 26 Volts
        .oorThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 24000
    },
    [SENSOR_CH_LOGIC_CURRENT_MA] = {
        .adcToValue = &adcConv_LogicCurrent_mA,
        .minADC = 0,
        .maxADC = CMR_ADC_MAX,
        .oorThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 20
    },
    [SENSOR_CH_LOAD_VOLTAGE_MV] = {
        .adcToValue = &adcConv_LoadVoltage_mV,
        .minADC = 2256, // 20 Volts
        .maxADC = 2933, // 26 Volts
        .oorThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 24000
    },
    [SENSOR_CH_LOAD_CURRENT_MA] = {
        .adcToValue = &adcConv_LoadCurrent_mA,
        .minADC = 0,
        .maxADC = CMR_ADC_MAX,
        .oorThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 20
    },
    [SENSOR_CH_FAN_CURRENT_MA] = {
        .adcToValue = &adcConv_FanCurrent_mA,
        .minADC = 0,
        .maxADC = CMR_ADC_MAX,
        .oorThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 20
    },
    [SENSOR_CH_BOARD_THERM_1] = {
        .adcToValue = &adcConv_SwitchTemp_C,
        .minADC = 0,
        .maxADC = CMR_ADC_MAX,
        .oorThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 20
    },
    [SENSOR_CH_BOARD_THERM_2] = {
        .adcToValue = &adcConv_SwitchTemp_C,
        .minADC = 0,
        .maxADC = CMR_ADC_MAX,
        .oorThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 20
    },
    [SENSOR_CH_PRE_RAD_THERM] = {
        .adcToValue = &adcConv_RadTherm,
        .minADC = 0,
        .maxADC = CMR_ADC_MAX,
        .oorThres_pcnt = 10,
        .error = SENSOR_ERR_NONE,
        .value = 20
    },
    [SENSOR_CH_POST_RAD_THERM] = {
        .adcToValue = &adcConv_RadTherm,
        .minADC = 0,
        .maxADC = CMR_ADC_MAX,
        .oorThres_pcnt = 10,
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

/**
 * @brief Conversions from thermistor resistance to temperature.
 * See
 */
static thermistorTempConversion_t thermTempConvs[] = {
    {
        .resistance_Ohm = 27280,
        .temp_C = 0
    },
    {
        .resistance_Ohm = 17960,
        .temp_C = 10,
    },
    {
        .resistance_Ohm = 12090,
        .temp_C = 20
    },
    {
        .resistance_Ohm = 8310,
        .temp_C = 30
    },
    {
        .resistance_Ohm = 5826,
        .temp_C = 40
    },
    {
        .resistance_Ohm = 4158,
        .temp_C = 50
    },
    {
        .resistance_Ohm = 3019,
        .temp_C = 60
    },
    {
        .resistance_Ohm = 2227,
        .temp_C = 70
    },
    {
        .resistance_Ohm = 1668,
        .temp_C = 80
    },
    {
        .resistance_Ohm = 1267,
        .temp_C = 90
    },
    {
        .resistance_Ohm = 975,
        .temp_C = 100
    },
    {
        .resistance_Ohm = 760,
        .temp_C = 110
    },
    {
        .resistance_Ohm = 599,
        .temp_C = 120
    },
    {
        .resistance_Ohm = 478,
        .temp_C = 130
    },
    {
        .resistance_Ohm = 385,
        .temp_C = 140
    },
    {
        .resistance_Ohm = 313,
        .temp_C = 150
    }
};

/** @brief Sensor update priority. */
static const uint32_t sensorUpdate_priority = 5;

/** @brief Sensor update period (milliseconds). */
static const TickType_t sensorUpdate_period_ms = 10;

/** @brief Sensor update task. */
static cmr_task_t sensorUpdate_task;

static void sensorUpdate(void *pvParameters) {
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
 * @brief Conversion function for ADC to logic voltage.
 *
 * @param s Logic voltage sensor.
 *
 * @return Logic voltage in mV.
 */
static int32_t adcConv_LogicVoltage_mV(cmr_sensor_t *s) {
    // value * 0.8 (mV per bit) * 11 (1:11 voltage divider)
    return adcFractionalConvert(s, 88, 10, 0);
}

/**
 * @brief Conversion function for ADC to logic current.
 *
 * @param s Logic current sensor.
 *
 * @return Logic current in mA.
 */
static int32_t adcConv_LogicCurrent_mA(cmr_sensor_t *s) {
    // mA * 1.5 Ohm * 20 V/V = mV = adcVal * 0.8 mV/bit
    // solve for mA = adcVal * 8 / 300
    return adcFractionalConvert(s, 8, 300, 0);
}

/**
 * @brief Conversion function for ADC to load voltage.
 *
 * @param s load voltage sensor.
 *
 * @return load voltage in mV.
 */
static int32_t adcConv_LoadVoltage_mV(cmr_sensor_t *s) {
    // value * 0.8 (mV per bit) * 11 (1:11 voltage divider)
    return adcFractionalConvert(s, 88, 10, 0);
}

/**
 * @brief Conversion function for ADC to load current.
 *
 * @param s Load current sensor.
 *
 * @return Load current in mA.
 */
static int32_t adcConv_LoadCurrent_mA(cmr_sensor_t *s) {
    // mA * 0.015 Ohm * 20 V/V = mV = adcVal * 0.8 mV/bit
    // Solve for mA = adcVal * 24 / 100
    return adcFractionalConvert(s, 24, 100, 0);
}

/**
 * @brief Conversion function for ADC to logic current.
 *
 * @param s Logic current sensor.
 *
 * @return Logic current in mA.
 */
// TODO this is wrong
static int32_t adcConv_FanCurrent_mA(cmr_sensor_t *s) {
    return adcFractionalConvert(s, 8, 200, 0);
}

/**
 * @brief Conversion function for ADC to switch temperature.
 *
 * @param s Switch temperature sensor
 *
 * @return Switch temperature in degrees C.
 */
static int32_t adcConv_SwitchTemp_C(cmr_sensor_t *s) {
    checkSensor(s);

    adcChannels_t adcChannel = sensorToADCChannel(s);
    uint32_t adcVal = adcChannels[adcChannel].value;

    // adcVal * 0.8 mV/bit = 3300 mV * (10 kOhm / (10 kOhm + Rth))
    // Rth = ((10 kOhm * 3300 mV) / (adcVal * 0.8 mV/bit)) - 10 kOhm
    // Rth = ((10000 Ohm * 3.3 V) / (adcVal * 0.0008 V/bit)) - 10000 Ohm
    // Rth = (33000 / adcVal * (8 / 10000)) - 10000
    // Rth = (33000 / (adcVal * 8) / 10000) - 10000
    // Rth = (330,000,000 / (adcVal * 8)) - 10000
    uint32_t thermistorResistance_Ohm = (330000000 / (adcVal * 8)) - 10000;

    size_t tableLen = sizeof(thermTempConvs) / sizeof(thermTempConvs[0]);

    for (size_t i = 0; i < tableLen; i++) {
        if (thermistorResistance_Ohm >= thermTempConvs[i].resistance_Ohm) {
            return thermTempConvs[i].temp_C;
        }
    }

    return thermTempConvs[tableLen - 1].temp_C;
}

/**
 * @brief Conversion function for ADC to logic current.
 *
 * @param s Logic current sensor.
 *
 * @return Logic current in mA.
 */
// TODO
static int32_t adcConv_RadTherm(cmr_sensor_t *s) {
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
 * @param numerator The numerator of the fraction.
 * @param divisor The denominator of the fraction.
 * @param offset Added to the raw ADC value before multiplying by fraction.
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

    sensor->error = SENSOR_ERR_NONE;

    // Return if sensor is too far above expected range
    // accounting for overflow
    uint32_t upperThres = sensor->maxADC + (adcRange * sensor->oorThres_pcnt / 100);
    if (upperThres >= sensor->maxADC && adcVal > upperThres) {
        sensor->error = SENSOR_ERR_OUT_OF_RANGE;
    }

    // Return if sensor is too far below expected range
    // accounting for underflow
    uint32_t lowerThres = sensor->minADC - (adcRange * sensor->oorThres_pcnt / 100);
    if (lowerThres <= sensor->minADC && adcVal < lowerThres) {
        sensor->error = SENSOR_ERR_OUT_OF_RANGE;
    }
}


/**
 * @brief Initializes the sensor interface.
 */
void sensorInit(void) {
    // Task creation.
    cmr_taskInit(
        &sensorUpdate_task,
        "sensorUpdate",
        sensorUpdate_priority,
        sensorUpdate,
        NULL
    );
}
