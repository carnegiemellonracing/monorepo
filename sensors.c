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

/** @brief Mapping of ADC channels to sensors. */
static const adcChannel_t sensorsADCChannels[SENSOR_CH_LEN] = {
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
 * @brief Gets a new value from an ADC sensor.
 *
 * This function simply copies data from the ADC - it does not make any requests
 * to ADC hardware.
 *
 * @param sensor The ADC sensor to update.
 *
 * @return The new value from the ADC.
 */
static uint32_t sampleADCSensor(const cmr_sensor_t *sensor) {
    sensorChannel_t sensorChannel = sensor - sensors;
    configASSERT(sensorChannel < SENSOR_CH_LEN);
    return adcRead(sensorsADCChannels[sensorChannel]);
}

/** @brief Represents a thermistor temperature conversion. */
typedef struct {
    uint32_t resistance_Ohm;    /**< @brief Resistance (Ohms). */
    int32_t temp_C;             /**< @brief Temperature (degrees Celsius). */
} thermistorTempConversion_t;

/**
 * @brief Conversions from thermistor resistance to temperature.
 *
 * @warning MUST be sorted in descending order of resistance!
 */
static thermistorTempConversion_t thermTempConvs[] = {{
    .resistance_Ohm = 27280,
    .temp_C = 0
}, {
    .resistance_Ohm = 17960,
    .temp_C = 10,
}, {
    .resistance_Ohm = 12090,
    .temp_C = 20
}, {
    .resistance_Ohm = 8310,
    .temp_C = 30
}, {
    .resistance_Ohm = 5826,
    .temp_C = 40
}, {
    .resistance_Ohm = 4158,
    .temp_C = 50
}, {
    .resistance_Ohm = 3019,
    .temp_C = 60
}, {
    .resistance_Ohm = 2227,
    .temp_C = 70
}, {
    .resistance_Ohm = 1668,
    .temp_C = 80
}, {
    .resistance_Ohm = 1267,
    .temp_C = 90
}, {
    .resistance_Ohm = 975,
    .temp_C = 100
}, {
    .resistance_Ohm = 760,
    .temp_C = 110
}, {
    .resistance_Ohm = 599,
    .temp_C = 120
}, {
    .resistance_Ohm = 478,
    .temp_C = 130
}, {
    .resistance_Ohm = 385,
    .temp_C = 140
}, {
    .resistance_Ohm = 313,
    .temp_C = 150
}};

/**
 * @brief Scales a raw ADC reading from a sensor.
 *
 * @param sensor The sensor.
 * @param reading The raw ADC reading to convert.
 * @param numerator The numerator of the fraction.
 * @param divisor The denominator of the fraction.
 * @param offset Added to the raw ADC value before multiplying by fraction.
 *
 * @return Scaled value. Sets channel.error field in case of error.
 */
static int32_t adcFractionalConvert(
    const cmr_sensor_t *sensor, uint32_t reading,
    int32_t numerator, int32_t divisor, uint32_t offset
) {
    (void) sensor;  // Placate compiler.
    return ((int32_t) (reading + offset)) * numerator / divisor;
}

/**
 * @brief Conversion function for ADC to logic voltage.
 *
 * @param s The sensor.
 * @param r The raw reading.
 *
 * @return Logic voltage in mV.
 */
static int32_t adcConvLogicVoltage_mV(const cmr_sensor_t *s, uint32_t r) {
    // value * 0.8 (mV per bit) * 11 (1:11 voltage divider)
    return adcFractionalConvert(s, r, 88, 10, 0);
}

/**
 * @brief Conversion function for ADC to logic current.
 *
 * @param s The sensor.
 * @param r The raw reading.
 *
 * @return Logic current in mA.
 */
static int32_t adcConvLogicCurrent_mA(const cmr_sensor_t *s, uint32_t r) {
    // mA * 1.5 Ohm * 20 V/V = mV = adcVal * 0.8 mV/bit
    // solve for mA = adcVal * 8 / 300
    return adcFractionalConvert(s, r, 8, 300, 0);
}

/**
 * @brief Conversion function for ADC to load voltage.
 *
 * @param s The sensor.
 * @param r The raw reading.
 *
 * @return load voltage in mV.
 */
static int32_t adcConvLoadVoltage_mV(const cmr_sensor_t *s, uint32_t r) {
    // value * 0.8 (mV per bit) * 11 (1:11 voltage divider)
    return adcFractionalConvert(s, r, 88, 10, 0);
}

/**
 * @brief Conversion function for ADC to load current.
 *
 * @param s The sensor.
 * @param r The raw reading.
 *
 * @return Load current in mA.
 */
static int32_t adcConvLoadCurrent_mA(const cmr_sensor_t *s, uint32_t r) {
    // mA * 0.015 Ohm * 20 V/V = mV = adcVal * 0.8 mV/bit
    // Solve for mA = adcVal * 24 / 100
    return adcFractionalConvert(s, r, 24, 100, 0);
}

/**
 * @brief Conversion function for ADC to fan current.
 *
 * @param s The sensor.
 * @param r The raw reading.
 *
 * @return Fan current in mA.
 */
static int32_t adcConvFanCurrent_mA(const cmr_sensor_t *s, uint32_t r) {
    // TODO this is wrong
    return adcFractionalConvert(s, r, 8, 200, 0);
}

/**
 * @brief Conversion function for ADC to switch temperature.
 *
 * @param s The sensor.
 * @param adcVal The raw reading.
 *
 * @return Switch temperature in degrees C.
 */
static int32_t adcConvSwitchTemp_C(const cmr_sensor_t *s, uint32_t adcVal) {
    (void) s;   // Placate compiler.

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
 * @brief Conversion function for ADC to radiator temperature.
 *
 * @param s The sensor.
 * @param r The raw reading.
 *
 * @return Radiator temperature in degrees C.
 */
static int32_t adcConv_RadTherm(const cmr_sensor_t *s, uint32_t r) {
    // TODO
    return adcFractionalConvert(s, r, 1, 1, 0);
}

// TODO calibrate all of these min/max values
cmr_sensor_t sensors[SENSOR_CH_LEN] = {
    [SENSOR_CH_LOGIC_VOLTAGE_MV] = {
        .sample = sampleADCSensor,
        .conv = adcConvLogicVoltage_mV,
        .readingMin = 2256, // 20 Volts
        .readingMax = 2933, // 26 Volts
        .outOfRange_pcnt = 10,
        .value = 24000
    },
    [SENSOR_CH_LOGIC_CURRENT_MA] = {
        .sample = sampleADCSensor,
        .conv = adcConvLogicCurrent_mA,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10,
        .value = 20
    },
    [SENSOR_CH_LOAD_VOLTAGE_MV] = {
        .sample = sampleADCSensor,
        .conv = adcConvLoadVoltage_mV,
        .readingMin = 2256, // 20 Volts
        .readingMax = 2933, // 26 Volts
        .outOfRange_pcnt = 10,
        .value = 24000
    },
    [SENSOR_CH_LOAD_CURRENT_MA] = {
        .sample = sampleADCSensor,
        .conv = adcConvLoadCurrent_mA,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10,
        .value = 20
    },
    [SENSOR_CH_FAN_CURRENT_MA] = {
        .sample = sampleADCSensor,
        .conv = adcConvFanCurrent_mA,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10,
        .value = 20
    },
    [SENSOR_CH_BOARD_THERM_1] = {
        .sample = sampleADCSensor,
        .conv = adcConvSwitchTemp_C,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10,
        .value = 20
    },
    [SENSOR_CH_BOARD_THERM_2] = {
        .sample = sampleADCSensor,
        .conv = adcConvSwitchTemp_C,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10,
        .value = 20
    },
    [SENSOR_CH_PRE_RAD_THERM] = {
        .sample = sampleADCSensor,
        .conv = adcConvRadTherm,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10,
        .value = 20
    },
    [SENSOR_CH_POST_RAD_THERM] = {
        .sample = sampleADCSensor,
        .conv = adcConvRadTherm,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10,
        .value = 20
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
        for (size_t i = 0; i < SENSOR_CH_LEN; i++) {
            cmr_sensorRead(sensors + i);
        }

        vTaskDelayUntil(&lastWakeTime, sensorsUpdate_period_ms);
    }
}

/**
 * @brief Initializes the sensor interface.
 */
void sensorsInit(void) {
    // Initialize each sensor.
    for (size_t i = 0; i < SENSOR_CH_LEN; i++) {
        cmr_sensorInit(sensors + i);
    }

    // Task creation.
    cmr_taskInit(
        &sensorsUpdate_task,
        "sensors update",
        sensorsUpdate_priority,
        sensorsUpdate,
        NULL
    );
}

