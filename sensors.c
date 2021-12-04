/**
 * @file sensors.c
 * @brief Board-specific sensors implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/tasks.h>  // Task interface

#include "thermistor_table.h"   // switchThermTempConvs, radThermTempConvs
#include "sensors.h"            // Interface to implement
#include "adc.h"                // adcChannels_t, adcChannels
#include <CMR/adc.h>            // ADC_MAX
#include <math.h>               // math.h

/** @brief Value of resistor divider for rad thermistors. */
#define SENSORS_RAD_TEMP_DIV_RES    5600

/** @brief Numerator for rad calc, 3300 for 3.3v supply. */
#define SENSORS_RAD_TEMP_MULT       SENSORS_RAD_TEMP_DIV_RES * 3300

// Forward declaration of sensor state.
static cmr_sensor_t sensors[SENSOR_CH_LEN];

/** @brief Mapping of ADC channels to sensors. */
static const adcChannel_t sensorsADCChannels[SENSOR_CH_LEN] = {
    [SENSOR_CH_LOAD_VOLTAGE_MV]  = ADC_POWER_VSENSE,
    [SENSOR_CH_LOAD_CURRENT_MA]  = ADC_POWER_ISENSE,
    [SENSOR_CH_LOGIC_VOLTAGE_MV]  = ADC_LOGIC_VSENSE,
    [SENSOR_CH_BOARD_THERM_1]  = ADC_BOARD_THERM_1,
    [SENSOR_CH_BOARD_THERM_2]  = ADC_BOARD_THERM_2,
    [SENSOR_CH_THERM_1]  = ADC_THERM_1,
    [SENSOR_CH_THERM_2]  = ADC_THERM_2,
    [SENSOR_CH_THERM_3]  = ADC_THERM_3,
    [SENSOR_CH_THERM_4]  = ADC_THERM_4,
    [SENSOR_CH_THERM_5]  = ADC_THERM_5,
    [SENSOR_CH_THERM_6]  = ADC_THERM_6,
    [SENSOR_CH_THERM_7]  = ADC_THERM_7,
    [SENSOR_CH_THERM_8]  = ADC_THERM_8,
    [SENSOR_CH_THERM_9]  = ADC_THERM_9
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
    sensorChannel_t sensorChannel = sensor - sensors;//Sensor index
    configASSERT(sensorChannel < SENSOR_CH_LEN);
    // sensorsADCChannels[sensorChannel] is ADC channel
    return adcRead(sensorsADCChannels[sensorChannel]);
}

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
    // mA * 0.010 Ohm * 20 V/V = mV = adcVal * 0.8 mV/bit
    // Solve for mA = adcVal * 24 / 100
    return adcFractionalConvert(s, r, 4, 1, 0);
}

/**
 * @brief Conversion function for ADC to switch temperature.
 *
 * @param s The sensor.
 * @param adcVal The raw reading.
 *
 * @return Switch temperature in 10th of degrees C.
 */
static int32_t adcConvSwitchTemp_dC(const cmr_sensor_t *s, uint32_t adcVal) {
    (void) s;   // Placate compiler.

    // adcVal * 0.8 mV/bit = 3300 mV * (10 kOhm / (10 kOhm + Rth))
    // Rth = ((10 kOhm * 3300 mV) / (adcVal * 0.8 mV/bit)) - 10 kOhm
    // Rth = ((10000 Ohm * 3.3 V) / (adcVal * 0.0008 V/bit)) - 10000 Ohm
    // Rth = (33000 / adcVal * (8 / 10000)) - 10000
    // Rth = (33000 / (adcVal * 8) / 10000) - 10000
    // Rth = (330,000,000 / (adcVal * 8)) - 10000
    uint32_t thermistorResistance_Ohm = (330000000 / (adcVal * 8)) - 10000;

    for (size_t i = 0; i < thermTempConvsSwitch_len; i++) {
        if (thermistorResistance_Ohm >= thermTempConvsSwitch[i].resistance_Ohm) {
            return thermTempConvsSwitch[i].temp_dC;
        }
    }

    return thermTempConvsSwitch[thermTempConvsSwitch_len - 1].temp_dC;
}

// returns in degrees C
// takes the B value, the resistance at given tmep, that temp, the resistor in the divider, the sensed voltage, and the bias voltage
// assumes thermistor is on the high side
float thermistorCalc(float B, float r1, float rTemp, float biasR, float vSense, float vBias) {
    float r2 = biasR * ((vBias / vSense) - 1);
    float temp = (B * (rTemp + 273.15f)) / (B - ((rTemp + 273.15f) * log(r1/r2)));
    return temp - 273.15f;
}

/**
 * @brief Conversion function for ADC to radiator temperature.
 *
 * @param s The sensor.
 * @param adcVal The raw reading.
 *
 * @return Cooling temperature in 10th of degrees C.
 */
static int32_t adcConvRadTherm_dC(const cmr_sensor_t *s, uint32_t adcVal) {
    float sensed_ratio = adcVal / ((float) 4095);
    float sensed_voltage = sensed_ratio * ((float) 3.3);
//B=3892 from thermistor
    float sensed_temp = thermistorCalc(3892.f, 10000.f, 25.f, 5.6e3, sensed_voltage, 2.6f);
    return sensed_temp * 10; //to report dC units
}

/**
 * @brief Sensor state.
 *
 * TODO calibrate all of these min/max values
 */
static cmr_sensor_t sensors[SENSOR_CH_LEN] = {
    [SENSOR_CH_LOAD_VOLTAGE_MV] = {
        .sample = sampleADCSensor,
        .conv = adcConvLoadVoltage_mV,
        .readingMin = 2256, // 20 Volts
        .readingMax = 2933, // 26 Volts
        .outOfRange_pcnt = 10
    },
    [SENSOR_CH_LOAD_CURRENT_MA] = {
        .sample = sampleADCSensor,
        .conv = adcConvLoadCurrent_mA,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10
    },
    [SENSOR_CH_LOGIC_VOLTAGE_MV] = {
        .sample = sampleADCSensor,
        .conv = adcConvLogicVoltage_mV,
        .readingMin = 2256, // 20 Volts
        .readingMax = 2933, // 26 Volts
        .outOfRange_pcnt = 10
    },
    [SENSOR_CH_BOARD_THERM_1] = {
        .sample = sampleADCSensor,
        .conv = adcConvSwitchTemp_dC,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10
    },
    [SENSOR_CH_BOARD_THERM_2] = {
        .sample = sampleADCSensor,
        .conv = adcConvSwitchTemp_dC,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10
    },
    [SENSOR_CH_THERM_1] = {
        .sample = sampleADCSensor,
        .conv = adcConvRadTherm_dC,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10
    },
    [SENSOR_CH_THERM_2] = {
        .sample = sampleADCSensor,
        .conv = adcConvRadTherm_dC,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10
    },
    [SENSOR_CH_THERM_3] = {
        .sample = sampleADCSensor,
        .conv = adcConvRadTherm_dC,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10
    },
    [SENSOR_CH_THERM_4] = {
        .sample = sampleADCSensor,
        .conv = adcConvRadTherm_dC,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10
    },
    [SENSOR_CH_THERM_5] = {
        .sample = sampleADCSensor,
        .conv = adcConvRadTherm_dC,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10
    },
    [SENSOR_CH_THERM_6] = {
        .sample = sampleADCSensor,
        .conv = adcConvRadTherm_dC,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10
    },
    [SENSOR_CH_THERM_7] = {
        .sample = sampleADCSensor,
        .conv = adcConvRadTherm_dC,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10
    },
    [SENSOR_CH_THERM_8] = {
        .sample = sampleADCSensor,
        .conv = adcConvRadTherm_dC,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10
    },
    [SENSOR_CH_THERM_9] = {
        .sample = sampleADCSensor,
        .conv = adcConvRadTherm_dC,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10
    }
};

/** @brief The sensors list. */
cmr_sensorList_t sensorList;

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

