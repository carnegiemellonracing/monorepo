/**
 * @file sensors.c
 * @brief Board-specific sensors implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/tasks.h>  // Task interface

#include "sensors.h"            // Interface to implement
#include "adc.h"                // adcChannels_t, adcChannels
#include "pilot_table.h"        // Tables for pilot signal conversions
#include <CMR/adc.h>            // ADC_MAX
#include <math.h>               // math.h

// Forward declaration of sensor state.
static cmr_sensor_t sensors[SENSOR_CH_LEN];

/**
 * @brief Mapping of sensor channels to ADC channels.
 */
const adcChannel_t sensorsADCChannels[SENSOR_CH_LEN] = {
    [SENSOR_CH_PILOT_DUTY] =    ADC_AVG_OUT,
    [SENSOR_CH_PILOT_VOLTAGE] = ADC_PEAK_OUT,
    [SENSOR_CH_THERM_1] = ADC_THERM_1,
    [SENSOR_CH_THERM_2] = ADC_THERM_2,
    [SENSOR_CH_SAFETY] = ADC_SAFETY
};

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
 * @brief Converts a raw ADC value into the pilot signal duty cycle
 *
 * @param sensor The sensor to read.
 *
 * @param reading The ADC value to convert.
 *
 * @return Pilot signal duty cycle percentage as an integer (e.g. 50% -> 50)
 */
static int32_t adcToPilotDutyCycle(const cmr_sensor_t *sensor, uint32_t reading) {
    (void) sensor;  // Placate compiler.

    /*
     * ADC value * 0.8 mV/bit = voltage
     */
    uint32_t voltage = reading * 8 / 10;

    for (int i = 0; i < dutyCycleVoltages_len - 1; i++) {
        if (dutyCycleVoltages[i].voltage <= voltage && dutyCycleVoltages[i + 1].voltage >= voltage) {
            uint32_t diff_x = dutyCycleVoltages[i + 1].voltage - dutyCycleVoltages[i].voltage;
            uint32_t diff_y = dutyCycleVoltages[i + 1].dutyCycle - dutyCycleVoltages[i].dutyCycle;

            return diff_y * (voltage - dutyCycleVoltages[i].voltage) / diff_x + dutyCycleVoltages[i].dutyCycle;
        }
    }

    /*
     * If we're getting an invalid value set the duty cycle to 0 to force maximum current to 0.
     */
    return 0;
}

/**
 * @brief Converts a raw ADC value into the pilot signal voltage
 *
 * @param sensor The sensor to read.
 *
 * @param reading The ADC value to convert.
 *
 * @return Pilot signal duty cycle voltage in volts
 */
static int32_t adcToPilotVoltage(const cmr_sensor_t *sensor, uint32_t reading) {
    (void) sensor;  // Placate compiler.

    /*
     * ADC value * 0.8 mV/bit = voltage
     */
    uint32_t voltage = reading * 8 / 10;

    if (voltage >= 436 && voltage <= 486) {
        return 3;
    } else if (voltage >= 800 && voltage <= 1212) {
        return 6;
    } else if (voltage >= 1780 && voltage <= 1946) {
        return 9;
    }

    return 0;
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
 * TODO: Are the min/max values correct?
 */
static cmr_sensor_t sensors[SENSOR_CH_LEN] = {
    [SENSOR_CH_PILOT_DUTY] = {
        .sample = sampleADCSensor,
        .conv = adcToPilotDutyCycle,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10
    },
    [SENSOR_CH_PILOT_VOLTAGE] = {
        .sample = sampleADCSensor,
        .conv = adcToPilotVoltage,
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
    [SENSOR_CH_SAFETY] = {
        .sample = sampleADCSensor,
        .conv = NULL,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10
    },
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
