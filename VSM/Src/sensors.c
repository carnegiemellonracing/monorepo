/**
 * @file sensors.c
 * @brief Board-specific sensors implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/tasks.h>  // Task interface

#include "sensors.h"    // Interface to implement
#include "adc.h"        // Board-specific ADC interface

/** @brief forward declare sensors vector. */
cmr_sensor_t sensors[SENSOR_CH_LEN];

/** @brief Mapping of ADC channels to sensors. */
static const adcChannel_t sensorsADCChannels[SENSOR_CH_LEN] =
{
    [SENSOR_CH_HALL_EFFECT_A] = ADC_HALL_EFFECT,
    [SENSOR_CH_BPRES_PSI]      = ADC_REAR_BRAKE_PRES,
    [SENSOR_CH_VOLTAGE_MV]     = ADC_VSENSE,
    [SENSOR_CH_SS_IN]          = ADC_SSIN,
    [SENSOR_CH_SS_OUT]         = ADC_SSOUT
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

/**
 * @brief Converts a raw sensor value into a low-voltage bus voltage.
 *
 * @param sensor The sensor.
 * @param value The raw value.
 *
 * @return Voltage in mV.
 */
static int32_t adcToBusVoltage_mV(const cmr_sensor_t *sensor, uint32_t value) {
    (void) sensor;  // Placate compiler.

    // value * 0.8 (mV per bit) * 11 (1:11 voltage divider)
    uint32_t busVoltage_mV = value * 8 * 11 / 10;
    return (int32_t) busVoltage_mV;
}

/**
 * @brief Converts a raw sensor value into a low-voltage bus voltage.
 *
 * @param sensor The sensor.
 * @param value The raw value.
 *
 * @return Voltage in qV.
 */
static int32_t adcToBusVoltage_eight_V(const cmr_sensor_t *sensor, uint32_t value) {
    (void) sensor;  // Placate compiler.
    uint32_t busVoltage_mV = adcToBusVoltage_mV(sensor, value);
    return busVoltage_mV / 125;
}

/**
 * @brief Converts a raw sensor value into a low-voltage bus current.
 *
 * @param sensor The sensor.
 * @param value The raw value.
 *
 * @return Current in mA.
 */
static int32_t adcToBusCurrent_mA(const cmr_sensor_t *sensor, uint32_t value) {
    (void) sensor;  // Placate compiler.

    /* value * 0.8 (mV per bit) / 20 (gain of current shunt monitor)
     * http://www.ti.com/lit/ds/symlink/ina196.pdf
     * page 3 section 5 for INA196 */
    uint32_t busCurrent_mA = value * 8 / 10 / 20;
    return (int32_t) busCurrent_mA;
}

/**
 * @brief Converts a raw sensor value into accumulator current draw.
 *
 * @param sensor The sensor.
 * @param value The raw value.
 *
 * @return Current in amps
 */
static int32_t adc_to_hv_current(const cmr_sensor_t *sensor, uint32_t value) {
    (void) sensor;  // Placate compiler.

    // https://www.lem.com/sites/default/files/products_datasheets/ho_50_250-s-0100_series.pdf
    float mcu_volts_per_adc = 3.3f / 4096.0f; 
    float offset_mcu_volts = 1.568f; // Reading at 0 amps off car
    float amps_per_sensor_volt = 125.0f / 2.0f;
    float sensor_volts_per_mcu_volt = 5.51f / 3.3f; // Based on voltage divider

    float mcu_volts_no_offset = value * mcu_volts_per_adc;
    float mcu_volts = mcu_volts_no_offset - offset_mcu_volts;
    float sensor_volts = mcu_volts * sensor_volts_per_mcu_volt;
    return (int8_t) (amps_per_sensor_volt * sensor_volts);
}


/**
 * @brief Converts a raw sensor value to a brake pressure in PSI.
 *
 * @param sensor The sensor.
 * @param value The raw value.
 *
 * @return Front brake pressure in PSI.
 */
static int32_t adcToBrakePres_PSI(const cmr_sensor_t *sensor, uint32_t value) {
    // https://www.variohm.com/images/datasheets/EPT3100_0113_F_1.pdf
    // EPT3100-H-10000 is (0, 100) bar, (0.5, 4.5) V
    // Divider is 5-to-3.3 V -> (0.333, 3) V
    // 100 bar is 1450 PSI
    static const uint32_t offset = 360;     // 0.333 V offset

    (void) sensor;  // Placate compiler.

    if (value < offset) {
        // Clamp to 0.
        value = offset;
    }

    uint32_t brakePres_PSI = (value - offset) * 1450 / 3313;
    return (int32_t) brakePres_PSI;
}

// TODO calibrate all of these min/max values
cmr_sensor_t sensors[SENSOR_CH_LEN] = {
    [SENSOR_CH_HALL_EFFECT_A] = {
        .sample = sampleADCSensor,
        .conv = adc_to_hv_current,
        .readingMin = 0,            // TODO
        .readingMax = CMR_ADC_MAX,  // TODO
        .outOfRange_pcnt = 10,
        .errorFlag = CMR_CAN_ERROR_NONE
    },
    [SENSOR_CH_BPRES_PSI] = {
        .sample = sampleADCSensor,
        .conv = adcToBrakePres_PSI,
        .readingMin = 0,            // TODO
        .readingMax = CMR_ADC_MAX,  // TODO
        .outOfRange_pcnt = 10,
        .errorFlag = CMR_CAN_ERROR_VSM_BPRES
    },
    [SENSOR_CH_VOLTAGE_MV] = {
        .conv = adcToBusVoltage_mV,
        .sample = sampleADCSensor,
        .readingMin = 2256, // 20 Volts
        .readingMax = 2933, // 26 Volts
        .outOfRange_pcnt = 10,
        .warnFlag = CMR_CAN_WARN_BUS_VOLTAGE
    },
    [SENSOR_CH_CURRENT_MA] = {
        .conv = adcToBusCurrent_mA,
        .sample = sampleADCSensor,
        .readingMin = 250,  // 10 mA
        .readingMax = 2500, // 100 mA
        .outOfRange_pcnt = 10,
        .warnFlag = CMR_CAN_WARN_BUS_CURRENT
    },
    [SENSOR_CH_SS_IN] = {
        .conv = adcToBusVoltage_eight_V,
        .sample = sampleADCSensor,
        .readingMin = 250,  
        .readingMax = 2933, // 26 Volts
        .outOfRange_pcnt = 10,
        .warnFlag = CMR_CAN_WARN_BUS_VOLTAGE
    },
    [SENSOR_CH_SS_OUT] = {
        .conv = adcToBusVoltage_eight_V,
        .sample = sampleADCSensor,
        .readingMin = 250,  // 10 mA
        .readingMax = 2933, // 26 Volts
        .outOfRange_pcnt = 10,
        .warnFlag = CMR_CAN_WARN_BUS_CURRENT
    },
};

/** @brief All sensors. */
cmr_sensorList_t sensorList;

/** @brief Sensors update priority. */
static const uint32_t sensorsUpdate_priority = 5;

/** @brief Sensors update period (milliseconds). */
static const TickType_t sensorsUpdate_period_ms = 5;

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
    // Initialize sensors.
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

