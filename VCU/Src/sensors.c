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

bool use_emd = false;

/**
 * @brief Mapping of sensor channels to ADC channels.
 */
 //DCM Values
const adcChannel_t sensorsADCChannels[SENSOR_CH_LEN] = {
    [SENSOR_CH_VOLTAGE_CV]  = ADC_THERM1,
    [SENSOR_CH_AVG_CURRENT_DA]  = ADC_THERM2
};

//VSM Values
static const adcChannel_t sensorsADCChannels[SENSOR_CH_LEN] =
{
    [SENSOR_CH_HALL_EFFECT_A] = ADC_HALL_EFFECT,
    [SENSOR_CH_BPRES_PSI]      = ADC_REAR_BRAKE_PRES,
    [SENSOR_CH_VOLTAGE_MV]     = ADC_VSENSE,
    [SENSOR_CH_SS_IN]          = ADC_SSIN,
    [SENSOR_CH_SS_OUT]         = ADC_SSOUT
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

//VSM Sensors
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

//DCM Sensors
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
    return (amps_per_sensor_volt * sensor_volts);
}

/**
 * @brief Gets the voltage from your preferred source
 *
 * @return voltage in mV
 */
int32_t getVoltage_mV(void) {
    if(use_emd) {
        cmr_canEMDMeasurements_t *EMD_Measurement = canTractiveGetPayload(CANRX_TRAC_EMD_MEASUREMENT);
        float EMD_voltage_V = (EMD_Measurement->voltage);  
        return ((int32_t) EMD_voltage_V * 1000.0f); 
    }
    cmr_canIVTreadings_t *IVT_Measurement = canTractiveGetPayload(CANRX_TRAC_IVT_VOLTAGE); 
    int32_t IVT_voltage_mV = big_endian_to_int32(&(IVT_Measurement->message));
    return IVT_voltage_mV;
}


/**
 * @brief Gets the current from your preferred source
 *
 * @return current in mA
 */
int32_t getCurrent_mA(void) {
    if(use_emd) {
        cmr_canEMDMeasurements_t *EMD_Measurement = canTractiveGetPayload(CANRX_TRAC_EMD_MEASUREMENT);
        float EMD_current_A = (EMD_Measurement->current);  
        return ((int32_t) EMD_current_A * 1000.0f); 
    }
	cmr_canIVTreadings_t *IVT_Measurement = canTractiveGetPayload(CANRX_TRAC_IVT_CURRENT); 
    int32_t IVT_current_mA = big_endian_to_int32(&(IVT_Measurement->message));
    return IVT_current_mA ;
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
