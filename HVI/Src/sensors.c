

/**
 * @file sensors.c
 * @brief Board-specific sensors implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "sensors.h"  // Interface to implement

#include <CMR/gpio.h>   // GPIO interface
#include <CMR/tasks.h>  // Task interface
#include <math.h>       // tanh()
#include <stdlib.h>     // abs()

#include "adc.h"   // Board-specific ADC interface
#include "can.h"   // Board-specific CAN interface
#include "gpio.h"  // Board-specific GPIO interface

/** @brief Number of samples for current measurement rolling average. */
#define BUS_CURRENT_SAMPLES 10

/** @brief Slope for voltage sense transfer function */
#define V_TRANS_M 19.506
/** @brief Intercept for voltage sense transfer function */
#define V_TRANS_B -8313.3

/** @brief See FSAE rule T.6.2.3 for definition of throttle implausibility. */
static const TickType_t TPOS_IMPLAUS_THRES_MS = 100;
/** @brief See FSAE rule T.6.2.3 for definition of throttle implausibility. */
static const uint32_t TPOS_IMPLAUS_THRES = UINT8_MAX / 10;

/** @brief Throttle threshold for brake implausibility. See FSAE rule EV.2.4. */
static const uint8_t BPP_TPOS_IMPLAUS_THRES = UINT8_MAX / 4;
/** @brief Throttle threshold for clearing brake implausibility. See FSAE rule EV.2.4. */
static const uint8_t BPP_TPOS_CLEAR_THRES = UINT8_MAX / 20;
/** @brief Threshold where brakes are considered to be actuated. */
static const uint8_t BRAKE_ACTIVE_THRES_PSI = 40;

/** @brief 90 degree sw left lock adc value. */
#define SWANGLE_90DEG_LEFT 1345
/** @brief -90 degree sw RIGHT lock adc value. */
#define SWANGLE_90DEG_RIGHT 3155

/**
 * @brief Mapping of sensor channels to ADC channels.
 */
const adcChannel_t sensorsADCChannels[SENSOR_CH_LEN] = {
    [SENSOR_CH_HV] = ADC_VSENSE,
    [SENSOR_CH_CURRENT] = ADC_ISENSE,
    [SENSOR_CH_VREF] = ADC_VREF
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
 * @brief Rescales ADC value from 12 bit to 8 bit.
 *
 * @param sensor The sensor to read.
 * @param reading The ADC value to convert.
 *
 * @return Value of sensor, rescaled to 0-255.
 */
static int32_t adcToUInt8(const cmr_sensor_t *sensor, uint32_t reading) {
    int32_t sensorVal = 0;
    if (reading >= sensor->readingMax) {
        sensorVal = UINT8_MAX;
    } else if (reading <= sensor->readingMin) {
        sensorVal = 0;
    } else {
        uint32_t sensorRange = sensor->readingMax - sensor->readingMin;
        uint32_t readingFromZero = reading - sensor->readingMin;
        // If UINT8_MAX * readingFromZero will overflow, do division first
        if (UINT32_MAX / readingFromZero < UINT8_MAX) {
            sensorVal = readingFromZero / sensorRange * UINT8_MAX;
        } else {
            sensorVal = UINT8_MAX * readingFromZero / sensorRange;
        }
    }

    return sensorVal;
}

/**
 * @brief Converts a raw ADC value to voltage in centivolts.
 *
 * @param sensor The sensor to read.
 *
 * @param reading The ADC value to convert.
 *
 * @return Voltage in centivolts.
 */
static uint16_t adcToVoltage(const cmr_sensor_t *sensor, uint32_t reading) {
    (void)sensor;

    uint16_t voltage = (V_TRANS_M*reading + (V_TRANS_B));

    return voltage;
}

/**
 * @brief Converts a raw ADC value into a current.
 *
 * @param sensor The sensor to read.
 *
 * @param reading The ADC value to convert.
 *
 * @return Current in A.
 */
static uint16_t adcToCurrent(const cmr_sensor_t *sensor, uint32_t reading) {
    (void)sensor;

    uint16_t current = (uint16_t) reading;

    return current;
}

static uint16_t adcToVref(const cmr_sensor_t *sensor, uint32_t reading) {
    (void)sensor;

    uint16_t vref = (uint16_t) reading;

    return vref;
}

static cmr_sensor_t sensors[SENSOR_CH_LEN] = {
    [SENSOR_CH_HV] = {
        .conv = adcToVoltage,
        .sample = sampleADCSensor,
        .readingMin = 0,
        .readingMax = 4096,
        .outOfRange_pcnt = 10,
        .warnFlag = 0 },
    [SENSOR_CH_CURRENT] = {
        .conv = adcToCurrent,
        .sample = sampleADCSensor,
        .readingMin = 0,
        .readingMax = 4096,
        .outOfRange_pcnt = 10,
        .warnFlag = 0
    }
    [SENSOR_CH_VREF] = {
        .conv = adcToVref,
        .sample = sampleADCSensor,
        .readingMin = 0,
        .readingMax = 4096,
        .outOfRange_pcnt = 10,
        .warnFlag = 0
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
    (void)pvParameters;  // Placate compiler.

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
        sensors, sizeof(sensors) / sizeof(sensors[0]));

    // Task creation.
    cmr_taskInit(
        &sensorsUpdate_task,
        "sensor update",
        sensorsUpdate_priority,
        sensorsUpdate,
        NULL);
}


