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

/** @brief See FSAE rule T.6.2.3 for definition of throttle implausibility. */
static const TickType_t TPOS_IMPLAUS_THRES_MS = 100;
/** @brief See FSAE rule T.6.2.3 for definition of throttle implausibility. */
static const uint32_t TPOS_IMPLAUS_THRES = UINT8_MAX / 10;

/** @brief Throttle threshold for brake implausibility. See FSAE rule EV.2.4. */
static const uint8_t BPP_TPOS_IMPLAUS_THRES = UINT8_MAX / 4;
/** @brief Throttle threshold for clearing brake implausibility. See FSAE rule EV.2.4. */
static const uint8_t BPP_TPOS_CLEAR_THRES = UINT8_MAX / 20;
/** @brief Threshold where brakes are considered to be actuated. */
static const uint8_t BRAKE_ACTIVE_THRES_PSI = 50;

/**
 * @brief Mapping of sensor channels to ADC channels.
 */
const adcChannel_t sensorsADCChannels[SENSOR_CH_LEN] = {
    [SENSOR_CH_TPOS_L] = ADC_CH1,
    [SENSOR_CH_TPOS_R] = ADC_CH3,
    [SENSOR_CH_BPOS_PCNT]   = ADC_CH4,
    [SENSOR_CH_BPRES_PSI]   = ADC_CH5,
    [SENSOR_CH_SWANGLE_DEG] = ADC_CH2,
    [SENSOR_CH_VOLTAGE_MV]  = ADC_VSENSE,
    [SENSOR_CH_AVG_CURRENT_MA]  = ADC_ISENSE,
    [SENSOR_CH_TPOS_IMPLAUS] = ADC_LEN  // Not an ADC channel!
};

/** @brief forward declaration */
static cmr_sensor_t sensors[];

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
 * @brief Converts a raw sensor ADC value to a percentage.
 *
 * @param sensor The sensor to read.
 * @param reading The ADC value to convert.
 *
 * @return Percentage value of sensor (0 to 100, inclusive).
 */
static int32_t adcToPcnt(const cmr_sensor_t *sensor, uint32_t reading) {
    // Scale to a percentage
    int32_t sensorPcnt = 0;
    if (reading >= sensor->readingMax) {
        sensorPcnt = 100;
    } else if (reading <= sensor->readingMin) {
        sensorPcnt = 0;
    } else {
        uint32_t sensorRange = sensor->readingMax - sensor->readingMin;
        uint32_t readingFromZero = reading - sensor->readingMin;
        // If 100 *readingFromZerowill overflow, do division first
        if (UINT32_MAX / readingFromZero < 100) {
            sensorPcnt = readingFromZero / sensorRange * 100;
        } else {
            sensorPcnt = 100 * readingFromZero / sensorRange;
        }
    }

    return sensorPcnt;
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
    }
    else if (reading <= sensor->readingMin) {
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
 * @brief Converts a raw ADC value to a brake pressure in PSI.
 *
 * @param sensor The sensor to read.
 *
 * @param reading The ADC value to convert.
 *
 * @return Front brake pressure in PSI.
 */
static int32_t adcToBPres_PSI(const cmr_sensor_t *sensor, uint32_t reading) {
    (void) sensor;  // Placate compiler.

    // Temporarily just rescale raw 12-bit value to 8 bits
    return (int32_t) (reading >> 4);
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
static int32_t adcToBusVoltage_mV(const cmr_sensor_t *sensor, uint32_t reading) {
    (void) sensor;  // Placate compiler.

    // value * 0.8 (mV per bit) * 11 (1:11 voltage divider)
    uint32_t busVoltage_mV = reading * 8 * 11 / 10;
    return (int32_t) busVoltage_mV;
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
static int32_t adcToAvgBusCurrent_mA(const cmr_sensor_t *sensor, uint32_t reading) {
    (void) sensor;  // Placate compiler.

    /** @brief Previous current values, in milliamps. */
    static uint32_t currentSamples_mA[BUS_CURRENT_SAMPLES] = { 0 };

    /** @brief Sum of prevCurrents_mA elements. */
    static uint32_t currentSamplesSum_mA = 0;

    /** @brief Index of oldest current. */
    static size_t oldestIndex = 0;

    /* value * 0.8 (mV per bit) / 20 (gain of current shunt monitor)
     * http://www.ti.com/lit/ds/symlink/ina196.pdf
     * page 3 section 5 for INA196 */
    uint32_t busCurrent_mA = reading * 8 / 10 / 20;

    // Update sum of currents
    currentSamplesSum_mA -= currentSamples_mA[oldestIndex];
    currentSamplesSum_mA += busCurrent_mA;

    // Overwrite oldest current with newest value.
    currentSamples_mA[oldestIndex] = busCurrent_mA;
    oldestIndex++;
    if (oldestIndex == BUS_CURRENT_SAMPLES) {
        oldestIndex = 0;
    }

    return (int32_t) currentSamplesSum_mA / BUS_CURRENT_SAMPLES;
}

/**
 * @brief Samples whether the throttle position is implausible.
 *
 * @param sensor The sensor (ignored).
 *
 * @return 1 if the throttle position is implausible, 0 otherwise.
 */
static uint32_t sampleTPOSDiff(const cmr_sensor_t *sensor) {
    (void) sensor;  // Placate compiler.

    /** @brief Last plausible time. */
    static TickType_t lastPlausible = 0;
    TickType_t now = xTaskGetTickCount();

    uint32_t diff;
    uint32_t leftPosition = cmr_sensorListGetValue(&sensorList, SENSOR_CH_TPOS_L);
    uint32_t rightPosition = cmr_sensorListGetValue(&sensorList, SENSOR_CH_TPOS_R);
    if (leftPosition > rightPosition) {
        diff = leftPosition - rightPosition;
    } else {
        diff = rightPosition - leftPosition;
    }

    if (diff < TPOS_IMPLAUS_THRES) {
        // Still plausible; move on.
        lastPlausible = now;
        return 0;
    }

    if (now - lastPlausible < TPOS_IMPLAUS_THRES_MS) {
        // Threshold not elapsed; move on.
        return 0;
    }

    return 1;   // Implausible!
}

/**
 * @brief Sample whether the brake pedal position is implausible.
 *
 * @param sensor The sensor (ignored).
 *
 * @return 1 if the brake pedal position is implausible, 0 otherwise.
 */
static uint32_t sampleBrakeImplaus(const cmr_sensor_t *sensor) {
    (void) sensor;  // Placate compiler.

    /** @brief Current implausibility status. */
    static bool implaus = false;

    cmr_canVSMSensors_t *vsmSensors = getPayload(CANRX_VSM_SENSORS);

    uint8_t throttlePos = throttleGetPos();
    uint16_t brakePres_PSI = vsmSensors->brakePressureRear_PSI;

    // Set brake implausibility
    if ((throttlePos > BPP_TPOS_IMPLAUS_THRES)
     && (brakePres_PSI > BRAKE_ACTIVE_THRES_PSI)) {
        implaus = true;
    }
    // Clear brake implausibility
    else if (throttlePos < BPP_TPOS_CLEAR_THRES) {
        implaus = false;
    }

    return implaus;
}

static cmr_sensor_t sensors[SENSOR_CH_LEN] = {
    [SENSOR_CH_TPOS_L] = {
        .conv = adcToUInt8,
        .sample = sampleADCSensor,
        .readingMin = 400,
        .readingMax = 3200,
        .outOfRange_pcnt = 10,
        .warnFlag = CMR_CAN_WARN_FSM_TPOS_L
    },
    [SENSOR_CH_TPOS_R] = {
        .conv = adcToUInt8,
        .sample = sampleADCSensor,
        .readingMin = 330,
        .readingMax = 3080,
        .outOfRange_pcnt = 10,
        .warnFlag = CMR_CAN_WARN_FSM_TPOS_R
    },
    // TODO calibrate min/max values for SENSOR_CH_BPOS_PCNT
    [SENSOR_CH_BPOS_PCNT] = {
        .conv = adcToPcnt,
        .sample = sampleADCSensor,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10,
        .warnFlag = CMR_CAN_WARN_FSM_BPOS
    },
    // TODO calibrate min/max values for SENSOR_CH_BPRES_PSI
    [SENSOR_CH_BPRES_PSI] = {
        .conv = adcToBPres_PSI,
        .sample = sampleADCSensor,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10,
        .warnFlag = CMR_CAN_WARN_FSM_BPRES
    },
    // TODO calibrate min/max values for SENSOR_CH_SWANGLE_DEG
    [SENSOR_CH_SWANGLE_DEG] = {
        .conv = NULL,
        .sample = sampleADCSensor,
        .readingMin = 0,
        .readingMax = CMR_ADC_MAX,
        .outOfRange_pcnt = 10,
        .warnFlag = CMR_CAN_WARN_FSM_SWANGLE
    },
    [SENSOR_CH_VOLTAGE_MV] = {
        .conv = adcToBusVoltage_mV,
        .sample = sampleADCSensor,
        .readingMin = 2256, // 20 Volts
        .readingMax = 2933, // 26 Volts
        .outOfRange_pcnt = 10,
        .warnFlag = CMR_CAN_WARN_BUS_VOLTAGE,
    },
    [SENSOR_CH_AVG_CURRENT_MA] = {
        .conv = adcToAvgBusCurrent_mA,
        .sample = sampleADCSensor,
        .readingMin = 250,  // 10 mA
        .readingMax = 2500, // 100 mA
        .outOfRange_pcnt = 10,
        .warnFlag = CMR_CAN_WARN_BUS_CURRENT
    },
    [SENSOR_CH_TPOS_IMPLAUS] = {
        .conv = NULL,
        .sample = sampleTPOSDiff,
        .readingMin = 0,    // reading will be true or false
        .readingMax = 0,
        .warnFlag = CMR_CAN_WARN_FSM_TPOS_IMPLAUSIBLE
    },
    [SENSOR_CH_BPP_IMPLAUS] = {
        .conv = NULL,
        .sample = sampleBrakeImplaus,
        .readingMin = 0,    // reading will be true or false
        .readingMax = 0,
        .warnFlag = CMR_CAN_WARN_FSM_BPP
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
        "sensor update",
        sensorsUpdate_priority,
        sensorsUpdate,
        NULL
    );
}

/**
 * @brief Gets average throttle position.
 *
 * @note If either sensor has an error or the throttle position is implausible,
 * 0 is returned.
 *
 * @return Throttle position (0-255, inclusive).
 */
uint8_t throttleGetPos(void) {
    if (/* cmr_sensorListGetError(&sensorList, SENSOR_CH_TPOS_L) != CMR_SENSOR_ERR_NONE ||
        cmr_sensorListGetError(&sensorList, SENSOR_CH_TPOS_R) != CMR_SENSOR_ERR_NONE || */
        cmr_sensorListGetValue(&sensorList, SENSOR_CH_TPOS_IMPLAUS) != 0
    ) {
        return 0;
    }

    int32_t tposL = cmr_sensorListGetValue(&sensorList, SENSOR_CH_TPOS_L);
    int32_t tposR = cmr_sensorListGetValue(&sensorList, SENSOR_CH_TPOS_R);

    if ((tposL == 0) || (tposR == 0)) {
        return 0;
    }

    return (uint8_t)((tposL + tposR) / 2);
}
