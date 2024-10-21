/**
 * @file sensors.c
 * @brief Sensor reading and conversion implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <FreeRTOS.h>   // configASSERT()

#include "CMR/sensors.h"     // Interface to implement

/**
 * @brief Initializes the sensor.
 *
 * @note The sensor's public fields should already be filled in!
 *
 * @param sensor The sensor to initialize.
 */
static void cmr_sensorInit(cmr_sensor_t *sensor) {
    configASSERT(sensor->readingMax >= sensor->readingMin);

    // Calculate tolerance.
    const uint32_t readingRange = sensor->readingMax - sensor->readingMin;
    uint32_t readingTolerance;
    // Prevent overflow.
    if (UINT32_MAX / sensor->outOfRange_pcnt < readingRange) {
        readingTolerance = (readingRange / 100) * sensor->outOfRange_pcnt;
    } else {
        readingTolerance = (readingRange * sensor->outOfRange_pcnt) / 100;
    }

    // Calculate thresholds from tolerance.
    uint32_t readingUpper = sensor->readingMax + readingTolerance;
    if (readingUpper < sensor->readingMax) {
        readingUpper = UINT32_MAX;  // Overflow!
    }

    uint32_t readingLower = sensor->readingMin - readingTolerance;
    if (readingLower > sensor->readingMin) {
        readingLower = 0;  // Underflow!
    }

    sensor->_ = (struct cmr_sensor_private) {
        .readingUpper = readingUpper,
        .readingLower = readingLower,
        .value = 0,
        .error = CMR_SENSOR_ERR_NONE
    };
}

/**
 * @brief Updates the sensor.
 *
 * @param sensor The sensor to sample.
 */
static void cmr_sensorUpdate(cmr_sensor_t *sensor) {
    uint32_t reading = sensor->sample(sensor);

    // Check if sensor has gone out-of-range.
    if (reading < sensor->_.readingLower || sensor->_.readingUpper < reading) {
        sensor->_.error = CMR_SENSOR_ERR_OUT_OF_RANGE;
    } else {
        sensor->_.error = CMR_SENSOR_ERR_NONE;
    }

    uint32_t value;
    if (sensor->conv != NULL) {
        value = sensor->conv(sensor, reading);
    } else {
        value = reading;
    }

    sensor->_.value = value;
}

/**
 * @brief Initializes the sensor list.
 *
 * @param list The list to initialize.
 * @param sensors The array of sensors in the list.
 * @param sensorsLen The total number of sensors in the array.
 */
void cmr_sensorListInit(
    cmr_sensorList_t *list,
    cmr_sensor_t *sensors, size_t sensorsLen
) {
    for (size_t i = 0; i < sensorsLen; i++) {
        cmr_sensorInit(sensors + i);
    }

    list->sensors = sensors;
    list->sensorsLen = sensorsLen;
}

/**
 * @brief Updates the given sensor list.
 *
 * @param list The list to update.
 */
void cmr_sensorListUpdate(cmr_sensorList_t *list) {
    for (size_t i = 0; i < list->sensorsLen; i++) {
        cmr_sensorUpdate(list->sensors + i);
    }
}

/**
 * @brief Sets the warning and/or error flags for each out-of-range sensor.
 *
 * @note The flags are bitwise-OR'd into the provided fields.
 *
 * @param list The list.
 * @param warnp The warnings to update, or NULL to ignore.
 * @param errorp The errors to update, or NULL to ignore.
 */
void cmr_sensorListGetFlags(
    cmr_sensorList_t *list,
    cmr_canWarn_t *warnp, cmr_canError_t *errorp
) {
    cmr_canWarn_t warn = (warnp == NULL) ? 0 : *warnp;
    cmr_canError_t error = (errorp == NULL) ? 0 : *errorp;

    for (size_t i = 0; i < list->sensorsLen; i++) {
        cmr_sensor_t *sensor = list->sensors + i;
        switch (sensor->_.error) {
            case CMR_SENSOR_ERR_OUT_OF_RANGE:
                warn |= sensor->warnFlag;
                error |= sensor->errorFlag;
                break;
            default:
                break;
        }
    }

    if (warnp != NULL) {
        *warnp |= warn;
    }

    if (errorp != NULL) {
        *errorp |= error;
    }
}

/**
 * @brief Gets the most recent converted value for the given channel.
 *
 * @param list The list to use.
 * @param channel The channel in the list (i.e., the index of the sensor in the
 * original array passed to `cmr_sensorListInit()`).
 *
 * @return The most recent converted value.
 */
int32_t cmr_sensorListGetValue(
    cmr_sensorList_t *list, size_t channel
) {
    return list->sensors[channel]._.value;
}

/**
 * @brief Gets the most recent error for the given channel.
 *
 * @param list The list to use.
 * @param channel The channel in the list (i.e., the index of the sensor in the
 * original array passed to `cmr_sensorListInit()`).
 *
 * @return The most recent error.
 */
cmr_sensorError_t cmr_sensorListGetError(
    cmr_sensorList_t *list, size_t channel
) {
    return list->sensors[channel]._.error;
}

