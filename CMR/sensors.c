/**
 * @file sensors.c
 * @brief Sensor abstraction implementation
 *
 * @author Carnegie Mellon Racing
 */

#include "sensors.h"    // Interface to implement
#include <stddef.h>     // NULL

/**
 * @brief Calculates how much a reading can be either below the min reading value or above the
 * max reading value.
 *
 * @param sensor The sensor.
 *
 * @return The amount that a reading can be outside of the normal reading range.
 */
static int32_t cmr_getReadingTolerance(cmr_sensor_t *sensor) {
    int32_t readingRange = sensor->maxReading - sensor->minReading;
    return readingRange / 100 * sensor->warnThres_pcnt;
}

/**
 * @brief Stores a converted sensor value and clears the error code if the value
 * is valid - otherwise the error code is set, but the sensor value is still
 * stored.
 *
 * @param sensor The sensor to get a value for.
 */
void cmr_getSensorValue(cmr_sensor_t *sensor) {
	// get error thresholds
    uint32_t reading = sensor->sampleSensor(sensor);
    uint32_t upperThres = sensor->maxReading + cmr_getReadingTolerance(sensor);
    uint32_t lowerThres = sensor->minReading - cmr_getReadingTolerance(sensor);

	// set error
    if (
        (upperThres >= sensor->maxReading && reading > upperThres) ||
        (lowerThres <= sensor->minReading && reading < lowerThres)
    ) {
        sensor->error = SENSOR_ERR_OUT_OF_RANGE;
    } else {
        sensor->error = SENSOR_ERR_NONE;
    }

	// convert reading to value
    sensor->value = sensor->readingToValue(sensor, reading);
}

