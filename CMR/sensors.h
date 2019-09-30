/**
 * @file sensors.h
 * @brief Sensor reading and conversion interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_SENSORS_H
#define CMR_SENSORS_H

#include <stdint.h>     // int32_t, uint32_t
#include <stddef.h>     // size_t

#include "can_types.h"  // cmr_canWarn_t, cmr_canError_t

/** @brief Sensor errors. */
typedef enum {
    CMR_SENSOR_ERR_NONE = 0,        /**< @brief No errors. */
    CMR_SENSOR_ERR_OUT_OF_RANGE,    /**< @brief Sensor out of range. */
    CMR_SENSOR_ERR_LEN              /**< @brief Number of sensor errors. */
} cmr_sensorError_t;

/** @brief Represents a sensor. */
typedef struct cmr_sensor cmr_sensor_t;

/**
 * @brief Callback for getting a raw sensor data sample.
 *
 * @param sensor The sensor being sampled.
 *
 * @return The raw sample.
 */
typedef uint32_t (*cmr_sensorSampleFn_t) (const cmr_sensor_t *sensor);

/**
 * @brief Callback for converting raw sensor readings to normalized values
 *
 * @param sensor The sensor being converted.
 * @param reading The raw sensor reading to convert.
 *
 * @return The converted value.
 */
typedef int32_t (*cmr_sensorConvFn_t) (const cmr_sensor_t *sensor, uint32_t reading);

/**
 * @brief Represents a sensor
 */
struct cmr_sensor {
    const cmr_sensorSampleFn_t sample;  /**< @brief Sample callback. */
    const cmr_sensorConvFn_t conv;      /**< @brief Conversion callback, or NULL for the identity function. */

    const uint32_t readingMin;          /**< @brief Minimum expected sensor reading. */
    const uint32_t readingMax;          /**< @brief Maximum expected sensor reading. Must be at least `readingMin`. */
    const uint32_t outOfRange_pcnt;     /**< @brief Out-of-range threshold, in percent. */

    const cmr_canWarn_t warnFlag;       /**< @brief Heartbeat warning flag to set when out of range. */
    const cmr_canError_t errorFlag;     /**< @brief Heartbeat error flag to set when out of range. */

    /**
     * @brief Private fields.
     *
     * This struct is opaque to the library consumer.
     */
    struct cmr_sensor_private {
        uint32_t readingUpper;      /**< @brief Upper reading threshold. */
        uint32_t readingLower;      /**< @brief Lower reading threshold. */

        volatile int32_t value;             /**< @brief Current value in proper units. */
        volatile cmr_sensorError_t error;   /**< @brief Sensor error status. */
    } _;                                /**< @brief Private fields; this struct is opaque to the library consumer. */
};

/**
 * @brief Represents a collection of sensors.
 *
 * @note The contents of this struct are opaque to the library consumer.
 */
typedef struct {
    cmr_sensor_t *sensors;  /**< @brief Array of sensors. */
    size_t sensorsLen;      /**< @brief Total number of sensors. */
} cmr_sensorList_t;

void cmr_sensorListInit(
    cmr_sensorList_t *list, cmr_sensor_t *sensors, size_t sensorsLen
);

void cmr_sensorListUpdate(cmr_sensorList_t *list);
void cmr_sensorListGetFlags(
    cmr_sensorList_t *list,
    cmr_canWarn_t *warnp, cmr_canError_t *errorp
);

int32_t cmr_sensorListGetValue(
    cmr_sensorList_t *list, size_t channel
);
cmr_sensorError_t cmr_sensorListGetError(
    cmr_sensorList_t *list, size_t channel
);

#endif /* CMR_SENSORS_H */

