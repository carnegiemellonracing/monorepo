/**
 * @file sensors.h
 * @brief Sensor reading interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_SENSORS_H
#define CMR_SENSORS_H

#include <stdint.h>
#include <CMR/can_types.h>
#include <stdbool.h>

/** @brief Sensor errors. */
typedef enum {
    SENSOR_ERR_NONE = 0,        /**< @brief No errors. */
    SENSOR_ERR_OUT_OF_RANGE,    /**< @brief Sensor out of range. */
    SENSOR_ERR_LEN              /**< @brief Number of sensor errors. */
} cmr_sensorError_t;

/** @brief Represents a sensor. */
typedef struct cmr_sensor cmr_sensor_t;

/**
 * @brief Function pointer typedef for converting sensor's raw data
 * value to a real value.
 */
typedef int32_t (*cmr_readingToValue_t) (cmr_sensor_t *, uint32_t);

/**
 * @brief Function pointer typedef for getting a raw reading from a sensor.
 */
typedef uint32_t (*cmr_sampleSensor_t) (cmr_sensor_t *);

/**
 * @brief Function pointer typedef for initializing a sensor.
 */
typedef void (*cmr_initSensor_t) (cmr_sensor_t *);

struct cmr_sensor {
    const cmr_readingToValue_t readingToValue; /**< @brief Function pointer for sensor reading to value. */
    const cmr_sampleSensor_t sampleSensor;     /**< @brief Function pointer for Sensor reading to value conversion . */

    uint32_t minReading;                       /**< @brief Minimum expected sensor reading. */
    uint32_t maxReading;                       /**< @brief Maximum expected sensor reading. */
    uint32_t warnThres_pcnt;                   /**< @brief Out-of-range error threshold percentage. */
    cmr_canWarn_t warnFlag;                    /**< @brief Heartbeat warning flag to set when out of range. */
    cmr_canError_t errorFlag;                  /**< @brief Heartbeat error flag to set when out of range. */

    volatile cmr_sensorError_t error;          /**< @brief Sensor error status. */
    volatile int32_t value;                    /**< @brief Current value in proper units. */
};

void cmr_getSensorValue(cmr_sensor_t *);

#endif /* CMR_SENSORS_H */

