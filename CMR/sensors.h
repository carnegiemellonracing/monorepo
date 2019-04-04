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

typedef enum {
    SENSOR_ERR_NONE = 0,
    SENSOR_ERR_OUT_OF_RANGE,
    SENSOR_ERR_LEN
} cmr_sensorError_t;

typedef struct cmr_sensor cmr_sensor_t;

/** 
 * @brief Function pointer typedef for converting sensor's raw data
 * value to a real value.
 */
typedef int32_t (*cmr_readingToValue_t) (cmr_sensor_t *, int32_t);
/**
 * @brief Function pointer typedef for getting a raw reading from a sensor.
 */
typedef int32_t (*cmr_sampleSensor_t) (cmr_sensor_t *);

/**
 * @brief Function pointer typedef for initializing a sensor.
 */
typedef void (*cmr_initSensor_t) (cmr_sensor_t *);

struct cmr_sensor {
    const cmr_readingToValue_t readingToValue; /**< @brief Function pointer for sensor reading to value. */
    const cmr_initSensor_t sensorInit; 	       /**< @brief Function pointer for initializing sensor. */
    const cmr_sampleSensor_t sampleSensor;     /**< @brief Function pointer for Sensor reading to value conversion . */

    bool isInitialized;	                       /**< @brief Whether the sensor is initialized or not. */

    uint32_t minReading;                       /**< @brief Minimum expected sensor reading. */
    uint32_t maxReading;                       /**< @brief Maximum expected sensor readinge. */
    uint32_t warnThres_pcnt;                   /**< @brief Out-of-range error threshold percentage. */
    cmr_canWarn_t flag;                        /**< @brief Heartbeat flag to set when out of range. */

    volatile cmr_sensorError_t error;          /**< @brief Sensor error status. */
    volatile int32_t value;                    /**< @brief Current value in proper units. */
};

void cmr_getSensorValue(cmr_sensor_t *);
void cmr_initializeSensor(cmr_sensor_t *);

#endif /* CMR_SENSORS_H */
