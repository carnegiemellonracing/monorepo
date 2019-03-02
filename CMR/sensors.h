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

typedef enum {
    SENSOR_ERR_NONE = 0,
    SENSOR_ERR_OUT_OF_RANGE,
    SENSOR_ERR_LEN
} cmr_sensorError_t;

typedef struct cmr_sensor cmr_sensor_t;

/** 
 * @brief Function pointer typedef for converting sensor's raw ADC
 * value to real value.
 */
typedef int32_t (*cmr_adcToValue_t) (cmr_sensor_t *);

struct cmr_sensor {
    const cmr_adcToValue_t adcToValue; /**< @brief ADC to value conversion function pointer. */
    uint32_t minADC;                   /**< @brief Minimum expected ADC value. */
    uint32_t maxADC;                   /**< @brief Maximum expected ADC value. */
    uint32_t warnThres_pcnt;           /**< @brief Out-of-range error threshold percentage. */
    cmr_canWarn_t flag;                /**< @brief Heartbeat flag to set when out of range. */
    volatile cmr_sensorError_t error;  /**< @brief Sensor error status. */
    volatile int32_t value;            /**< @brief Current value in proper units. */
};

#endif /* CMR_SENSORS_H */
