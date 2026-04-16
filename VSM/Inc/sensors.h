/**
 * @file sensors.h
 * @brief Board-specific sensor interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <CMR/sensors.h>

/** @brief Array indexes for sensor value calibration array. */
typedef enum {
    SENSOR_CH_HALL_EFFECT_CA = 0,   /**< @brief Hall effect sensor for accumulator current. */
    SENSOR_CH_BPRES_PSI,            /**< @brief Rear brake pressure sensor. */
    SENSOR_CH_VOLTAGE_MV,           /**< @brief Board voltage sense. */
    SENSOR_CH_CURRENT_MA,           /**< @brief Board current sense. */
    SENSOR_CH_SS_IN,                /**< @brief Safety Circuit voltage before latches. */
    SENSOR_CH_SS_OUT,               /**< @brief SS voltage after latches. */
    SENSOR_CH_LEN                   /**< @brief Total number of sensors. */
} sensorChannel_t;

extern cmr_sensorList_t sensorList;

void sensorsInit(void);

#endif /* SENSORS_H */
