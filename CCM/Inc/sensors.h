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
    SENSOR_CH_PILOT_DUTY,           /**< @brief Pilot signal duty cycle */
    SENSOR_CH_PILOT_VOLTAGE,        /**< @brief Pilot signal voltage */
    SENSOR_CH_THERM_1,
    SENSOR_CH_THERM_2,
    SENSOR_CH_SAFETY,
    SENSOR_CH_LEN                   /**< @brief Total number of sensors. */
} sensorChannel_t;

extern cmr_sensorList_t sensorList;

void sensorsInit(void);

#endif /* SENSORS_H */
