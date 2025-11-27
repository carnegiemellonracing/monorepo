/**
 * @file sensors.h
 * @brief Board-specific sensor interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <CMR/sensors.h>
#include <stdbool.h>

#include "adc.h"

/** @brief Array indexes for sensor value calibration array. */
typedef enum {
    SENSOR_CH_HV,  /**< @brief Voltage */
    SENSOR_CH_CURRENT,      /**< @brief Current */
    SENSOR_CH_VREF,        /**< @brief VREF */
    SENSOR_CH_LEN,
} sensorChannel_t;

extern cmr_sensorList_t sensorList;

extern const adcChannels_t sensorsADCChannels[SENSOR_CH_LEN];

void sensorsInit(void);

#endif /* SENSORS_H */
