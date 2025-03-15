/**
 * @file sensors.h
 * @brief Board-specific sensor interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>

#include <CMR/sensors.h>
#include "adc.h"

/** @brief Array indexes for sensor value calibration array. */
typedef enum {
	SENSOR_CH_VOLTAGE_CV = 0,   /**< @brief High voltage rail. */
	SENSOR_CH_AVG_CURRENT_DA,   /**< @brief HV current draw. */
    SENSOR_CH_LEN               /**< @brief Number of sensors. */
} sensorChannel_t;

extern cmr_sensorList_t sensorList;

extern const adcChannel_t sensorsADCChannels[SENSOR_CH_LEN];

void sensorsInit(void);

int32_t getVoltage(void);

int32_t getCurrent(void);

#endif /* SENSORS_H */

