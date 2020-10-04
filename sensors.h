/*
 * sensors.h
 *
 *  Created on: Jun 15, 2020
 *      Author: vamsi
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#include <stdbool.h>

#include <CMR/sensors.h>
#include "adc.h"

/** @brief Array indexes for sensor value calibration array. */
typedef enum {
	SENSOR_CH_V24V       = 0,     
	SENSOR_CH_HV_PLUS    = 1,
	SENSOR_CH_HV_MINUS   = 2,
	SENSOR_CH_BATT_PLUS  = 3,
    SENSOR_CH_BATT_MINUS = 4,
	SENSOR_CH_SHUNT_LV   = 5,
	SENSOR_CH_AIR_POWER  = 6,
	SENSOR_CH_DCDC_CURR  = 7,
	SENSOR_CH_LV_CURR    = 8,
	SENSOR_CH_LEN     /**< @brief Total ADC channels. */
} sensorChannel_t;

extern cmr_sensorList_t sensorList;

void sensorsInit(void);

#endif /* SENSORS_H_ */
