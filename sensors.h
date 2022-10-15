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
	SENSOR_CH_V24V           = 0,
	SENSOR_CH_AIR_POWER      = 1,
	SENSOR_CH_SAFETY         = 2,
	SENSOR_CH_IBATT_FILTERED = 3,
	SENSOR_CH_VSENSE         = 4,
	SENSOR_CH_ISENSE         = 5,
	SENSOR_CH_LEN     /**< @brief Total ADC channels. */
} sensorChannel_t;

extern cmr_sensorList_t sensorList;

void sensorsInit(void);

int32_t getLVmillivolts();
int32_t getLVmilliamps();
int32_t getAIRmillivolts();
int32_t getHVmillivolts();
int32_t getHVmilliamps();

#endif /* SENSORS_H_ */
