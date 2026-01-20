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
	SENSOR_CH_V24V           = 0, /** @brief LV Battery voltage */
	SENSOR_CH_AIR_POWER      = 1, /** @brief Voltage on AIR coil */
	SENSOR_CH_SAFETY         = 2, /** @brief Safety Circuit Input Voltage */
	SENSOR_CH_VSENSE         = 3, /** @brief TS Voltage */
	SENSOR_CH_ISENSE         = 4, /** @brief TS Current */ 
    SENSOR_CH_VREF,        /**< @brief Hall Effect Reference Voltage */ 
	SENSOR_CH_LEN     /**< @brief Total ADC channels. */
} sensorChannel_t;

extern cmr_sensorList_t sensorList;

void sensorsInit(void);

int32_t getLVmillivolts();
int32_t getLVmilliamps();
int32_t getAIRmillivolts();
int32_t getSafetymillivolts();
int32_t getHVmillivolts();
int32_t getHVmilliamps();
int32_t getHVIvoltage(); 
int32_t getHVIcurrent(); 
int32_t getHVIvref(); 
int32_t getHVmilliamps_avg(); 


#endif /* SENSORS_H_ */
