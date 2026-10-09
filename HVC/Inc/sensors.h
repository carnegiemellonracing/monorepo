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
	SENSOR_CH_AIR_POWER      = 0, /** @brief Voltage on AIR coil */
	SENSOR_CH_SAFETY         = 1, /** @brief Safety Circuit Input Voltage */
	SENSOR_CH_VSENSE         = 2, /** @brief TS Voltage */
	SENSOR_CH_ISENSE         = 3, /** @brief TS Current */ 
    SENSOR_CH_VREF           = 4,  /**< @brief Hall Effect Reference Voltage */ 
	SENSOR_CH_HALL_EFFECT_A  = 5,  /**< @brief Hall effect sensor for accumulator current. */
    SENSOR_CH_BPRES_PSI      = 6,       /**< @brief Rear brake pressure sensor. */
	SENSOR_CH_LEN     /**< @brief Total ADC channels. */
} sensorChannel_t;

extern cmr_sensorList_t sensorList;

void sensorsInit(void);
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
