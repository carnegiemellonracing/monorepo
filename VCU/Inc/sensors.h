/**
 * @file sensors.h
 * @brief Board-specific sensor interface.
 *
 * @author Carnegie Mellon Racing
 */

#pragma once

#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>

#include <CMR/sensors.h>
#include "adc.h"

//DCM Values
/** @brief Array indexes for sensor value calibration array. */
typedef enum {
	SENSOR_CH_VOLTAGE_CV = 0,   /**< @brief High voltage rail. */
	SENSOR_CH_AVG_CURRENT_DA,   /**< @brief HV current draw. */
    SENSOR_CH_LEN               /**< @brief Number of sensors. */
} sensorChannel_t;

//VSM Values
typedef enum {
    SENSOR_CH_HALL_EFFECT_A = 0,    /**< @brief Hall effect sensor for accumulator current. */
    SENSOR_CH_BPRES_PSI,            /**< @brief Rear brake pressure sensor. */
    SENSOR_CH_VOLTAGE_MV,           /**< @brief Board voltage sense. */
    SENSOR_CH_CURRENT_MA,           /**< @brief Board current sense. */
    SENSOR_CH_SS_IN,                /**< @brief Safety Circuit voltage before latches. */
    SENSOR_CH_SS_OUT,               /**< @brief SS voltage after latches. */
    SENSOR_CH_LEN                   /**< @brief Total number of sensors. */
} sensorChannel_t;

extern cmr_sensorList_t sensorList;

extern const adcChannel_t sensorsADCChannels[SENSOR_CH_LEN];

void sensorsInit(void);

int32_t getVoltage_mV(void);

int32_t getCurrent_mA(void);

#endif /* SENSORS_H */