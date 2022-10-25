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
    SENSOR_CH_TPOS_L_U8 = 0,    /**< @brief Left throttle position sensor. */
    SENSOR_CH_TPOS_R_U8,        /**< @brief Right throttle position sensor. */
    SENSOR_CH_BPOS_U8,          /**< @brief Brake pedal position sensor. */
    SENSOR_CH_BPRES_PSI,        /**< @brief Brake pressure sensor. */
    SENSOR_CH_SWANGLE_DEG,      /**< @brief Steering angle sensor. */ 
    SENSOR_CH_VOLTAGE_MV,       /**< @brief Board voltage rail. */ 
    SENSOR_CH_AVG_CURRENT_MA,   /**< @brief Board average current draw. */ 
    SENSOR_CH_TPOS_IMPLAUS,     /**< @brief Throttle implausibility. */ 
    SENSOR_CH_BPP_IMPLAUS,      /**< @brief Brake pedal position implausibility. */ 
    SENSOR_CH_SS_MODULE,        /**< @brief Safety circuit module */
    SENSOR_CH_SS_COCKPIT,       /**< @brief Safety circuit cockpit */
    SENSOR_CH_SS_FRHUB,         /**< @brief Safety circuit FR hub */
    SENSOR_CH_SS_INERTIA,       /**< @brief Safety circuit inertia */
    SENSOR_CH_SS_FLHUB,         /**< @brief Safety circuit FL hub */
    SENSOR_CH_SS_BOTS,          /**< @brief Safety circuit bots */
    SENSOR_CH_LEN               /**< @brief Number of sensors. */
} sensorChannel_t;

extern cmr_sensorList_t sensorList;

extern const adcChannel_t sensorsADCChannels[SENSOR_CH_LEN];

void sensorsInit(void);
uint8_t throttleGetPos(void);

#endif /* SENSORS_H */

