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
    SENSOR_CH_TPOS_L_U8 = 0,                        /**< @brief Left throttle position sensor. */
    SENSOR_CH_TPOS_R_U8,                            /**< @brief Right throttle position sensor. */
    SENSOR_CH_BPOS_U8,                              /**< @brief Brake pedal position sensor. */
    SENSOR_CH_BPRES_PSI,                            /**< @brief Brake pressure sensor. */
    SENSOR_CH_BPRES_IMPLUS,                         /**< @brief Checks if the brake value is implausible */
    SENSOR_CH_SWANGLE_DEG_FL,                       /**< @brief Steering angle sensor left side */
    SENSOR_CH_SWANGLE_DEG_FR,                       /**< @brief Steering angle sensor right side */
    SENSOR_CH_TPOS_IMPLAUS,                         /**< @brief Throttle implausibility. */
    SENSOR_CH_BPP_IMPLAUS,                          /**< @brief Brake pedal position implausibility. */
    SENSOR_CH_EBS_PRESSURE_1_DECI_BAR,              /**< @brief Tank Pressure 1 for DV System */
    SENSOR_CH_EBS_PRESSURE_2_DECI_BAR,              /**< @brief Tank Pressure 2 for DV System */
    SENSOR_CH_EBS_PRESSURE_1_DECI_BAR_NON_DV_CHECK, /**< @brief Tank Pressure 1 for DV System. Used for non dv error check */
    SENSOR_CH_EBS_PRESSURE_2_DECI_BAR_NON_DV_CHECK, /**< @brief Tank Pressure 2 for DV System. Used for non dv error check */
    SENSOR_CH_EBS_CURRENT_1_MA,                     /**< @brief Solenoid Current 1 for DV System */
    SENSOR_CH_EBS_CURRENT_2_MA,                     /**< @brief Solenoid Current 2 for DV System */
    SENSOR_CH_LEN                                   /**< @brief Number of sensors. */
} sensorChannel_t;

extern cmr_sensorList_t sensorList;

extern const adcChannel_t sensorsADCChannels[SENSOR_CH_LEN];


void sensorsInit(void);
uint8_t throttleGetPos(void);

#endif /* SENSORS_H */
