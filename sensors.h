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
    SENSOR_CH_LOAD_VOLTAGE_MV = 0,
    SENSOR_CH_LOAD_CURRENT_MA,
    SENSOR_CH_LOGIC_VOLTAGE_MV,
    SENSOR_CH_BOARD_THERM_1,
    SENSOR_CH_BOARD_THERM_2,
    SENSOR_CH_THERM_1,
    SENSOR_CH_THERM_2,
    SENSOR_CH_THERM_3,
    SENSOR_CH_THERM_4,
    SENSOR_CH_THERM_5,
    SENSOR_CH_THERM_6,
    SENSOR_CH_THERM_7,
    SENSOR_CH_THERM_8,
    SENSOR_CH_THERM_9,
    SENSOR_CH_LEN
} sensorChannel_t;

/** @brief Exported sensorlist for interface consumers.
 * Populated automatically. */
extern cmr_sensorList_t sensorList;

void sensorsInit(void);

#endif /* SENSORS_H */

