/**
 * @file sensors.h
 * @brief Board-specific sensor interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <CMR/sensors.h>

extern const uint32_t TPOS_IMPLAUS_THRES_MS;

/** @brief Array indexes for sensor value calibration array. */
typedef enum {
    SENSOR_CH_LOGIC_VOLTAGE_MV = 0,
    SENSOR_CH_LOGIC_CURRENT_MA,
    SENSOR_CH_LOAD_VOLTAGE_MV,
    SENSOR_CH_LOAD_CURRENT_MA,
    SENSOR_CH_LEN
} sensorChannel_t;

extern cmr_sensor_t sensors[SENSOR_CH_LEN];

void sensorInit(void);

#endif /* SENSORS_H */
