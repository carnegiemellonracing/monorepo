/**
 * @file 26x_sensors.h
 * @brief Sensor abstraction layer: routes CAN parsing + state access to Movella or Sensoric.
 *
 * Usage:
 *  - Call sensors_init(...) once at boot OR rely on compile-time default.
 *  - In CAN RX callback: call sensors_parse(canID, payload).
 *  - otherwise, call sensors_get_*() to read whichever backend is active.
 */

#ifndef CMR_26X_SENSORS_H
#define CMR_26X_SENSORS_H

#include <stdint.h>
#include <stdbool.h>

#include "movella.h"
#include "sensoric.h"

// TODO: check units


typedef enum {
    SENSORS_SRC_NONE = 0,
    SENSORS_SRC_MOVELLA,
    SENSORS_SRC_SENSORIC,
} sensors_source_t;

/**
 * Compile-time default selection:
 *   - Define CMR_SENSORS_SOURCE to one of SENSORS_SRC_MOVELLA / SENSORS_SRC_SENSORIC
 *   - If not defined, defaults to MOVELLA.
 *
 * use it like
 *   -DCMR_SENSORS_SOURCE=SENSORS_SRC_SENSORIC
 */
#ifndef CMR_SENSORS_SOURCE
#define CMR_SENSORS_SOURCE SENSORS_SRC_SENSORIC
#endif

/**
 * init, if we're none then do nothing
 */
void sensors_init(sensors_source_t src);

/** Runtime override of active sensor source. */
void sensors_set_source(sensors_source_t src);

/** Returns current active sensor source. */
sensors_source_t sensors_get_source(void);

/** call in place of movella_parse / sensoric_parse */
void sensors_parse(uint16_t canID, volatile void *payload);

/**
 * "unified" get functions. These return NULL if that backend isn't active.
 */
const volatile movella_state_t *sensors_get_movella_state(void);
const volatile car_state_t     *sensors_get_car_state(void);
const volatile sensoric_state_t *sensors_get_sensoric_state(void);

/**
 * These return false if unavailable for the active backend.
 *
 * Sensoric scaling/units are not applied here TODO: FIX THIS SHIT
 *       This exposes raw-int values cast to float for Sensoric, and true float values for Movella.
 */
bool sensors_get_gyro_xyz(float *gx, float *gy, float *gz);
bool sensors_get_accel_xyz(float *ax, float *ay, float *az);
bool sensors_get_vel_xy(float *vx, float *vy);


#endif // CMR_26X_SENSORS_H