/**
 * @file 26x_sensors.c
 * @brief Implementation of sensor abstraction layer.
 */

#include "../Inc/26x_sensors.h"

static volatile sensors_source_t sensor_src = (sensors_source_t)CMR_SENSORS_SOURCE;

void sensors_init(sensors_source_t src) {
    if (src != SENSORS_SRC_NONE) {
        sensor_src = src;
    }
}

void sensors_set_source(sensors_source_t src) {
    sensor_src = src;
}

sensors_source_t sensors_get_source(void) {
    return (sensors_source_t)sensor_src;
}

void sensors_parse(uint16_t canID, volatile void *payload) {
    switch (sensor_src) {
        case SENSORS_SRC_MOVELLA:
            movella_parse(canID, payload);
            break;

        case SENSORS_SRC_SENSORIC:
            sensoric_parse(canID, payload);
            break;

        case SENSORS_SRC_NONE:
        default:
            // do nothing
            break;
    }
}

const volatile movella_state_t *sensors_get_movella_state(void) {
    return (sensor_src == SENSORS_SRC_MOVELLA) ? &movella_state : (const volatile movella_state_t *)0;
}

const volatile car_state_t *sensors_get_car_state(void) {
    return (sensor_src == SENSORS_SRC_MOVELLA) ? &car_state : (const volatile car_state_t *)0;
}

const volatile sensoric_state_t *sensors_get_sensoric_state(void) {
    return (sensor_src == SENSORS_SRC_SENSORIC) ? &sensoric_state : (const volatile sensoric_state_t *)0;
}

bool sensors_get_gyro_xyz(float *gx, float *gy, float *gz) {
    if (!gx || !gy || !gz) return false;

    if (sensor_src == SENSORS_SRC_MOVELLA) {
        // car_state gyro is already in car frame
        *gx = car_state.gyro.x;
        *gy = car_state.gyro.y;
        *gz = car_state.gyro.z;
        return true;
    }

    if (sensor_src == SENSORS_SRC_SENSORIC) {
        // umm we need to fix units here
        *gx = (float)sensoric_state.rate.x;
        *gy = (float)sensoric_state.rate.y;
        *gz = (float)sensoric_state.rate.z;
        return true;
    }

    return false;
}

bool sensors_get_accel_xyz(float *ax, float *ay, float *az) {
    if (!ax || !ay || !az) return false;

    if (sensor_src == SENSORS_SRC_MOVELLA) {
        *ax = car_state.accel.x;
        *ay = car_state.accel.y;
        *az = car_state.accel.z;
        return true;
    }

    if (sensor_src == SENSORS_SRC_SENSORIC) {
        // umm we need to fix units here
        *ax = (float)sensoric_state.acc.x;
        *ay = (float)sensoric_state.acc.y;
        *az = (float)sensoric_state.acc.z;
        return true;
    }

    return false;
}

bool sensors_get_vel_xy(float *vx, float *vy) {
    if (!vx || !vy) return false;

    // check sensoric case

    if (sensor_src == SENSORS_SRC_MOVELLA) {
        *vx = car_state.velocity.x;
        *vy = car_state.velocity.y;
        return true;
    }


    if (sensor_src == SENSORS_SRC_SENSORIC) {
        // umm we need to fix units here
        // (still raw int16 -> float here)
        *vx = (float)sensoric_state.vel_ang.x;
        *vy = (float)sensoric_state.vel_ang.y;
        return true;
    }

    return false;
}