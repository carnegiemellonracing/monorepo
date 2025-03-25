/**
 * @file evse.h
 * @brief EVSE interface
 *
 * @author Carnegie Mellon Racing
 */

#ifndef EVSE_H
#define EVSE_H

#include <stdint.h>

typedef enum {
    EVSE_ERROR,
    EVSE_OFF,
    EVSE_READY,
    EVSE_VEHICLE_DETECTED,
    EVSE_STANDBY,
    EVSE_LEN
} evseState_t;

evseState_t getEvseState(int32_t voltage);
uint32_t getEvseCurrentLimit(int32_t dutyCycle);

#endif
