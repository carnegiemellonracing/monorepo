/**
 * @file pilot_table.h
 * @brief Headers for pilot signal-related tables
 *
 * @author Carnegie Mellon Racing
 */

#ifndef PILOT_TABLE_H
#define PILOT_TABLE_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t voltage;     // in mV
    uint32_t dutyCycle;   // as a percentage
} pilotDutyCycleConversion_t;

extern const pilotDutyCycleConversion_t dutyCycleVoltages[];

extern const size_t dutyCycleVoltages_len;

#endif
