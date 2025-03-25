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

const pilotDutyCycleConversion_t dutyCycleVoltages[] = {
    {
            .voltage = 350,
            .dutyCycle = 10
    },
    {
            .voltage = 470,
            .dutyCycle = 16
    },
    {
            .voltage = 790,
            .dutyCycle = 25
    },
    {
            .voltage = 920,
            .dutyCycle = 30
    },
    {
            .voltage = 1190,
            .dutyCycle = 40
    },
    {
            .voltage = 1600,
            .dutyCycle = 50
    },
    {
            .voltage = 1900,
            .dutyCycle = 60
    },
    {
            .voltage = 2270,
            .dutyCycle = 70
    },
    {
            .voltage = 2600,
            .dutyCycle = 80
    },
    {
            .voltage = 2900,
            .dutyCycle = 90
    },
    {
            .voltage = 3100,
            .dutyCycle = 96
    }
};

const size_t dutyCycleVoltages_len = sizeof(dutyCycleVoltages) / sizeof(dutyCycleVoltages[0]);

#endif