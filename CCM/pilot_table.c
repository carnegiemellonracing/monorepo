#include "pilot_table.h"

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
