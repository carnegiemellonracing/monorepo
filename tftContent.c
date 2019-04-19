/**
 * @file tftContent.c
 * @brief TFT content implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "tftContent.h"     // Interface to implement

static const uint8_t tftContent_RobotoMono_Bold_72_L4_data[] = {
#include "ESE/content/RobotoMono-Bold_72_L4.rawh"
};

static const uint8_t tftContent_RobotoMono_Bold_40_L4_data[] = {
#include "ESE/content/RobotoMono-Bold_40_L4.rawh"
};

const tftContent_t tftContent_RobotoMono_Bold_72_L4 = {
    .len = sizeof(tftContent_RobotoMono_Bold_72_L4_data),
    .addr = 1000,
    .data = tftContent_RobotoMono_Bold_72_L4_data
};

const tftContent_t tftContent_RobotoMono_Bold_40_L4 = {
    .len = sizeof(tftContent_RobotoMono_Bold_40_L4_data),
    .addr = 45192,
    .data = tftContent_RobotoMono_Bold_40_L4_data
};

