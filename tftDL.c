/**
 * @file tftDL.c
 * @brief TFT display list implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "tftDL.h"  // Interface to implement.

static uint32_t tftDLStartupData[] = {
    0xFFFFFF00,     // Start new display list
    0x02C1272D,     // Set clear color to #C1272D
    0x26000007,     // Clear using color
    0x04FFFFFF,     // Set text color to #FFFFFF
    0xFFFFFF0C,     // Draw text
    (120 << 16) | (160),    // Set Y and X to center of screen
    (30) | (((1 << 9) | (1 << 10)) << 16),    // Set font #30, center horizontal and vertical
    ('C') | ('M' << 8) | ('R' << 16) | ('\0' << 24),
    0x00000000,     // End display list
    0xFFFFFF01      // Swap display list
};

static uint32_t tftDLRTDData[] = {
#include "ESE/RTD.rawh"
};

const tftDL_t tftDLStartup = {
    .len = sizeof(tftDLStartupData),
    .data = tftDLStartupData
};

const tftDL_t tftDLRTD = {
    .len = sizeof(tftDLRTDData),
    .data = tftDLRTDData
};

