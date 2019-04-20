/**
 * @file tftDL.c
 * @brief TFT display list implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <stdio.h>      // snprintf()

#include "tftDL.h"          // Interface to implement
#include "tftContent.h"     // Content interface

/** @brief Represents a display list. */
struct tftDL {
    size_t len;             /**< @brief Length of the display list, in bytes. */
    const uint32_t *data;   /**< @brief The display list data. */

    size_t contentLen;              /**< @brief Number of content items. */
    const tftContent_t **content;   /**< @brief Associated content. */
};

static uint32_t tftDL_startupData[] = {
#include "ESE/startup.rawh"
};

static const tftContent_t *tftDL_startupContent[] = {
    &tftContent_startup_lut,
    &tftContent_startup
};

const tftDL_t tftDL_startup = {
    .len = sizeof(tftDL_startupData),
    .data = tftDL_startupData,

    .contentLen = sizeof(tftDL_startupContent) / sizeof(tftDL_startupContent[0]),
    .content = tftDL_startupContent
};

static uint32_t tftDL_RTDData[] = {
#include "ESE/RTD.rawh"
};

static const tftContent_t *tftDL_RTDContent[] = {
    &tftContent_RobotoMono_Bold_72_L4,
    &tftContent_RobotoMono_Bold_40_L4,
};

/** @brief Ready-to-drive screen. */
const tftDL_t tftDL_RTD = {
    .len = sizeof(tftDL_RTDData),
    .data = tftDL_RTDData,

    .contentLen = sizeof(tftDL_RTDContent) / sizeof(tftDL_RTDContent[0]),
    .content = tftDL_RTDContent
};

typedef struct {
    uint32_t *addr;
    uint8_t topY;
    uint8_t botY;
    int32_t maxVal;
    int32_t minVal;
} tftDL_bar_t;

#define TFT_DL_VERTEX_Y_BIT 12

static void tftDL_barSetY(const tftDL_bar_t *bar, int32_t val) {
    uint8_t y;
    if (val < bar->minVal) {
        y = bar->botY;
    } else if (val > bar->maxVal) {
        y = bar->topY;
    } else {
        int32_t len = (
            (val - bar->minVal) * (int32_t) (bar->botY - bar->topY)
        ) / (bar->maxVal - bar->minVal);
        y = bar->botY - (uint8_t) len;
    }

    uint32_t vertex = *bar->addr;
    vertex &= ~(((1 << 8) - 1) << TFT_DL_VERTEX_Y_BIT);
    vertex |= y << TFT_DL_VERTEX_Y_BIT;
    *bar->addr = vertex;
}

/**
 * @brief Updates the ready-to-drive screen.
 *
 * @param speed_mph Current speed (miles per hour).
 * @param hvVoltage High-voltage bus voltage (Volts).
 * @param power_kW Power (kiloWatts).
 */
void tftDL_RTDUpdate(
    uint32_t speed_mph,
    int32_t hvVoltage,
    int32_t power_kW
) {
    static char *speed_mph_10 = (char *) (tftDL_RTDData + 112);
    static char *speed_mph_1 = (char *) (tftDL_RTDData + 114);

    static char *hvVoltage_100 = (char *) (tftDL_RTDData + 136);
    static char *hvVoltage_10 = (char *) (tftDL_RTDData + 137);
    static char *hvVoltage_1 = (char *) (tftDL_RTDData + 138);

    static const tftDL_bar_t hvVoltage_bar = {
        .addr = tftDL_RTDData + 129,
        .topY = 12,
        .botY = 168,
        .maxVal = 400000,
        .minVal = 270000
    };

    static char *power_kW_10 = (char *) (tftDL_RTDData + 164);
    static char *power_kW_1 = (char *) (tftDL_RTDData + 165);

    static const tftDL_bar_t power_kW_bar = {
        .addr = tftDL_RTDData + 157,
        .topY = 12,
        .botY = 168,
        .maxVal = 85,
        .minVal = 0
    };

    char buf[4];
    snprintf(buf, sizeof(buf), "%2lu", speed_mph);
    *speed_mph_10 = buf[0];
    *speed_mph_1 = buf[1];

    snprintf(buf, sizeof(buf), "%3ld", hvVoltage / 1000);
    *hvVoltage_100 = buf[0];
    *hvVoltage_10 = buf[1];
    *hvVoltage_1 = buf[2];
    tftDL_barSetY(&hvVoltage_bar, hvVoltage);

    snprintf(buf, sizeof(buf), "%2ld", power_kW);
    *power_kW_10 = buf[0];
    *power_kW_1 = buf[1];
    tftDL_barSetY(&power_kW_bar, power_kW);
}

/**
 * @brief Loads the display list's content into graphics memory.
 *
 * @param tft The display.
 * @param tftDL The display list.
 */
void tftDLContentLoad(tft_t *tft, const tftDL_t *tftDL) {
    for (size_t i = 0; i < tftDL->contentLen; i++) {
        const tftContent_t *tftContent = tftDL->content[i];
        tftContentLoad(tft, tftContent);
    }
}

/**
 * @brief Writes a display list.
 *
 * @param tft The display.
 * @param tftDL The display list.
 */
void tftDLWrite(tft_t *tft, const tftDL_t *tftDL) {
    tftCoCmd(tft, tftDL->len, tftDL->data, true);
}

