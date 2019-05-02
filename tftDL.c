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
    int32_t power_kW,
    int32_t dcdcTemp,
    int32_t motorTemp,
    int32_t acTemp,
    int32_t mcTemp
) {
    static struct {
        char buf[3];
    } *const speed_mph_str = (void *) (tftDL_RTDData + 72);

    static struct {
        char buf[4];
    } *const hvVoltage_str = (void *) (tftDL_RTDData + 91);

    static struct {
        char buf[3];
    } *const dcdcTemp_str = (void *) (tftDL_RTDData + 125);

    static struct {
        char buf[3];
    } *const motorTemp_str = (void *) (tftDL_RTDData + 130);

    static struct {
        char buf[3];
    } *const acTemp_str = (void *) (tftDL_RTDData + 120);

    static struct {
        char buf[3];
    } *const mcTemp_str = (void *) (tftDL_RTDData + 135);

    static const tftDL_bar_t hvVoltage_bar = {
        .addr = tftDL_RTDData + 85,
        .topY = 12,
        .botY = 168,
        .maxVal = 400000,
        .minVal = 270000
    };

    static struct {
        char buf[3];
    } *const power_kW_str = (void *) (tftDL_RTDData + 115);

    static const tftDL_bar_t power_kW_bar = {
        .addr = tftDL_RTDData + 109,
        .topY = 12,
        .botY = 168,
        .maxVal = 85,
        .minVal = 0
    };

    snprintf(
        speed_mph_str->buf, sizeof(speed_mph_str->buf),
        "%2lu", speed_mph
    );

    snprintf(
        hvVoltage_str->buf, sizeof(hvVoltage_str->buf),
        "%3ld", hvVoltage / 1000
    );

    snprintf(
        dcdcTemp_str->buf, sizeof(dcdcTemp_str->buf),
        "%2ld", dcdcTemp
    );

    snprintf(
        motorTemp_str->buf, sizeof(motorTemp_str->buf),
        "%2ld", motorTemp
    );

    snprintf(
        acTemp_str->buf, sizeof(acTemp_str->buf),
        "%2ld", acTemp
    );

    snprintf(
        mcTemp_str->buf, sizeof(mcTemp_str->buf),
        "%2ld", mcTemp
    );

    snprintf(
        power_kW_str->buf, sizeof(power_kW_str->buf),
        "%2ld", power_kW
    );

    tftDL_barSetY(&hvVoltage_bar, hvVoltage);

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

