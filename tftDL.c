/**
 * @file tftDL.c
 * @brief TFT display list implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "tftDL.h"          // Interface to implement
#include "tftContent.h"     // Content interface

/** @brief Represents a display list. */
struct tftDL {
    size_t len;             /**< @brief Length of the display list, in bytes. */
    const uint32_t *data;   /**< @brief The display list data. */

    size_t contentLen;              /**< @brief Number of content items. */
    const tftContent_t **content;   /**< @brief Associated content. */
};

static uint32_t tftDLStartupData[] = {
#include "ESE/startup.rawh"
};

static const tftContent_t *tftDLStartupContent[] = {
    &tftContent_startup_lut,
    &tftContent_startup
};

const tftDL_t tftDLStartup = {
    .len = sizeof(tftDLStartupData),
    .data = tftDLStartupData,

    .contentLen = sizeof(tftDLStartupContent) / sizeof(tftDLStartupContent[0]),
    .content = tftDLStartupContent
};

static uint32_t tftDLRTDData[] = {
#include "ESE/RTD.rawh"
};

static const tftContent_t *tftDLRTDContent[] = {
    &tftContent_RobotoMono_Bold_72_L4,
    &tftContent_RobotoMono_Bold_40_L4,
};

const tftDL_t tftDLRTD = {
    .len = sizeof(tftDLRTDData),
    .data = tftDLRTDData,

    .contentLen = sizeof(tftDLRTDContent) / sizeof(tftDLRTDContent[0]),
    .content = tftDLRTDContent
};

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

