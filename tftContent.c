/**
 * @file tftContent.c
 * @brief TFT content implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "tftContent.h"     // Interface to implement

/** @brief Represents TFT content. */
struct tftContent {
    size_t len;             /**< @brief The content's length, in bytes. */
    size_t addr;            /**< @brief The content's address in `RAM_G`. */
    const uint8_t *data;    /**< @brief The content. */
};

/** @brief Startup image lookup table data. */
static const uint8_t tftContent_startup_lut_data[] = {
#include "ESE/content/startup.lut.binh"
};

/** @brief Startup image lookup table. */
const tftContent_t tftContent_startup_lut = {
    .len = sizeof(tftContent_startup_lut_data),
    .addr = 0,
    .data = tftContent_startup_lut_data
};

/** @brief Startup image data. */
static const uint8_t tftContent_startup_data[] = {
#include "ESE/content/startup.binh"
};

/** @brief Startup image. */
const tftContent_t tftContent_startup = {
    .len = sizeof(tftContent_startup_data),
    .addr = 1024,
    .data = tftContent_startup_data
};

/** @brief Roboto Mono, bold, 72-pt, 4-bit font data. */
static const uint8_t tftContent_RobotoMono_Bold_72_L4_data[] = {
#include "ESE/content/RobotoMono-Bold_72_L4.binh"
};

/** @brief Roboto Mono, bold, 72-pt, 4-bit font. */
const tftContent_t tftContent_RobotoMono_Bold_72_L4 = {
    .len = sizeof(tftContent_RobotoMono_Bold_72_L4_data),
    .addr = 1000,
    .data = tftContent_RobotoMono_Bold_72_L4_data
};

/** @brief Roboto Mono, bold, 40-pt, 4-bit font data. */
static const uint8_t tftContent_RobotoMono_Bold_40_L4_data[] = {
#include "ESE/content/RobotoMono-Bold_40_L4.binh"
};

/** @brief Roboto Mono, bold, 40-pt, 4-bit font. */
const tftContent_t tftContent_RobotoMono_Bold_40_L4 = {
    .len = sizeof(tftContent_RobotoMono_Bold_40_L4_data),
    .addr = 45192,
    .data = tftContent_RobotoMono_Bold_40_L4_data
};

/**
 * @brief Loads content in graphics memory.
 *
 * @param tft The display.
 * @param tftContent The content.
 */
void tftContentLoad(tft_t *tft, const tftContent_t *tftContent) {
    // Set up inflate coprocessor command.
    uint32_t coCmdInflate[] = {
        0xFFFFFF22,         // CMD_INFLATE
        tftContent->addr    // Destination address.
    };
    tftCoCmd(tft, sizeof(coCmdInflate), coCmdInflate, false);

    // Write compressed data.
    tftCoCmd(tft, tftContent->len, tftContent->data, true);
}

