/**
 * @file config.c
 * @brief Manages settings across reset
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface

#include <FreeRTOS.h>   // FreeRTOS interface
#include <semphr.h>     // Semaphore interface

#include "CMR/config.h"     // Board-specific flash interface
#include "CMR/can_types.h"  // default settings
#include "CMR/panic.h"      // bad things
#include <string.h>         // memcpy

#include "config.h"  // interface to implement
#include "can.h"     // default settings

/**@brief 12-byte unique product ID */
#define UID_PTR ((uint32_t *) UID_BASE)
/**@brief Some string to distinguish first-time-boots */
#define CANARY "\xbd\xa6\x27\x59\xe2\xcf\x25\x88\x95\x24\xed\xec"

/**@brief flash driver wrapping the settings */
static cmr_config_t cfg;

/**@brief Index of the page to use
 * @note  On a target with 128KB of flash, this is the last page. */
static const uint32_t settings_page_id = 0x3F;

/**@brief mcache of current settings
 *
 * @note Updates to this alias into the flash driver's cache automatically
 *
 * @note no effort is made to make this concurrency-safe (here)
 */
cfg_settings_t current_settings;

/**
 * @brief Set the default settings during
 * reset or first-time-boot.
 *
 */
void set_default_settings(void) {
    cfg_settings_t defaults = {
        .mcu_serial = {
            UID_PTR[0],
            UID_PTR[1],
            UID_PTR[2],
        },
        .canary = CANARY,
    };

    current_settings = defaults;
    commit_settings();
}

/**
 * @brief Check current_settings for any out-of-ranges.
 *
 * @note Panics on failure.
 *
 */
static void validate_settings() {
}

/**
 * @brief Commit current settings to flash after validating
 *
 * @note Copies the (unexposed) canary in to make sure
 * we don't lose track of it over resets
 */
void commit_settings(void) {
    memcpy(current_settings.canary, CANARY, sizeof(current_settings.canary));
    validate_settings();
    cmr_configCommit(&cfg);
}

/**
 * @brief Initialize flash cache, pull in current settings,
 * and perform first-boot-time initialization.
 *
 */
void configInit(void) {
    cmr_configInit(
        &cfg,
        (volatile uint32_t *) &current_settings, sizeof(current_settings),
        settings_page_id
    );

    /* Do a vaguely sketchy uninitialized flash check to
     * set defaults on the first boot */
    if (memcmp(current_settings.canary, CANARY, sizeof(current_settings.mcu_serial))) {
        set_default_settings();
    }
}
