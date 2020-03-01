/**
 * @file config.h
 * @brief Manages settings across reset
 *
 * @author Carnegie Mellon Racing
 */

#ifndef _CFG_H_
#define _CFG_H_

#include "parser.h"         /* MAX_SIGNALS */
#include <arm_neon.h>       /* float32 */

/**
 * @brief Valid cutoff frequencies at which to downsample.
 */
enum signal_sample_freq {
    SAMPLE_0HZ      = 0,    /**< @brief X Hz cutoff */
    SAMPLE_1HZ,             /**< @brief X Hz cutoff */
    SAMPLE_5HZ,             /**< @brief X Hz cutoff */
    SAMPLE_10HZ,            /**< @brief X Hz cutoff */
    SAMPLE_50HZ,            /**< @brief X Hz cutoff */
    SAMPLE_100HZ,           /**< @brief X Hz cutoff */
    SAMPLE_NUM_FREQS        /**< @brief X Hz cutoff */
};

/**
 * @brief Per-signal configuration, indexed by ID in the JSON configuration
 * vector.
 */
struct signal_cfg {
    uint8_t sample_cutoff_freq;     /**< @brief Enum of type signal_sample_freq */
    float32_t conversion_scale;     /**< @brief Multiply by this, then */
    float32_t conversion_bias;      /**< @brief Add this to get converted
                                     * sample value. */
};

/** @brief List of exposed settings stored in flash */
typedef struct {
    /**
     * @brief Per-signal configuration, indexed by ID in the JSON configuration
     * vector.
     */
    struct signal_cfg signal_cfg[MAX_SIGNALS];
    uint32_t mcu_serial[3];         /**<@brief STM32 mcu's have a unique 96 byte ID */
    uint8_t  canary[12];            /**<@brief Some value to distinguish first-time boots */
} cfg_settings_t;

void configInit(void);
void commit_settings(void);
void set_default_settings(void);

/** @brief (externally visible) mcache of current settings
 * @note Updates to this data are not concurrent-access safe by default
 */
extern cfg_settings_t current_settings;

#endif /* _CFG_H_ */
