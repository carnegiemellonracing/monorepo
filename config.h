/**
 * @file config.h
 * @brief Manages settings across reset
 *
 * @author Carnegie Mellon Racing
 */

#ifndef _CFG_H_
#define _CFG_H_

#include "can.h"     // Board-specific can interface

/** @brief List of exposed settings stored in flash */
typedef struct {
    uint32_t mcu_serial[3];         /**<@brief STM32 mcu's have a unique 96 byte ID */
    uint8_t  canary[12];            /**<@brief Some value to distinguish first-time boots */
} cfg_settings_t;

void configInit(void);
void commit_settings(void);
void set_default_settings(void);

/**@brief (externally visible) mcache of current settings
 * @note Updates to this data are not concurrent-access safe by default
 */
extern cfg_settings_t current_settings;

#endif /* _CFG_H_ */
