/**
 * @file config.h
 * @brief Configuration system with flash persistence.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_CONFIG_H
#define CMR_CONFIG_H

#include <stm32f4xx_hal.h> // HAL_FLASH_MODULE_ENABLED

#ifdef HAL_FLASH_MODULE_ENABLED

#include <stdint.h>
#include <stddef.h>

/**
 * @brief the configuration storage len in words. this is the maximum
 *        flash sector size (0x20000 / 4).
 */
#define CONFIG_CACHE_LEN 4096

void cmr_configInit(uint32_t sector);

int cmr_configSet(size_t addr, uint32_t data);

int cmr_configGet(size_t addr, uint32_t *dest);

void cmr_configPull();

void cmr_configCommit();

#endif /* HAL_FLASH_MODULE_ENABLED */

#endif /* CMR_CONFIG_H */


