/**
 * @file config.h
 * @brief Configuration system with flash persistence.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_CONFIG_H
#define CMR_CONFIG_H

#include "platform.h"   // HAL_FLASH_MODULE_ENABLED

#ifdef HAL_FLASH_MODULE_ENABLED

#include <stdint.h>
#include <stddef.h>

typedef struct {
    volatile uint32_t *cache;
    size_t cacheLen;
    uint32_t flashSector;
    volatile uint32_t *flashStart;
    volatile size_t flashSize;
} cmr_config_t;

void cmr_configInit(cmr_config_t *config, volatile uint32_t *cache, size_t cacheLen, uint32_t sector);

int cmr_configSet(cmr_config_t *config, size_t addr, uint32_t data);

void cmr_configPull(cmr_config_t *config);

int cmr_configGet(cmr_config_t *config, size_t addr, uint32_t *dest);

void cmr_configCommit(cmr_config_t *config);

/* External Platform-specific dependecies */
extern void _platform_configInit(cmr_config_t *config, volatile uint32_t *cache, size_t cacheLen, uint32_t sector);
extern void _platform_configCommit(cmr_config_t *config);

#endif /* HAL_FLASH_MODULE_ENABLED */

#endif /* CMR_CONFIG_H */


