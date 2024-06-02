/**
 * @file config.c
 * @brief Configuration system implementation.
 *
 * The HAL flash program implementations (both polling mode, as used here,
 * and interrupt mode) wait for prior operations to finish for a timeout
 * defined by HAL.
 *
 * @bug After erase/writing flash, the board can no longer be programmed
 * without performing a full chip erase via ST-Link Utility. A fix is
 * in progress.
 *
 * @author Carnegie Mellon Racing
 */

#include "config.h" // Interface to implement

#ifdef HAL_FLASH_MODULE_ENABLED

#include <string.h> // memcpy()
#include "panic.h"  // cmr_panic()

/**
 * @brief Initializes the configuration system with a base address.
 *
 * @note Also pulls in from flash
 *
 * @param config The interface to initalize.
 * @param cache The config cache to use.
 * @param cacheLen The size of the config cache to use.
 * @param sector The flash sector to use. See the HAL documentation @ref FLASHEx_Sectors.
 */
void cmr_configInit(cmr_config_t *config, volatile uint32_t *cache, size_t cacheLen, uint32_t sector) {
    _platform_configInit(config, cache, cacheLen, sector);
}

/**
 * @brief Sets a configuration setting.
 *
 * @param config The interface to use.
 * @param addr The word address to write to.
 * @param data The datum to write.
 *
 * @return -1 if addr out of config->cache bounds, 0 otherwise.
 */
int cmr_configSet(cmr_config_t *config, size_t addr, uint32_t data) {
    if (addr >= config->cacheLen) {
        return -1;
    }

    config->cache[addr] = data;

    return 0;
}

/**
 * @brief Gets a configuration setting.
 *
 * @param config The interface to use.
 * @param addr The word address to read from (0 to CONFIG_CACHE_LEN).
 * @param dest The destination to write the data to.
 * @return -1 if addr out of config->cache bounds, 0 otherwise.
 */
int cmr_configGet(cmr_config_t *config, size_t addr, uint32_t *dest) {
    if (addr >= config->cacheLen || dest == NULL) {
        return -1;
    }

    *dest = config->cache[addr];

    return 0;
}

/**
 * @brief Pulls the config from flash into the local config cache.
 *
 * @param config The interface to use.
 */
void cmr_configPull(cmr_config_t *config) {
    memcpy((void *) config->cache, (void *) config->flashStart, config->cacheLen * sizeof(config->cache[0]));
}

/**
 * @brief Commits the local config cache to flash.
 *
 * @param config The interface to use.
 */
void cmr_configCommit(cmr_config_t *config) {
    _platform_configCommit(config);
}

#endif /* HAL_FLASH_MODULE_ENABLED */
