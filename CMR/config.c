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
 * @brief Instantiates the macro for each flash sector.
 *
 * The parameter are, in order: sector number, corresponding base address, and cooresponding size.
 * The sectors are defined at HAL @ref FLASHEx_Sectors.
 * The sector addresses and sizes are defined in the STM32F413 reference manual.
 *
 * @param f The macro to instantiate.
 */
#define SECTOR_FOREACH(f) \
    f(FLASH_SECTOR_0, 0x08000000, 0x4000) \
    f(FLASH_SECTOR_1, 0x08004000, 0x4000) \
    f(FLASH_SECTOR_2, 0x08008000, 0x4000) \
    f(FLASH_SECTOR_3, 0x0800C000, 0x4000) \
    f(FLASH_SECTOR_4, 0x08010000, 0x10000) \
    f(FLASH_SECTOR_5, 0x08020000, 0x20000) \
    f(FLASH_SECTOR_6, 0x08040000, 0x20000) \
    f(FLASH_SECTOR_7, 0x08060000, 0x20000) \
    f(FLASH_SECTOR_8, 0x08080000, 0x20000) \
    f(FLASH_SECTOR_9, 0x080A0000, 0x20000) \
    f(FLASH_SECTOR_10, 0x080C0000, 0x20000) \
    f(FLASH_SECTOR_11, 0x080E0000, 0x20000) \
    f(FLASH_SECTOR_12, 0x08100000, 0x20000) \
    f(FLASH_SECTOR_13, 0x08120000, 0x20000) \
    f(FLASH_SECTOR_14, 0x08140000, 0x20000) \
    f(FLASH_SECTOR_15, 0x08160000, 0x20000)

static void *getSectorBase(uint32_t sector) {
#define GET_ADDR(sec, base, ...) \
    if (sector == sec) { \
        return (void *) base; \
    }
SECTOR_FOREACH(GET_ADDR)
#undef GET_ADDR

    cmr_panic("Invalid sector!");
}

static size_t getSectorSize(uint32_t sector) {
#define GET_SIZE(sec, base, size) \
    if (sector == sec) { \
        return size; \
    }
SECTOR_FOREACH(GET_SIZE)
#undef GET_SIZE

    cmr_panic("Invalid sector!");
}

/**
 * @brief Initializes the configuration system with a base address.
 * 
 * @param config The interface to initalize.
 * @param cache The config cache to use.
 * @param cacheLen The size of the config cache to use.
 * @param sector The flash sector to use. See the HAL documentation @ref FLASHEx_Sectors.
 */
void cmr_configInit(cmr_config_t *config, volatile uint32_t *cache, size_t cacheLen, uint32_t sector) {
    config->cache = cache;
    config->cacheLen = cacheLen;
    config->flashSector = sector;
    config->flashStart = (volatile uint32_t *) getSectorBase(sector);
    config->flashSize = getSectorSize(sector);

    if (config->cacheLen > config->flashSize) {
        cmr_panic("Config cache is larger than sector!");
    }

    cmr_configPull(config);
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
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return;
    }

    // Clears all the error bits. See the HAL documentation @ref FLASH_Flag_definition.
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | 
            FLASH_FLAG_OPERR | 
            FLASH_FLAG_WRPERR |
            FLASH_FLAG_PGAERR | 
            FLASH_FLAG_PGSERR);
  
    FLASH_EraseInitTypeDef eraseInit = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Sector = config->flashSector,
        .NbSectors = 1,
        .VoltageRange = VOLTAGE_RANGE_3,
    };

    // Use HAL_FLASHEx_Erase instead of HAL_FLASH_Erase, as it clears the FLASH control register.
    uint32_t error;
    if (HAL_FLASHEx_Erase(&eraseInit, &error) != HAL_OK) {
        cmr_panic("Flash erase failed!");
    }

    size_t idx = 0;
    while (idx < config->cacheLen) {
        if (HAL_FLASH_Program(TYPEPROGRAM_WORD, (uint32_t) (config->flashStart + idx), 
                config->cache[idx]) == HAL_OK) {
            idx++;
        }
    }

    HAL_FLASH_Lock();
}

#endif /* HAL_FLASH_MODULE_ENABLED */
