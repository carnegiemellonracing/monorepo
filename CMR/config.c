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
 * @brief The configuration storage len in words. This is the maximum
 *        flash sector size (0x20000 / 4).
 */
#define CONFIG_CACHE_LEN 4096

typedef struct {
    volatile uint32_t cache[CONFIG_CACHE_LEN];
    uint32_t flashSector;
    volatile uint32_t *flashStart;
    volatile size_t flashSize;
} cmr_config_t;

static cmr_config_t config;

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
 * @param addr A base address.
 */
void cmr_configInit(uint32_t sector) {
    config.flashSector = sector;
    config.flashStart = (volatile uint32_t *) getSectorBase(sector);
    config.flashSize = getSectorSize(sector);

    if (sizeof(config.cache) > config.flashSize) {
        cmr_panic("The flash sector is not large enough!");
    }

    cmr_configPull();
}

/**
 * @brief Sets a configuration setting.
 *
 * @param addr A word address to write to (0 to CONFIG_CACHE_LEN).
 * @param data A datum to write.
 */
int cmr_configSet(size_t addr, uint32_t data) {
    if (addr >= CONFIG_CACHE_LEN) {
        return -1;
    }

    config.cache[addr] = data;

    return 0;
}

/**
 * @brief Gets a configuration setting.
 *
 * @param addr A word address to read from (0 to CONFIG_CACHE_LEN).
 * @return The data at the address.
 */
int cmr_configGet(size_t addr, uint32_t *dest) {
    if (addr >= CONFIG_CACHE_LEN || dest == NULL) {
        return -1; 
    }

    *dest = config.cache[addr];

    return 0;
}

/**
 * @brief Pulls the config from flash into the local config cache.
 */
void cmr_configPull() {
    memcpy((void *) config.cache, (void *) config.flashStart, sizeof(config.cache));
}

/**
 * @brief Commits the local config cache to flash.
 */
void cmr_configCommit() {
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
        .Sector = config.flashSector,
        .NbSectors = 1,
        .VoltageRange = VOLTAGE_RANGE_3,
    };

    // Use HAL_FLASHEx_Erase instead of HAL_FLASH_Erase, as it clears the FLASH control register.
    uint32_t error;
    if (HAL_FLASHEx_Erase(&eraseInit, &error) != HAL_OK) {
        cmr_panic("Flash erase failed!");
    }

    size_t idx = 0;
    while (idx < CONFIG_CACHE_LEN) {
        if (HAL_FLASH_Program(TYPEPROGRAM_WORD, (uint32_t) (config.flashStart + idx), 
                config.cache[idx]) == HAL_OK) {
            idx++;
        }
    }

    HAL_FLASH_Lock();
}

#endif /* HAL_FLASH_MODULE_ENABLED */
