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
 * @brief The configuration storage size in words. 
 *
 * It may be wise to parameterize this into a cmr_config_t, thus allowing multiple
 * concurrent flash configuration systems with configurable size. I imagine each 
 * PCB will have different configuration size requirements...
 */
#define CONFIG_LEN 1024

/**
 * @brief The configuration cache. It is updated by calling cmr_configCommit() and
 * cmr_configPull().
 */
static volatile uint32_t configCache[CONFIG_LEN];

/**
 * @brief The configuration base address. It is initialized by cmr_configInit().
 */
static volatile uint32_t *configFlashStart;

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
    f(0,  0x08000000, 0x4000) \
    f(1,  0x08004000, 0x4000) \
    f(2,  0x08008000, 0x4000) \
    f(3,  0x0800C000, 0x4000) \
    f(4,  0x08010000, 0x10000) \
    f(5,  0x08020000, 0x20000) \
    f(6,  0x08040000, 0x20000) \
    f(7,  0x08060000, 0x20000) \
    f(8,  0x08080000, 0x20000) \
    f(9,  0x080A0000, 0x20000) \
    f(10, 0x080C0000, 0x20000) \
    f(11, 0x080E0000, 0x20000) \
    f(12, 0x08100000, 0x20000) \
    f(13, 0x08120000, 0x20000) \
    f(14, 0x08140000, 0x20000) \
    f(15, 0x08160000, 0x20000)

/**
 * @brief Gets the sector corresponding to an address. The address
 * must be within sectors 0 through 15.
 *
 * @param addr An address to get the corresponding sector of.
 *
 * @return The corresponding sector.
 */
static uint32_t getSector(void *addr) {
#define FLASH_SECTOR(num, base, size) \
    if ((addr >= (void *) base) && (addr < (void *) (base + size))) { \
        return FLASH_SECTOR_ ## num; \
    } 
SECTOR_FOREACH(FLASH_SECTOR) 
#undef FLASH_SECTOR

    cmr_panic("Invalid sector!");
}

/**
 * @brief Gets the base address corresponding to a sector.
 *
 * @param sector A sector to get the corresponding base address of. It must be
 *               one of the values defined in HAL @ref FLASHEx_Sectors.
 *
 * @return The corresponding base address.
 */
static void *getSectorAddr(uint32_t sector) {
#define FLASH_SECTOR_ADDR(num, addr, ...) \
    if (sector == FLASH_SECTOR_ ## num) { \
        return (void *) addr; \
    } 

SECTOR_FOREACH(FLASH_SECTOR_ADDR)
#undef FLASH_SECTOR_ADDR

    cmr_panic("Invalid sector!");
}

static size_t getSectorSize(uint32_t sector) {
#define FLASH_SECTOR_SIZE(num, addr, size) \
    if (sector == FLASH_SECTOR_ ## num) { \
        return size; \
    }

SECTOR_FOREACH(FLASH_SECTOR_SIZE)
#undef FLASH_SECTOR_SIZE

    cmr_panic("Invalid sector!");
}

/**
 * @brief Initializes the configuration system with a base address.
 *
 * @param addr A base address.
 */
void cmr_configInit(void *addr) {
    if (getSector(addr) != getSector(addr + CONFIG_LEN)) {
        cmr_panic("The address is too close to the end of a sector!");   
    }

    configFlashStart = (volatile uint32_t *) addr;
    cmr_configPull();
}

/**
 * @brief Tests the configuration system with a base address.
 *
 * @param sector A sector to test.
 */
void cmr_configTest(uint32_t sector) {
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return;
    }

    // Clears all the error bits. See the HAL documentation @ref FLASH_Flag_definition.
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | \
            FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR);
  
    uint32_t *flashStart = getSectorAddr(sector);
    uint32_t *flashEnd = flashStart + getSectorSize(sector) / sizeof(uint32_t);
    uint32_t byte = *flashStart + 1;
    
    FLASH_EraseInitTypeDef eraseInit = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Sector = sector,
        .NbSectors = 1,
        .VoltageRange = VOLTAGE_RANGE_3,
    };

    uint32_t error;
    if (HAL_FLASHEx_Erase(&eraseInit, &error) != HAL_OK) {
        cmr_panic("Flash erase failed!");
    }

    uint32_t *addr = flashStart;
    while (addr < flashEnd) {
        if (HAL_FLASH_Program(TYPEPROGRAM_WORD, (uint32_t) addr, byte) != HAL_OK) {
            cmr_panic("Failed programming word!");
        }
        addr += 1;
    }

    HAL_FLASH_Lock();

    addr = flashStart;
    size_t errors = 0;
    while (addr < flashEnd) {
        if (*addr != byte) {
            errors += 1;
        }
        addr += 1;
    }

    if (errors != 0) {
        cmr_panic("The write was unsuccessful!");
    }
}

/**
 * @brief Sets a configuration setting.
 *
 * @param addr A word address to write to (0 to CONFIG_LEN).
 * @param data A datum to write.
 */
int cmr_configSet(size_t addr, uint32_t data) {
    if (addr >= CONFIG_LEN) {
        return -1;
    }

    configCache[addr] = data;

    return 0;
}

/**
 * @brief Gets a configuration setting.
 *
 * @param addr A word address to read from (0 to CONFIG_LEN).
 * @return The data at the address.
 */
int cmr_configGet(size_t addr, uint32_t *dest) {
    if (addr >= CONFIG_LEN || dest == NULL) {
        return -1; 
    }

    *dest = configCache[addr];

    return 0;
}

/**
 * @brief Pulls the config from flash into the local config cache.
 */
void cmr_configPull() {
    memcpy((uint32_t *) configCache, (uint32_t *) configFlashStart, CONFIG_LEN);
}

/**
 * @brief Commits the local config cache to flash.
 */
void cmr_configCommit() {
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return;
    }

    // Clears all the error bits. See the HAL documentation @ref FLASH_Flag_definition.
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | \
            FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR);
  
    uint32_t sector = getSector((void *) configFlashStart);
    
    FLASH_EraseInitTypeDef eraseInit = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Sector = sector,
        .NbSectors = 1,
        .VoltageRange = VOLTAGE_RANGE_3,
    };

    uint32_t error;
    if (HAL_FLASHEx_Erase(&eraseInit, &error) != HAL_OK) {
        cmr_panic("Flash erase failed!");
    }

    size_t idx = 0;
    while (idx < CONFIG_LEN) {
        if (HAL_FLASH_Program(TYPEPROGRAM_WORD, (uint32_t) configFlashStart + idx, configCache[idx]) == HAL_OK) {
            idx++;
        }
    }

    HAL_FLASH_Lock();
}

#endif /* HAL_FLASH_MODULE_ENABLED */
