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

#include "panic.h"  // cmr_panic()

/**
 * @brief The base addresses of each flash sector. See  the HAL
 * documentation @ref FLASHEx_Sectors.
 */
#define ADDR_FLASH_SECTOR_0     ((size_t) 0x08000000) 
#define ADDR_FLASH_SECTOR_1     ((size_t) 0x08004000) 
#define ADDR_FLASH_SECTOR_2     ((size_t) 0x08008000) 
#define ADDR_FLASH_SECTOR_3     ((size_t) 0x0800C000) 
#define ADDR_FLASH_SECTOR_4     ((size_t) 0x08010000) 
#define ADDR_FLASH_SECTOR_5     ((size_t) 0x08020000) 
#define ADDR_FLASH_SECTOR_6     ((size_t) 0x08040000) 
#define ADDR_FLASH_SECTOR_7     ((size_t) 0x08060000) 
#define ADDR_FLASH_SECTOR_8     ((size_t) 0x08080000) 
#define ADDR_FLASH_SECTOR_9     ((size_t) 0x080A0000) 
#define ADDR_FLASH_SECTOR_10    ((size_t) 0x080C0000) 
#define ADDR_FLASH_SECTOR_11    ((size_t) 0x080E0000) 
#define ADDR_FLASH_SECTOR_12    ((size_t) 0x08100000) 
#define ADDR_FLASH_SECTOR_13    ((size_t) 0x08120000) 
#define ADDR_FLASH_SECTOR_14    ((size_t) 0x08140000) 
#define ADDR_FLASH_SECTOR_15    ((size_t) 0x08160000)
#define ADDR_FLASH_SECTOR_16    ((size_t) 0x08180000) 

/**
 * @brief The configuration storage size in words.
 */
#define CONFIG_LEN ((size_t) 1024)

/**
 * @brief The configuration cache. It is updated by calling cmr_configCommit() and
 * cmr_configPull().
 */
static volatile uint32_t configBuf[CONFIG_LEN];

/**
 * @brief The configuration base address. It is initialized by cmr_configInit().
 */
static size_t startAddr;

/**
 * @brief Instantiates the macro for each flash sector.
 *
 * @note Is there a better way to macro num+1? Maybe just stick
 * the ADDR_FLASH_SECTOR_* in an array?
 *
 * @param f The macro to instantiate.
 */
#define SECTOR_FOREACH(f) \
    f(0, 1) \
    f(1, 2) \
    f(2, 3) \
    f(3, 4) \
    f(4, 5) \
    f(5, 6) \
    f(6, 7) \
    f(7, 8) \
    f(8, 9) \
    f(9, 10) \
    f(10, 11) \
    f(11, 12) \
    f(12, 13) \
    f(13, 14) \
    f(14, 15) \
    f(15, 16)

/**
 * @brief Gets the sector corresponding to an address. The address
 * must be within sectors 0 through 15.
 *
 * @param addr An address to get the corresponding sector of.
 *
 * @return The corresponding sector.
 */
static uint32_t getSector(size_t addr) {
#define FLASH_SECTOR(num, next) \
    if ((addr >= ADDR_FLASH_SECTOR_ ## num) && (addr < ADDR_FLASH_SECTOR_ ## next)) { \
        return FLASH_SECTOR_ ## num; \
    }
SECTOR_FOREACH(FLASH_SECTOR) 
#undef FLASH_SECTOR

    cmr_panic("Invalid sector!");
}

/**
 * @brief Gets the base address corresponding to a sector. The sector
 * must be within 0 and 15.
 *
 * @param sector A sector to get the corresponding base address of.
 *
 * @return The corresponding base address.
 */
static uint32_t getSectorAddr(uint32_t sector) {
#define FLASH_SECTOR_ADDR(sec, ...) \
    if (sector == FLASH_SECTOR_ ## sec) { \
        return ADDR_FLASH_SECTOR_ ## sec; \
    }

SECTOR_FOREACH(FLASH_SECTOR_ADDR)
#undef FLASH_SECTOR_ADDR

    cmr_panic("Invalid sector!");
}

/**
 * @brief Gets the number of sectors required corresponding to a base address.
 * Consider the case when the base address is near the end of a sector.
 *
 * @param addr The config base address.
 *
 * @returns The number of sectors.
 */
static size_t getNumberSectors(size_t addr) {
    if (getSector(addr+CONFIG_LEN) != getSector(addr)) {
        return 2;
    }

    return 1;
}

/**
 * @brief Initializes the configuration system with a base address.
 *
 * @param addr A base address.
 */
void cmr_configInit(size_t addr) {
    startAddr = addr;
    cmr_configPull();
}

/**
 * @brief Tests the configuration system with a base address.
 *
 * @param addr A base address.
 */
void cmr_configTest(uint32_t sector) {
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return;
    }

    // Clears all the error bits. See the HAL documentation @ref FLASH_Flag_definition.
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | \
            FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR);
  
    size_t startAddr = getSectorAddr(sector);
    size_t endAddr = getSectorAddr(sector+1)-1;
    uint32_t byte = (*(uint32_t *) startAddr) + 1;
    
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

    size_t addr = startAddr;
    while (addr < endAddr) {
        if (HAL_FLASH_Program(TYPEPROGRAM_WORD, addr, byte) != HAL_OK) {
            cmr_panic("Failed programming word!");
        }
        addr += sizeof(uint32_t);
    }

    HAL_FLASH_Lock();

    addr = startAddr;
    size_t errors = 0;
    while (addr < endAddr) {
        if ((*(uint32_t *) addr) != byte) {
            errors += 1;
        }
        addr += sizeof(uint32_t);
    }

    if (errors != 0) {
        cmr_panic("The write was unsuccessful!");
    }
}

/**
 * @brief Sets a configuration from a buffer.
 *
 * @param data A buffer.
 * @param dataLen The buffers length.
 */
void cmr_configBufSet(const uint8_t *data, size_t dataLen) {
    if (dataLen != sizeof(cmr_bufConfigMsg_t)) {
        return;
    }

    cmr_bufConfigMsg_t conf = *(cmr_bufConfigMsg_t *)data;

    cmr_configSet(conf.addr, conf.data);
}

/**
 * @brief Sets a configuration setting.
 *
 * @param addr A word address to write to (0 to CONFIG_LEN).
 * @param data A datum to write.
 */
void cmr_configSet(size_t addr, uint32_t data) {
    if (addr >= CONFIG_LEN) {
        cmr_panic("cmr_configSet(): Address too big!");
    }

    configBuf[addr] = data;
}

/**
 * @brief Gets a configuration setting.
 *
 * @param addr A word address to read from (0 to CONFIG_LEN).
 * @return The data at the address.
 */
uint32_t cmr_configGet(size_t addr) {
    if (addr >= CONFIG_LEN) {
        cmr_panic("cmr_configSet(): Address too big!");
    }

    return configBuf[addr];
}

/**
 * @brief Pulls the config from flash into the local config cache.
 */
void cmr_configPull() {
   __IO uint32_t *flash = (__IO uint32_t *) startAddr;
 
    for (size_t idx = 0; idx < CONFIG_LEN; idx++) {
        configBuf[idx] = flash[idx];
    } 
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
  
    uint32_t sector = getSector(startAddr);
    
    FLASH_EraseInitTypeDef eraseInit = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Sector = sector,
        .NbSectors = getNumberSectors(startAddr),
        .VoltageRange = VOLTAGE_RANGE_3,
    };

    uint32_t error;
    if (HAL_FLASHEx_Erase(&eraseInit, &error) != HAL_OK) {
        cmr_panic("Flash erase failed!");
    }

    size_t idx = 0;
    size_t addr = startAddr;
    while (addr < startAddr + CONFIG_LEN) {
        if (HAL_FLASH_Program(TYPEPROGRAM_WORD, addr, configBuf[idx]) == HAL_OK) {
            idx++;
            addr += sizeof(uint32_t);
        }
    }

    HAL_FLASH_Lock();
}

#endif /* HAL_FLASH_MODULE_ENABLED */
