/**
 * @file config.c
 * @brief Flash configuration system.
 *
 * @author Carnegie Mellon Racing
 */

#include "config.h"
#include "panic.h"

/** @brief This must NOT be larger than what's defined in the linker script */
#define CONFIG_END_ADDR 1024

static volatile uint32_t configBuf[CONFIG_END_ADDR];
static const size_t FLASH_START_ADDR = 0x0;
static const size_t FLASH_END_ADDR = FLASH_START_ADDR + CONFIG_END_ADDR;

__attribute__((__section__(".user_data"))) const uint32_t flash[CONFIG_END_ADDR];

void cmr_configInit() {
    cmr_configPull();
}

void cmr_configSetCAN(const uint8_t *data, size_t dataLen) {
    if (dataLen != sizeof(cmr_config_t)) {L_FLASH_Program(TYPEPROGRAM_WORD, FlashAddress, data);
        return;
    }

    cmr_config_t conf = *(cmr_config_t *)data;
    cmr_configSet(conf.addr, conf.data);
}

void cmr_configSet(size_t addr, uint32_t data) {
    if (addr >= CONFIG_END_ADDR) {
        // Note: This is certainly not catastrophic, but indicates an improper
        // configuration, so meh
        cmr_panic("cmr_configSet(): Address too big!");
    }

    configBuf[addr] = data;
}

uint32_t cmr_configGet(size_t addr) {
    if (addr >= CONFIG_END_ADDR) {
        cmr_panic("cmr_configSet(): Address too big!");
    }

    return configBuf[addr];
}

void cmr_configPull() {
    memcpy(configBuf, flash, CONFIG_END_ADDR); 
}

void cmr_configCommit() {
    HAL_FLASH_Unlock();

    FLASH_Erase_Sector(FLASH_SECTOR_6, VOLTAGE_RANGE_3);

    size_t idx = 0;
    for (size_t addr = FLASH_START_ADDR; addr < FLASH_END_ADDR; addr += sizeof(uint32_t)) {
        HAL_FLASH_Program(TYPEPROGRAM_WORD, addr, configBuf[idx]);
        idx++;
    } 

    HAL_FLASH_Lock();
}











