/**
 * @file config.c
 * @brief Flash configuration system.
 *
 * @author Carnegie Mellon Racing
 */

#include "config.h"
#include "panic.h"
#include <stm32f4xx_hal.h>

#define CONFIG_END_ADDR 1024
#define FLASH_START_ADDR 0x080E0000
#define FLASH_END_ADDR (FLASH_START_ADDR + CONFIG_END_ADDR)

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) 
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) 
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) 
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) 
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) 
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) 
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) 
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) 
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) 
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) 
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) 
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) 
#define ADDR_FLASH_SECTOR_12    ((uint32_t)0x08100000) 
#define ADDR_FLASH_SECTOR_13    ((uint32_t)0x08120000) 
#define ADDR_FLASH_SECTOR_14    ((uint32_t)0x08140000) 
#define ADDR_FLASH_SECTOR_15    ((uint32_t)0x08160000) 

// static const volatile uint32_t *flash = (const volatile uint32_t *) FLASH_START_ADDR;
__attribute__((__section__(".user_data"))) const uint32_t flash[64];

static volatile uint32_t configBuf[CONFIG_END_ADDR];

void HAL_FLASH_EndOfOperationCallback(uint32_t returnValue) {


}

void HAL_FLASH_OperationErrorCallback(uint32_t returnValue) {

}

void cmr_configInit() {
    cmr_configPull();
}

void cmr_configSetCAN(const uint8_t *data, size_t dataLen) {
    if (dataLen != sizeof(cmr_canConfigMsg_t)) {
        return;
    }

    cmr_canConfigMsg_t conf = *(cmr_canConfigMsg_t *)data;

    cmr_configSet(conf.addr, conf.data);
}

void cmr_configSet(size_t addr, uint32_t data) {
    if (addr >= CONFIG_END_ADDR) {
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
    for (size_t idx = 0; idx < CONFIG_END_ADDR; idx++) {
        configBuf[idx] = flash[idx];
    } 
}

void cmr_configCommit() {
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | \
            FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR);
   
    FLASH_EraseInitTypeDef eraseInit = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Sector = FLASH_SECTOR_11,
        .NbSectors = 1,
        .VoltageRange = VOLTAGE_RANGE_3,
    };

    uint32_t error;
    if (HAL_FLASHEx_Erase(&eraseInit, &error) != HAL_OK) {
        HAL_FLASH_Unlock();
        cmr_panic("Flash erase failed!");
    }

    size_t idx = 0;
    for (size_t addr = FLASH_START_ADDR; addr < FLASH_END_ADDR; addr += sizeof(uint32_t)) {
        if (HAL_FLASH_Program(TYPEPROGRAM_WORD, addr, configBuf[idx]) != HAL_OK) {
            HAL_FLASH_Unlock();
            cmr_panic("Flash programming timed out!");
        }
        idx++;
    } 

    HAL_FLASH_Lock();
}











