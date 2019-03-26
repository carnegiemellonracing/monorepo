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
 * @brief Represents a config message sent over CAN.
 */
typedef struct {
    uint32_t addr; 
    uint32_t data;
} cmr_bufConfigMsg_t;

void cmr_configInit();

void cmr_configTest(uint32_t sector);

void cmr_configBufSet(const uint8_t *data, size_t dataLen);

void cmr_configSet(size_t addr, uint32_t data);

uint32_t cmr_configGet(size_t addr);

void cmr_configPull();

void cmr_configCommit();

#endif /* HAL_FLASH_MODULE_ENABLED */

#endif /* CMR_CONFIG_H */


