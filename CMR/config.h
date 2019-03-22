/**
 * @file config.h
 * @brief Flash configuration system.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_CONFIG_H
#define CMR_CONFIG_H

#include <stm32f4xx_hal.h> // HAL_FLASH_MODULE_ENABLED

#ifdef HAL_FLASH_MODULE_ENABLED

#include <stdint.h>
#include <stddef.h>

/** @brief Each PCB has its own config CAN ID. The IDs are defined here rather 
 *         than in each of the client codes to prevent collisions. Fill in
 *         as necessary.
 */
#define CMR_CONFIG_CANID_TOM 0x500


typedef struct {
    size_t addr; 
    uint32_t data;
} cmr_canConfigMsg_t;

void cmr_configInit();

void cmr_configSetCAN(const uint8_t *data, size_t dataLen);

void cmr_configSet(size_t addr, uint32_t data);

uint32_t cmr_configGet(size_t addr);

void cmr_configPull();

void cmr_configCommit();

#endif /** HAL_FLASH_MODULE_ENABLED */

#endif /** CMR_CONFIG_H */


