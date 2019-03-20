/**
 * @file config.h
 * @brief Flash configuration system.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_CONFIG_H
#define CMR_CONFIG_H

#include <stdint.h>
#include <stddef.h>

/** @brief Each PCB has its own config CAN ID. The IDs are defined here rather 
 *         than in each of the client codes to prevent collisions. Fill in
 *         as necessary.
 */
#define CMR_CONFIG_CANID_TOM 0x500

// XXX: Consider adding a "command" field so that the config message
//      can request "read, cantx, uarttx, etc"
typedef struct {
    size_t addr;
    uint32_t data;
} cmr_config_t;

void cmr_configInit();

void cmr_configSetCAN(const uint8_t *data, size_t dataLen);

void cmr_configSet(size_t addr, uint32_t data);

uint32_t cmr_configGet(size_t addr);

void cmr_configPull();

void cmr_configCommit();

#endif /** CMR_CONFIG_H */
