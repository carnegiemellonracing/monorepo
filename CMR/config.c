/**
 * @file config.c
 * @brief Flash configuration system.
 *
 * @author Carnegie Mellon Racing
 */

#include "config.h"
#include "panic.h"

#define CONFIG_MAX_ADDR 1024

static volatile uint32_t configBuf[CONFIG_MAX_ADDR];

void cmr_configInit() {
    cmr_configPull();
}

void cmr_configSetCAN(const uint8_t *data, size_t dataLen) {
    if (dataLen != sizeof(cmr_config_t)) {
        return;
    }

    cmr_config_t conf = *(cmr_config_t *)data;
    cmr_configSet(conf.addr, conf.data);
}

void cmr_configSet(size_t addr, uint32_t data) {
    if (addr >= CONFIG_MAX_ADDR) {
        // Note: This is certainly not catastrophic, but indicates an improper
        // configuration, so meh
        cmr_panic("cmr_configSet(): Address too big!");
    }

    configBuf[addr] = data;
}

uint32_t cmr_configGet(size_t addr) {
    if (addr >= CONFIG_MAX_ADDR) {
        cmr_panic("cmr_configSet(): Address too big!");
    }

    return configBuf[addr];
}

void cmr_configPull() {

}

void cmr_configCommit() {

}
