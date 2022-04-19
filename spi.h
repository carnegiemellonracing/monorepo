/*
 * spi.h
 *
 *  Created on: Sep 7, 2021
 *      Author: vamsi
 */

#ifndef SPI_H_
#define SPI_H_

#include <CMR/spi.h>
#include <CMR/tasks.h>

/** @brief Addresses of registers within HV Sense */
typedef enum {
    IWV = 0x0,          /**<@brief Instantaneous value of Current I. */
    V1WV = 0x1,         /**<@brief Instantaneous value of Voltage V1. */
    V2WV = 0x2,         /**<@brief Instantaneous value of Voltage V2. */
    ADC_CRC = 0x4,      /**<@brief CRC value of IWV, V1WV, and V2WV registers. */
    CTRL_CRC = 0x5,     /**<@brief CRC value of configuration registers. */
    CNT_SNAPSHOT = 0x7, /**<@brief CRC value of configuration registers. */
    CONFIG = 0x8,       /**<@brief Configuration register. */
    STATUS0 = 0x9,      /**<@brief Status register. */
    LOCK = 0xA,         /**<@brief Memory protection register. */
    SYNC_SNAP = 0xB,    /**<@brief Synchronization register. */
    COUNTER0 = 0xC,     /**<@brief Contains the least significant eight bits of the internal synchronization counter. */
    COUNTER1 = 0xD,     /**<@brief COUNTER1[3:0] bits contain the most significant four bits of the internal synchronization counter. */
    EMI_CTRL = 0xE,     /**<@brief EMI control register. Manages the PWM control block of the isolated dc-to-dc converter to reduce EMI emissions. */
    STATUS1 = 0xF,      /**<@brief Status Register. */
    TEMPOS = 0x18,      /**<@brief Temperature sensor offset. */
} hvSenseRegister_t;

/** @brief Bit position of RESET_ON within STATUS0 Register */
#define STATUS0_RESET_ON (1 << 0)

/** @brief Lock register - protect configuration registers */
#define LOCK_KEY_EN (0xCA)

/** @brief Lock register - don't protect configuration registers */
#define LOCK_KEY_DIS (0x9C)

/** @brief Sets ADC output every 1ms */
#define ADC_FREQ_1kHz ((0b11) << 4)

/** @brief Sets ADC output every 0.5ms */
#define ADC_FREQ_2kHz ((0b01) << 4) | 0b1

/** @brief Sampling time */
#define EMI_CONFIG 0x55

int32_t getHVmillivolts();
int32_t getCurrentInstant();
int32_t getCurrentAverage();

void spiInit(void);

#endif /* SPI_H_ */
