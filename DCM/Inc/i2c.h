/**
 * @file i2c.h
 * @brief All i2c activity on the CDC (i.e. the RTC and FRAM)
 *
 * @author Carnegie Mellon Racing
 */

#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <CMR/i2c.h>
#include <CMR/config_screen_helper.h>
#include <CMR/can_types.h>

/**********
 * COMMON *
 **********/

void i2cInit(void);

/**************
 * FRAM STUFF *
 **************/

/** @brief FRAM supports address between 0 and (1<<16)-1 */
#define FRAM_NUM_ADDRESS (1<<16)

/** @brief Metadata for variables stored in FRAM */
typedef struct {
    uint16_t startAddress;  /**< @brief Starting Address to be stored in FRAM */
    uint16_t dataLength;    /**< @brief Data Length in Bytes */
    // uint32_t currentValue;	/**< @brief Most Recent Value Read from FRAM */
} cmr_framVariable_t;

/** @brief Represents a variable stored in FRAM */
typedef cmr_driver_profile_t framVariable_t;

int framRead(framVariable_t variable, uint8_t *data);
int framWrite(framVariable_t variable, uint8_t *data);

/*************
 * RTC STUFF *
 *************/

cmr_can_rtc_data_t getRTCData(void);
void setRTCData(const cmr_can_rtc_data_t *time);

#endif /* I2C_H */
