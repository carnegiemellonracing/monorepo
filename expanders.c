/**
 * @file expanders.c
 * @brief GPIO expanders implementation
 * 
 * GPIO expanders in use:
 *      Main Board Digital 1:   PCA9555DBR      (ROT_{1,2}_{5..8})
 *      Main Board Digital 2:   PCA9555DBR      (ROT_{1,2}_{1..4}, LED_{1,2}, PUSH{1..4})
 *      Daughter Board Digital: PCA9554PW,118   (PUSH{1..3})
 *      Daughter Board Analog:  AD5593RBRUZ     (CLUTCH{1,2})
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/i2c.h>    // I2C interface

#include "expanders.h"          // Interface to implement
#include "expandersPrivate.h"   // Private interface


/** @brief Primary I2C interface */
static cmr_i2c_t i2c;

/** @brief DIM I2C address */
static const uint32_t ownAddress = 0x00; // 0x00 = 0b0000000

// TODO: Fix addresses
/** @brief Main Board Digital 1 expander I2C address */
static const uint16_t mainDigital1Address = 0x27; // 0x27 = 0b0100111

/** @brief Main Board Digital 2 expander I2C address */
static const uint16_t mainDigital2Address = 0x26; // 0x26 = 0b0100110

/** @brief Daughter Board Digital expander I2C address */
static const uint16_t daughterDigitalAddress = 0x25; // 0x27 = 0b0100101

/** @brief Daughter Board Analog expander I2C address */
static const uint16_t daughterAnalogAddress = 0x27; // 0x27 = 0b0100111

/** @brief I2C Timeout (milliseconds). */
static const uint32_t i2cTimeout_ms = 1;

/**
 * @brief Array of expander button config and states
 * 
 */
static expanderButtonState_t buttons[EXP_BUTTON_LEN] = {
    [EXP_DASH_BUTTON_1] = {
        .expanderAddr = mainDigital2Address,
        .port = 1,
        .pin = 2,
        .value = false
    },
    [EXP_DASH_BUTTON_2] = {
        .expanderAddr = mainDigital2Address,
        .port = 1,
        .pin = 3,
        .value = false
    },
    [EXP_DASH_BUTTON_3] = {
        .expanderAddr = mainDigital2Address,
        .port = 1,
        .pin = 4,
        .value = false
    },
    [EXP_DASH_BUTTON_4] = {
        .expanderAddr = mainDigital2Address,
        .port = 1,
        .pin = 5,
        .value = false
    },
    [EXP_WHEEL_BUTTON_1] = {
        .expanderAddr = daughterDigitalAddress,
        .port = 0,
        .pin = 0,
        .value = false
    },
    [EXP_WHEEL_BUTTON_2] = {
        .expanderAddr = daughterDigitalAddress,
        .port = 0,
        .pin = 1,
        .value = false
    },
    [EXP_WHEEL_BUTTON_3] = {
        .expanderAddr = daughterDigitalAddress,
        .port = 0,
        .pin = 2,
        .value = false
    },
};

bool expanderGetButton(expanderButton_t button)
{
    return buttons[button].value;
}

uint8_t expanderGetRotary(expanderRotary_t rotary)
{
    return 0;
}
uint32_t expanderGetClutch(expanderClutch_t clutch)
{
    return 0;
}
void expanderSetLED(expanderLED_t led, bool on)
{
    return;
}

/**
 * @brief Initializes the GPIO expander interface.
 */
void expandersInit(void) {
    // I2C initialization
    cmr_i2cInit(
        &i2c, I2C1,
        I2C_CLOCK_LOW, ownAddress,
        GPIOB, GPIO_PIN_6,
        GPIOB, GPIO_PIN_7
    );

    // Main Board Digital 1 expander has all inputs on Port 0, nothing on Port 1
    uint8_t mainDigital1Config[2] = {
        PCA9555_CONFIG_PORT_0,
        0xFF
    };
    // Main Board Digital 2 expander has all inputs except outputs on Port 1 Pins 0 and 1
    uint8_t mainDigital2Config[3] = {
        PCA9555_CONFIG_PORT_0,
        0xFF,
        0xFC    // 0b11111100
    };

    // Daughter Board Digital expander has all inputs
    uint8_t daughterDigitalConfig[2] = {
        PCA9554_CONFIG_PORT,
        0xFF
    };

    // Transmit config to expanders
    cmr_i2cTX(
        &i2c,
        mainDigital1Address, mainDigital1Config,
        sizeof(mainDigital1Config) / sizeof(mainDigital1Config[0]),
        i2cTimeout_ms
    );
    cmr_i2cTX(
        &i2c,
        mainDigital2Address, mainDigital2Config,
        sizeof(mainDigital2Config) / sizeof(mainDigital2Config[0]),
        i2cTimeout_ms
    );
    cmr_i2cTX(
        &i2c,
        daughterDigitalAddress, daughterDigitalConfig,
        sizeof(daughterDigitalConfig) / sizeof(daughterDigitalConfig[0]),
        i2cTimeout_ms
    );

}



