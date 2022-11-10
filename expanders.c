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
#include <CMR/tasks.h>  // Task interface

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
 * @brief Array of expander button pin configs
 * 
 */
static expanderPinConfig_t buttons[EXP_BUTTON_LEN] = {
    [EXP_DASH_BUTTON_1] = {
        .expanderAddress = mainDigital2Address,
        .port = 1,
        .pin = 2
    },
    [EXP_DASH_BUTTON_2] = {
        .expanderAddress = mainDigital2Address,
        .port = 1,
        .pin = 3
    },
    [EXP_DASH_BUTTON_3] = {
        .expanderAddress = mainDigital2Address,
        .port = 1,
        .pin = 4
    },
    [EXP_DASH_BUTTON_4] = {
        .expanderAddress = mainDigital2Address,
        .port = 1,
        .pin = 5
    },
    [EXP_WHEEL_BUTTON_1] = {
        .expanderAddress = daughterDigitalAddress,
        .port = 0,
        .pin = 0
    },
    [EXP_WHEEL_BUTTON_2] = {
        .expanderAddress = daughterDigitalAddress,
        .port = 0,
        .pin = 1
    },
    [EXP_WHEEL_BUTTON_3] = {
        .expanderAddress = daughterDigitalAddress,
        .port = 0,
        .pin = 2
    }
};

/**
 * @brief Array of expander rotary pin configs
 * 
 */
static expanderRotaryConfig_t rotaries[EXP_ROTARY_LEN] = {
    [EXP_ROTARY_1] = {
        .pins = {
            [ROTARY_POS_1] = {
                .expanderAddress = mainDigital2Address,
                .port = 0,
                .pin = 1
            },
            [ROTARY_POS_2] = {
                .expanderAddress = mainDigital2Address,
                .port = 0,
                .pin = 3
            },
            [ROTARY_POS_3] = {
                .expanderAddress = mainDigital2Address,
                .port = 0,
                .pin = 5
            },
            [ROTARY_POS_4] = {
                .expanderAddress = mainDigital2Address,
                .port = 0,
                .pin = 7
            },
            [ROTARY_POS_5] = {
                .expanderAddress = mainDigital1Address,
                .port = 0,
                .pin = 1
            },
            [ROTARY_POS_6] = {
                .expanderAddress = mainDigital1Address,
                .port = 0,
                .pin = 3
            },
            [ROTARY_POS_7] = {
                .expanderAddress = mainDigital1Address,
                .port = 0,
                .pin = 5
            },
            [ROTARY_POS_8] = {
                .expanderAddress = mainDigital1Address,
                .port = 0,
                .pin = 7
            }
        }
    },
    [EXP_ROTARY_2] = {
        .pins = {
            [ROTARY_POS_1] = {
                .expanderAddress = mainDigital2Address,
                .port = 0,
                .pin = 0
            },
            [ROTARY_POS_2] = {
                .expanderAddress = mainDigital2Address,
                .port = 0,
                .pin = 2
            },
            [ROTARY_POS_3] = {
                .expanderAddress = mainDigital2Address,
                .port = 0,
                .pin = 4
            },
            [ROTARY_POS_4] = {
                .expanderAddress = mainDigital2Address,
                .port = 0,
                .pin = 6
            },
            [ROTARY_POS_5] = {
                .expanderAddress = mainDigital1Address,
                .port = 0,
                .pin = 0
            },
            [ROTARY_POS_6] = {
                .expanderAddress = mainDigital1Address,
                .port = 0,
                .pin = 2
            },
            [ROTARY_POS_7] = {
                .expanderAddress = mainDigital1Address,
                .port = 0,
                .pin = 4
            },
            [ROTARY_POS_8] = {
                .expanderAddress = mainDigital1Address,
                .port = 0,
                .pin = 6
            }
        }
    }
};

/** @brief Array of bytes containing data for the pins of each digital GPIO expander */
uint8_t mainDigital1Data[2];
uint8_t mainDigital2Data[2];
uint8_t daughterDigitalData[1];

/** @brief GPIO expander update 100 Hz priority. */
static const uint32_t expanderUpdate100Hz_priority = 4;

/** @brief GPIO expander update 100 Hz TX period (milliseconds). */
static const TickType_t expanderUpdate100Hz_period_ms = 10;

/** @brief GPIO expander update 100 Hz TX task. */
static cmr_task_t expanderUpdate100Hz_task;

void getExpanderData(uint16_t addr, uint8_t cmd, uint8_t *data, size_t len)
{
    cmr_i2cTX(&i2c, addr, &cmd, 1, i2cTimeout_ms);
    cmr_i2cRX(&i2c, addr, data, len, i2cTimeout_ms);
}

static void expanderUpdate100Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        getExpanderData(
            mainDigital1Address, PCA9555_INPUT_PORT_0,
            mainDigital1Data, 2
        );
        getExpanderData(
            mainDigital2Address, PCA9555_INPUT_PORT_0,
            mainDigital2Data, 2
        );
        getExpanderData(
            daughterDigitalAddress, PCA9554_INPUT_PORT,
            daughterDigitalData, 1
        );

        vTaskDelayUntil(&lastWakeTime, expanderUpdate100Hz_period_ms);
    }
}

static bool getDataBitFromConfig(expanderPinConfig_t config)
{
    uint16_t addr = config.expanderAddress;

    // Select correct expander data based on config
    uint8_t *data;
    if (addr == mainDigital1Address) 
        data = mainDigital1Data;
    else if (addr == mainDigital2Address)
        data = mainDigital2Data;
    else if (addr == daughterAnalogAddress)
        data = daughterDigitalData;
    else
        return false;

    // Mask out bit corresponding to pin
    uint8_t mask = 1 << config.pin;
    return data[config.port] & mask;
}

bool expanderGetButtonPressed(expanderButton_t button)
{
    expanderPinConfig_t buttonConfig = buttons[button];
    return getDataBitFromConfig(buttonConfig);
}

expanderRotaryPosition_t expanderGetRotary(expanderRotary_t rotary)
{
    expanderRotaryConfig_t rotaryConfig = rotaries[rotary];

    for (size_t pos = ROTARY_POS_1; pos < ROTARY_POS_LEN; pos++)
    {
        if (getDataBitFromConfig(rotaryConfig.pins[pos]))
        {
            return pos;
        }
    }
    return ROTARY_POS_INVALID;
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
    
    cmr_taskInit(
        &expanderUpdate100Hz_task,
        "GPIO Expander Update 100Hz",
        expanderUpdate100Hz_priority,
        expanderUpdate100Hz,
        NULL
    );

}



