/**
 * @file expanders.c
 * @brief GPIO expanders implementation
 *
 * GPIO expanders in use:
 *      Main Board Digital 1:   PCF8574PW       (ROT_{0..2},ROT_SEL,BUTTON{0..3})
 *      Steering Wheel SPI  :   ADS7038IRTET    (PUSH{0..3}, PPOS{0,1})
 *      Old:
 *      Daughter Board Digital: PCA9554PW,118   (PUSH{1..3})
 *      Daughter Board Analog:  AD5593RBRUZ     (CLUTCH{1,2})
 * @author Carnegie Mellon Racing
 */

#include <CMR/i2c.h>    // I2C interface
#include <CMR/spi.h>    // SPI interface
#include <CMR/tasks.h>  // Task interface

#include "spi.h"
#include "expanders.h"          // Interface to implement
#include "expandersPrivate.h"   // Private interface

#define NO_COMMAND 0

/** @brief Primary I2C interface */
static cmr_i2c_t i2c;

/** @brief SPI Interface*/
static cmr_spi_t spi;

/** @brief DIM I2C address */
static const uint32_t ownAddress = 0x00; // 0x00 = 0b0000000

// TODO: Fix addresses
/** @brief Main Board Digital 1 expander I2C address */
static const uint16_t mainDigital1Address = 0x20; // 0x23 = 0b010_0000

/** @brief Daughter Board Digital expander I2C address */
static const uint16_t daughterDigitalAddress = 0x25; // 0x25 = 0b010_0101

/** @brief Daughter Board Analog expander I2C address */
static const uint16_t daughterAnalogAddress = 0x11; // 0x11 = 0b0010001

/** @brief I2C Timeout (milliseconds). */
static const uint32_t i2cTimeout_ms = 1;

/**
 * @brief Array of expander button pin configs
 *
 */
static expanderPinConfig_t buttons[EXP_BUTTON_LEN] = {
    [EXP_DASH_BUTTON_0] = {
        .expanderAddress = mainDigital1Address,
        .port = 0,
        .pin = 4
    },
    [EXP_DASH_BUTTON_1] = {
        .expanderAddress = mainDigital1Address,
        .port = 0,
        .pin = 5
    },
    [EXP_DASH_BUTTON_2] = {
        .expanderAddress = mainDigital1Address,
        .port = 0,
        .pin = 6
    },
    [EXP_DASH_BUTTON_3] = {
        .expanderAddress = mainDigital1Address,
        .port = 0,
        .pin = 7
    },
    [EXP_WHEEL_BUTTON_0] = {
        .expanderAddress = daughterDigitalAddress,
        .port = 0,
        .pin = 5
    },
    [EXP_WHEEL_BUTTON_1] = {
        .expanderAddress = daughterDigitalAddress,
        .port = 0,
        .pin = 2
    },
    [EXP_WHEEL_BUTTON_2] = {
        .expanderAddress = daughterDigitalAddress,
        .port = 0,
        .pin = 3
    },
    [EXP_WHEEL_BUTTON_3] = {
        .expanderAddress = daughterDigitalAddress,
        .port = 0,
        .pin = 4
    }
};

/**
 * @brief Array of expander rotary pin configs
 *
 */
static expanderPinConfig_t rotaries[EXP_ROTARY_LEN] = {
    [EXP_ROTARY_0] = {
        .expanderAddress = mainDigital1Address,
        .port = 0,
        .pin = 2
    },
    [EXP_ROTARY_1] = {
        .expanderAddress = mainDigital1Address,
        .port = 0,
        .pin = 1

    },
    [EXP_ROTARY_2] = {
        .expanderAddress = mainDigital1Address,
        .port = 0,
        .pin = 0
    }
};

static expanderPinConfig_t EXP_ROTARY_SEL = {
    .expanderAddress = mainDigital1Address,
    .port = 0,
    .pin  = 3
};


/**
 * @brief Array of expander clutch pin configs
 *
 */
static expanderPinConfig_t clutchPaddles[EXP_CLUTCH_LEN] = {
    [EXP_CLUTCH_1] = {
        .expanderAddress = daughterAnalogAddress,
        .port = 0,
        .pin = 0
    },
    [EXP_CLUTCH_2] = {
        .expanderAddress = daughterAnalogAddress,
        .port = 0,
        .pin = 1
    }
};

/** @brief Array to store target LED states */
// bool ledTargets[EXP_LED_LEN];

/** @brief Array of bytes containing data for the pins of each digital GPIO expander */
// Number of elements in the array corresponds to number of ports for GPIO
uint8_t mainDigital1Data[8];
// uint8_t mainDigital2Data[1];
uint8_t daughterDigitalData[1];
// TODO: Look into switching to uint16_t and reverse byte order
uint8_t daughterAnalogData[2 * EXP_CLUTCH_LEN];

/** @brief GPIO expander update 100 Hz priority. */
static const uint32_t expanderUpdate100Hz_priority = 7;

/** @brief GPIO expander update 100 Hz TX period (milliseconds). */
static const TickType_t expanderUpdate100Hz_period_ms = 10;

/** @brief GPIO expander update 100 Hz TX task. */
static cmr_task_t expanderUpdate100Hz_task;


bool checkStatus(int status){
    return status == 0;
}

static int getExpanderData(uint16_t addr, uint8_t cmd, uint8_t *data, size_t len)
{
    if (addr == mainDigital1Address)
    {
        int status = cmr_i2cTX(&i2c, addr, &cmd, 1, i2cTimeout_ms);
        if (status < 0)
            return -1;
        return cmr_i2cRX(&i2c, addr, data, len, i2cTimeout_ms);
    }
    if (addr == daughterDigitalAddress){
        *data = ADS7038_read(cmd);
        return 0; // TODO fix this
    }
    if (addr == daughterAnalogAddress){
        ADS7038_adcManualRead(data);
        return 0;
    }
    return -1; // Invalid Address
}

static int updateExpanderDataDaughter(){
    // don't really need status but returning anyway

    int status = getExpanderData(
         daughterDigitalAddress, NO_COMMAND,
         daughterDigitalData, 1
     );

    if (!checkStatus(status)) return status;

    status |= getExpanderData(
         daughterAnalogAddress, NO_COMMAND,
         daughterAnalogData, 2 * EXP_CLUTCH_LEN
     );

    return status;
}

static int updateExpanderDataMain()
{
    int status = getExpanderData(
        mainDigital1Address, NULL,
        mainDigital1Data, 2
    );

    return status;
}

/**
 * @brief Checks current LED state and updates if different from `ledTargets`
 *
 * Not wholly generic since it's too complicated,
 * so it uses the fact that all LEDs are on Main Digital 2 Port 1
 *
 */
// static void checkLEDState()
// {
//     uint8_t targetMask = 0;
//     uint8_t targetState = 0;

//     for (size_t l = EXP_LED_1; l < EXP_LED_LEN; l++)
//     {
//         uint8_t pin = leds[l].pin;
//         targetMask |= 1 << pin;
//         targetState |= ledTargets[l] << pin;
//     }

//     // See note above about non-generic code
//     uint8_t currentState = mainDigital2Data[0];
//     if ((currentState & targetMask) != targetState)
//     {
//         uint8_t cmd[2] = {
//             PCA9554_OUTPUT_PORT,
//             targetState
//         };

//         cmr_i2cTX(
//             &i2c,
//             mainDigital2Address, cmd,
//             sizeof(cmd) / sizeof(cmd[0]),
//             i2cTimeout_ms
//         );
//     }
// }


static int configAnalogADCDaughter(){
    // Daughter Board Analog expander has ADC inputs on pins 0 and 1
    uint8_t daughterAnalogADCConfig[3] = {
        AD5593R_CTRL_REG_ADC_CONFIG,
        0x00,
        0x03    // 0b00000011
    };
    // Set REP bit so ADC conversions are repeated (see datasheet)
    uint8_t daughterAnalogADCSequence[3] = {
        AD5593R_CTRL_REG_ADC_SEQ,
        0x02,   // 0b00000010 (REP bit)
        0x03    // 0b00000011
    };

    int status;

    status = cmr_i2cTX(
         &i2c,
         daughterAnalogAddress, daughterAnalogADCConfig,
         sizeof(daughterAnalogADCConfig) / sizeof(daughterAnalogADCConfig[0]),
         i2cTimeout_ms
     );

    if (!checkStatus(status)) return status;

    status |= cmr_i2cTX(
         &i2c,
         daughterAnalogAddress, daughterAnalogADCSequence,
         sizeof(daughterAnalogADCSequence) / sizeof(daughterAnalogADCSequence[0]),
         i2cTimeout_ms
     );

    return status;
}

#define SYSTICKCLOCK 96000000ULL
#define SYSTICKPERUS (SYSTICKCLOCK / 1000000UL)

// delay has to constant expression
static void inline __attribute__((always_inline)) delayus(unsigned delay)
{
    uint32_t ticks = SYSTICKPERUS * delay;
    uint32_t start_tick = SysTick -> VAL;

    while(SysTick -> VAL - start_tick < ticks);
}

void resetClock() {
    GPIO_InitTypeDef pinConfig = { //clock
        .Pin = GPIO_PIN_8,
        .Mode = GPIO_MODE_OUTPUT_OD,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH
    };

    HAL_GPIO_Init(GPIOA, &pinConfig);


    pinConfig.Pin = GPIO_PIN_4; //data
    pinConfig.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOB, &pinConfig);

    for (int i = 0; i < 10; i++) {
        HAL_GPIO_WritePin( // clock
            GPIOA, GPIO_PIN_8,
            GPIO_PIN_RESET
        );
    	delayus(5);
        HAL_GPIO_WritePin( // data
            GPIOA, GPIO_PIN_8,
            GPIO_PIN_SET
        );
        delayus(5);
    }

    pinConfig.Pin = GPIO_PIN_8; //clock
    pinConfig.Mode = GPIO_MODE_AF_OD;
    pinConfig.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &pinConfig);

    pinConfig.Pin = GPIO_PIN_4; //data
    pinConfig.Alternate = GPIO_AF9_I2C3;
    HAL_GPIO_Init(GPIOB, &pinConfig);

    // i2cInitChain();
}

static int configDaughterDigital(){
    // Daughter Board Digital expander has all inputs
    uint8_t daughterDigitalConfig[2] = {
        PCA9554_CONFIG_PORT,
        0xFF
    };

    int status =cmr_i2cTX(
         &i2c,
         daughterDigitalAddress, daughterDigitalConfig,
         sizeof(daughterDigitalConfig) / sizeof(daughterDigitalConfig[0]),
         i2cTimeout_ms
     );

    return status;
}

static int configMainDigital1()
{
    // Main Board Digital 1 expander has all inputs on Ports 0 and 1
    uint8_t mainDigital1Config[3] = {
        //    PCA9555_CONFIG_PORT_0,
        NULL,
        0xFF,
        0xFF};
    //    // Main Board Digital 2 expander has all inputs except outputs on Pins 0 and 1
    //    uint8_t mainDigital2Config[2] = {
    //        PCA9554_CONFIG_PORT,
    //        0xFC    // 0b11111100
    //    };

    int status;
    status = cmr_i2cTX(
        &i2c,
        mainDigital1Address, mainDigital1Config,
        sizeof(mainDigital1Config) / sizeof(mainDigital1Config[0]),
        i2cTimeout_ms
    );
    return status;
}

static void expanderUpdate100Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    int status = configMainDigital1();

    status |= configDaughterDigital();

    status |= configAnalogADCDaughter();

    TickType_t lastWakeTime = xTaskGetTickCount();

    int badStatus_count = 0;

    while (1) {
        status = 0;

        // check status of each function
        if (checkStatus(status)){
            status |= updateExpanderDataMain();
        }

        // dont check status of daughter board, if steering is removed, DIM should still work
        configAnalogADCDaughter();
        // ADS7038Init();
        updateExpanderDataDaughter();

        // if (checkStatus(status)) checkLEDState();

        vTaskDelayUntil(&lastWakeTime, expanderUpdate100Hz_period_ms);

        if (checkStatus(status)){
            badStatus_count = 0;
        }
        else{
            badStatus_count += 1;
        }
        // if we've seen a badStatus for more than 4 times,
        // reset clock and delay for a while so that the timed out function finishes.
        if (badStatus_count > 4) {
            lastWakeTime = xTaskGetTickCount();
            vTaskDelayUntil(&lastWakeTime, 100);
            resetClock();

            // config everything again since clock reset
            configMainDigital1and2();
            configDaughterDigital();
            configAnalogADCDaughter();
        }


    }
}

static bool getPinValueFromConfig(expanderPinConfig_t config)
{
    uint16_t addr = config.expanderAddress;

    // Select correct expander data based on config
    uint8_t *data = NULL;
    if (addr == mainDigital1Address){
        data = mainDigital1Data;
    }
//    else if (addr == mainDigital2Address)
//        data = mainDigital2Data;
    else if (addr == daughterAnalogAddress){
        data = daughterAnalogData;

    }
    else if (addr == daughterDigitalAddress){
        data = daughterDigitalData;
    }
    else
        return false;

    // Mask out bit corresponding to pin
    uint8_t mask = 1 << config.pin;
    configASSERT(data != NULL);
    // ~data because sense lines are active low
    return (~(data[config.port])) & mask;
}

/**
 * @brief Get expander button states
 *
 * @param button
 * @return true
 * @return false
 */
bool expanderGetButtonPressed(expanderButton_t button)
{
    expanderPinConfig_t buttonConfig = buttons[button];
    // Buttons are active low, negate for active high
    return getPinValueFromConfig(buttonConfig);
}

// expanderRotaryPosition_t expanderGetRotary(expanderRotary_t rotary)
// {
//     expanderRotaryConfig_t rotaryConfig = rotaries[rotary];

//     for (size_t pos = ROTARY_POS_1; pos < ROTARY_POS_LEN; pos++)
//     {
//         if (getPinValueFromConfig(rotaryConfig.pins[pos]))
//         {
//             return pos;
//         }
//     }
//     return ROTARY_POS_INVALID;
// }
// TODO: scale down to uint8_t and send over CAN
uint32_t expanderGetClutch(expanderClutch_t clutch)
{
    return daughterAnalogData[clutch*2];
}
//void expanderSetLED(expanderLED_t led, bool isOn)
//{
//    ledTargets[led] = isOn;
//}

/**
 * @brief Initializes the GPIO expander interface.
 */
void expandersInit(void) {
    // I2C initialization
    cmr_i2cInit(
        &i2c, I2C3,
        I2C_CLOCK_HI, ownAddress,
        GPIOA, GPIO_PIN_8,
        GPIOB, GPIO_PIN_4
    );

	if (HAL_I2CEx_ConfigAnalogFilter(&(i2c.handle), I2C_ANALOGFILTER_DISABLE) != HAL_OK)
		{
		cmr_panic("Failed to disable analog filter");
		}
		/** Configure Digital filter
		*/
	if (HAL_I2CEx_ConfigDigitalFilter(&(i2c.handle), 15) != HAL_OK)
		{
		  cmr_panic("Failed to enable digital filter");
		}

    cmr_taskInit(
        &expanderUpdate100Hz_task,
        "GPIO Expander Update 100Hz",
        expanderUpdate100Hz_priority,
        expanderUpdate100Hz,
        NULL
    );

}



