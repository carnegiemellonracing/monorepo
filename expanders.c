/**
 * @file expanders.c
 * @brief GPIO expanders implementation
 *
 * GPIO expanders in use:
 *      Main Board Digital 1:   PCF8574       (ROT_{0..2},ROT_SEL,BUTTON{0..3})
 *      Steering Wheel SPI  :   ADS7038IRTET    (PUSH{0..3}, PPOS{0,1})
 * @author Carnegie Mellon Racing
 */

#include "expanders.h"  // Interface to implement

#include <CMR/gpio.h>   // GPIO interface
#include <CMR/tasks.h>  // Task interface

#include "ADS7038.h"  //SPI interface
#include "PCF8574.h"  //I2c interface

#define SYSTICKCLOCK 96000000ULL
#define SYSTICKPERUS (SYSTICKCLOCK / 1000000UL)

// These addresses do not matter, they can be any unique number
/** @brief Daughter Board Digital expander Identifier */
static const uint16_t daughterDigitalAddress = 0x1;

/** @brief Daughter Board Analog expander Identifier */
static const uint16_t daughterAnalogAddress = 0x2;

/**
 * @brief Array of expander button pin configs
 *
 */
static expanderPinConfig_t buttons[EXP_BUTTON_LEN] = {
    [EXP_DASH_BUTTON_0] = {
        .expanderAddress = PCF8574_EXPANDER_ADDR,
        .port = 0,
        .pin = 4 },
    [EXP_DASH_BUTTON_1] = { .expanderAddress = PCF8574_EXPANDER_ADDR, .port = 0, .pin = 5 },
    [EXP_DASH_BUTTON_2] = { .expanderAddress = PCF8574_EXPANDER_ADDR, .port = 0, .pin = 6 },
    [EXP_DASH_BUTTON_3] = { .expanderAddress = PCF8574_EXPANDER_ADDR, .port = 0, .pin = 7 },
    [EXP_WHEEL_BUTTON_0] = { .expanderAddress = daughterDigitalAddress, .port = 0, .pin = 5 },
    [EXP_WHEEL_BUTTON_1] = { .expanderAddress = daughterDigitalAddress, .port = 0, .pin = 2 },
    [EXP_WHEEL_BUTTON_2] = { .expanderAddress = daughterDigitalAddress, .port = 0, .pin = 3 },
    [EXP_WHEEL_BUTTON_3] = { .expanderAddress = daughterDigitalAddress, .port = 0, .pin = 4 }
};


static rotaryPinConfig_t rotaryPositions[ROTARY_POS_LEN] = {
     [ROTARY_POS_0] = { .pin = 0b111},
     [ROTARY_POS_1] = { .pin = 0b011},
     [ROTARY_POS_2] = { .pin = 0b101},
     [ROTARY_POS_3] = { .pin = 0b001},
     [ROTARY_POS_4] = { .pin = 0b110},
     [ROTARY_POS_5] = { .pin = 0b010},
     [ROTARY_POS_6] = { .pin = 0b100},
     [ROTARY_POS_7] = { .pin = 0b000},
};

/**
 * @brief Array of expander clutch pin configs
 *
 */
static expanderPinConfig_t clutchPaddles[EXP_CLUTCH_LEN] = {
    [EXP_CLUTCH_LEFT] = {
        .expanderAddress = daughterAnalogAddress,
        .port = 0,
        .pin = 0 },
    [EXP_CLUTCH_RIGHT] = { .expanderAddress = daughterAnalogAddress, .port = 0, .pin = 1 }
};

/** @brief Array of bytes containing data for the pins of each digital GPIO expander */
// Number of elements in the array corresponds to number of ports for GPIO
uint8_t mainDigital1Data[1];
uint8_t daughterDigitalData[1];
uint16_t daughterAnalogData[EXP_CLUTCH_LEN];

/** @brief GPIO expander update 100 Hz priority. */
static const uint32_t expanderUpdate100Hz_priority = 7;

/** @brief GPIO expander update 100 Hz TX period (milliseconds). */
static const TickType_t expanderUpdate100Hz_period_ms = 10;

/** @brief GPIO expander update 100 Hz TX task. */
static cmr_task_t expanderUpdate100Hz_task;

bool checkStatus(int status) {
    return status == 0;
}

static int getExpanderData(uint16_t addr, void *data) {
    if (addr == PCF8574_EXPANDER_ADDR) {
        return PCF8574_expanderRead(data);
    }
    if (addr == daughterDigitalAddress) {
        uint8_t temp[3];
        int status = ADS7038_read(GPI_VALUE_REG, temp);
        *((uint8_t *)data) = temp[0];
        return status;
    }
    if (addr == daughterAnalogAddress) {
        return ADS7038_adcManualRead(data);
    }
    return -1;
}

static int updateExpanderDataDaughter() {
    // don't really need status but returning anyway

    int status = getExpanderData(
        daughterDigitalAddress,
        daughterDigitalData);

    if (checkStatus(status)) {
        status |= getExpanderData(
            daughterAnalogAddress,
            daughterAnalogData);
    }
    return status;
}

static int updateExpanderDataMain() {
    return getExpanderData(
        PCF8574_EXPANDER_ADDR,
        mainDigital1Data);
}

// delay has to constant expression
static inline void __attribute__((always_inline)) delayus(unsigned delay) {
    uint32_t ticks = SYSTICKPERUS * delay;
    uint32_t start_tick = SysTick->VAL;

    while (SysTick->VAL - start_tick < ticks);
}

void resetClock() {
    GPIO_InitTypeDef pinConfig = { // clock
                                   .Pin = GPIO_PIN_8,
                                   .Mode = GPIO_MODE_OUTPUT_OD,
                                   .Pull = GPIO_NOPULL,
                                   .Speed = GPIO_SPEED_FREQ_VERY_HIGH
    };

    HAL_GPIO_Init(GPIOA, &pinConfig);

    pinConfig.Pin = GPIO_PIN_4;  // data
    pinConfig.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOB, &pinConfig);

    for (int i = 0; i < 10; i++) {
        HAL_GPIO_WritePin(  // clock
            GPIOA, GPIO_PIN_8,
            GPIO_PIN_RESET);
        delayus(5);
        HAL_GPIO_WritePin(  // data
            GPIOA, GPIO_PIN_8,
            GPIO_PIN_SET);
        delayus(5);
    }

    pinConfig.Pin = GPIO_PIN_8;  // clock
    pinConfig.Mode = GPIO_MODE_AF_OD;
    pinConfig.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &pinConfig);

    pinConfig.Pin = GPIO_PIN_4;  // data
    pinConfig.Alternate = GPIO_AF9_I2C3;
    HAL_GPIO_Init(GPIOB, &pinConfig);
}

static bool getPinValueFromConfig(expanderPinConfig_t config) {
    uint16_t addr = config.expanderAddress;
    uint8_t mask = 1 << config.pin;
    // Select correct expander data based on config
    uint8_t *data = NULL;
    if (addr == PCF8574_EXPANDER_ADDR) {
        data = mainDigital1Data;
    } else if (addr == daughterAnalogAddress) {
        // Analog data is 16 bits, special case
        uint16_t *data16 = daughterAnalogData;
        configASSERT(data16 != NULL);
        return (~(data16[config.port])) & mask;

    } else if (addr == daughterDigitalAddress) {
        data = daughterDigitalData;
    } else {
        return false;
    }

    // Mask out bit corresponding to pin
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
bool expanderGetButtonPressed(expanderButton_t button) {
    expanderPinConfig_t buttonConfig = buttons[button];
    // Buttons are active low, negate for active high
    return getPinValueFromConfig(buttonConfig);
}

expanderRotaryPosition_t expanderGetRotary(bool select) {
    uint8_t pin = 0;
    PCF8574_expanderWrite(I2C_ROTSEL, select);
    PCF8574_expanderRead(&pin);
    pin &= 0b111;
    for (size_t pos = ROTARY_POS_0; pos < ROTARY_POS_LEN; pos++)
    {
        if (rotaryPositions[pos].pin == pin)
        {
            return pos;
        }
    }
    return ROTARY_POS_INVALID;
}

uint32_t expanderGetClutch(expanderClutch_t clutch) {
    return daughterAnalogData[clutch];
}

static void expanderUpdate100Hz(void *pvParameters) {
    (void)pvParameters;  // Placate compiler.
    int status = 0;
    int badStatusCount = 0;

    TickType_t lastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&lastWakeTime, 1000);
    PCF8574Init();
    // 0 == no issue
    int possibleDisconnected = ADS7038Init();

    while (1) {
        status = 0;
        status |= updateExpanderDataMain();

        // dont check status of daughter board, if steering is removed, DIM should still work
        if (possibleDisconnected != 0) {
            possibleDisconnected = ADS7038Init();
        } else {
            updateExpanderDataDaughter();
        }

        vTaskDelayUntil(&lastWakeTime, expanderUpdate100Hz_period_ms);

        if (checkStatus(status)) {
            badStatusCount = 0;  // Clear Counter
        } else {
            badStatusCount += 1;
        }
        // if we've seen a badStatus for more than 4 times,
        // reset clock and delay for a while so that the timed out function finishes.
        if (badStatusCount > 4) {
            lastWakeTime = xTaskGetTickCount();
            vTaskDelayUntil(&lastWakeTime, 100);
            resetClock();
        }
    }
}
/**
 * @brief Initializes the GPIO expander interface.
 */
void expandersInit(void) {
    cmr_taskInit(
        &expanderUpdate100Hz_task,
        "GPIO Expander Update 100Hz",
        expanderUpdate100Hz_priority,
        expanderUpdate100Hz,
        NULL);
}
