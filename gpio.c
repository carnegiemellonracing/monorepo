/**
 * @file gpio.c
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface
#include <FreeRTOS.h>       // FreeRTOS API

#include "state.h"
#include "gpio.h"       // Interface to implement
#include "expanders.h"  // GPIO expanders interface


/** @brief Array to store target LED states */
// bool ledTargets[EXP_LED_LEN];

/**
 * @brief Board-specific pin configuration.
 *
 * Replace/add more pin configurations here as appropriate. Each enumeration
 * value of `gpio_t` should get a configuration.
 *
 * @see `stm32f4xx_hal_gpio.h` for various initialization values.
 */
static const cmr_gpioPinConfig_t gpioPinConfigs[GPIO_LEN] = {
    [GPIO_LED_0] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_6,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_LED_1] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_5,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_LED_2] = { // not in the schematic
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_4,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_LED_AMS] = {
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_11,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_LED_IMD] = {
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_9,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_LED_BSPD] = {
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_10,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
	[GPIO_PTT_N] = {
	        .port = GPIOC,
	        .init = {
	            .Pin = GPIO_PIN_8,
	            .Mode = GPIO_MODE_OUTPUT_PP,
	            .Pull = GPIO_NOPULL,
	            .Speed = GPIO_SPEED_FREQ_LOW
	        }
	},
    [GPIO_PD_N] = {
            .port = GPIOA,
            .init = {
                .Pin = GPIO_PIN_5,
	            .Mode = GPIO_MODE_OUTPUT_PP,
	            .Pull = GPIO_NOPULL,
	            .Speed = GPIO_SPEED_FREQ_LOW
            }
    }
};

static const uint32_t buttonsInput_priority = 4;

/** @brief Button input task period (milliseconds). */
static const TickType_t buttonsInput_period = 10;

/** @brief Button input task task. */
static cmr_task_t buttonsInput_task;

/** @brief Current regen step */
uint32_t regenStep = 0;

#define BUTTON_DEBOUNCE_TIME 250

static expanderButtonEvent_t expanderButtons[EXP_BUTTON_LEN] = {
    [EXP_DASH_BUTTON_0] = {
        .buttonState = true,
        .setAction = &downButton,
        .lastPressed = 0,
        .debounce = BUTTON_DEBOUNCE_TIME,
    },
    [EXP_DASH_BUTTON_1] = {
        .buttonState = true,
        .setAction = &upButton,
        .lastPressed = 0,
        .debounce = BUTTON_DEBOUNCE_TIME,
    },
    [EXP_DASH_BUTTON_2] = {
        .buttonState = true,
        .setAction = &rightButton,
        .lastPressed = 0,
        .debounce = BUTTON_DEBOUNCE_TIME,
    },
    [EXP_DASH_BUTTON_3] = {
        .buttonState = true,
        .setAction = &leftButton,
        .lastPressed = 0,
        .debounce = BUTTON_DEBOUNCE_TIME,
    },
    [EXP_WHEEL_BUTTON_0] = {
        .buttonState = false,
        .setAction = &actionOneButton,
        .lastPressed = 0,
        .debounce = BUTTON_DEBOUNCE_TIME,
    },
    [EXP_WHEEL_BUTTON_1] = {
        .buttonState = true,
        .setAction = &actionTwoButton,
        .lastPressed = 0,
        .debounce = BUTTON_DEBOUNCE_TIME,
    },
    [EXP_WHEEL_BUTTON_2] = {
        .buttonState = true,
        .setAction = &drsButton,
        .lastPressed = 0,
        .debounce = BUTTON_DEBOUNCE_TIME,
    },
    [EXP_WHEEL_BUTTON_3] = {
        .buttonState = true,
        .setAction = &actionZeroButton,
        .lastPressed = 0,
        .debounce = BUTTON_DEBOUNCE_TIME,
    }
};

/**
 * @brief Handles button actions.
 *
 * @param pvParameters Ignored.
 */
static void buttonsInput(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    TickType_t currentTime;
    bool select = 0;
    while (1) {
        /* if vsm has changed state unexpectedly we
         * need to adjust our req to still be valid */
        if(!stateVSMReqIsValid(stateGetVSM(), stateGetVSMReq()))
        {
            updateReq();
        }

        currentTime = xTaskGetTickCount();

        // updating each button and updating states according to button presses
        for (expanderButton_t i = EXP_DASH_BUTTON_0; i < EXP_BUTTON_LEN; i ++) {
            bool currState = expanderGetButtonPressed(i);
            expanderButtonEvent_t *currButton = &expanderButtons[i];

            if (currButton->buttonState != currState &&
                currentTime - currButton->lastPressed > currButton->debounce) {
                (*(currButton->setAction))(currState);
                currButton->lastPressed = currentTime;
                currButton->buttonState = currState;
            }
        }

            expanderRotaryPosition_t rotaryPos = expanderGetRotary(select);
            rotaries(select,rotaryPos);
            if(!select) {
            	stateGearSwitch(rotaryPos);
            }
            select = !select;

        vTaskDelayUntil(&lastWakeTime, buttonsInput_period);
    }

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
//     if ((currentState & targetMask) != targetState)
//     {
//         // uint8_t cmd[2] = {
//         //     PCA9554_OUTPUT_PORT,
//         //     targetState
//         // };

//     }
// }

// void expanderSetLED(expanderLED_t led, bool isOn)
// {
//    ledTargets[led] = isOn;
// }



/**
 * @brief Initializes the GPIO interface.
 */
void gpioInit(void) {
    cmr_gpioPinInit(
        gpioPinConfigs, sizeof(gpioPinConfigs) / sizeof(gpioPinConfigs[0])
    );
     cmr_taskInit(
          &buttonsInput_task,
          "buttonsInput",
          buttonsInput_priority,
          buttonsInput,
          NULL
      );
}

