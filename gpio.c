/**
 * @file gpio.c
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface
#include <FreeRTOS.h>       // FreeRTOS API
#include <task.h>           // Task interface
#include <queue.h>          // Queue interface

#include "state.h"      // state handling stuff
#include "gpio.h"       // Interface to implement
#include "expanders.h"   // GPIO expanders interface

/**
 * @brief Board-specific pin configuration.
 *
 * Replace/add more pin configurations here as appropriate. Each enumeration
 * value of `gpio_t` should get a configuration.
 *
 * @see `stm32f4xx_hal_gpio.h` for various initialization values.
 */
static const cmr_gpioPinConfig_t gpioPinConfigs[GPIO_LEN] = {
    [GPIO_LED_STATUS] = { // not in the schematic
        .port = GPIOC,
        .init = { 
            .Pin = GPIO_PIN_0,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_LED_IMD] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_9,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_LED_AMS] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_8,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_LED_BSPD] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_7,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_SS_MODULE] = {
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_12,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
        }
    },
    [GPIO_SS_COCKPIT] = {
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_11,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
        }
    },
    [GPIO_SS_FRHUB] = {
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_10,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
        }
    },
    [GPIO_SS_INERTIA] = {
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_9,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
        }
    },
    [GPIO_SS_FLHUB] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_6,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
        }
    },
    [GPIO_SS_BOTS] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_10,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
        }
    },
    [GPIO_PD_N] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_3,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    }
};

/** @brief Button input task priority. */
static const uint32_t buttonsInput_priority = 4;

/** @brief Button input task period (milliseconds). */
static const TickType_t buttonsInput_period = 10;

/** @brief Button input task task. */
static cmr_task_t buttonsInput_task;

/** @brief AE/DRS button value */
bool drsButtonPressed;
/** @brief Action 1 button value */
bool action1ButtonPressed;
/** @brief Action 2 button value */
bool action2ButtonPressed;

/** @brief Current regen step */
unsigned int regenStep = 0;

static void actionOneButtonAction(bool pressed)
{
    action1ButtonPressed = pressed;
    actionOneButton(pressed);
}

static void actionTwoButtonAction(bool pressed)
{
    action2ButtonPressed = pressed;
    actionTwoButton(pressed);
}

static void drsButtonAction(bool pressed)
{
    drsButtonPressed = pressed;
}

#define BUTTON_DEBOUNCE_TIME 200

static expanderButtonEvent_t expanderButtons[EXP_BUTTON_LEN] = {
    [EXP_DASH_BUTTON_1] = {
        .buttonState = false,
        .setAction = &stateVSMDownButton,
        .lastPressed = 0,
        .debounce = BUTTON_DEBOUNCE_TIME * 5, // State down has a long debounce time so don't accidentally drop out of HVEN
    },
    [EXP_DASH_BUTTON_2] = {
        .buttonState = false,
        .setAction = &stateGearDownButton,
        .lastPressed = 0,
        .debounce = BUTTON_DEBOUNCE_TIME,
    },
    [EXP_DASH_BUTTON_3] = {
        .buttonState = false,
        .setAction = &stateVSMUpButton,
        .lastPressed = 0,
        .debounce = BUTTON_DEBOUNCE_TIME,
    },
    [EXP_DASH_BUTTON_4] = {
        .buttonState = false,
        .setAction = &stateGearUpButton,
        .lastPressed = 0,
        .debounce = BUTTON_DEBOUNCE_TIME,
    },
    [EXP_WHEEL_BUTTON_1] = {
        .buttonState = false,
        .setAction = &actionOneButtonAction,
        .lastPressed = 0,
        .debounce = BUTTON_DEBOUNCE_TIME,
    },
    [EXP_WHEEL_BUTTON_2] = {
        .buttonState = false,
        .setAction = &drsButtonAction,
        .lastPressed = 0,
        .debounce = BUTTON_DEBOUNCE_TIME,
    },
    [EXP_WHEEL_BUTTON_3] = {
        .buttonState = false,
        .setAction = &actionTwoButtonAction,
        .lastPressed = 0,
        .debounce = BUTTON_DEBOUNCE_TIME,
    }
};

static expanderRotaryEvent_t rotaries[EXP_ROTARY_LEN] = {
    [EXP_ROTARY_1] = {
        .position = ROTARY_POS_INVALID,
        .setAction = &stateDrsModeSwitch
    },
    [EXP_ROTARY_2] = {
        .position = ROTARY_POS_INVALID,
        .setAction = &stateRotary2Switch
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

    while (1) {
        /* if vsm has changed state unexpectedly we
         * need to adjust our req to still be valid */
        if(!stateVSMReqIsValid(stateGetVSM(), stateGetVSMReq()))
        {
            updateReq();
        }

        currentTime = xTaskGetTickCount();

        // updating each button and updating states according to button presses
        for (expanderButton_t i = EXP_DASH_BUTTON_1; i < EXP_BUTTON_LEN; i ++) {
            bool currState = expanderGetButtonPressed(i);
            expanderButtonEvent_t *currButton = &expanderButtons[i];

            if (currButton->buttonState != currState && 
                currentTime - currButton->lastPressed > currButton->debounce) {
                (*(currButton->setAction))(currState);
                currButton->lastPressed = currentTime;  
                currButton->buttonState = currState;
            }
        }

        for (expanderRotary_t i = EXP_ROTARY_1; i < EXP_ROTARY_LEN; i++) {
            expanderRotaryPosition_t rotaryPos = expanderGetRotary(i);
            expanderRotaryEvent_t *currRotary =  &rotaries[i];

            if (rotaryPos != currRotary->position) {
                (*(currRotary->setAction))(rotaryPos);
                currRotary->position = rotaryPos;
            }
        }

        vTaskDelayUntil(&lastWakeTime, buttonsInput_period);
    }

}

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

