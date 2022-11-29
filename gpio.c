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

/** @brief Maximum number of button events in the queue. */
#define BUTTON_EVENTS_MAX 64


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
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_11,
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

/** @brief AE/DRS button value */
bool drsButtonPressed;
/** @brief Action 1 button value */
bool action1ButtonPressed;
/** @brief Action 2 button value */
bool action2ButtonPressed;

/** @brief Current regen step */
unsigned int regenStep = 0;


static expanderButtonEvent_t[EXP_BUTTON_LEN] expanderButtons = {
    [EXP_DASH_BUTTON_1] = {
        .button = EXP_DASH_BUTTON_1,
        .buttonState = expanderGetButtonPressed(EXP_DASH_BUTTON_1),
        .setAction = &actionFunc,
        .lastPressed = 0,
    },
    [EXP_DASH_BUTTON_2] = {
        .button = EXP_DASH_BUTTON_2,
        .buttonState = expanderGetButtonPressed(EXP_DASH_BUTTON_2),
        .setAction = &actionFunc,
        .lastPressed = 0,
    },
    [EXP_DASH_BUTTON_3] = {
        .button = EXP_DASH_BUTTON_3,
        .buttonState = expanderGetButtonPressed(EXP_DASH_BUTTON_3),
        .setAction = &actionFunc,
        .lastPressed = 0,
    },
    [EXP_DASH_BUTTON_4] = {
        .button = EXP_DASH_BUTTON_3,
        .buttonState = expanderGetButtonPressed(EXP_DASH_BUTTON_4),
        .setAction = &actionFunc,
        .lastPressed = 0,
    },
    [EXP_WHEEL_BUTTON_1] = {
        .button = EXP_DASH_BUTTON_3,
        .buttonState = expanderGetButtonPressed(EXP_WHEEL_BUTTON_1),
        .setAction = &actionFunc,
        .lastPressed = 0,
    },
    [EXP_WHEEL_BUTTON_2] = {
        .button = EXP_DASH_BUTTON_3,
        .buttonState = expanderGetButtonPressed(EXP_WHEEL_BUTTON_2),
        .setAction = &actionFunc,
        .lastPressed = 0,
    },
    [EXP_WHEEL_BUTTON_3] = {
        .button = EXP_DASH_BUTTON_3,
        .buttonState = expanderGetButtonPressed(EXP_WHEEL_BUTTON_3),
        .setAction = &actionFunc,
        .lastPressed = 0,
    }
}

/**
 * @brief Handles button events.
 *
 * @param pvParameters Ignored.
 */

static void buttonsInput_task(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    #define BUTTON_DEBOUNCE_TIME 200

    while (1) {
        currentTime = xTaskGetTickCount();
        // updating each button and updating states according to button presses
        for (expanderButton_t i = EXP_DASH_BUTTON_1; i < EXP_BUTTON_LEN; i ++){
            if (expanderGetButtonPressed(i)){
                if (expanderButtons[i].buttonState != expanderGetButtonPressed(i) && 
                    currentTime - expanderButtons[i].lastPressed > BUTTON_DEBOUNCE_TIME){
                    *expanderButtons[i].setAction(i);
                    expanderButtons[i].lastPressed = currentTime;  
                }
                
            }
            expanderButtons[i].buttonState = expanderGetButtonPressetd(i);
        } 

        vTaskDelayUntil(&lastWakeTime, buttonsInput_period);
    }

    
    // while (1) {

    //     volatile int value = cmr_gpioRead(GPIO_BUTTON_1);
    //     drsButtonPressed = value;
    //     value = cmr_gpioRead(GPIO_BUTTON_3);
    //     action1ButtonPressed = value;
    //     value = cmr_gpioRead(GPIO_BUTTON_8);
    //     action2ButtonPressed = value;

    //     /* if vsm has changed state unexpectedly we
    //      * need to adjust our req to still be valid */
    //     if(!stateVSMReqIsValid(stateGetVSM(), stateGetVSMReq()))
    //     {
    //         updateReq();
    //     }
        
    //     currentTime = xTaskGetTickCount();

	
    //     buttonEvent_t event;
    //     while (xQueueReceive(buttons.events.q, &event, 0) == pdTRUE) {

	// 		switch (event.pin) {
    //         	case GPIO_BUTTON_3:
    //         		if((currentTime - lastButtonPress) > BUTTON_DEBOUNCE_TIME) {
    //                     actionOneButton(event.pressed);
	// 				}
    //         		break;
    //         	case GPIO_BUTTON_8:
    //         		if((currentTime - lastButtonPress) > BUTTON_DEBOUNCE_TIME) {
    //                     actionTwoButton(event.pressed);
	// 				}
    //         		break;
    //             case GPIO_BUTTON_9:
    //             	if((currentTime - lastButtonPress) > BUTTON_DEBOUNCE_TIME) {
    //                     regenUpButton(event.pressed);
	// 				}
    //             	break;
    //             case GPIO_BUTTON_2:
    //             	if((currentTime - lastButtonPress) > BUTTON_DEBOUNCE_TIME) {
    //                     regenDownButton(event.pressed);
	// 				}
    //                 break;
    //             case GPIO_BUTTON_7:
    //                 stateVSMUpButton(event.pressed);
    //                 break;
    //             case GPIO_BUTTON_6:
    //                 /* Avoid accidental double clicks on state down button
    //                 (the transition back into hv_en takes a while) */
    //                 if((currentTime - lastButtonPress) > 1000) {
    //                     stateVSMDownButton(event.pressed);
    //                 }
    //                 break;
    //             case GPIO_BUTTON_4:
    //                 stateGearUpButton(event.pressed);
    //                 break;
    //             case GPIO_BUTTON_5:
    //             	stateGearDownButton(event.pressed);
    //                 break;
    //             default:
    //                 break;
    //         }
            
    //         // Record the time of the last button press
            // lastButtonPress = xTaskGetTickCount();
    //     }

    //     vTaskDelayUntil(&lastWakeTime, buttonsInput_period);
    // } 
}

/**
 * @brief Initializes the GPIO interface.
 */
void gpioInit(void) {
    cmr_gpioPinInit(
        gpioPinConfigs, sizeof(gpioPinConfigs) / sizeof(gpioPinConfigs[0])
    );

    
}

