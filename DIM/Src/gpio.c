/**
 * @file gpio.c
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#include "gpio.h"  // Interface to implement
#include "adc.h"

#include <CMR/gpio.h>   // GPIO interface
#include <stm32f4xx_hal.h>  // HAL interface

#include "state.h"

//bool gpioButtonStates[NUM_BUTTONS];

button_t buttonStates[NUM_BUTTONS];


gpio_t gpioButtonPins[NUM_BUTTONS] = {GPIO_BUTTON_UP, GPIO_BUTTON_DOWN, GPIO_BUTTON_LEFT,
                                    GPIO_BUTTON_RIGHT, GPIO_BUTTON_SW_LEFT, GPIO_BUTTON_SW_RIGHT};

static const uint32_t gpioReadButtons_priority = 5;

/** @brief Button input task task. */
static cmr_task_t gpioReadButtons_task;

static bool eabReq = false;

/**
 * @brief Board-specific pin configuration.
 *
 * Replace/add more pin configurations here as appropriate. Each enumeration
 * value of `gpio_t` should get a configuration.
 *
 * @see `stm32f4xx_hal_gpio.h` for various initialization values.
 */
// TODO: change GPIO pin configs based on new schematic
static const cmr_gpioPinConfig_t gpioPinConfigs[GPIO_LEN] = {

// // D_BUTTON_2
    // [GPIO_BUTTON_UP] = {
	// 	.port = GPIOC,
	// 	.init = {
	// 		.Pin = GPIO_PIN_10,
	// 		.Mode = GPIO_MODE_INPUT,
	// 		.Pull = GPIO_PULLUP,
	// 		.Speed = GPIO_SPEED_FREQ_LOW
	// 	}
	// },
	// // D_BUTTON_1
	// [GPIO_BUTTON_DOWN] = {
	// 	.port = GPIOB,
	// 	.init = {
	// 		.Pin = GPIO_PIN_5,
	// 		.Mode = GPIO_MODE_INPUT,
	// 		.Pull = GPIO_PULLUP,
	// 		.Speed = GPIO_SPEED_FREQ_LOW
	// 	}
	// },
	// // D_BUTTON_4
	// [GPIO_BUTTON_LEFT] = {
	// 	.port = GPIOC,
	// 	.init = {
	// 		.Pin = GPIO_PIN_0,
	// 		.Mode = GPIO_MODE_INPUT,
	// 		.Pull = GPIO_PULLUP,
	// 		.Speed = GPIO_SPEED_FREQ_LOW
	// 	}
	// },
	// // D_BUTTON_3
	// [GPIO_BUTTON_RIGHT] = {
	// 	.port = GPIOB,
	// 	.init = {
	// 		.Pin = GPIO_PIN_9,
	// 		.Mode = GPIO_MODE_INPUT,
	// 		.Pull = GPIO_PULLUP,
	// 		.Speed = GPIO_SPEED_FREQ_LOW
	// 	}
	// },
	// OLD DIM PINS	
	// D_BUTTON_2
    [GPIO_BUTTON_UP] = {
		.port = GPIOC,
		.init = {
			.Pin = GPIO_PIN_10,
			.Mode = GPIO_MODE_INPUT,
			.Pull = GPIO_PULLUP,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	},
	// D_BUTTON_1
	[GPIO_BUTTON_DOWN] = {
		.port = GPIOB,
		.init = {
			.Pin = GPIO_PIN_4,
			.Mode = GPIO_MODE_INPUT,
			.Pull = GPIO_PULLUP,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	},
	// D_BUTTON_4
	[GPIO_BUTTON_LEFT] = {
		.port = GPIOB,
		.init = {
			.Pin = GPIO_PIN_5,
			.Mode = GPIO_MODE_INPUT,
			.Pull = GPIO_PULLUP,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	},
	// D_BUTTON_3
	[GPIO_BUTTON_RIGHT] = {
		.port = GPIOB,
		.init = {
			.Pin = GPIO_PIN_9,
			.Mode = GPIO_MODE_INPUT,
			.Pull = GPIO_PULLUP,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	},
	// S_BUTTON_1
	[GPIO_BUTTON_SW_LEFT] = {
		.port = GPIOB,
		.init = {
			.Pin = GPIO_PIN_8,
			.Mode = GPIO_MODE_INPUT,
			.Pull = GPIO_PULLUP,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	},
	// S_BUTTON_2
	[GPIO_BUTTON_SW_RIGHT] = {
		.port = GPIOD,
		.init = {
			.Pin = GPIO_PIN_2,
			.Mode = GPIO_MODE_INPUT,
			.Pull = GPIO_PULLUP,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	},
	[GPIO_CTRL_SWITCH] = {
		.port = GPIOC,
		.init = {
			.Pin = GPIO_PIN_8,
			.Mode = GPIO_MODE_INPUT,
			.Pull = GPIO_PULLUP,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	},
	[GPIO_LED_AMS] = {
		.port = GPIOB,
		.init = {
			.Pin = GPIO_PIN_12,
			.Mode = GPIO_MODE_OUTPUT_PP,
			.Pull = GPIO_PULLUP,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	},
	// [GPIO_LED_IMD] = {
	// 	.port = GPIOA,
	// 	.init = {
	// 		.Pin = GPIO_PIN_10,
	// 		.Mode = GPIO_MODE_OUTPUT_PP,
	// 		.Pull = GPIO_PULLUP,
	// 		.Speed = GPIO_SPEED_FREQ_LOW
	// 	}
	// },
	// OLD DIM PINS
	[GPIO_LED_IMD] = {
		.port = GPIOA,
		.init = {
			.Pin = GPIO_PIN_8,
			.Mode = GPIO_MODE_OUTPUT_PP,
			.Pull = GPIO_PULLUP,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	},
	[GPIO_LED_BSPD] = {
		.port = GPIOA,
		.init = {
			.Pin = GPIO_PIN_9,
			.Mode = GPIO_MODE_OUTPUT_PP,
			.Pull = GPIO_PULLUP,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	},
	[GPIO_ASMS_ON] = {
		.port = GPIOB,
		.init = {
			.Pin = GPIO_PIN_13,
			.Mode = GPIO_MODE_INPUT,
			.Pull = GPIO_PULLUP,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	},
	[GPIO_LED_STATUS] = {
		.port = GPIOC,
		.init = {
			.Pin = GPIO_PIN_12,
			.Mode = GPIO_MODE_OUTPUT_PP,
			.Pull = GPIO_PULLUP,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	},
	[GPIO_PD_N] = {
		.port = GPIOB,
		.init = {
			.Pin = GPIO_PIN_14,
			.Mode = GPIO_MODE_OUTPUT_PP,
			.Pull = GPIO_NOPULL,
			.Speed = GPIO_SPEED_FREQ_LOW
		}
	}
};

/**
 * @brief This function is a wrapper that lets you see if ASMS is on
 *
 * @return 1 iff ASMS is on
 */
uint8_t getASMS(){
    return cmr_gpioRead(GPIO_ASMS_ON);
}

/**
 * @brief This function is a wrapper that lets you see if EAB is on
 *
 * @return 1 iff EAB is on
 */
bool getEAB(){
	uint8_t *eabStatus = (uint8_t*)getPayload(CANRX_EAB_STATUS);
	return *eabStatus;
}

/* Debouncing for button presses. */
# define DEBOUNCE_DELAY 50

static uint32_t lastPress[NUM_BUTTONS] = {0};
static bool lastState[NUM_BUTTONS] = {false};

/**
 * @brief Converts analog ADC to 0-3.3V, then scales that to 0-5V
 *
 * @param analog ADC value between 0-4096. Helper function for adcToXY
 *
 * @return a float between 0-5V.
 */

static float adcToVoltage(uint32_t analog){

    //error check
    if(analog > 4096){
        return -1;
    }else{
        //Scale analog to voltage between 0-5V
        float finalVolt = ((float)analog/4096.0f) * 5.0f;

        return finalVolt;
    }
}

/**
 * @brief reads state of all buttons
 */
static void gpioReadButtons(void *pvParameters) {
    (void)pvParameters;
    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        uint8_t paddle = adcRead(ADC_PADDLE);
        if(getCurrState() == CONFIG) {
            if (paddle > 100) config_increment_up_requested = true;
        }

        // Direct assignment for CAN buttons

        for(int i=0; i<NUM_BUTTONS; i++){
            // Active Low
            buttonStates[i].gpioState = !cmr_gpioRead(gpioButtonPins[i]);
            if (buttonStates[i].prevState != buttonStates[i].gpioState){
                buttonStates[i].isPressed = buttonStates[i].gpioState;
            }
            buttonStates[i].prevState = buttonStates[i].gpioState;
        }
        vTaskDelayUntil(&lastWakeTime, 100);
    }
}

/**
 * @brief Initializes the GPIO interface.
 */
void gpioInit(void) {
    cmr_gpioPinInit(
        gpioPinConfigs, sizeof(gpioPinConfigs) / sizeof(gpioPinConfigs[0]));
    cmr_taskInit(
        &gpioReadButtons_task,
        "gpioReadButtons",
        gpioReadButtons_priority,
        gpioReadButtons,
        NULL);
}
