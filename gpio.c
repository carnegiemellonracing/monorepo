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

/**
 * @brief Handles button events.
 *
 * @param pvParameters Ignored.
 */

static void buttonsInput_task(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        

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

    
}

