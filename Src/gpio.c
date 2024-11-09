/**
 * @file gpio.c
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#include "gpio.h"  // Interface to implement

#include <stm32f4xx_hal.h>  // HAL interface

#include "expanders.h"  // GPIO expanders interface
#include "state.h"

static const uint32_t gpioReadButtons_priority = 4;

/** @brief Button input task period (milliseconds). */
static const TickType_t gpioReadButtons_period = 10;

/** @brief Button input task task. */
static cmr_task_t gpioReadButtons_task;

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
    [GPIO_LED_AMS] = { .port = GPIOA, .init = { .Pin = GPIO_PIN_6,
                       .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL,
                       .Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_LED_IMD] = { .port = GPIOA, .init = { .Pin = GPIO_PIN_7,
                       .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL,
                       .Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_LED_BSPD] = { .port = GPIOA, .init = { .Pin = GPIO_PIN_8,
                        .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL,
                        .Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_BUTTON_L] = { .port = GPIOC, .init = { .Pin = GPIO_PIN_0, 
                        .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP, 
                        .Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_BUTTON_R] = { .port = GPIOC, .init = { .Pin = GPIO_PIN_1, 
                        .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP, 
                        .Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_BUTTON_U] = { .port = GPIOC, .init = { .Pin = GPIO_PIN_2, 
                        .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP, 
                        .Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_BUTTON_D] = { .port = GPIOC, .init = { .Pin = GPIO_PIN_3, 
                        .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP, 
                        .Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_BUTTON_SW1] = { .port = GPIOC, .init = { .Pin = GPIO_PIN_4, 
                        .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP, 
                        .Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_BUTTON_SW2] = { .port = GPIOC, .init = { .Pin = GPIO_PIN_5, 
                        .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP, 
                        .Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_BUTTON_PUSH] = { .port = GPIOC, .init = { .Pin = GPIO_PIN_9, 
                        .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP, 
                        .Speed = GPIO_SPEED_FREQ_LOW } },
};

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
 * @brief reads state of all buttons
 */
static void gpioReadButtons(void *pvParameters) {
    (void)pvParameters;
    bool pressConfirmed = false;
    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        // Direct assignment for CAN buttons
        for(int i=0; i<NUM_BUTTONS; i++){
            canButtonStates[i] = (HAL_GPIO_ReadPin(gpioPinConfigs[i].port, gpioPinConfigs[i].pin) == GPIO_PIN_RESET);
        }
        //the for loop iterates over all the pins including the LEDs will it mess up the LEDs (should i change it repeat for the num of buttons insteaed)??
        for(int i=0; i<GPIO_LEN; i++) {
            while(HAL_GPIO_ReadPin(gpioPinConfigs[i].port, gpioPinConfigs[i].pin)
                == GPIO_PIN_RESET) { 
                //when you press it goes to zero
                vTaskDelayUntil(&lastWakeTime, buttonsInput_period);
                pressConfirmed = true;        
            }
            if(pressConfirmed){
                gpioButtonStates[i] = true;
            }
        }
        vTaskDelayUntil(&lastWakeTime, gpioReadButtons_period);
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
