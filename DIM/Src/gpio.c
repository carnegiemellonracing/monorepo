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

/**
 * @brief This function is a wrapper that lets you see if ASMS is on
 *
 * @return 1 iff ASMS is on
 */
uint8_t getASMS(){
    return 0;
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

/**
 * @brief reads state of all buttons
 */
static void gpioReadButtons(void *pvParameters) {
    (void)pvParameters;
    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        // Direct assignment for CAN buttons

        for(int i=0; i<NUM_BUTTONS; i++){
            buttonStates[i].gpioState = 0;
            buttonStates[i].isPressed = 0;
            buttonStates[i].prevState = 0;
        }
        vTaskDelayUntil(&lastWakeTime, 100);
    }
}

/**
 * @brief Initializes the GPIO interface.
 */
void gpioInit(void) {
    cmr_taskInit(
        &gpioReadButtons_task,
        "gpioReadButtons",
        gpioReadButtons_priority,
        gpioReadButtons,
        NULL);
}
