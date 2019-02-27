/**
 * @file gpio.c
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface
#include <FreeRTOS.h>   // FreeRTOS API
#include <task.h>       // Task interface
#include <queue.h>      // Queue interface

#include <CMR/gpio.h>   // GPIO interface

#include "gpio.h"   // Interface to implement

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
    [GPIO_LED_STATUS] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_6,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW
        }
    },
    [GPIO_BUTTON_0] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_6,
            .Mode = GPIO_MODE_IT_RISING_FALLING,
            .Pull = GPIO_NOPULL
        }
    },
    [GPIO_BUTTON_1] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_7,
            .Mode = GPIO_MODE_IT_RISING_FALLING,
            .Pull = GPIO_NOPULL
        }
    },
    [GPIO_BUTTON_2] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_8,
            .Mode = GPIO_MODE_IT_RISING_FALLING,
            .Pull = GPIO_NOPULL
        }
    },
    [GPIO_BUTTON_3] = {
        .port = GPIOC,
        .init = {
            .Pin = GPIO_PIN_9,
            .Mode = GPIO_MODE_IT_RISING_FALLING,
            .Pull = GPIO_NOPULL
        }
    },
    [GPIO_BUTTON_4] = {
        .port = GPIOA,
        .init = {
            .Pin = GPIO_PIN_8,
            .Mode = GPIO_MODE_IT_RISING_FALLING,
            .Pull = GPIO_NOPULL
        }
    },
    [GPIO_BEEPER] = {
        .port = GPIOB,
        .init = {
            .Pin = GPIO_PIN_5,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH
        }
    }
};

/** @brief Button state. */
static struct {
    /** @brief Event queue. */
    struct {
        QueueHandle_t q;        /**< @brief Message queue. */
        StaticQueue_t qBuf;     /**< @brief Queue storage. */
        /** @brief Queue item storage. */
        buttonEvent_t qItemBuf[BUTTON_EVENTS_MAX];
    } events;

    /** @brief Input task. */
    struct {
        /** @brief Stack buffer. */
        StackType_t stackBuf[configMINIMAL_STACK_SIZE];
        StaticTask_t taskBuf;   /**< @brief Task buffer. */
    } taskInput;
} buttons;

/** @brief Button input task priority. */
static const uint32_t buttonsInput_priority = 4;

/** @brief Button input task period (milliseconds). */
static const TickType_t buttonsInput_period = 10;

/**
 * @brief Handles button events.
 *
 * @param pvParameters Ignored.
 */
static void buttonsInput_task(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        volatile int value = cmr_gpioRead(GPIO_BUTTON_0);
        (void) value;

        buttonEvent_t event;
        while (xQueueReceive(buttons.events.q, &event, 0) == pdTRUE) {
            switch (event.pin) {
                case GPIO_BUTTON_0:
                    DIM_requested_state += 1;
                    break;
                case GPIO_BUTTON_1:
                    DIM_requested_state -= 1;
                    break;
                case GPIO_BUTTON_2:
                    break;
                case GPIO_BUTTON_3:
                    break;
                case GPIO_BUTTON_4:
                    break;
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

    buttons.events.q = xQueueCreateStatic(
        sizeof(buttons.events.qItemBuf) / sizeof(buttons.events.qItemBuf[0]),
        sizeof(buttons.events.qItemBuf[0]),
        (void *) buttons.events.qItemBuf,
        &buttons.events.qBuf
    );
    configASSERT(buttons.events.q != NULL);

    xTaskCreateStatic(
        buttonsInput_task,
        "GPIO button input",
        sizeof(buttons.taskInput.stackBuf) / sizeof(buttons.taskInput.stackBuf[0]),
        NULL, buttonsInput_priority,
        buttons.taskInput.stackBuf,
        &buttons.taskInput.taskBuf
    );

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**
 * @brief HAL GPIO interrupt handler.
 *
 * @param gpioPin Ignored.
 */
void HAL_GPIO_EXTI_Callback(uint16_t gpioPin) {
    /** @brief Represents a button's state. */
    typedef struct {
        const gpio_t pin;   /**< @brief Associated GPIO pin. */
        uint32_t lastTick;  /**< @brief Last HAL tick for the interrupt. */
        bool value;         /**< @brief Last read value. */
    } buttonState_t;

    /** @brief Button states. */
    static buttonState_t states[] = {
        { .pin = GPIO_BUTTON_0 },
        { .pin = GPIO_BUTTON_1 },
        { .pin = GPIO_BUTTON_2 },
        { .pin = GPIO_BUTTON_3 },
        { .pin = GPIO_BUTTON_4 }
    };

    (void) gpioPin;

    uint32_t now = HAL_GetTick();

    for (size_t i = 0; i < sizeof(states) / sizeof(states[0]); i++) {
        buttonState_t *state = states + i;
        bool value = cmr_gpioRead(state->pin);
        if (value == state->value) {
            continue;   // Button did not change; move on.
        }

        if (now < state->lastTick + 1) {
            continue;   // "Software debounce" by ignoring too-recent interrupt.
        }
        state->lastTick = now;

        state->value = value;   // Update button value.

        // Enqueue a button event.
        buttonEvent_t event = { .pin = state->pin, .pressed = !value };
        BaseType_t higherWoken;
        if (
            xQueueSendFromISR(buttons.events.q, &event, &higherWoken) != pdTRUE
        ) {
            (void) 0;   // Buffer is full; too bad.
        }
        portYIELD_FROM_ISR(higherWoken);
    }
}
