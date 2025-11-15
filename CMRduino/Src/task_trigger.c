/**
 * @brief Camera trigger task configuration and implementation.
 */

 #include <stdint.h>
 #include "task_trigger.h"

#define CAMERA_TRIGGER_PORT     GPIOA  // TODO: Change to your port
#define CAMERA_TRIGGER_PIN      GPIO_PIN_0  // TODO: Change to your pin

 

/** @brief Camera trigger task. */
static cmr_task_t cameraTrigger_task;
/** @brief Camera trigger task priority (highest). */
static const uint32_t cameraTrigger_priority = 0;
/** @brief Camera trigger task period (milliseconds). */
static const TickType_t cameraTrigger_period_ms = 50; // 20 Hz

/** @brief Camera exposure time (milliseconds). */
static TickType_t camera_exposure_ms = 10; // Default 10ms

/** @brief Camera trigger task handle. */
TaskHandle_t cameraTrigger_taskHandle = NULL; // Made non-static for extern access

/**
 * @brief Task for triggering camera GPIO at fixed intervals.
 *
 * This task generates a trigger pulse on a GPIO pin at a fixed rate.
 * The GPIO goes HIGH for the exposure duration, then LOW.
 * Can be delayed by CAN message reception via xTaskNotifyFromISR.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void cameraTrigger(void *pvParameters) {
    (void) pvParameters; // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    uint32_t notificationValue = 0;
    
    while (1) {
        // Check if we received a delay notification from CAN RX
        if (xTaskNotifyWait(0x00, 0xFFFFFFFF, &notificationValue, 0) == pdTRUE) {
            // Notification received - apply the delay
            // The notification value contains the delay amount in ms
            TickType_t delay_ticks = pdMS_TO_TICKS(notificationValue);
            vTaskDelay(delay_ticks);
        }

        // Wait for next trigger period
        vTaskDelayUntil(&lastWakeTime, cameraTrigger_period_ms);

        cmr_gpioWrite(GPIO_OUT_CAMERA_TRIGGER, 1);

        // Hold HIGH for exposure time
        vTaskDelay(camera_exposure_ms);

        cmr_gpioWrite(GPIO_OUT_CAMERA_TRIGGER, 0);
    }
}

/**
 * @brief Delays the next camera trigger by a specified amount.
 *
 * Call this function from CAN RX ISR or task to delay the next trigger.
 * Uses xTaskNotifyFromISR to communicate with the trigger task.
 *
 * @param delay_ms Delay amount in milliseconds.
 * @param pxHigherPriorityTaskWoken Pointer to flag for context switch.
 *
 * @return pdPASS if notification sent successfully, pdFAIL otherwise.
 */
BaseType_t delayCameraTrigger(uint32_t delay_ms, BaseType_t *pxHigherPriorityTaskWoken) {
    if (cameraTrigger_taskHandle == NULL) {
        return pdFAIL;
    }

    return xTaskNotifyFromISR(
        cameraTrigger_taskHandle,
        delay_ms,
        eSetValueWithOverwrite,
        pxHigherPriorityTaskWoken
    );
}

/**
 * @brief Initializes the camera trigger task.
 *
 * @param exposure_ms Camera exposure time in milliseconds (0 = use default 10ms).
 */
void cameraTriggerInit(TickType_t exposure_ms) {
    // Set exposure time (use default if 0)
    if (exposure_ms > 0) {
        camera_exposure_ms = exposure_ms;
    }

    // Task initialization
    cmr_taskInit(
        &cameraTrigger_task,
        "Camera Trigger",
        cameraTrigger_priority,
        cameraTrigger,
        NULL
    );
    
    // Get task handle for notifications
    cameraTrigger_taskHandle = cameraTrigger_task.taskHandle;
}