/**
 * @brief Camera trigger task configuration and implementation.
 */

 #include <stdint.h>
 #include <CMR/gpio.h>
 #include "task_trigger.h"
 #include "gpio.h"
 #include "can.h"
#include <stdatomic.h>
#include <stdbool.h>
#include "timer.h"

#define CAN_ID_FRAME_TIMESTAMP 0x67
#define CAMERA_TRIGGER_PORT     GPIOC
#define CAMERA_TRIGGER_PIN      GPIO_PIN_3

extern atomic_bool delay_flag;
 

/** @brief Camera trigger task. */
static cmr_task_t cameraTrigger_task;
/** @brief Camera trigger task priority (highest). */
static const uint32_t cameraTrigger_priority = 1;
/** @brief Camera trigger task period (milliseconds). */
static const TickType_t cameraTrigger_period_ms = 50; // 20 Hz

/** @brief Camera exposure time (milliseconds). */
static TickType_t camera_exposure_ms = 2; // Default 10ms

/** @ brief variable for setting delay time.*/

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
    	int32_t signed_period = (int32_t)cameraTrigger_period_ms;
    	static uint8_t delay_count = 0;

    	if (atomic_load(&delay_flag)) {
    	    delay_count++;
    	    if (delay_count >= 5) {
    	        int32_t delay = getDelay();
    	        if (delay < signed_period && delay > -signed_period) {
    	            signed_period -= delay;
    	        }
    	        delay_count = 0;
    	    }
    	    atomic_store(&delay_flag, false);
    	}

		vTaskDelayUntil(&lastWakeTime, (TickType_t)signed_period);
        uint32_t timestamp = get_time_us();
        canTX(CAN_ID_FRAME_TIMESTAMP, &timestamp, sizeof(timestamp), cameraTrigger_period_ms);
        
        cmr_gpioWrite(GPIO_FAN_ON, 1);

        vTaskDelay(camera_exposure_ms);

        cmr_gpioWrite(GPIO_FAN_ON, 0);
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
/*BaseType_t delayCameraTrigger(uint32_t delay_ms, BaseType_t *pxHigherPriorityTaskWoken) {
    if (cameraTrigger_taskHandle == NULL) {
        return pdFAIL;
    }

    return xTaskNotifyFromISR(
        cameraTrigger_taskHandle,
        delay_ms,
        eSetValueWithOverwrite,
        pxHigherPriorityTaskWoken
    );
}*/

/**
 * @brief Initializes the camera trigger task.
 *
 * @param exposure_ms Camera exposure time in milliseconds (0 = use default 10ms).
 */
void cameraTriggerInit() {
    // Set exposure time (use default if 0)

    // Task initialization
    cmr_taskInit(
        &cameraTrigger_task,
        "Camera Trigger",
        cameraTrigger_priority,
        cameraTrigger,
        NULL
    );
    
    // Get task handle for notifications
    //cameraTrigger_taskHandle = cameraTrigger_task.taskHandle;
}
