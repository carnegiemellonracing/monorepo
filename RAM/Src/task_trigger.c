/**
 * @brief Camera trigger task configuration and implementation.
 */

 #include <stdint.h>
 #include <FreeRTOS.h>
 #include <CMR/tasks.h>
 #include <CMR/gpio.h>
 #include <CMR/can_ids.h>
 #include <CMR/timer.h>

 #include "task_trigger.h"
 #include "gpio.h"
 #include "can.h"
 #include <stdbool.h>


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

static volatile int32_t trigger_delay = 0;

void setDelay(int32_t delay) {
    trigger_delay = delay;
}

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
    
    uint8_t delay_count = 0;

    while (1) {
    	int32_t signed_period = (int32_t)cameraTrigger_period_ms;

        delay_count++;
        if (delay_count >= 5) {
            int32_t delay = trigger_delay;
            if (delay < signed_period && delay > -signed_period) {
                signed_period -= delay;
            }
            delay_count = 0;
        }
    

		vTaskDelayUntil(&lastWakeTime, (TickType_t)signed_period);
        uint32_t timestamp = get_time_us();
        canTX(CMR_CAN_BUS_DAQ, CMR_CANID_DV_FRAME_TIME, 
            &timestamp, sizeof(timestamp), cameraTrigger_period_ms);
        
        cmr_gpioWrite(GPIO_CAMERA_TRIGGER, 1);

        vTaskDelay(camera_exposure_ms);

        cmr_gpioWrite(GPIO_CAMERA_TRIGGER, 0);
    }
}

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
