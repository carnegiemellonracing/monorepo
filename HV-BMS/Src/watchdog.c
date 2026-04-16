/**
 * @file watchdog.c
 * @brief Board-specific Watchdog implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "watchdog.h"
#include <CMR/tasks.h>

/** @brief Watchdog priority */
static const uint32_t Watchdog_priority = 2;

/** @brief Watchdog task period (milliseconds). */
static const TickType_t Watchdog_period_ms = 2;

/** @brief Task for watchdog */
static cmr_task_t Watchdog_task;

/** @brief WWDG interface */
static cmr_wwdg_t wwdg;

/**
 * @brief Task for updating the Voltage and Current
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void WatchdogUpdate(void *pvParameters) {
    (void) pvParameters;
    cmr_wwdgStart(&wwdg);
    TickType_t lastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&lastWakeTime, Watchdog_period_ms);
    while(1) {
        cmr_wwdgKick(&wwdg);
        vTaskDelayUntil(&lastWakeTime, Watchdog_period_ms);
    }
}

/** @brief
 * 
 */
void wwdgInit() {
    /* Want to reset watchdog with a window between 1 ms and 10 ms
    APB1 = 48 MHz
    WWDG clock counter = (10ms * 48*10^6 Hz) / (4096 * 2^(prescalar value 0) * 1000) + 64 = 182
    WWDG Window value = counter - (1ms * 46*10^6 Hz) / 4096 *2^(prescalar value 0) * 1000) = 111 
    WWDG Window value means that the WWDG counter should be refreshed only
    when the counter is below 111 (and greater than 64) otherwise a reset will
    be generated.
    */
    const WWDG_InitTypeDef wwdgInit = {
        .Prescaler = WWDG_PRESCALER_1,
        .Window = 111,
        .Counter = 182,
        .EWIMode = WWDG_EWI_DISABLE
    };

    cmr_wwdgInit(&wwdg, &wwdgInit);

    cmr_taskInit(
        &Watchdog_task,
        "Watchdog",
        Watchdog_priority,
        WatchdogUpdate,
        NULL
    );
}
