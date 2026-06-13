/**
 * @file tssi.c
 * @brief Tractive System Status Indicators.
 * 
 * @author Carnegie Mellon Racing
 */

#include "error.h"       // Board-specific error interfaces
#include "gpio.h"       // Board-specific GPIO interface
#include "pwm.h"
#include "state.h"    // Board Specific State Interface
#include "tssi.h"      // Interface to implement



#include <CMR/pwm.h>           // PWM interface
#include <CMR/tasks.h>

/** @brief TSSI control task priority. */
static const uint32_t tssiControlPriority = 2;
/** @brief TSSI control task period. */
static const TickType_t tssiControl_period_ms = 250;
/** @brief TSSI control task. */
static cmr_task_t tssiControl_task;

static void flash_error_state() {
    pwmSetDutyCycle(PWM_GREEN, 0);
    pwmSetDutyCycle(PWM_RED, 50);
}

static void flash_normal_state() {
    pwmSetDutyCycle(PWM_GREEN, 100);
    pwmSetDutyCycle(PWM_RED, 0);
}

/**
 * @brief Task for controlling the TSSI.     
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void tssiControl(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    static bool initStarted = false;
    static TickType_t initStartTime = 0;
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1) {
        bool tssi_red_error =   getAMSError() || 
                                !cmr_gpioRead(GPIO_IN_IMD_ERR_N) || 
                                !cmr_gpioRead(GPIO_IN_IMD_ERR_COND_N);

        if (tssi_red_error && exitedErrorState) {
            flash_error_state();
        }
        else{
            if (!tssi_red_error) {
                exitedErrorState = true;
            }
            flash_normal_state(); 
        }
       
        vTaskDelayUntil(&lastWakeTime, tssiControl_period_ms);
    }
}

/**
 * @brief Initialization of ASSI control task.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
void tssiInit() {

    cmr_taskInit(
        &tssiControl_task,
        "TSSIControl",
        tssiControlPriority,
        tssiControl,
        NULL
    );
}