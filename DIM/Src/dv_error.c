/**
 * @file as_error.c
 * @brief  Checks for any DV errors and if they exist then trigger the DV Relay
 *
 * @author Ayush Garg
 */

#include "can.h"        
#include "gpio.h" 
#include "sensors.h"       

/** @brief DV Error priority. */
static const uint32_t dv_error_priority = 2;

/** @brief DV Error period (milliseconds). */
static const TickType_t dv_error_period_ms = 10;

/** @brief DV Error task. */
static cmr_task_t dv_error_task;

/**
 * @brief Task for checking if any DV Errors are present. T
 * These are continuous error checks
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void dv_error_checks(void *pvParameters) {
    (void)pvParameters;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while(1)
    {
        bool error_present = false;
        if (cmr_sensorListGetError(&sensorList, SENSOR_CH_EBS_CURRENT_1_MA) != CMR_SENSOR_ERR_NONE)
            error_present = true;
    
        if (cmr_sensorListGetError(&sensorList, SENSOR_CH_EBS_CURRENT_2_MA) != CMR_SENSOR_ERR_NONE)
            error_present = true;
    
        if (cmr_sensorListGetError(&sensorList, SENSOR_CH_EBS_PRESSURE_1_DECI_BAR) != CMR_SENSOR_ERR_NONE)
            error_present = true;

        if (cmr_sensorListGetError(&sensorList, SENSOR_CH_EBS_PRESSURE_2_DECI_BAR) != CMR_SENSOR_ERR_NONE)
            error_present = true;

        cmr_gpioWrite(GPIO_AS_ERROR, error_present);
        vTaskDelayUntil(&lastWakeTime, dv_error_period_ms);
    }
}

/**
 * @brief inits the dv_error
 * 
 */
void dv_error_init(void){
    cmr_taskInit(
        &dv_error_task,
        "dv_error",
        dv_error_priority,
        dv_error_checks,
        NULL);
}