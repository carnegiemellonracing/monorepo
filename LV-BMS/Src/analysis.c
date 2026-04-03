/**
 * @file analysis.h
 * @brief Helpers for analyzing data
 *
 * @author Ayush Garg
 */
#include "stdint.h.h"  // Task interface

#include <CMR/tasks.h>      // Task interface

#include "bq_interface.h"  // Task interface


/** @brief analysis priority. */
static const uint32_t analysis_priority = 2;

/** @brief analysis period (milliseconds). */
static const TickType_t analysis_period_ms = 10;

/** @brief analysis task. */
static cmr_task_t analysis_task;

/**
 * @brief Task for toggling the analysis
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void analysis(void *pvParameters) {
	(void) pvParameters; // placate compiler

    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1) {
        uint16_t max_cell_voltage_mV = 0;
        uint16_t min_cell_voltage_mV = UINT16_MAX;
        uint8_t  max_cell_voltage_idx = 0;
        uint8_t  min_cell_voltage_idx = 0;
        
        uint16_t max_cell_temp_centi_deg = 0;
        uint16_t min_cell_temp_centi_deg = UINT16_MAX;
        uint8_t  max_cell_temp_idx = 0;
        uint8_t  min_cell_temp_idx = 0;
        
        uint16_t pack_voltage_mV = 0; 

        for (uint8_t cell_index = 0; cell_index < NUM_CELLS; cell_index++) {
            uint16_t curr_cell_voltage_mV = getVoltageData_mV(cell_index);
            uint16_t curr_cell_temp_centi_C = getTempData_centi_C(cell_index);

            // Process Voltages 
            if (curr_cell_voltage_mV > max_cell_voltage_mV) {
                max_cell_voltage_mV = curr_cell_voltage_mV;
                max_cell_voltage_idx = cell_index; 
            }

            if (curr_cell_voltage_mV < min_cell_voltage_mV) { 
                min_cell_voltage_mV = curr_cell_voltage_mV;
                min_cell_voltage_idx = cell_index;
            }
            
            pack_voltage_mV += curr_cell_voltage_mV;

            // Process Temperatures
            if (curr_cell_temp_centi_C > max_cell_temp_centi_deg) {
                max_cell_temp_centi_deg = curr_cell_temp_centi_C;
                max_cell_temp_idx = cell_index;
            }

            if (curr_cell_temp_centi_C < min_cell_temp_centi_deg) {
                min_cell_temp_centi_deg = curr_cell_temp_centi_C;
                min_cell_temp_idx = cell_index;
            }
        }

        vTaskDelayUntil(&lastWakeTime, analysis_period_ms);
    }
}

void analysisInit(){
	cmr_taskInit(
		&analysis_task,
		"Analysis Task",
		analysis_priority,
		analysis,
		NULL
	);
}

