/**
 * @file data.h
 * @brief Implements LV-BMS CAN transmission functions
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/tasks.h>
#include <CMR/uart.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "adc.h"
#include "bq_interface.h"
#include "can.h"
#include "dwt.h"
#include "gpio.h"
#include "uart.h"
#include "analysis.h"

/** @brief Sample Task Priority priority. */
static const uint32_t sampleTaskPriority = 3;

/** @brief Sample Task period (milliseconds). */
static const TickType_t sampleTaskPeriod_ms = 100;

/** @brief Sample task. */
static cmr_task_t sampleTask;

#define BALANCE_EN true
#define BALANCE_DIS false
#define MUX_CHANNELS 4

extern BMB_Data_t BMBData; 


bool isBalanceCommanded() {
	TickType_t xLastWakeTime = xTaskGetTickCount();

	if(cmr_canRXMetaTimeoutError(&(canRXMeta[CANRX_BALANCE_COMMAND]), xLastWakeTime) < 0) {
		return false;
	}
	
	volatile cmr_canHVCBalanceCommand_t *balanceCommand = getPayload(CANRX_BALANCE_COMMAND);
	return balanceCommand->balanceRequest;
}

// Determines if we are balancing and performs the necessary actions 
static void handle_balancing (void){
    bool currentlyBalancing = false;
	uint16_t threshold = 0;
	uint32_t settleTimer_ms;
	bool settlingTimerStarted = false;

	// Main BMS control loop
	while (1) {
        // Balancing logic

		// If we get a balancing command and we weren't previously balancing, enable
		// cells to balance
		// if (isBalanceCommanded() && !currentlyBalancing) {
		// 	if(getBalDone() == 1){
        //         threshold = BMS_stats.min_cell_voltage_mV + 5;
		// 		cellBalancing(BALANCE_EN, threshold); 
		// 		currentlyBalancing = true;
		// 	} 
		// }

		// If the balancing command stops and we were balancing previously, disable
		// all balancing cells
		// else if(!isBalanceCommanded() && currentlyBalancing){
		// 	cellBalancing(BALANCE_DIS, threshold);
		// 	currentlyBalancing = false;
		// }

		// Once balancing timers have expired we stop balancing
		if(getBalDone()==1 && currentlyBalancing && !settlingTimerStarted) {
			settlingTimerStarted = true;
			settleTimer_ms = xTaskGetTickCount(); 
		}

		// After a 500 ms resting period, recheck the voltages
		// and enable whichever cells are above threshold
		// else if (settlingTimerStarted && (xTaskGetTickCount()-settleTimer_ms)>500) {
		// 	currentlyBalancing = false;
		// 	settlingTimerStarted = false;
		// }
    }

}

// Main sample task entry point for BMS
void vBMBSampleTask(void *pvParameters) {
    (void) pvParameters;
    static TickType_t const ADC_settlingTime_ms = 10;
    static bool ledToggle = false;
    
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(1){
		handle_balancing();
		vTaskDelayUntil(&xLastWakeTime, ADC_settlingTime_ms);


		// Loop through the different MUX channels and select a different one
		// We still monitor all voltages each channel switch
		for (uint8_t j = 0; j < MUX_CHANNELS; j++){
			setMuxOutput(j);
			vTaskDelayUntil(&xLastWakeTime, ADC_settlingTime_ms);
			pollAllTemperatureData(j);
		}

		pollAllVoltageData();
		writeLED(ledToggle);
		ledToggle = !ledToggle;
		vTaskDelayUntil(&xLastWakeTime, (sampleTaskPeriod_ms - (ADC_settlingTime_ms * MUX_CHANNELS)));
	}

    
}



void sampleInit(){
    cmr_taskInit(
        &sampleTask,
        "BMS Sample Init",
        sampleTaskPriority,
        vBMBSampleTask,
        NULL
    );
}