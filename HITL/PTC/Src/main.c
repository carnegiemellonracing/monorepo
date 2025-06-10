
#include <stm32f4xx_hal.h>  // HAL interface

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/tasks.h>  // Task interface

#include "i2c.h"
#include "gpio.h"
#include "can.h"
#include "pwm.h"

// -------------- DEFINE TESTBENCH FUNCTIONS FUNCTIONS HERE -------------- //
void brakelightTestbench(uint8_t brakeThresh) {
	setVSMHeartbeat(CMR_CAN_HV_EN);
	setVSMStatus(CMR_CAN_VSM_STATE_HV_EN);
	setBrakePressure(brakeThresh+1);
	vTaskDelay(2000);
	if(cmr_gpioRead(DIGITAL_IN_24V_MCU2_BRAKELIGHT) != 1) {
		//set error screen here
		cmr_panic("l");
	}
	setBrakePressure(brakeThresh-1);
	vTaskDelay(2000);
	if(cmr_gpioRead(DIGITAL_IN_24V_MCU2_BRAKELIGHT) != 0) {
		//set error screen here
		cmr_panic("l");
	}

	vTaskDelay(2000);

	//try it at 10 above/below for sanity check
	setBrakePressure(brakeThresh+10);
	vTaskDelay(2000);
	if(cmr_gpioRead(DIGITAL_IN_24V_MCU2_BRAKELIGHT) != 1) {
		//set error screen here
		cmr_panic("l");
	}
	setBrakePressure(brakeThresh-10);
	vTaskDelay(2000);
	if(cmr_gpioRead(DIGITAL_IN_24V_MCU2_BRAKELIGHT) != 0) {
		//set error screen here
		cmr_panic("l");
	}
	//write passed test
}

void mcPowerTestbench() {

	//check that mc power turns on in HV_EN, RTD, and INVERTER_EN for VSM and Car

	setVSMHeartbeat(CMR_CAN_RTD);
	vTaskDelay(100);
	if(cmr_gpioRead(DIGITAL_IN_24V_MCU1_MC) != 1) {
		//set error screen here
		return;
	}

	vTaskDelay(2000);

	setVSMHeartbeat(CMR_CAN_HV_EN);
	setVSMStatus(CMR_CAN_VSM_STATE_INVERTER_EN );
	vTaskDelay(100);
	if(cmr_gpioRead(DIGITAL_IN_24V_MCU1_MC) != 1) {
		//set error screen here
		cmr_panic("l");
	}

	vTaskDelay(2000);

	setVSMHeartbeat(CMR_CAN_HV_EN);
	setVSMStatus(CMR_CAN_VSM_STATE_HV_EN);
	vTaskDelay(100);
	if(cmr_gpioRead(DIGITAL_IN_24V_MCU1_MC) != 1) {
		//set error screen here
		cmr_panic("l");
	}

	vTaskDelay(2000);

	//make sure this is off in GLV_ON and ERROR
	setVSMHeartbeat(CMR_CAN_GLV_ON);
	setVSMStatus(CMR_CAN_VSM_STATE_GLV_ON);
	vTaskDelay(100);
	if(cmr_gpioRead(DIGITAL_IN_24V_MCU1_MC) != 0) {
		//set error screen here
		cmr_panic("l");
	}

	setVSMHeartbeat(CMR_CAN_ERROR);
	vTaskDelay(2000);
	if(cmr_gpioRead(DIGITAL_IN_24V_MCU1_MC) != 0) {
		//set error screen here
		cmr_panic("l");
	}
}

void constantPWM_AC_Testbench(uint16_t AC_LowTemp, uint16_t AC_HighTemp, uint16_t lowPWM, uint16_t highPWM) {
	//iterate through range of temperatures in 5 degree increments for low and high
	uint32_t freq;
	uint32_t dutyCycle;
	for(int i = 0; i < AC_LowTemp; i+=5) {
		setACTemps(i, i);
		vTaskDelay(500);

		//change to correct timer from pin
		pwmRead(1, &freq, &dutyCycle);

		//check for 5% difference
		if((float)(abs(dutyCycle - lowPWM)/lowPWM) > 0.05) {
			return;
		}
	}
	vTaskDelay(1000);
	for(int i = AC_HighTemp; i < AC_HighTemp+50; i+=5) {
		setACTemps(i, i);
		vTaskDelay(500);

		//change to correct timer from pin
		pwmRead(1, &freq, &dutyCycle);

		//check for 5% difference
		if((float)(abs(dutyCycle - highPWM)/highPWM) > 0.05) {
			//error screen
			return;
		}
	}
}

void constantPWM_IC_Testbench(uint16_t IC_LowTemp, uint16_t IC_HighTemp, uint16_t lowPWM, uint16_t highPWM) {
	//iterate through range of temperatures in 5 degree increments for low and high
	uint32_t freq;
	uint32_t dutyCycle;
	for(int i = 0; i < IC_LowTemp; i+=5) {
		//set all IGBTs to same
		setIGBT1Temp(i);
		setIGBT2Temp(i);
		setIGBT3Temp(i);
		setIGBT4Temp(i);

		vTaskDelay(500);

		//change to correct timer from pin
		pwmRead(2, &freq, &dutyCycle);

		//check for 5% difference
		if((float)(abs(dutyCycle - lowPWM)/lowPWM) > 0.05) {
			//error screen
			return;
		}
	}
	vTaskDelay(1000);
	for(int i = IC_HighTemp; i < IC_HighTemp+50; i+=5) {
		//set all IGBTs to same
		setIGBT1Temp(i);
		setIGBT2Temp(i);
		setIGBT3Temp(i);
		setIGBT4Temp(i);
		vTaskDelay(500);

		//change to correct timer from pin
		pwmRead(2, &freq, &dutyCycle);

		//check for 5% difference
		if((float)(abs(dutyCycle - highPWM)/highPWM) > 0.05) {
			//error screen
			return;
		}
	}
}

void intermediatePWM_TestbenchAC(uint16_t AC_LowTemp, uint16_t AC_HighTemp, uint16_t lowPWM, uint16_t highPWM) {
	//iterate through range of temperatures in 5 degree increments for low and high
	uint32_t freq;
	uint32_t dutyCycle;
	for(int i = AC_LowTemp; i < AC_HighTemp; i+=5) {
		setACTemps(i, i);
		vTaskDelay(500);

		//change to correct timer from pin
		pwmRead(1, &freq, &dutyCycle);

		uint16_t targetPWM = ((highPWM-lowPWM) * (i-AC_LowTemp)/(AC_HighTemp)) + lowPWM;
		//check for 5% difference
		if((float)(abs(dutyCycle - targetPWM)/targetPWM) > 0.05) {
			return;
		}
	}
}

void intermediatePWM_TestbenchIC(uint16_t IC_LowTemp, uint16_t IC_HighTemp, uint16_t lowPWM, uint16_t highPWM) {
	//iterate through range of temperatures in 5 degree increments for low and high
	uint32_t freq;
	uint32_t dutyCycle;
	for(int i = IC_LowTemp; i < IC_HighTemp; i+=5) {
		//set all IGBTs to same
		setIGBT1Temp(i);
		setIGBT2Temp(i);
		setIGBT3Temp(i);
		setIGBT4Temp(i);
		vTaskDelay(500);

		//change to correct timer from pin
		pwmRead(1, &freq, &dutyCycle);

		uint16_t targetPWM = ((highPWM-lowPWM) * (i-IC_LowTemp)/(IC_HighTemp)) + lowPWM;
		//check for 5% difference
		if((float)(abs(dutyCycle - targetPWM)/targetPWM) > 0.05) {
			return;
		}
	}
}

// ---------------------------------------------------------------------------- //

static void testbenchTask(void *pvParameters) {
	TickType_t lastWakeTime = xTaskGetTickCount();
	while (1) {
		//brakelightTestbench(20); //change to actual threshold
		mcPowerTestbench();
//		constantPWM_AC_Testbench(0, 0, 0, 0);
//		constantPWM_IC_Testbench(0, 0, 0, 0);
//
//		intermediatePWM_TestbenchAC(0, 0, 0, 0);
//		intermediatePWM_TestbenchIC(0, 0, 0, 0);
		vTaskDelayUntil(&lastWakeTime, 100); // sets 1Hz frequency (every 1000 ms)
	}
}

/** @brief Testbench Task priority. */
static const uint32_t testbenchPriority = 2;
/** @brief Testbench Task period (milliseconds). */
static const TickType_t canTX5Hz_period_ms = 1000;
/** @brief CAN 5 Hz TX task. */
static cmr_task_t testbenchTask_PTC;

int main(void) {

	// Initialising interfaces: GPIO, PWM, CAN, I2C
    HAL_Init();
    cmr_rccSystemClockEnable();

	gpioInit();
//	i2cInit();
//	pwmReadInit();
	canInit();

	cmr_taskInit(
		&testbenchTask_PTC,
		"CAN TX 100Hz",
		testbenchPriority,
		testbenchTask,
		NULL
	);

	vTaskStartScheduler();



	return 0;
}
