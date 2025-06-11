
#include <stm32h7xx_hal.h>  // HAL interface

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/tasks.h>  // Task interface

#include "i2c.h"
#include "gpio.h"
#include "can.h"
#include "pwm.h"
#include <CMR/can_types.h>

#include <stdbool.h>

/** @brief Status LED priority. */
static const uint32_t statusLED_priority = 2;

/** @brief Status LED period (milliseconds). */
static const TickType_t statusLED_period_ms = 250;

/** @brief Status LED task. */
static cmr_task_t statusLED_task;


uint8_t zeroes[2] = {0, 0};

/**
 * @brief Task for toggling the status LED.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void statusLED(void *pvParameters) {
    (void) pvParameters;

    cmr_gpioWrite(DIGITAL_OUT_5V_MCU, 0);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_gpioToggle(DIGITAL_OUT_5V_MCU);

        vTaskDelayUntil(&lastWakeTime, statusLED_period_ms);
    }
}

bool testSoftwareError(void) {
	setHVC_heartbeat(0, 0, 0, 0, 0);
	vTaskDelay(1000);
	cmr_gpioWrite(DIGITAL_IN_3V3_MCU4, 1);
	vTaskDelay(1000);
	cmr_gpioWrite(DIGITAL_IN_3V3_MCU4, 0);
	setHVC_heartbeat(CMR_CAN_HVC_ERROR_PACK_OVERVOLT, 0, 0, 0, 0);
	vTaskDelay(1000);
	setHVC_heartbeat(0, 0, 0, 0, 0);
	cmr_gpioWrite(DIGITAL_IN_3V3_MCU4, 1);
	vTaskDelay(1000);
	cmr_gpioWrite(DIGITAL_IN_3V3_MCU4, 0);
//	vTaskDelay(1000);
//	setHVC_heartbeat(0, 0, 0, 0, 0);
//	vTaskDelay(1000);
//	setHVC_heartbeat(CMR_CAN_HVC_ERROR_CELL_OVERVOLT, 0, 0, 0, 0);
//	vTaskDelay(1000);
//	setHVC_heartbeat(0, 0, 0, 0, 0);
//	vTaskDelay(1000);
//	setHVC_heartbeat(CMR_CAN_HVC_ERROR_CELL_OVERTEMP, 0, 0, 0, 0);
	vTaskDelay(1000);
	setHVC_heartbeat(0, 0, 0, 0, 0);
}

bool testIMDError(void) {
	//send imd clear
	cmr_gpioWrite(DIGITAL_OUT_24V_MCU, 1);
	vTaskDelay(1000);

	//send reset signal
	cmr_gpioWrite(DIGITAL_IN_3V3_MCU4, 1);
	vTaskDelay(50);
	cmr_gpioWrite(DIGITAL_IN_3V3_MCU4, 0);
	vTaskDelay(1000);

	//cmr_canVSMStatus_t *latch = (cmr_canVSMStatus_t*)(canRXMeta_VEH[4].payload);
	//uint8_t badState = latch->badStateMatrix;
	//send imd clear
	cmr_gpioWrite(DIGITAL_OUT_24V_MCU, 0);
	vTaskDelay(1000);
	//latch = (cmr_canVSMStatus_t*)(canRXMeta_VEH[4].payload);
	//badState = latch->badStateMatrix;

}

bool testRTD(void) {
	vTaskDelay(1000);
	setDIM_request(1,0,0,0);
	for(int i = 0; i < 1000; i++) {
		cmr_canHeartbeat_t *vsm = getPayloadVEH(0);
		uint8_t state = vsm->state;
		setCDC_heartbeat(state,zeroes, zeroes);
		setDIM_heartbeat(state,zeroes, zeroes);
		setPTC_heartbeat(state,zeroes, zeroes);
		vTaskDelay(1);
	}
	vTaskDelay(1000);
	setDIM_request(2,0,0,0);
	setinverter1(256, 0, 0, 0);
	setinverter2(256, 0, 0, 0);
	setinverter3(256, 0, 0, 0);
	setinverter4(256, 0, 0, 0);
	while(1) {
		cmr_canHeartbeat_t *vsm = getPayloadVEH(0);
		cmr_canHVCCommand_t *status = getPayloadVEH(CANRX_HEARTBEAT_VSM);
		uint8_t state = vsm->state;
		setHVC_heartbeat(0, status->modeRequest, 3, 1, 1);
		setCDC_heartbeat(state,zeroes, zeroes);
		setDIM_heartbeat(state,zeroes, zeroes);
		setPTC_heartbeat(state,zeroes, zeroes);
		vTaskDelay(1);
	}
}

bool testBDPError(void) {
	i2c_writeI2CDAC2(7, 127);
	i2c_writeI2CDAC2(7, 255);

	//send reset signal
	cmr_gpioWrite(DIGITAL_IN_3V3_MCU4, 1);
	vTaskDelay(50);
	cmr_gpioWrite(DIGITAL_IN_3V3_MCU4, 0);
	vTaskDelay(1000);

}

bool testDCM() {
	cmr_canVSMStatus_t *vsm = getPayloadVEH(CANRX_HEARTBEAT_VSM);
	uint8_t state = vsm->internalState;

	// first set VSM to CLEAR_ERROR

	// setHVC_heartbeat(0, 1, 0x0B, 1, 1); 
	// setCDC_heartbeat(5,zeroes, zeroes);
	// setDIM_heartbeat(5,zeroes, zeroes);
	// setPTC_heartbeat(5,zeroes, zeroes);

	// now VSM in GLV_ON
	setDIM_request(1, 0, 0, 0);

	// setHVC_heartbeat(0, 1, 2, 1, 1);
	// setCDC_heartbeat(1,zeroes, zeroes);
	// setDIM_heartbeat(1,zeroes, zeroes);
	// setPTC_heartbeat(1,zeroes, zeroes);

	// check that pumps, fans, inverters all on:

	// check pumps and fans
	// cmr_canPTCDriverStatus_t *pumpsFans = getPayloadVEH(CANRX_HEARTBEAT_VSM);
	// uint8_t fanState = pumpsFans->fan1DutyCycle_pcnt, pumpsState = pumpsFans->pump1DutyCycle_pcnt;

	// while (fanState != 1 && pumpsState != 1) {
	// 	pumpsFans = getPayloadVEH(CANRX_HEARTBEAT_VSM);
	// 	fanState = pumpsFans->fan1DutyCycle_pcnt;
	// 	pumpsState = pumpsFans->pump1DutyCycle_pcnt;
	// }

	// check inverters:
	cmr_canAMKSetpoints_t *motors = getPayloadVEH(CANRX_HEARTBEAT_INV);
	uint8_t motorsState = motors->control_bv;

	while (motorsState != CMR_CAN_AMK_CTRL_INV_EN) {
		motors = getPayloadVEH(CANRX_HEARTBEAT_VSM);
		motorsState = motors->control_bv;
	}

	// then put VSM in HV_EN:

	setDIM_request(CMR_CAN_HV_EN, 0, 0, 0);

	// setinverter1(1, 0, 0, 0);
	// setinverter2(1, 0, 0, 0);
	// setinverter3(1, 0, 0, 0);
	// setinverter4(1, 0, 0, 0);

	//check pumps and fans

	// pumpsFans = getPayloadVEH(CANRX_HEARTBEAT_VSM);
	// fanState = pumpsFans->fan1DutyCycle_pcnt, pumpsState = pumpsFans->pump1DutyCycle_pcnt;
	
	// while (fanState != 1 && pumpsState != 1) {
	// 	pumpsFans = getPayloadVEH(CANRX_HEARTBEAT_VSM);
	// 	fanState = pumpsFans->fan1DutyCycle_pcnt;
	// 	pumpsState = pumpsFans->pump1DutyCycle_pcnt;
	// }

	// check inverters
	motors = getPayloadVEH(CANRX_HEARTBEAT_INV);
	motorsState = motors->control_bv;

	while (motorsState != CMR_CAN_AMK_CTRL_INV_EN) {
		motors = getPayloadVEH(CANRX_HEARTBEAT_INV);
		motorsState = motors->control_bv;
	}

	// put VSM in RTD:
	setDIM_request(CMR_CAN_RTD, 0, 0, 0);

	// check pumps and fans
	// pumpsFans = getPayloadVEH(CANRX_HEARTBEAT_VSM);
	// fanState = pumpsFans->fan1DutyCycle_pcnt, pumpsState = pumpsFans->pump1DutyCycle_pcnt;
	
	// while (fanState != 1 && pumpsState != 1) {
	// 	pumpsFans = getPayloadVEH(CANRX_HEARTBEAT_VSM);
	// 	fanState = pumpsFans->fan1DutyCycle_pcnt;
	// 	pumpsState = pumpsFans->pump1DutyCycle_pcnt;
	// }

	// check inverters
	motors = getPayloadVEH(CANRX_HEARTBEAT_INV);
	motorsState = motors->control_bv;

	while (motorsState != CMR_CAN_AMK_CTRL_INV_EN) {
		motors = getPayloadVEH(CANRX_HEARTBEAT_INV);
		motorsState = motors->control_bv;
	}


	motors = getPayloadVEH(CANRX_HEARTBEAT_INV);
	motorsState = motors->control_bv;

	return motorsState == CMR_CAN_AMK_CTRL_INV_EN;
}

static void testbenchTask(void *pvParameters) {
	TickType_t lastWakeTime = xTaskGetTickCount();
	while (1) {
//		testRTD();
		testDCM();
		vTaskDelayUntil(&lastWakeTime, 1000); // sets 1Hz frequency (every 1000 ms)
	}
}

/** @brief Testbench Task priority. */
static const uint32_t testbenchPriority = 2;
/** @brief Testbench Task period (milliseconds). */
static const TickType_t canTX5Hz_period_ms = 1000;
/** @brief CAN 5 Hz TX task. */
static cmr_task_t testbenchTask_VSM;

int main(void) {

	// Initialising interfaces: GPIO, PWM, CAN, I2C
	HAL_Init();
	cmr_rccSystemClockEnable();
	gpioInit();
	i2cInit();
	//pwmReadInit();
	canInit();

//	setHVC_heartbeat(0, 1, 1, 1, 1);
//	cmr_gpioWrite(DIGITAL_OUT_24V, 1);
//	uint8_t zeroes[2] = {0,0};
//	setCDC_heartbeat(4,zeroes, zeroes);
//	setDIM_heartbeat(4,zeroes, zeroes);
//	setPTC_heartbeat(4,zeroes, zeroes);
//	testClearErrorState();

	// Setting outputs
	//cmr_gpioWrite(GPIO_MCU1, 0);
	//cmr_gpioWrite(DIGITAL_OUT_5V_MCU, 1);


	// Reading inputs to verify board
//	i2c_writeExtRefDAC1();
//	HAL_Delay(100);
//
//	uint8_t val = 0;
//	uint16_t y = 0;
//	while (1) {
//		i2c_writeI2CDAC1(15, val);
//		HAL_Delay(100);
//		y = i2c_readI2CADC1(0);
//		HAL_Delay(100);
//		val = val + 10;
//	}
	cmr_taskInit(&statusLED_task, "statusLED", statusLED_priority, statusLED,
		                NULL);

	cmr_taskInit(
			&testbenchTask_VSM,
			"CAN TX 100Hz",
			testbenchPriority,
			testbenchTask,
			NULL
	);

	vTaskStartScheduler();




	return 0;
}
