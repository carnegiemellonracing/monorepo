
#include <stm32f4xx_hal.h>  // HAL interface

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

    cmr_gpioWrite(GPIO_LED_STATUS, 0);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_gpioToggle(GPIO_LED_STATUS);

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
	cmr_gpioWrite(DIGITAL_OUT_24V, 1);
	vTaskDelay(1000);

	//send reset signal
	cmr_gpioWrite(DIGITAL_IN_3V3_MCU4, 1);
	vTaskDelay(50);
	cmr_gpioWrite(DIGITAL_IN_3V3_MCU4, 0);
	vTaskDelay(1000);

	//cmr_canVSMStatus_t *latch = (cmr_canVSMStatus_t*)(canRXMeta_VEH[4].payload);
	//uint8_t badState = latch->badStateMatrix;
	//send imd clear
	cmr_gpioWrite(DIGITAL_OUT_24V, 0);
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
		cmr_canHVCCommand_t *status = getPayloadVEH(CANRX_HEARTBEAT_VSM2);
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

bool testClearErrorState(void) {
	//vTaskDelay(50);

	// set other boards to ERROR
	setHVC_heartbeat(1, 1, 0, 1, 1); // errorState = 1 (PACK_UNDERVOLT), hvcState = 0 (ERROR_STATE)
	setCDC_heartbeat(4,zeroes, zeroes);
	setDIM_heartbeat(4,zeroes, zeroes);
	setPTC_heartbeat(4,zeroes, zeroes);

	// vsm external state
	cmr_canHeartbeat_t *vsm = getPayloadVEH(CANRX_HEARTBEAT_VSM);
	uint8_t currentState = vsm->state;

	//vTaskDelay(50);

	// all boards in error -- VSM should be in error
	if (currentState != CMR_CAN_ERROR) {
		return false;
	}

	//dim request for GLV_ON
	setDIM_request(1, 0, 0, 0); 

	vsm = getPayloadVEH(CANRX_HEARTBEAT_VSM);
	currentState = vsm->state;

	vTaskDelay(50);

	// GLV_ON sets VSM to CLEAR_ERROR
	while (currentState != CMR_CAN_CLEAR_ERROR) {
		vsm = getPayloadVEH(CANRX_HEARTBEAT_VSM);
		currentState = vsm->state;
	}

	// set other boards to CLEAR_ERROR
	setHVC_heartbeat(0, 1, 0x0B, 1, 1); // errorStatus = 0 (none), hvcState = 0x0B (CLEAR_ERROR)
	setCDC_heartbeat(5,zeroes, zeroes);
	setDIM_heartbeat(5,zeroes, zeroes);
	setPTC_heartbeat(5,zeroes, zeroes);

	vsm = getPayloadVEH(CANRX_HEARTBEAT_VSM);
	currentState = vsm->state;


	while (currentState != CMR_CAN_GLV_ON) {
		vsm = getPayloadVEH(CANRX_HEARTBEAT_VSM);
		currentState = vsm->state;
	}

	// set other boards to GLV_ON
	setHVC_heartbeat(0, 1, 2, 1, 1); // errorStatus = 0 (none), hvcState = 2 (STANDBY -- GLV_ON)
	setCDC_heartbeat(1,zeroes, zeroes);
	setDIM_heartbeat(1,zeroes, zeroes);
	setPTC_heartbeat(1,zeroes, zeroes);

	// DIM request for HV_EN sets VSM to REQ_PRECHARGE
	setDIM_request(CMR_CAN_HV_EN, 0, 0, 0);


	cmr_canVSMStatus_t *vsmInternal = getPayloadVEH(CANRX_HEARTBEAT_VSM);
	uint8_t internalState = vsmInternal->internalState;

	while (internalState != CMR_CAN_VSM_STATE_REQ_PRECHARGE) {
		vsmInternal = getPayloadVEH(CANRX_HEARTBEAT_VSM);
		internalState = vsmInternal->internalState;
	}

	// set other boards to HV_EN
	setCDC_heartbeat(CMR_CAN_HV_EN, zeroes, zeroes);
	setDIM_heartbeat(CMR_CAN_HV_EN, zeroes, zeroes);
	setPTC_heartbeat(CMR_CAN_HV_EN, zeroes, zeroes);

	// make HVC go thru precharge states
	setHVC_heartbeat(0, 1, CMR_CAN_HVC_STATE_DRIVE_PRECHARGE, 1, 1); 
	setHVC_heartbeat(0, 1, CMR_CAN_HVC_STATE_DRIVE_PRECHARGE_COMPLETE, 1, 1);
	setHVC_heartbeat(0, 1, CMR_CAN_HVC_STATE_DRIVE, 1, 1);

	// HVCMode to CMR_CAN_HVC_MODE_START sets VSM to RUN_BMS
	setHVC_heartbeat(0, CMR_CAN_HVC_MODE_START, 0x06, 1, 1);

	vsmInternal = getPayloadVEH(CANRX_HEARTBEAT_VSM);
	internalState = vsmInternal->internalState;

	while (internalState != CMR_CAN_VSM_STATE_RUN_BMS) {
		vsmInternal = getPayloadVEH(CANRX_HEARTBEAT_VSM);
		internalState = vsmInternal->internalState;
	}

	// HVCMode to CMR_CAN_HVC_MODE_RUN sets VSM to DCDC_EN (kinda useless state?)
	setHVC_heartbeat(0, CMR_CAN_HVC_MODE_RUN, CMR_CAN_HVC_STATE_DRIVE_PRECHARGE, 1, 1);

	vsmInternal = getPayloadVEH(CANRX_HEARTBEAT_VSM);
	internalState = vsmInternal->internalState;

	while (internalState != CMR_CAN_VSM_STATE_DCDC_EN) {
		vsmInternal = getPayloadVEH(CANRX_HEARTBEAT_VSM);
		internalState = vsmInternal->internalState;
	}

	// set AMK values to CMR_CAN_AMK_STATUS_SYSTEM_READY get VSM in HV_EN
	setinverter1(1, 0, 0, 0);
	setinverter2(1, 0, 0, 0);
	setinverter3(1, 0, 0, 0);
	setinverter4(1, 0, 0, 0);

	vsm = getPayloadVEH(CANRX_HEARTBEAT_VSM);
	currentState = vsm->state;

	while (currentState != CMR_CAN_HV_EN) {
		vsm = getPayloadVEH(CANRX_HEARTBEAT_VSM);
		currentState = vsm->state;
	}
	
	// other boards already in HV_EN

	// dim request for RTD sets along with other throttle and brake pos condition sets VSM to RTD
	setDIM_request(CMR_CAN_RTD, 0, 0, 0);
	setFSM_data(0, 5, 40, 0, 0); // throttlePosition = 0, brakePressureThreshold_PSI = 40

	vsmInternal = getPayloadVEH(CANRX_HEARTBEAT_VSM);
	internalState = vsmInternal->internalState;

	while (internalState != CMR_CAN_VSM_STATE_RTD) {
		vsmInternal = getPayloadVEH(CANRX_HEARTBEAT_VSM);
		internalState = vsmInternal->internalState;
	}

	// set other boards to RTD
	setHVC_heartbeat(0, 1, 0x05, 1, 1); // hvcState = 0x05 (CMR_CAN_HVC_STATE_DRIVE)
	setCDC_heartbeat(CMR_CAN_RTD, zeroes, zeroes);
	setDIM_heartbeat(CMR_CAN_RTD, zeroes, zeroes);
	setPTC_heartbeat(CMR_CAN_RTD, zeroes, zeroes);

	vsmInternal = getPayloadVEH(CANRX_HEARTBEAT_VSM);
	internalState = vsmInternal->internalState;

	return internalState == CMR_CAN_VSM_STATE_RTD;
}

static void testbenchTask(void *pvParameters) {
	TickType_t lastWakeTime = xTaskGetTickCount();
	while (1) {
//		testRTD();
		testClearErrorState();
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
