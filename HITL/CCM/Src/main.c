
#include <stm32f4xx_hal.h>  // HAL interface

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/tasks.h>  // Task interface

#include "i2c.h"
#include "gpio.h"
#include "can.h"
#include "pwm.h"

// Testbentch function
/**
* TODO: maybe adding timeout and return something when successfully
* entered charging state
**/ 

typedef enum {
    CMR_CCM_STATE_ERROR = 0,        /**< @brief Error state. */
    CMR_CCM_STATE_CLEAR_ERROR,      /**< @brief Clear error state. */
    CMR_CCM_STATE_CLEAR_HVC,        /**< @brief Wait for HVC to enter clear error state */
    CMR_CCM_STATE_IDLE_HVC,         /**< @brief Wait for HVC to enter standby state */
    CMR_CCM_STATE_STANDBY,          /**< @brief Charger control module on. */
    CMR_CCM_STATE_CHARGE_REQ,       /**< @brief Requesting charging mode */
    CMR_CCM_STATE_CHARGE,           /**< @brief Charging. */
    CMR_CCM_STATE_SLOW_CHARGE,      /**< @brief Cell Balancing */
    CMR_CCM_STATE_SHUTDOWN,         /**< @brief Shutdown chargers before HVC shutdown */
    CMR_CCM_STATE_LEN               /**< @brief Number of CCM states. */
} cmr_CCMState_t;

bool ChargingStartTestFuntion() {
	cmr_canRXMeta_t *metaCCMState = canRXMeta_VEH + CANRX_CCM_HEARTBEAT;
	volatile cmr_canHeartbeat_t *canCCMHeartbeat = getPayloadVEH(CANRX_CCM_HEARTBEAT);
	uint8_t state = canCCMHeartbeat->state;

	uint8_t timeout_counter = 0;

	// Entering CMR_CCM_STATE_CLEAR_ERROR 
	setCCMCommand(CMR_CAN_CCM_MODE_RESET);
	while (state != CMR_CCM_STATE_CLEAR_ERROR){
		if (timeout_counter > 10){return false;}
		vTaskDelay(100);
		volatile cmr_canHeartbeat_t *canCCMHeartbeat = getPayloadVEH(CANRX_CCM_HEARTBEAT);
		uint8_t state = canCCMHeartbeat->state;

		timeout_counter ++;
	}

	// Entering CMR_·CCM_STATE_CLEAR_HVC
	timeout_counter = 0;
	setHVCHeartbeat(CMR_CAN_HVC_STATE_CLEAR_ERROR, 0);
	while (state != CMR_CCM_STATE_CLEAR_HVC){
		if (timeout_counter > 10){return false;}
		vTaskDelay(100);
		volatile cmr_canHeartbeat_t *canCCMHeartbeat = getPayloadVEH(CANRX_CCM_HEARTBEAT);
		uint8_t state = canCCMHeartbeat->state;

		timeout_counter ++;
	}
	
	// Entering CMR_CCM_STATE_IDLE_HVC
	timeout_counter = 0;
	setHVCHeartbeat(CMR_CAN_HVC_STATE_STANDBY, CMR_CAN_HVC_MODE_IDLE);
	while (state != CMR_CCM_STATE_IDLE_HVC){
		if (timeout_counter > 10){return false;}
		vTaskDelay(100);
		volatile cmr_canHeartbeat_t *canCCMHeartbeat = getPayloadVEH(CANRX_CCM_HEARTBEAT);
		uint8_t state = canCCMHeartbeat->state;

		timeout_counter ++;
	}
	
	// Entering CMR_CCM_STATE_CHARGE_REQ	
	timeout_counter = 0;
	setCCMCommand(CMR_CAN_CCM_MODE_RUN);
	while (state != CMR_CCM_STATE_CHARGE_REQ){
		if (timeout_counter > 10){return false;}
		vTaskDelay(100);
		volatile cmr_canHeartbeat_t *canCCMHeartbeat = getPayloadVEH(CANRX_CCM_HEARTBEAT);
		uint8_t state = canCCMHeartbeat->state;

		timeout_counter ++;
	}

	// Entering CMR_CCM_STATE_CHARGE
	timeout_counter = 0;
	setHVCHeartbeat(CMR_CAN_HVC_STATE_CHARGE_TRICKLE, CMR_CAN_HVC_MODE_IDLE);
	while (state != CMR_CCM_STATE_CHARGE){
		if (timeout_counter > 10){return false;}
		vTaskDelay(100);
		timeout_counter ++;
	}
	return true;
}

/** @brief Testbench Task priority. */
static const uint32_t testbenchPriority = 2;
/** @brief Testbench Task period (milliseconds). */
static const TickType_t canTX5Hz_period_ms = 1000;
/** @brief CAN 5 Hz TX task. */
static cmr_task_t testbenchTask_CCM;

static void testbenchTask(void *pvParameters) {
	TickType_t lastWakeTime = xTaskGetTickCount();
	while (1) {
		ChargingStartTestFuntion();
		vTaskDelayUntil(&lastWakeTime, 1000); // sets 1Hz frequency (every 1000 ms)
	}
}



int main(void) {

	// Initialising interfaces: GPIO, PWM, CAN, I2C
	gpioInit();
	i2cInit();
	pwmReadInit();
	canInit();


	// Setting outputs
	cmr_gpioWrite(GPIO_MCU1, 0);
	cmr_gpioWrite(DIGITAL_OUT_5V_MCU, 1);

	cmr_taskInit(
		&testbenchTask_CCM,
		"CAN TX 100Hz",
		testbenchPriority,
		testbenchTask,
		NULL
	);
	
	// Reading inputs to verify board
	TickType_t lastWakeTime = xTaskGetTickCount();
	while (1) {


		vTaskDelayUntil(&lastWakeTime, 1000); // sets 1Hz frequency (every 1000 ms)
	}





	return 0;
}
