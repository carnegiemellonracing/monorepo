/**
 * @file can.c
 * @brief Board-specific CAN implementation.
 *
 * Adding a new periodic message struct:
 *
 * 1. Add the corresponding index to the `canRX_t` enum in `can.h`.
 * 2. Add a configuration entry in `canRXMeta` at that index.
 * 3. Access the message using `canRXMeta[index]`.
 *
 * @author Carnegie Mellon Racing
 */

#include <string.h>     // memcpy()
#include <CMR/tasks.h>  // Task interface
#include "can.h"      // Interface to implement

/** @brief Primary CAN interface. */
static cmr_can_t veh_can;
static cmr_can_t daq_can;
static cmr_can_t trac_can;



// -------------- DEFINE STATIC VARIABLES TO BE CHANGED (WITH GETTERS AND SETTERS) HERE -------------- //
static cmr_canVSMSensors_t sensors = {.brakePressureRear_PSI = 0,
        							  .hallEffect_cA = 0,
									  .coulombCount_C = 0.0
};


static cmr_canHeartbeat_t VSMHeartbeat;

static cmr_canVSMStatus_t VSMStatus = {.internalState = CMR_CAN_VSM_STATE_GLV_ON,
									   .badStateMatrix = CMR_CAN_VSM_ERROR_SOURCE_NONE,
									   .latchMatrix = CMR_CAN_VSM_LATCH_NONE,
									   .moduleTimeoutMatrix = CMR_CAN_VSM_ERROR_SOURCE_NONE
};

static cmr_canAMKActualValues2_t inv1 = {.coldPlateTemp_dC = 0,
		                                 .motorTemp_dC = 0,
										 .errorCode = 0,
										 .igbtTemp_dC = 0

};
static cmr_canAMKActualValues2_t inv2 = {.coldPlateTemp_dC = 0,
		                                 .motorTemp_dC = 0,
										 .errorCode = 0,
										 .igbtTemp_dC = 0

};
static cmr_canAMKActualValues2_t inv3 = {.coldPlateTemp_dC = 0,
		                                 .motorTemp_dC = 0,
										 .errorCode = 0,
										 .igbtTemp_dC = 0

};
static cmr_canAMKActualValues2_t inv4 = {.coldPlateTemp_dC = 0,
		                                 .motorTemp_dC = 0,
										 .errorCode = 0,
										 .igbtTemp_dC = 0

};
static cmr_canFSMData_t fsmData = {.brakePedalPosition = 0,
								   .brakePressureFront_PSI = 0,
								   .steeringWheelAngle_deg = 0,
								   .throttlePosition = 0,
								   .steeringWheelAngle_deg = 0
};
static cmr_canBMSMinMaxCellTemperature_t AC_Temps = {.maxCellTemp_C = 0,
													 .minCellTemp_C = 0,
													 .minTempBMBNum = 0,
													 .maxTempBMBNum = 0,
													 .maxTempCellNum = 0,
													 .minTempCellNum = 0
};
// --------------------------------------------------------------------------------------------------- //



cmr_canRXMeta_t canRXMeta_VEH[] = {
    [CANRX_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    }, // and more (refer to VSM repo CAN file) i.e. vsm status, sensors, etc.
};

cmr_canRXMeta_t canRXMeta_DAQ[] = {
    [CANRX_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    }, // and more (refer to VSM repo CAN file) i.e. vsm status, sensors, etc.
};

cmr_canRXMeta_t canRXMeta_TRAC[] = {
    [CANRX_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    }, // and more (refer to VSM repo CAN file) i.e. vsm status, sensors, etc.
};

int canTX_VEH(cmr_canID_t id, const void *data, size_t len, TickType_t timeout) {
    return cmr_canTX(&veh_can, id, data, len, timeout);
}

int canTX_DAQ(cmr_canID_t id, const void *data, size_t len, TickType_t timeout) {
    return cmr_canTX(&daq_can, id, data, len, timeout);
}

int canTX_TRAC(cmr_canID_t id, const void *data, size_t len, TickType_t timeout) {
    return cmr_canTX(&trac_can, id, data, len, timeout);
}


// -------------- DEFINE CAN MESSAGE CONSTRUCTION FUNCTIONS HERE -------------- //
void setVSMHeartbeat(uint8_t state) {
	VSMHeartbeat.state = state;
	uint8_t errors[2] = {0, 0};
	uint8_t warnings[2] = {0, 0};

    memcpy(&VSMHeartbeat.error, &errors, sizeof(VSMHeartbeat.error));
    memcpy(&VSMHeartbeat.warning, &warnings, sizeof(VSMHeartbeat.warning));

}

void setVSMStatus(uint16_t internalState) {
	VSMStatus.internalState = internalState;
}

void setACTemps(uint8_t min, uint8_t max) {
	AC_Temps.minCellTemp_C = min;
	AC_Temps.maxCellTemp_C = max;
}

void setIGBT1Temp(uint8_t temp)
{
	inv2.igbtTemp_dC = temp;
}
void setIGBT2Temp(uint8_t temp)
{
	inv2.igbtTemp_dC = temp;
}
void setIGBT3Temp(uint8_t temp)
{
	inv2.igbtTemp_dC = temp;
}
void setIGBT4Temp(uint8_t temp)
{
	inv2.igbtTemp_dC = temp;
}

void setBrakePressure(uint16_t brakePressure) {
	sensors.brakePressureRear_PSI = brakePressure;
}

// ---------------------------------------------------------------------------- //

/** @brief CAN Tx function */
int canTX(cmr_can_t bus, cmr_canID_t id, const void *data, size_t len, TickType_t timeout) {
    return cmr_canTX(&bus, id, data, len, timeout);
}


/** @brief CAN 1 Hz TX priority. */
static const uint32_t canTX1Hz_priority = 3;

/** @brief CAN 1 Hz TX period (milliseconds). */
static const TickType_t canTX1Hz_period_ms = 1000;

/** @brief CAN 1 Hz TX task. */
static cmr_task_t canTX1Hz_task;

/**
 * @brief Task for sending CAN messages at 1 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX1Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {

        // insert 1Hz messages

    	// send bms pack temps
        canTX(veh_can, CMR_CANID_HVC_MIN_MAX_CELL_TEMPERATURE, &AC_Temps, sizeof(AC_Temps), canTX1Hz_period_ms);
        vTaskDelayUntil(&lastWakeTime, canTX1Hz_period_ms);
    }
}


/** @brief CAN 5 Hz TX priority. */
static const uint32_t canTX5Hz_priority = 2;
/** @brief CAN 5 Hz TX period (milliseconds). */
static const TickType_t canTX5Hz_period_ms = 100;
/** @brief CAN 5 Hz TX task. */
static cmr_task_t canTX5Hz_task;


/**
 * @brief Task for sending CAN messages at 5 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX5Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {

        // insert 5Hz messages

    	// send inverter temps
        canTX(veh_can, CMR_CANID_AMK_1_ACT_2, &inv1, sizeof(inv1), canTX5Hz_period_ms);
        canTX(veh_can, CMR_CANID_AMK_2_ACT_2, &inv2, sizeof(inv2), canTX5Hz_period_ms);
        canTX(veh_can, CMR_CANID_AMK_3_ACT_2, &inv3, sizeof(inv3), canTX5Hz_period_ms);
        canTX(veh_can, CMR_CANID_AMK_4_ACT_2, &inv4, sizeof(inv4), canTX5Hz_period_ms);

        vTaskDelayUntil(&lastWakeTime, canTX5Hz_period_ms);
    }
}


/** @brief CAN 10 Hz TX priority. */
static const uint32_t canTX10Hz_priority = 3;

/** @brief CAN 10 Hz TX period (milliseconds). */
static const TickType_t canTX10Hz_period_ms = 100;

/** @brief CAN 10 Hz TX task. */
static cmr_task_t canTX10Hz_task;

/**
 * @brief Task for sending CAN messages at 10 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX10Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {

        // insert 10Hz messages

        vTaskDelayUntil(&lastWakeTime, canTX10Hz_period_ms);
    }
}


/** @brief CAN 100 Hz TX priority. */
static const uint32_t canTX100Hz_priority = 5;

/** @brief CAN 100 Hz TX period (milliseconds). */
static const TickType_t canTX100Hz_period_ms = 10;

/** @brief CAN 100 Hz TX task. */
static cmr_task_t canTX100Hz_task;

static void canTX100Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        
        // insert 100Hz messages
        // send VSM heartbeat
        canTX(veh_can, CMR_CANID_HEARTBEAT_VSM, &VSMHeartbeat, sizeof(VSMHeartbeat), canTX100Hz_period_ms);
        // send VSM status
        canTX(veh_can, CMR_CANID_VSM_STATUS, &VSMStatus, sizeof(VSMStatus), canTX100Hz_period_ms);
        // send FSM
        canTX(veh_can, CMR_CANID_FSM_DATA, &fsmData, sizeof(fsmData), canTX100Hz_period_ms);

        vTaskDelayUntil(&lastWakeTime, canTX100Hz_period_ms);
    }
}


/** @brief CAN 200 Hz TX priority. */
static const uint32_t canTX200Hz_priority = 5;

/** @brief CAN 200 Hz TX period (milliseconds). */
static const TickType_t canTX200Hz_period_ms = 5;

/** @brief CAN 200 Hz TX task. */
static cmr_task_t canTX200Hz_task;

/**
 * @brief Task for sending CAN messages at 200 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canTX200Hz(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {

        // insert 200Hz messages

    	//send VSM sensors
        canTX(veh_can, CMR_CANID_VSM_SENSORS, &sensors, sizeof(sensors), canTX200Hz_period_ms);
        vTaskDelayUntil(&lastWakeTime, canTX200Hz_period_ms);
    }
}

void *getPayloadDAQ(canRX_DAQ_t rxMsg) {
    if (rxMsg >= CANRX_LEN_DAQ) {
        return NULL; // TODO switch to configassert
    }

    cmr_canRXMeta_t *rxMeta = &(canRXMeta_DAQ[rxMsg]);

    return (void *)(&rxMeta->payload);
}

void *getPayloadVEH(canRX_VEH_t rxMsg) {
    if (rxMsg >= CANRX_LEN_VEH) {
        return NULL; // TODO switch to configassert
    }

    cmr_canRXMeta_t *rxMeta = &(canRXMeta_VEH[rxMsg]);

    return (void *)(&rxMeta->payload);
}

void *getPayloadTRAC(canRX_TRAC_t rxMsg) {
    if (rxMsg >= CANRX_LEN_TRAC) {
        return NULL; // TODO switch to configassert
    }

    cmr_canRXMeta_t *rxMeta = &(canRXMeta_TRAC[rxMsg]);

    return (void *)(&rxMeta->payload);
}


/**
 * @brief Initializes the CAN interface.
 * @note No CAN filtering since we want to observe all messages produced by the test board
 */
void canInit(void) {
    // CAN2 initialization.
    cmr_canInit(
        &daq_can, CAN1,
		CMR_CAN_BITRATE_500K,
        canRXMeta_DAQ, sizeof(canRXMeta_VEH) / sizeof(canRXMeta_VEH[0]),
        NULL,
        GPIOA, GPIO_PIN_11,     // CAN2 RX port/pin.
        GPIOA, GPIO_PIN_12      // CAN2 TX port/pin.
    );
    cmr_canInit(
        &veh_can, CAN2,
		CMR_CAN_BITRATE_500K,
        canRXMeta_VEH, sizeof(canRXMeta_VEH) / sizeof(canRXMeta_VEH[0]),
        NULL,
        GPIOB, GPIO_PIN_12,     // CAN2 RX port/pin.
        GPIOB, GPIO_PIN_13      // CAN2 TX port/pin.
    );
    cmr_canInit(
        &trac_can, CAN3,
		CMR_CAN_BITRATE_500K,
        canRXMeta_TRAC, sizeof(canRXMeta_VEH) / sizeof(canRXMeta_VEH[0]),
        NULL,
        GPIOA, GPIO_PIN_8,     // CAN2 RX port/pin.
        GPIOB, GPIO_PIN_4      // CAN2 TX port/pin.
    );

    // Task initialization.
    cmr_taskInit(
        &canTX1Hz_task,
        "CAN TX 1Hz",
        canTX1Hz_priority,
        canTX1Hz,
        NULL
    );
    cmr_taskInit(
    	&canTX5Hz_task,
        "CAN TX 5Hz",
        canTX5Hz_priority,
        canTX5Hz,
        NULL
    );
    cmr_taskInit(
        &canTX10Hz_task,
        "CAN TX 10Hz",
        canTX10Hz_priority,
        canTX10Hz,
        NULL
    );
    cmr_taskInit(
        &canTX100Hz_task,
        "CAN TX 100Hz",
        canTX100Hz_priority,
        canTX100Hz,
        NULL
    );
    cmr_taskInit(
        &canTX200Hz_task,
        "CAN TX 200Hz",
        canTX200Hz_priority,
        canTX200Hz,
        NULL
    );
}
