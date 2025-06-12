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
static cmr_can_t vsm_can;


// -------------- DEFINE STATIC VARIABLES TO BE CHANGED (WITH GETTERS AND SETTERS) HERE -------------- //


// --------------------------------------------------------------------------------------------------- //
static cmr_canHVCHeartbeat_t HVC_heartbeat = {.errorStatus = 0, .hvcMode = 0, .hvcState = 0, .relayStatus = 0, .uptime_s = 0};
static cmr_canHVIHeartbeat_t HVI_heartbeat = {.packCurrent_dA =0 ,.packVoltage_cV =0,.packPower_W = 0};
static cmr_canHeartbeat_t FSM_heartbeat = {.state =0 ,.error ={0,0},.warning ={0,0}};
static cmr_canHeartbeat_t PTC_heartbeat = {.state =0 ,.error ={0,0},.warning ={0,0}};
static cmr_canHeartbeat_t DIM_heartbeat = {.state =0 ,.error ={0,0},.warning ={0,0}};
static cmr_canHeartbeat_t CDC_heartbeat = {.state =0 ,.error ={0,0},.warning ={0,0}};
static cmr_canFSMData_t FSM_data = {.torqueRequested = 0, .throttlePosition = 0, .brakePressureFront_PSI = 0, .brakePedalPosition = 0};
static cmr_canFSMSWAngle_t SWAngle_data = {.steeringWheelAngle_millideg_FL = 0, .steeringWheelAngle_millideg_FR = 0};
static cmr_canDIMRequest_t DIM_request = {.requestedState = 0, .requestedGear = 0, .requestedDrsMode = 0, .requestedDriver = 0};
static cmr_canAMKActualValues1_t inverter1 = {.status_bv = 0, .velocity_rpm = 0, .torqueCurrent_raw = 0, .magCurrent_raw = 0};
static cmr_canAMKActualValues1_t inverter2 = {.status_bv = 0, .velocity_rpm = 0, .torqueCurrent_raw = 0, .magCurrent_raw = 0};
static cmr_canAMKActualValues1_t inverter3 = {.status_bv = 0, .velocity_rpm = 0, .torqueCurrent_raw = 0, .magCurrent_raw = 0};
static cmr_canAMKActualValues1_t inverter4 = {.status_bv = 0, .velocity_rpm = 0, .torqueCurrent_raw = 0, .magCurrent_raw = 0};






cmr_canRXMeta_t canRXMeta_VEH[] = {
		[CANRX_HEARTBEAT_VSM] = {
		        .canID = CMR_CANID_HEARTBEAT_VSM,
		        .timeoutError_ms = 50,
		        .timeoutWarn_ms = 25
		    }, // and more (refer to VSM repo CAN file) i.e. vsm status, sensors, etc.
			[1] = {
					.canID = CMR_CANID_VSM_SENSORS,
					.timeoutError_ms = 50,
					.timeoutWarn_ms = 25
			},
			[2] = {
					.canID = CMR_CANID_HVC_COMMAND,
					.timeoutError_ms = 50,
					.timeoutWarn_ms = 25
			},
			[3] = {
					.canID = CMR_CANID_VSM_POWER_DIAGNOSTICS,
					.timeoutError_ms = 50,
					.timeoutWarn_ms = 25
			},
			[4] = {
					.canID = CMR_CANID_VSM_STATUS,
					.timeoutError_ms = 50,
					.timeoutWarn_ms = 25
			},
			[5] = {
					.canID = CMR_CANID_VSM_LATCHED_STATUS,
					.timeoutError_ms = 150,
					.timeoutWarn_ms = 100
			}
};

cmr_canRXMeta_t canRXMeta_DAQ[] = {

};

cmr_canRXMeta_t canRXMeta_TRAC[] = {
    [CANRX_HEARTBEAT_VSM] = {
        .canID = CMR_CANID_HEARTBEAT_VSM,
        .timeoutError_ms = 50,
        .timeoutWarn_ms = 25
    }, // and more (refer to VSM repo CAN file) i.e. vsm status, sensors, etc.
};

//cmr_canRX_Meta_t canRXMeta_VSM[] = {
//    [CANRX_HEARTBEAT_VSM] = {
//        .canID = CMR_CANID_HEARTBEAT_VSM
//    }
//};

int canTX_VEH(cmr_canID_t id, const void *data, size_t len, TickType_t timeout) {
    return cmr_canTX(&veh_can, id, data, len, timeout);
}

int canTX_DAQ(cmr_canID_t id, const void *data, size_t len, TickType_t timeout) {
    return cmr_canTX(&daq_can, id, data, len, timeout);
}

int canTX_TRAC(cmr_canID_t id, const void *data, size_t len, TickType_t timeout) {
    return cmr_canTX(&trac_can, id, data, len, timeout);
}

int canTX_VSM(cmr_canID_t id, const void *data, size_t len, TickType_t timeout) {
    return cmr_canTX(&vsm_can, id, data, len, timeout);
}


// -------------- DEFINE CAN MESSAGE CONSTRUCTION FUNCTIONS HERE -------------- //
void setHVC_heartbeat(uint16_t errorStatus, uint8_t hvcMode, uint8_t hvcState, uint8_t relayStatus, uint8_t uptime_s) {
    HVC_heartbeat.errorStatus = errorStatus;
    HVC_heartbeat.hvcMode = hvcMode;
    HVC_heartbeat.hvcState = hvcState;
    HVC_heartbeat.relayStatus = relayStatus;
    HVC_heartbeat.uptime_s = uptime_s;
}

void setHVI_heartbeat(int16_t packCurrent_dA, uint16_t packVoltage_cV, int32_t packPower_W) {
    HVI_heartbeat.packCurrent_dA = packCurrent_dA;
    HVI_heartbeat.packVoltage_cV = packVoltage_cV;
    HVI_heartbeat.packPower_W = packPower_W;
}

void setFSM_heartbeat(uint8_t state) {
    FSM_heartbeat.state = state;
    uint8_t errors[2] = {0,0};
    uint8_t warnings[2] = {0,0};

    memcpy(&FSM_heartbeat.error, &errors, sizeof(FSM_heartbeat.error));
    memcpy(&FSM_heartbeat.warning, &warnings, sizeof(FSM_heartbeat.warning));
}

void setPTC_heartbeat(uint8_t state, uint8_t *error, uint8_t *warning) {
    PTC_heartbeat.state = state;
    uint8_t errors[2] = {0,0};
    uint8_t warnings[2] = {0,0};

    memcpy(&PTC_heartbeat.error, &errors, sizeof(PTC_heartbeat.error));
    memcpy(&PTC_heartbeat.warning, &warnings, sizeof(PTC_heartbeat.warning));
}

void setDIM_heartbeat(uint8_t state, uint8_t *error, uint8_t *warning) {
    DIM_heartbeat.state = state;
    uint8_t errors[2] = {0,0};
    uint8_t warnings[2] = {0,0};

    memcpy(&DIM_heartbeat.error, &errors, sizeof(DIM_heartbeat.error));
    memcpy(&DIM_heartbeat.warning, &warnings, sizeof(DIM_heartbeat.warning));
}

void setCDC_heartbeat(uint8_t state, uint8_t *error, uint8_t *warning) {
    CDC_heartbeat.state = state;
    uint8_t errors[2] = {0,0};
    uint8_t warnings[2] = {0,0};

    memcpy(&CDC_heartbeat.error, &errors, sizeof(CDC_heartbeat.error));
    memcpy(&CDC_heartbeat.warning, &warnings, sizeof(CDC_heartbeat.warning));
}

void setFSM_data(uint8_t torqueRequested, uint8_t throttlePosition, uint16_t brakePressureFront_PSI, uint8_t brakePedalPosition, int32_t steeringWheelAngle_millideg_FL, int32_t steeringWheelAngle_millideg_FR) {
    FSM_data.torqueRequested = torqueRequested;
    FSM_data.throttlePosition = throttlePosition;
    FSM_data.brakePressureFront_PSI = brakePressureFront_PSI;
    FSM_data.brakePedalPosition = brakePedalPosition;
    SWAngle_data.steeringWheelAngle_millideg_FL = steeringWheelAngle_millideg_FL;
    SWAngle_data.steeringWheelAngle_millideg_FR = steeringWheelAngle_millideg_FR;
}

void setDIM_request(uint8_t requestedState, uint8_t requestedGear, uint8_t requestedDrsMode, uint8_t requestedDriver) {
    DIM_request.requestedState = requestedState;
    DIM_request.requestedGear = requestedGear;
    DIM_request.requestedDrsMode = requestedDrsMode;
    DIM_request.requestedDriver = requestedDriver;
}

void setinverter1(uint16_t status_bv, int16_t velocity_rpm, int16_t torqueCurrent_raw, int16_t magCurrent_raw) {
    inverter1.status_bv = status_bv;
    inverter1.velocity_rpm = velocity_rpm;
    inverter1.torqueCurrent_raw = torqueCurrent_raw;
    inverter1.magCurrent_raw = magCurrent_raw;
}

void setinverter2(uint16_t status_bv, int16_t velocity_rpm, int16_t torqueCurrent_raw, int16_t magCurrent_raw) {
    inverter2.status_bv = status_bv;
    inverter2.velocity_rpm = velocity_rpm;
    inverter2.torqueCurrent_raw = torqueCurrent_raw;
    inverter2.magCurrent_raw = magCurrent_raw;
}

void setinverter3(uint16_t status_bv, int16_t velocity_rpm, int16_t torqueCurrent_raw, int16_t magCurrent_raw) {
    inverter3.status_bv = status_bv;
    inverter3.velocity_rpm = velocity_rpm;
    inverter3.torqueCurrent_raw = torqueCurrent_raw;
    inverter3.magCurrent_raw = magCurrent_raw;
}

void setinverter4(uint16_t status_bv, int16_t velocity_rpm, int16_t torqueCurrent_raw, int16_t magCurrent_raw) {
    inverter4.status_bv = status_bv;
    inverter4.velocity_rpm = velocity_rpm;
    inverter4.torqueCurrent_raw = torqueCurrent_raw;
    inverter4.magCurrent_raw = magCurrent_raw;
}

// ---------------------------------------------------------------------------- //




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

        vTaskDelayUntil(&lastWakeTime, canTX1Hz_period_ms);
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
    	cmr_canTX(&veh_can, CMR_CANID_HEARTBEAT_HVC, &HVC_heartbeat, sizeof(HVC_heartbeat), 100);
    	cmr_canTX(&veh_can, CMR_CANID_HEARTBEAT_CDC, &CDC_heartbeat, sizeof(CDC_heartbeat), 100);
    	cmr_canTX(&veh_can, CMR_CANID_HEARTBEAT_DIM, &DIM_heartbeat, sizeof(DIM_heartbeat), 100);
    	cmr_canTX(&veh_can, CMR_CANID_HEARTBEAT_PTC, &PTC_heartbeat, sizeof(PTC_heartbeat), 100);
    	cmr_canTX(&veh_can, CMR_CANID_FSM_DATA, &FSM_data, sizeof(FSM_data), 100);
    	cmr_canTX(&veh_can, CMR_CANID_DIM_REQUEST, &DIM_request, sizeof(DIM_request), 100);
    	cmr_canTX(&veh_can, CMR_CANID_AMK_RR_ACT_1, &inverter1, sizeof(inverter1), 100);
    	cmr_canTX(&veh_can, CMR_CANID_AMK_FR_ACT_1, &inverter2, sizeof(inverter1), 100);
    	cmr_canTX(&veh_can, CMR_CANID_AMK_RL_ACT_1, &inverter3, sizeof(inverter1), 100);
    	cmr_canTX(&veh_can, CMR_CANID_AMK_FL_ACT_1, &inverter4, sizeof(inverter1), 100);


        // send VSM heartbeat
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
//
//void *getPayloadVSM(canRX_VSM_t rxMsg) {
//    if (rxMsg >= CANRX_LEN_VSM) {
//        return NULL;
//    }

//    cmr_canRXMeta_t *rxMeta = &(canRXMeta_VSM[rxMsg]);
//
//    return (void *)(&rxMeta->payload);
//}

/**
 * @brief Initializes the CAN interface.
 * @note No CAN filtering since we want to observe all messages produced by the test board
 */
void canInit(void) {
    // CAN2 initialization.
	const cmr_canFilter_t canFilters[] = {
			{
					.isMask = false,
					.rxFIFO = CAN_RX_FIFO0,
					.ids = {
							0x100,
							0x100,
							0x130,
							0x130
					}
			}
	};
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
    cmr_canFilter(&veh_can, canFilters, sizeof(canFilters)/sizeof(canFilters));

    // Task initialization.
    cmr_taskInit(
        &canTX1Hz_task,
        "CAN TX 1Hz",
        canTX1Hz_priority,
        canTX1Hz,
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
