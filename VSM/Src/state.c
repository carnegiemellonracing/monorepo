/**
 * @file state.c
 * @brief Vehicle Safety Module state machine.
 *
 * @note Full state machine description available at
 * http://cmr-linux.club.cc.cmu.edu/confluence/display/EN/19e+Vehicle+Safety+Module
 *
 * @author Carnegie Mellon Racing
 */

// ------------------------------------------------------------------------------------------------
// Includes

#include <CMR/tasks.h>      // Task interface, taskENTER_CRITICAL(), taskEXIT_CRITICAL()
#include <CMR/gpio.h>       // cmr_gpioRead()

#include "state.h"          // Interface to implement
#include "error.h"          // updateCurrentErrors, updateCurrentWarnings
#include "can.h"            // cmr_canRXMeta_t, canRXMeta[], canRX_t,
                            // getPayload(), getModuleState()
#include "sensors.h"        // Sensors interface
#include "gpio.h"           // gpio_t

// ------------------------------------------------------------------------------------------------
// Globals
#define AS_WAKEUP_TIME 5000
#define AS_FINISHED_TIME 30000

/** @brief Mapping of VSM internal states to vehicle states. Indexed by cmr_canVSMState_t. */
cmr_canState_t vsmToCANState[] = {
    [CMR_CAN_VSM_STATE_ERROR]           = CMR_CAN_ERROR,
    [CMR_CAN_VSM_STATE_CLEAR_ERROR]     = CMR_CAN_CLEAR_ERROR,
    [CMR_CAN_VSM_STATE_GLV_ON]          = CMR_CAN_GLV_ON,
    [CMR_CAN_VSM_STATE_REQ_PRECHARGE]   = CMR_CAN_GLV_ON,
    [CMR_CAN_VSM_STATE_RUN_BMS]         = CMR_CAN_GLV_ON,
    [CMR_CAN_VSM_STATE_INVERTER_EN]     = CMR_CAN_GLV_ON,
    [CMR_CAN_VSM_STATE_HV_EN]           = CMR_CAN_HV_EN,
    [CMR_CAN_VSM_STATE_RTD]             = CMR_CAN_RTD,
    [CMR_CAN_VSM_STATE_AS_READY]        = CMR_CAN_AS_READY,
    [CMR_CAN_VSM_STATE_AS_DRIVING]      = CMR_CAN_AS_DRIVING,
    [CMR_CAN_VSM_STATE_AS_FINISHED]     = CMR_CAN_AS_FINISHED,
    [CMR_CAN_VSM_STATE_AS_EMERGENCY]    = CMR_CAN_AS_EMERGENCY
};

/** @brief Last time that vehicle state was changed */
volatile TickType_t lastStateChangeTime_ms = 0;

/** @brief HVC mode request. */
volatile cmr_canHVCMode_t hvcModeRequest = CMR_CAN_HVC_MODE_ERROR;

// ------------------------------------------------------------------------------------------------
// Static globals

/** @brief Time to sound RTD buzzer. See rule EV.6.11.5.a. */
static const TickType_t rtdBuzzerTime_ms = 1500;

/** @brief Time to sound AS Emergency buzzer. See rule T.14.9.5 of FSG Rules */
static const TickType_t ASEmergencyBuzzerTime_ms = 9000;

/** @brief Time between on and off of AS Emergency buzzer. See rule T.14.9.5 of FSG Rules
 * Provides frequency of 5 hz
 */
static const TickType_t ASEmergencySwitchingTime_ms = 100;

/** @brief Current Vehicle Safety Module state and errors. */
static volatile vsmStatus_t vsmStatus = {
    .heartbeatErrors = CMR_CAN_ERROR_NONE,
    .heartbeatWarnings = CMR_CAN_WARN_NONE,
    .canVSMStatus = {
        .internalState = CMR_CAN_VSM_STATE_ERROR,
        .moduleTimeoutMatrix = CMR_CAN_VSM_ERROR_SOURCE_NONE,
        .badStateMatrix = CMR_CAN_VSM_ERROR_SOURCE_NONE,
        .latchMatrix = CMR_CAN_VSM_LATCH_NONE
    },
    .canVSMLatchedStatus = {
        .moduleTimeoutMatrix = CMR_CAN_VSM_ERROR_SOURCE_NONE,
        .badStateMatrix = CMR_CAN_VSM_ERROR_SOURCE_NONE,
        .latchMatrix = CMR_CAN_VSM_LATCH_NONE
    }
};

/** @brief State update priority. */
static const uint32_t stateUpdate_priority = 3;

/** @brief State update period. */
static const TickType_t stateUpdate_period_ms = 100;

/** @brief State update task. */
static cmr_task_t stateUpdate_task;

// ------------------------------------------------------------------------------------------------
// Static function forward declarations

static cmr_canVSMState_t getNextState(TickType_t lastWakeTime_ms);
static void setStateOutputs(TickType_t lastWakeTime_ms);
static void stateUpdate(void *pvParameters);

static bool getBrakeStatus();
static bool getMissionFinished();
static bool getMissionSelected();
static bool TSActive();
static bool AutonomousClear();
static bool EBSActive();
static bool vehicleStill();
static bool getVehicleFinished(bool vehicleStill);
static bool getRESGo();
static bool RESTriggered();

// ------------------------------------------------------------------------------------------------
// Interface functions

/**
 * @brief Initializes the state machine.
 */
void stateInit(void) {
    // Task creation.
    cmr_taskInit(
        &stateUpdate_task,
        "State update",
        stateUpdate_priority,
        stateUpdate,
        NULL
    );
}

/**
 * @brief Gets the current state.
 *
 * @return Current state.
 */
cmr_canVSMState_t getCurrentState(void) {
    return vsmStatus.canVSMStatus.internalState;
}

/**
 * @brief Gets the current status.
 *
 * @return Pointer to const struct containing current status.
 */
const vsmStatus_t *getCurrentStatus(void) {
    return (const vsmStatus_t *)(&vsmStatus);
}

/**
 * @brief Gets the current warnings.
 *
 * @return Warnings field for VSM heartbeat.
 */
uint16_t getCurrentWarnings(void) {
    return vsmStatus.heartbeatWarnings;
}

/**
 * @brief Updates errors and warnings in the vsmStatus struct.
 *
 * @param lastWakeTime Pass in from any FreeRTOS task. Used to check for module timeouts.
 */
void updateErrorsAndWarnings(TickType_t lastWakeTime) {
    updateCurrentErrors(&vsmStatus, lastWakeTime);
    updateCurrentWarnings(&vsmStatus, lastWakeTime);
}

// ------------------------------------------------------------------------------------------------
// Static local functions

/**
 * @brief Gets the next state based on the current state.
 *
 * @param lastWakeTime_ms Pass in from stateUpdate.
 * Used to update errors and warnings.
 *
 * @return Next VSM state.
 */
static cmr_canVSMState_t getNextState(TickType_t lastWakeTime_ms) {
    updateErrorsAndWarnings(lastWakeTime_ms);

    //If RES triggered stop everything
    if(RESTriggered() && getASMSState()){
        return CMR_CAN_VSM_STATE_AS_EMERGENCY;
    }

    // TE (Immediately return error if anything is wrong)
    if ((vsmStatus.heartbeatErrors != CMR_CAN_ERROR_NONE)
     || (vsmStatus.canVSMStatus.moduleTimeoutMatrix != CMR_CAN_VSM_ERROR_SOURCE_NONE)
     || (vsmStatus.canVSMStatus.latchMatrix != CMR_CAN_VSM_LATCH_NONE)) {
        return CMR_CAN_VSM_STATE_ERROR;
    }

    cmr_canVSMState_t state = vsmStatus.canVSMStatus.internalState;

    // Nothing is wrong, default nextState to error and begin state transition logic
    cmr_canVSMState_t nextState = CMR_CAN_VSM_STATE_ERROR;

    // DIM request timeout/reject handling.
    cmr_canDIMRequest_t *dimRequest = getPayload(CANRX_DIM_REQUEST);
    cmr_canState_t dimRequestedState = (cmr_canState_t)(dimRequest->requestedState);
    if (
        cmr_canRXMetaTimeoutError(&(canRXMeta[CANRX_DIM_REQUEST]), lastWakeTime_ms) ||
        (vsmStatus.heartbeatWarnings & CMR_CAN_WARN_VSM_DIM_REQ_NAK)
    ) {
        // Stale/NAK'd DIM request; assume it is requesting the current state.
        dimRequestedState = vsmToCANState[state];
    }

    // Get all info for state transitions
    cmr_canHVCHeartbeat_t *hvcHeartbeat = getPayload(CANRX_HEARTBEAT_HVC);

    cmr_canFSMData_t *fsmData = getPayload(CANRX_FSM_DATA);
    uint16_t throttlePosition = fsmData->throttlePosition;

    uint32_t brakePressureRear_PSI = cmr_sensorListGetValue(
        &sensorList, SENSOR_CH_BPRES_PSI
    );

    cmr_canAMKActualValues1_t *amk1Actual = getPayload(CANRX_INVERTER_1);
    cmr_canAMKActualValues1_t *amk2Actual = getPayload(CANRX_INVERTER_2);
    cmr_canAMKActualValues1_t *amk3Actual = getPayload(CANRX_INVERTER_3);
    cmr_canAMKActualValues1_t *amk4Actual = getPayload(CANRX_INVERTER_4);

    bool vehicleStill = (amk1Actual->velocity_rpm == 0) && (amk2Actual->velocity_rpm == 0) &&
                        (amk3Actual->velocity_rpm == 0) && (amk4Actual->velocity_rpm == 0);
    
    //If need to go to AS Emergency do so immediatly
    if(EBSActive() && !getVehicleFinished(vehicleStill) && getASMSState()){
        return CMR_CAN_VSM_STATE_AS_EMERGENCY;
    }

    taskENTER_CRITICAL();

    switch (state) {
        case CMR_CAN_VSM_STATE_ERROR: {
            // T0
            if (dimRequestedState == CMR_CAN_GLV_ON) {
                nextState = CMR_CAN_VSM_STATE_CLEAR_ERROR;
            }
            else {
                nextState = CMR_CAN_VSM_STATE_ERROR;
            }

            break;
        }

        case CMR_CAN_VSM_STATE_CLEAR_ERROR: {
            // T1
            if ((vsmStatus.canVSMStatus.badStateMatrix == CMR_CAN_VSM_ERROR_SOURCE_NONE) &&
                (dimRequestedState == CMR_CAN_GLV_ON)
            ) {
                nextState = CMR_CAN_VSM_STATE_GLV_ON;
            }
            else {
                nextState = CMR_CAN_VSM_STATE_CLEAR_ERROR;
            }

            break;
        }

        case CMR_CAN_VSM_STATE_GLV_ON: {
            // T2
            if (dimRequestedState == CMR_CAN_HV_EN || dimRequestedState == CMR_CAN_AS_READY) {
                nextState = CMR_CAN_VSM_STATE_REQ_PRECHARGE;
            }
            // T11
            else if (dimRequestedState == CMR_CAN_GLV_ON) {
                nextState = CMR_CAN_VSM_STATE_GLV_ON;
            }

            break;
        }

        case CMR_CAN_VSM_STATE_REQ_PRECHARGE: {
            // T3
            if ((hvcHeartbeat->hvcMode == CMR_CAN_HVC_MODE_START) &&
                (dimRequestedState == CMR_CAN_HV_EN || dimRequestedState == CMR_CAN_AS_READY)
            ) {
                 nextState = CMR_CAN_VSM_STATE_RUN_BMS;
            }
            else if (dimRequestedState == CMR_CAN_HV_EN || dimRequestedState == CMR_CAN_AS_READY){
                nextState = CMR_CAN_VSM_STATE_REQ_PRECHARGE;
            }
            else if (dimRequestedState == CMR_CAN_GLV_ON) {
                nextState = CMR_CAN_VSM_STATE_GLV_ON;
            }

            break;
        }

        case CMR_CAN_VSM_STATE_RUN_BMS: {
            // T4
            if ((hvcHeartbeat->hvcMode == CMR_CAN_HVC_MODE_RUN) &&
                (dimRequestedState == CMR_CAN_HV_EN || dimRequestedState == CMR_CAN_AS_READY)
            ) {
                nextState = CMR_CAN_VSM_STATE_INVERTER_EN;
            }
            else if (dimRequestedState == CMR_CAN_HV_EN || dimRequestedState == CMR_CAN_AS_READY) {
                nextState = CMR_CAN_VSM_STATE_RUN_BMS;
            }
            else if (dimRequestedState == CMR_CAN_GLV_ON) {
                nextState = CMR_CAN_VSM_STATE_GLV_ON;
            }

            break;
        }
        
        case CMR_CAN_VSM_STATE_INVERTER_EN: {
            if (
                // TODO: change back before comp so that don't unnecessarily error out
                ((amk1Actual->status_bv & CMR_CAN_AMK_STATUS_SYSTEM_READY) &&
                    !cmr_canRXMetaTimeoutError(&(canRXMeta[CANRX_INVERTER_1]), lastWakeTime_ms)) ||
                ((amk2Actual->status_bv & CMR_CAN_AMK_STATUS_SYSTEM_READY) &&
                    !cmr_canRXMetaTimeoutError(&(canRXMeta[CANRX_INVERTER_2]), lastWakeTime_ms)) ||
                ((amk3Actual->status_bv & CMR_CAN_AMK_STATUS_SYSTEM_READY) &&
                    !cmr_canRXMetaTimeoutError(&(canRXMeta[CANRX_INVERTER_3]), lastWakeTime_ms)) ||
                ((amk4Actual->status_bv & CMR_CAN_AMK_STATUS_SYSTEM_READY) &&
                    !cmr_canRXMetaTimeoutError(&(canRXMeta[CANRX_INVERTER_4]), lastWakeTime_ms))
                ){
                //if (!EBSActive() && AutonomousClear() && brakePressureRear_PSI >= brakePressureThreshold_PSI) {
                // if (true){
                //     nextState = CMR_CAN_VSM_STATE_AS_READY;
                // } else
                if (getASMSState()){ //Trying to enter DV mode but failed previous conditions
                    nextState = CMR_CAN_VSM_STATE_AS_READY;
                    //nextState = CMR_CAN_VSM_STATE_GLV_ON;
                }
                else{
                    nextState = CMR_CAN_VSM_STATE_HV_EN;
                }
            }
            else if (dimRequestedState == CMR_CAN_GLV_ON) {
                nextState = CMR_CAN_VSM_STATE_GLV_ON;
            }
            else{
                nextState = CMR_CAN_VSM_STATE_INVERTER_EN;
            }
            break;
        }

        case CMR_CAN_VSM_STATE_HV_EN: {
            // T6
            if (dimRequestedState == CMR_CAN_RTD) {
                if (
                    throttlePosition <= 5 &&
                    brakePressureRear_PSI >= brakePressureThreshold_PSI
                ) {
                    nextState = CMR_CAN_VSM_STATE_RTD;
                } else {
                    nextState = CMR_CAN_VSM_STATE_HV_EN;
                }
            }
            // T12
            else if (dimRequestedState == CMR_CAN_HV_EN)
            {
                nextState = CMR_CAN_VSM_STATE_HV_EN;
            }
            // T8
            else if (dimRequestedState == CMR_CAN_GLV_ON) {
                nextState = CMR_CAN_VSM_STATE_GLV_ON;
            }

            break;
        }

        case CMR_CAN_VSM_STATE_AS_READY: {
            // if (!EBSActive() && !AutonomousClear()){
            //     nextState = CMR_CAN_VSM_STATE_GLV_ON;
            // }
            // else if (brakePressureRear_PSI < brakePressureThreshold_PSI){ //If brakes off then AS off
            //     nextState = CMR_CAN_VSM_STATE_GLV_ON;
            // }

            //T6
            if (getRESGo() && getASMSState()){
                if (lastWakeTime_ms > lastStateChangeTime_ms + AS_WAKEUP_TIME){
                    nextState = CMR_CAN_VSM_STATE_AS_DRIVING;
                }
                else{
                    nextState = CMR_CAN_VSM_STATE_AS_READY;
                }
            }
            else if (dimRequestedState == CMR_CAN_AS_READY && getASMSState()){
                nextState = CMR_CAN_VSM_STATE_AS_READY;
            }
            //T8
            else{
                nextState = CMR_CAN_VSM_STATE_ERROR;
            }
            break;
        }

        case CMR_CAN_VSM_STATE_AS_DRIVING: {
            //T13
            if (!RESTriggered() && getASMSState()){
                nextState = CMR_CAN_VSM_STATE_AS_DRIVING;
            }
            else if (getVehicleFinished(vehicleStill)){
                nextState = CMR_CAN_VSM_STATE_AS_FINISHED;
            }
            else{
                nextState = CMR_CAN_VSM_STATE_ERROR;
            }

            break;
        }

        case CMR_CAN_VSM_STATE_AS_FINISHED: {
            if (!EBSActive() && !AutonomousClear()){
                nextState = CMR_CAN_VSM_STATE_ERROR;
            }
            else if (getVeichleFinished(veichleStill) && !RESTriggered() && (lastWakeTime_ms > lastStateChangeTime_ms + AS_FINISHED_TIME)){
                nextState = CMR_CAN_VSM_STATE_ERROR;
            }
            else{
                nextState = CMR_CAN_VSM_STATE_AS_FINISHED;
            }

            break;
        }

        case CMR_CAN_VSM_STATE_AS_EMERGENCY: {
            if (lastStateChangeTime_ms + AS_FINISHED_TIME > lastWakeTime_ms){
                nextState = CMR_CAN_VSM_STATE_AS_EMERGENCY;
            }
            else if (dimRequestedState == CMR_CAN_GLV_ON){
                nextState = CMR_CAN_VSM_STATE_CLEAR_ERROR;
            }
            else{
                nextState = CMR_CAN_VSM_STATE_ERROR;
            }

            break;
        }

        case CMR_CAN_VSM_STATE_RTD: {
            // T13
            if (dimRequestedState == CMR_CAN_RTD)
            {
                nextState = CMR_CAN_VSM_STATE_RTD;
            }
            // T7
            else if (dimRequestedState == CMR_CAN_HV_EN) {
                nextState = CMR_CAN_VSM_STATE_HV_EN;
            }

            break;
        }

        default: {
            nextState = CMR_CAN_VSM_STATE_ERROR;
            break;
        }
    }

    taskEXIT_CRITICAL();

    return nextState;
}

/**
 * @brief Sets Vehicle Safety Module outputs based on current state.
 *
 * @param lastWakeTime_ms Pass in from stateUpdate. Used for RTD buzzer timing.
 */
static void setStateOutputs(TickType_t lastWakeTime_ms) {
    switch (vsmStatus.canVSMStatus.internalState) {
        case CMR_CAN_VSM_STATE_ERROR:
            cmr_gpioWrite(GPIO_OUT_RTD_SIGNAL, 0);
            hvcModeRequest = CMR_CAN_HVC_MODE_IDLE;
            break;

        case CMR_CAN_VSM_STATE_CLEAR_ERROR:
            cmr_gpioWrite(GPIO_OUT_RTD_SIGNAL, 0);
            hvcModeRequest = CMR_CAN_HVC_MODE_ERROR;
            break;

        case CMR_CAN_VSM_STATE_AS_FINISHED:
        case CMR_CAN_VSM_STATE_GLV_ON:
            cmr_gpioWrite(GPIO_OUT_RTD_SIGNAL, 0);
            hvcModeRequest = CMR_CAN_HVC_MODE_IDLE;
            break;

        case CMR_CAN_VSM_STATE_REQ_PRECHARGE:
            cmr_gpioWrite(GPIO_OUT_RTD_SIGNAL, 0);
            hvcModeRequest = CMR_CAN_HVC_MODE_START;
            break;

        case CMR_CAN_VSM_STATE_RUN_BMS: //Fallthrough
        case CMR_CAN_VSM_STATE_INVERTER_EN:
        case CMR_CAN_VSM_STATE_HV_EN:
        case CMR_CAN_VSM_STATE_AS_READY:
            cmr_gpioWrite(GPIO_OUT_RTD_SIGNAL, 0);
            hvcModeRequest = CMR_CAN_HVC_MODE_RUN;
            break;

        case CMR_CAN_VSM_STATE_RTD: //Fallthrough
        case CMR_CAN_VSM_STATE_AS_DRIVING:
            // Enable RTD buzzer for the first rtdBuzzerTime_ms after getting into RTD
            if (lastWakeTime_ms < lastStateChangeTime_ms + rtdBuzzerTime_ms) {
                cmr_gpioWrite(GPIO_OUT_RTD_SIGNAL, 1);
            }
            else {
                cmr_gpioWrite(GPIO_OUT_RTD_SIGNAL, 0);
            }

            hvcModeRequest = CMR_CAN_HVC_MODE_RUN;
            break;
        
        case CMR_CAN_VSM_STATE_AS_EMERGENCY: {
            TickType_t timeinASEmergency_ms = lastWakeTime_ms - lastStateChangeTime_ms;
            if (timeinASEmergency_ms < ASEmergencyBuzzerTime_ms)
            {
                //Modulate buzzer at 2.5 Hz
                TickType_t cyclesPassed = timeinASEmergency_ms / ASEmergencySwitchingTime_ms;
                cmr_gpioWrite(GPIO_OUT_RTD_SIGNAL, cyclesPassed % 2);
            }
            else
            {
                cmr_gpioWrite(GPIO_OUT_RTD_SIGNAL, 0);
            }
            // Go quiet mode
            // cmr_gpioWrite(GPIO_OUT_RTD_SIGNAL, 0);

            hvcModeRequest = CMR_CAN_HVC_MODE_IDLE;
            break;
        }
        
        default:
            cmr_gpioWrite(GPIO_OUT_RTD_SIGNAL, 0);
            hvcModeRequest = CMR_CAN_HVC_MODE_IDLE;
            break;

    }
}

/**
 * @brief State update task.
 */
static void stateUpdate(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    cmr_canVSMState_t lastState = vsmStatus.canVSMStatus.internalState;

    TickType_t lastWakeTime_ms = xTaskGetTickCount();
    while (1) {
        cmr_canVSMState_t nextState = getNextState(lastWakeTime_ms);

        if (nextState != lastState) {
            lastStateChangeTime_ms = lastWakeTime_ms;
            lastState = vsmStatus.canVSMStatus.internalState;
        }

        vsmStatus.canVSMStatus.internalState = nextState;
        setStateOutputs(lastWakeTime_ms);

        vTaskDelayUntil(&lastWakeTime_ms, stateUpdate_period_ms);
    }
}

/**
 * @brief Checks if possible for Autonomous Braking System to Deploy
 */
static inline bool getBrakeStatus(){
    cmr_canDVPressureReadings_t* pressureReading = (cmr_canDVPressureReadings_t*) getPayload(CANRX_AS_PRESSURE_READING);
    return pressureReading->tankPressurePercentofThreshold > 100;
}

/**
 * @brief Checks if an autonomous mission is selected
 */
static inline bool getMissionSelected(){
    return true;
}

/**
 * @brief Checks if TS is active
 */
static inline bool TSActive(){
    cmr_canHVCHeartbeat_t* HVCState = (cmr_canHVCHeartbeat_t*) (getPayload(CANRX_HEARTBEAT_HVC));
    return CMR_CAN_HVC_STATE_DRIVE == HVCState->hvcState;
}

/**
 * @brief Checks if autnomous systems are working and 
 * that the car can continue being in AS Ready or AS Driving
 */
static inline bool AutonomousClear(){
    return getASMSState() && getBrakeStatus() && getMissionSelected() && TSActive();
}

/**
 * @brief Checks if EBS is trying to brake
 */
static inline bool EBSActive(){
	return false;
//    cmr_canDVPressureReadings_t* pressureReading = (cmr_canDVPressureReadings_t*) getPayload(CANRX_AS_PRESSURE_READING);
//    return pressureReading->ebsPressurePercentofThreshold > 100;
}

 
/**
 * @brief Check if autonomous mission has finshed.  
 */
static inline bool getMissionFinished(){
    return false;
}

/**
 * @brief Check if veichle is finished
 * 
 * @note Veichle still passed in as a parameters since it is computed once
 *       per hot loop and is somewhat expensive to compute  
 */
static bool getVeichleFinished(bool veichleStill){
    return veichleStill && getMissionFinished();
}


/**
 * @brief Checks if RES is giving a go ahead
 */
static inline bool getRESGo() {
	uint8_t *data = (uint8_t*)(getPayload(CANRX_RES));
    return (data[0] & 4);
}

/**
 * @brief Checks if RES is activated
 * @todo IMPLEMENT
 */
static inline bool RESTriggered(){
	uint8_t *data = (uint8_t*)(getPayload(CANRX_RES));
	return !(data[0] & 1);
}