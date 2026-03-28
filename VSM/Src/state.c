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
#define DV_TANK_PRESSURE_MINIMUM_DECIBAR 6
#define FRONT_MINIMUM_BRAKING_PSI 800
#define REAR_MINIMUM_BRAKING_PSI  500



/** @brief Mapping of VSM internal states to vehicle states. Indexed by cmr_canVSMState_t. */
cmr_canState_t vsmToCANState[] = {
    [CMR_CAN_VSM_STATE_ERROR]           = CMR_CAN_ERROR,
    [CMR_CAN_VSM_STATE_CLEAR_ERROR]     = CMR_CAN_CLEAR_ERROR,
    [CMR_CAN_VSM_STATE_GLV_ON]          = CMR_CAN_GLV_ON,
    [CMR_CAN_VSM_STATE_REQ_PRECHARGE]   = CMR_CAN_GLV_ON,
    [CMR_CAN_VSM_STATE_RUN_BMS]         = CMR_CAN_GLV_ON,
    [CMR_CAN_VSM_STATE_INVERTER_EN]     = CMR_CAN_GLV_ON,
    [CMR_CAN_VSM_STATE_BRAKE_TEST]      = CMR_CAN_GLV_ON,
    [CMR_CAN_VSM_STATE_HV_EN]           = CMR_CAN_HV_EN,
    [CMR_CAN_VSM_STATE_RTD]             = CMR_CAN_RTD,
    [CMR_CAN_VSM_STATE_AS_READY]        = CMR_CAN_AS_READY,
    [CMR_CAN_VSM_STATE_AS_DRIVING]      = CMR_CAN_AS_DRIVING,
    [CMR_CAN_VSM_STATE_AS_FINISHED]     = CMR_CAN_AS_FINISHED,
    [CMR_CAN_VSM_STATE_AS_EMERGENCY]    = CMR_CAN_AS_EMERGENCY
};

/** @brief DTI Facing motor setpoints struct*/
typedef struct{
    int32_t velocity_erpm;          /**< @brief Velocity setpoint (ERPM). */
    int16_t torqueLimPos_dA;        /**< @brief Positive torque limit. */
    int16_t torqueLimNeg_dA;        /**< @brief Negative torque limit. */
    int16_t ACCurrent_dA;     /**< @brief Negative torque limit. */      
} cmr_DTI_RX_Message_t;

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

/** @brief ASState flag. Keeps track of the "true" ASMS State. Set during GLV_ON only. 
 */
static bool ASState = false;

/** @brief Autonomous brake test state is not started at the beginning*/
static brakeTestState_t brakeTestState = BRAKE_TEST_NOT_STARTED;
/** @brief Start time of the brake test  */
static TickType_t brakeTestStartTime = 0;
/** @brief Time to detect pressure rise (3 seconds) @todo time*/
static const TickType_t brakeTestTimeout = 3000;
/** @brief Expected pressure increase for working brakes @todo threshold*/
static const uint16_t brakePressureRiseThreshold = 50;
/** @brief Brake pressure at the start of the brake test*/
static int32_t initialBrakePressure = 0;

/** @brief Current Vehicle Safety Module state and errors. */
static volatile vsmStatus_t vsmStatus = {
    .heartbeatErrors = CMR_CAN_ERROR_NONE,
    .heartbeatWarnings = CMR_CAN_WARN_NONE,
    .canVSMStatus = {
        .internalState = CMR_CAN_VSM_STATE_ERROR,
        .moduleTimeoutMatrix = CMR_CAN_VSM_TIMEOUT_SOURCE_NONE,
        .badStateMatrix = CMR_CAN_VSM_BADSTATE_SOURCE_NONE,
        .latchMatrix = CMR_CAN_VSM_LATCH_NONE
    },
    .canVSMLatchedStatus = {
        .moduleTimeoutMatrix = CMR_CAN_VSM_TIMEOUT_SOURCE_NONE,
        .badStateMatrix = CMR_CAN_VSM_BADSTATE_SOURCE_NONE,
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

static bool getDVBrakeActive();
static bool getDVBrakeDeployable();
static bool getMissionFinished();
static bool getMissionSelected();
static bool TSActive();
static bool AutonomousClear();
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
    if(RESTriggered() && ASState){
        return CMR_CAN_VSM_STATE_AS_EMERGENCY;
    }

    // TE (Immediately return error if anything is wrong)
    if ((vsmStatus.heartbeatErrors != CMR_CAN_ERROR_NONE)
     || (vsmStatus.canVSMStatus.moduleTimeoutMatrix != CMR_CAN_VSM_TIMEOUT_SOURCE_NONE)
     || (vsmStatus.canVSMStatus.latchMatrix != CMR_CAN_VSM_LATCH_NONE)
     /*|| (ASState != getASMSState())*/) {
        // if(ASState) {
        //     return CMR_CAN_VSM_STATE_AS_EMERGENCY;
        // }
        if(lastWakeTime_ms == 10000000) {
            return 10;
        }
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

    cmr_canDTI_TX_TempFault_t *dti_fl_tempfault = getPayload(CANRX_FL_TEMPFAULT);
    cmr_canDTI_TX_TempFault_t *dti_fr_tempfault = getPayload(CANRX_FR_TEMPFAULT);
    cmr_canDTI_TX_TempFault_t *dti_rl_tempfault = getPayload(CANRX_RL_TEMPFAULT);
    cmr_canDTI_TX_TempFault_t *dti_rr_tempfault = getPayload(CANRX_RR_TEMPFAULT);
    cmr_canDTI_TX_IOStatus_t *dti_fl_io_status = getPayload(CANRX_FL_IO_STATUS);
    cmr_canDTI_TX_IOStatus_t *dti_fr_io_status = getPayload(CANRX_FR_IO_STATUS);
    cmr_canDTI_TX_IOStatus_t *dti_rl_io_status = getPayload(CANRX_RL_IO_STATUS);
    cmr_canDTI_TX_IOStatus_t *dti_rr_io_status = getPayload(CANRX_RR_IO_STATUS);
    cmr_DTI_RX_Message_t *dti_fl_erpm = getPayload(CANRX_FL_ERPM);
    cmr_DTI_RX_Message_t *dti_fr_erpm = getPayload(CANRX_FR_ERPM);
    cmr_DTI_RX_Message_t *dti_rl_erpm = getPayload(CANRX_RL_ERPM);
    cmr_DTI_RX_Message_t *dti_rr_erpm = getPayload(CANRX_RR_ERPM);

    bool vehicleStill = (dti_fl_erpm->velocity_erpm == 0) && (dti_fr_erpm->velocity_erpm == 0) &&
                        (dti_rl_erpm->velocity_erpm == 0) && (dti_rr_erpm->velocity_erpm == 0);

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
            if ((vsmStatus.canVSMStatus.badStateMatrix == CMR_CAN_VSM_BADSTATE_SOURCE_NONE) &&
                (dimRequestedState == CMR_CAN_GLV_ON)
            ) {
                nextState = CMR_CAN_VSM_STATE_GLV_ON;
                resetError(); // Reset error latch if system returns to GLV_ON
            }
            else {
                nextState = CMR_CAN_VSM_STATE_CLEAR_ERROR;
            }
            break;
        }

        case CMR_CAN_VSM_STATE_GLV_ON: {
            // T2
            ASState = getASMSState();
            if (dimRequestedState == CMR_CAN_AS_READY
                && ASState && getMissionSelected() && getDVBrakeDeployable() 
                && getDVBrakeActive()){
                nextState = CMR_CAN_VSM_STATE_REQ_PRECHARGE;
            }
            else if (dimRequestedState == CMR_CAN_AS_READY) {
                // if we fail to state up go to error
                nextState = CMR_CAN_VSM_STATE_ERROR;
            }
            else if (dimRequestedState == CMR_CAN_HV_EN){
                nextState = CMR_CAN_VSM_STATE_REQ_PRECHARGE;
            }
            // T11
            else if (dimRequestedState == CMR_CAN_GLV_ON ){
                nextState = CMR_CAN_VSM_STATE_GLV_ON;
            }
            else{
                nextState = CMR_CAN_VSM_STATE_ERROR;
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
            if (invertersPass(lastWakeTime_ms)){
                if (AutonomousClear()) {
                    nextState = CMR_CAN_VSM_STATE_AS_READY;
                } else if (ASState){ 
                    //Trying to enter DV mode but failed previous conditions
                    nextState = CMR_CAN_VSM_STATE_GLV_ON;
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
                if ((throttlePosition <= 5 &&
                    brakePressureRear_PSI >= brakePressureThreshold_PSI)
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

        case CMR_CAN_VSM_STATE_AS_READY: {
            if (!AutonomousClear()){
                nextState = CMR_CAN_VSM_STATE_AS_EMERGENCY;
            }
            else if (getRESGo()){
                if (lastWakeTime_ms > lastStateChangeTime_ms + AS_WAKEUP_TIME){
                    nextState = CMR_CAN_VSM_STATE_AS_DRIVING;
                }
                else{
                    nextState = CMR_CAN_VSM_STATE_AS_READY;
                }
            }
            else if (dimRequestedState == CMR_CAN_AS_READY && ASState){
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
            if (!AutonomousClear()){
                nextState = CMR_CAN_VSM_STATE_AS_EMERGENCY;
            }
            else if (getVehicleFinished(vehicleStill)){
                nextState = CMR_CAN_VSM_STATE_AS_FINISHED;
            }
            else if (!RESTriggered()){
                nextState = CMR_CAN_VSM_STATE_AS_DRIVING;
            }
            else{
                nextState = CMR_CAN_VSM_STATE_AS_EMERGENCY;
            }

            break;
        }

        case CMR_CAN_VSM_STATE_AS_FINISHED: {
            if (!getDVBrakeActive() || RESTriggered()){
                nextState = CMR_CAN_VSM_STATE_AS_EMERGENCY;
            }
            else if ((lastWakeTime_ms > lastStateChangeTime_ms + AS_FINISHED_TIME)){
                nextState = CMR_CAN_GLV_ON;
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
            else{
                nextState = CMR_CAN_VSM_STATE_ERROR;
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

        case CMR_CAN_VSM_STATE_BRAKE_TEST: //Fallthrough
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
 * @brief Checks if we are able to deploy the DV Brakes
 * @note This should be active for the entirety of DV
 */
static bool getDVBrakeDeployable(){
    cmr_canDVPressureReadings_t* pressureReading = (cmr_canDVPressureReadings_t*) getPayload(CANRX_AS_PRESSURE_READING);
   return   pressureReading->ebsPressure_1 > DV_TANK_PRESSURE_MINIMUM_DECIBAR &&  
            pressureReading->ebsPressure_2 > DV_TANK_PRESSURE_MINIMUM_DECIBAR;
}

/**
 * @brief Checks if the DV brakes are currently deployed
 * @note  This should be active before attempting to state up
 */
static bool getDVBrakeActive(){
    uint32_t brakePressureRear_PSI = cmr_sensorListGetValue(&sensorList, SENSOR_CH_BPRES_PSI);
    cmr_canFSMData_t *fsmData = getPayload(CANRX_FSM_DATA);
    uint16_t brakePressureFront_PSI = fsmData->brakePressureFront_PSI;
    // @todo Add check to ensure solonoid current is 0 from DIM

   return   brakePressureFront_PSI > FRONT_MINIMUM_BRAKING_PSI &&  
            brakePressureRear_PSI  > REAR_MINIMUM_BRAKING_PSI;
}

/**
 * @brief Checks if an autonomous mission is selected
 */
static inline bool getMissionSelected(){
    cmr_canDIMRequest_t *dimRequest = getPayload(CANRX_DIM_REQUEST);
    return (dimRequest->requestedGear > CMR_CAN_GEAR_DV_MISSION_MIN && dimRequest->requestedGear < CMR_CAN_GEAR_DV_MISSION_MAX);
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
    return ASState &&  getDVBrakeDeployable() && getMissionSelected() && TSActive() && !RESTriggered();
}
 
/**
 * @brief Check if autonomous mission has finshed.  
 */
static inline bool getMissionFinished(){ //can from compute
    uint8_t *missionFinished = getPayload(CANRX_AS_MISSION_FINISHED);
    if(*missionFinished) return true;
    else return false;
}

/**
 * @brief Check if vehicle is finished
 * 
 * @note Vehicle still passed in as a parameters since it is computed once
 *       per hot loop and is somewhat expensive to compute  
 */
static bool getVehicleFinished(bool vehicleStill){
    return vehicleStill && getMissionFinished();
}

/**
 * @brief Go-signal for switching from “Ready” to “Autonomous” state 
 * 
 * More: https://doc.fs-quiz.eu/FSG2017_DV_Technical_Specifications_v1.0.pdf
 */
static inline bool getRESGo() {
	uint8_t *data = (uint8_t*)(getPayload(CANRX_RES));
    return (data[0] & CMR_CAN_RES_GO);
}

/**
 * @brief Checks if RES is activated
 */
static inline bool RESTriggered(){
	uint8_t *data = (uint8_t*)(getPayload(CANRX_RES));
	return !(data[0] & CMR_CAN_RES_TRIG);
}