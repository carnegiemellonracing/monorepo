#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "fff.h"

#include "can.h"
#include "error.h"
#include "gpio.h"
#include "sensors.h"

DEFINE_FFF_GLOBALS;

cmr_canRXMeta_t canRXMeta[CANRX_LEN];
cmr_sensorList_t sensorList;
const uint16_t brakePressureThreshold_PSI = 40;

static cmr_canDIMRequest_t dimRequest;
static cmr_canHVCHeartbeat_t hvcHeartbeat;
static cmr_canFSMData_t fsmData;
static cmr_canDVPressureReadings_t dvPressure;
static uint8_t resPayload[8];
static uint8_t missionFinished;

FAKE_VOID_FUNC(
    cmr_taskInit,
    cmr_task_t *,
    const char *,
    UBaseType_t,
    TaskFunction_t,
    void *
);
FAKE_VALUE_FUNC(TickType_t, xTaskGetTickCount);
FAKE_VOID_FUNC(vTaskDelayUntil, TickType_t *, TickType_t);

FAKE_VOID_FUNC(cmr_gpioWrite, size_t, int);
FAKE_VALUE_FUNC(int, cmr_gpioRead, size_t);

FAKE_VALUE_FUNC(void *, getPayload, canRX_t);
FAKE_VALUE_FUNC(cmr_canState_t, getModuleState, canRX_t);
FAKE_VALUE_FUNC(uint8_t, getASMSState);
FAKE_VOID_FUNC(sendFirstError, uint8_t);
FAKE_VOID_FUNC(resetError);
FAKE_VALUE_FUNC(int32_t, getDTIERPM, canRX_t);
FAKE_VALUE_FUNC(int16_t, getDTIACCurrent_dA, canRX_t);
FAKE_VALUE_FUNC(int16_t, getDTIDCCurrent_dA, canRX_t);
FAKE_VALUE_FUNC(int16_t, getDTICtlrTemp_dC, canRX_t);
FAKE_VALUE_FUNC(int16_t, getDTIMotorTemp_dC, canRX_t);

FAKE_VALUE_FUNC(int, cmr_canRXMetaTimeoutError, const cmr_canRXMeta_t *, TickType_t);
FAKE_VALUE_FUNC(int32_t, cmr_sensorListGetValue, cmr_sensorList_t *, size_t);

FAKE_VOID_FUNC(updateCurrentErrors, volatile vsmStatus_t *, TickType_t);
FAKE_VOID_FUNC(updateCurrentWarnings, volatile vsmStatus_t *, TickType_t);
FAKE_VALUE_FUNC(bool, invertersPass, TickType_t);
FAKE_VALUE_FUNC(bool, getAMSError);

//TODO: this bypasses a lot of stuff, should populate rxMeta array and get from there instead
// also ideally check can filters before returning 
static void *getPayload_custom(canRX_t rxMsg) {
    switch (rxMsg) {
        case CANRX_DIM_REQUEST:
            return &dimRequest;
        case CANRX_HEARTBEAT_HVC:
            return &hvcHeartbeat;
        case CANRX_FSM_DATA:
            return &fsmData;
        case CANRX_AS_PRESSURE_READING:
            return &dvPressure;
        case CANRX_RES:
            return resPayload;
        case CANRX_AS_MISSION_FINISHED:
            return &missionFinished;
        default:
            return NULL;
    }
}

#include "../Src/state.c"

static void reset_payloads(void) {
    memset(&dimRequest, 0, sizeof(dimRequest));
    memset(&hvcHeartbeat, 0, sizeof(hvcHeartbeat));
    memset(&fsmData, 0, sizeof(fsmData));
    memset(&dvPressure, 0, sizeof(dvPressure));
    memset(resPayload, 0, sizeof(resPayload));
    missionFinished = 0;
}

static void reset_vsm_state(void) {
    vsmStatus.heartbeatErrors = CMR_CAN_ERROR_NONE;
    vsmStatus.heartbeatWarnings = CMR_CAN_WARN_NONE;
    vsmStatus.dimRequestReject = CMR_CAN_UNKNOWN;
    vsmStatus.canVSMStatus.internalState = CMR_CAN_VSM_STATE_ERROR;
    vsmStatus.canVSMStatus.moduleTimeoutMatrix = CMR_CAN_VSM_TIMEOUT_SOURCE_NONE;
    vsmStatus.canVSMStatus.badStateMatrix = CMR_CAN_VSM_BADSTATE_SOURCE_NONE;
    vsmStatus.canVSMStatus.latchMatrix = CMR_CAN_VSM_LATCH_NONE;
    vsmStatus.canVSMLatchedStatus.moduleTimeoutMatrix = CMR_CAN_VSM_TIMEOUT_SOURCE_NONE;
    vsmStatus.canVSMLatchedStatus.badStateMatrix = CMR_CAN_VSM_BADSTATE_SOURCE_NONE;
    vsmStatus.canVSMLatchedStatus.latchMatrix = CMR_CAN_VSM_LATCH_NONE;

    ASState = false;
    lastStateChangeTime_ms = 0;
    hvcModeRequest = CMR_CAN_HVC_MODE_ERROR;
}

void setUp(void) {
    RESET_FAKE(cmr_taskInit);
    RESET_FAKE(xTaskGetTickCount);
    RESET_FAKE(vTaskDelayUntil);
    RESET_FAKE(cmr_gpioWrite);
    RESET_FAKE(cmr_gpioRead);
    RESET_FAKE(getPayload);
    RESET_FAKE(getModuleState);
    RESET_FAKE(getASMSState);
    RESET_FAKE(sendFirstError);
    RESET_FAKE(resetError);
    RESET_FAKE(getDTIERPM);
    RESET_FAKE(getDTIACCurrent_dA);
    RESET_FAKE(getDTIDCCurrent_dA);
    RESET_FAKE(getDTICtlrTemp_dC);
    RESET_FAKE(getDTIMotorTemp_dC);
    RESET_FAKE(cmr_canRXMetaTimeoutError);
    RESET_FAKE(cmr_sensorListGetValue);
    RESET_FAKE(updateCurrentErrors);
    RESET_FAKE(updateCurrentWarnings);
    RESET_FAKE(invertersPass);
    RESET_FAKE(getAMSError);
    FFF_RESET_HISTORY();

    reset_payloads();
    reset_vsm_state();

    getPayload_fake.custom_fake = getPayload_custom;
    cmr_canRXMetaTimeoutError_fake.return_val = 0;
}

void tearDown(void) {
}

void test_glv_on_goes_to_req_precharge_when_autonomous_checks_pass(void) {
    vsmStatus.canVSMStatus.internalState = CMR_CAN_VSM_STATE_GLV_ON;

    dimRequest.requestedState = CMR_CAN_AS_READY;
    dimRequest.requestedGear = CMR_CAN_GEAR_DV_MISSION_ACCEL;

    getASMSState_fake.return_val = 1;
    dvPressure.ebsPressure_1_deci_bar = 100;
    dvPressure.ebsPressure_2_deci_bar = 100;
    fsmData.brakePressureFront_PSI = 700;
    cmr_sensorListGetValue_fake.return_val = 450;

    TEST_ASSERT_EQUAL(
        CMR_CAN_VSM_STATE_REQ_PRECHARGE,
        getNextState(100)
    );
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_glv_on_goes_to_req_precharge_when_autonomous_checks_pass);
    return UNITY_END();
}
