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

// 1. Helpers that populate/read the fake CAN RX metadata payload array.
static void *getPayload_custom(canRX_t rxMsg) {
    if (rxMsg >= CANRX_LEN) {
        return NULL;
    }

    return (void *)(&canRXMeta[rxMsg].payload);
}

#include "../Src/state.c"

static cmr_canDIMRequest_t *dim_req(void) {
    return (cmr_canDIMRequest_t *)(void *)(&canRXMeta[CANRX_DIM_REQUEST].payload);
}

static cmr_canHVCHeartbeat_t *hvc_hb(void) {
    return (cmr_canHVCHeartbeat_t *)(void *)(&canRXMeta[CANRX_HEARTBEAT_HVC].payload);
}

static cmr_canFSMData_t *fsm_data(void) {
    return (cmr_canFSMData_t *)(void *)(&canRXMeta[CANRX_FSM_DATA].payload);
}

static cmr_canDVPressureReadings_t *dv_press(void) {
    return (cmr_canDVPressureReadings_t *)(void *)(&canRXMeta[CANRX_AS_PRESSURE_READING].payload);
}

static uint8_t *res(void) {
    return (uint8_t *)(void *)(&canRXMeta[CANRX_RES].payload);
}

static void reset_rx(void) {
    for (size_t i = 0; i < CANRX_LEN; i++) {
        for (size_t j = 0; j < sizeof(canRXMeta[i].payload); j++) {
            canRXMeta[i].payload[j] = 0;
        }
    }
}

static void reset_state(void) {
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

    reset_rx();
    reset_state();

    getPayload_fake.custom_fake = getPayload_custom;
    cmr_canRXMetaTimeoutError_fake.return_val = 0;
}

void tearDown(void) {
}

// 2. State machine setup/assertion helpers.
static void setup_as_clear(void) {
    ASState = true;
    dim_req()->requestedState = CMR_CAN_AS_READY;
    dim_req()->requestedGear = CMR_CAN_GEAR_DV_MISSION_ACCEL;
    dv_press()->ebsPressure_1_deci_bar = 100;
    dv_press()->ebsPressure_2_deci_bar = 100;
    fsm_data()->brakePressureFront_PSI = 700;
    hvc_hb()->hvcState = CMR_CAN_HVC_STATE_DRIVE;
    res()[0] = CMR_CAN_RES_TRIG;
    cmr_sensorListGetValue_fake.return_val = 450;
}

static void assert_error_fault(
    cmr_canVSMState_t state,
    uint16_t heartbeatErrors,
    uint8_t moduleTimeoutMatrix,
    uint8_t latchMatrix
) {
    vsmStatus.canVSMStatus.internalState = state;
    vsmStatus.heartbeatErrors = heartbeatErrors;
    vsmStatus.canVSMStatus.moduleTimeoutMatrix = moduleTimeoutMatrix;
    vsmStatus.canVSMStatus.latchMatrix = latchMatrix;

    TEST_ASSERT_EQUAL(CMR_CAN_VSM_STATE_ERROR, getNextState(100));
}

static void assert_as_emerg_fault(
    cmr_canVSMState_t state,
    uint16_t heartbeatErrors,
    uint8_t moduleTimeoutMatrix,
    uint8_t latchMatrix
) {
    setup_as_clear();
    vsmStatus.canVSMStatus.internalState = state;
    vsmStatus.heartbeatErrors = heartbeatErrors;
    vsmStatus.canVSMStatus.moduleTimeoutMatrix = moduleTimeoutMatrix;
    vsmStatus.canVSMStatus.latchMatrix = latchMatrix;

    TEST_ASSERT_EQUAL(CMR_CAN_VSM_STATE_AS_EMERGENCY, getNextState(100));
}

void test_glv_as_to_precharge(void) {
    vsmStatus.canVSMStatus.internalState = CMR_CAN_VSM_STATE_GLV_ON;

    dim_req()->requestedState = CMR_CAN_AS_READY;
    dim_req()->requestedGear = CMR_CAN_GEAR_DV_MISSION_ACCEL;

    getASMSState_fake.return_val = 1;
    dv_press()->ebsPressure_1_deci_bar = 100;
    dv_press()->ebsPressure_2_deci_bar = 100;
    fsm_data()->brakePressureFront_PSI = 700;
    cmr_sensorListGetValue_fake.return_val = 450;

    TEST_ASSERT_EQUAL(
        CMR_CAN_VSM_STATE_REQ_PRECHARGE,
        getNextState(100)
    );
}

void test_error_on_hb_fault(void) {
    assert_error_fault(
        CMR_CAN_VSM_STATE_GLV_ON,
        CMR_CAN_ERROR_VSM_MODULE_TIMEOUT,
        CMR_CAN_VSM_TIMEOUT_SOURCE_NONE,
        CMR_CAN_VSM_LATCH_NONE
    );
}

void test_error_on_timeout(void) {
    assert_error_fault(
        CMR_CAN_VSM_STATE_HV_EN,
        CMR_CAN_ERROR_NONE,
        CMR_CAN_VSM_TIMEOUT_SOURCE_DIM,
        CMR_CAN_VSM_LATCH_NONE
    );
}

void test_error_on_latch(void) {
    assert_error_fault(
        CMR_CAN_VSM_STATE_RTD,
        CMR_CAN_ERROR_NONE,
        CMR_CAN_VSM_TIMEOUT_SOURCE_NONE,
        CMR_CAN_VSM_LATCH_IMD
    );
}

void test_as_emerg_on_hb_fault(void) {
    assert_as_emerg_fault(
        CMR_CAN_VSM_STATE_AS_READY,
        CMR_CAN_ERROR_VSM_MODULE_TIMEOUT,
        CMR_CAN_VSM_TIMEOUT_SOURCE_NONE,
        CMR_CAN_VSM_LATCH_NONE
    );
}

void test_as_emerg_on_timeout(void) {
    assert_as_emerg_fault(
        CMR_CAN_VSM_STATE_AS_DRIVING,
        CMR_CAN_ERROR_NONE,
        CMR_CAN_VSM_TIMEOUT_SOURCE_DIM,
        CMR_CAN_VSM_LATCH_NONE
    );
}

void test_as_emerg_on_latch(void) {
    assert_as_emerg_fault(
        CMR_CAN_VSM_STATE_AS_FINISHED,
        CMR_CAN_ERROR_NONE,
        CMR_CAN_VSM_TIMEOUT_SOURCE_NONE,
        CMR_CAN_VSM_LATCH_IMD
    );
}

void test_as_emerg_on_res(void) {
    setup_as_clear();
    vsmStatus.canVSMStatus.internalState = CMR_CAN_VSM_STATE_AS_DRIVING;
    res()[0] = 0;

    TEST_ASSERT_EQUAL(CMR_CAN_VSM_STATE_AS_EMERGENCY, getNextState(100));
}

void test_inv_en_as_clear_fail(void) {
    setup_as_clear();
    vsmStatus.canVSMStatus.internalState = CMR_CAN_VSM_STATE_INVERTER_EN;
    invertersPass_fake.return_val = true;
    hvc_hb()->hvcState = CMR_CAN_HVC_STATE_STANDBY;

    TEST_ASSERT_EQUAL(CMR_CAN_VSM_STATE_AS_EMERGENCY, getNextState(100));
}

void test_as_ready_clear_fail(void) {
    setup_as_clear();
    vsmStatus.canVSMStatus.internalState = CMR_CAN_VSM_STATE_AS_READY;
    dv_press()->ebsPressure_1_deci_bar = 0;

    TEST_ASSERT_EQUAL(CMR_CAN_VSM_STATE_AS_EMERGENCY, getNextState(100));
}

void test_as_ready_bad_req(void) {
    setup_as_clear();
    vsmStatus.canVSMStatus.internalState = CMR_CAN_VSM_STATE_AS_READY;
    dim_req()->requestedState = CMR_CAN_GLV_ON;

    TEST_ASSERT_EQUAL(CMR_CAN_VSM_STATE_AS_EMERGENCY, getNextState(100));
}

void test_as_driving_clear_fail(void) {
    setup_as_clear();
    vsmStatus.canVSMStatus.internalState = CMR_CAN_VSM_STATE_AS_DRIVING;
    dim_req()->requestedGear = CMR_CAN_GEAR_UNKNOWN;

    TEST_ASSERT_EQUAL(CMR_CAN_VSM_STATE_AS_EMERGENCY, getNextState(100));
}

void test_as_finished_no_brake(void) {
    setup_as_clear();
    vsmStatus.canVSMStatus.internalState = CMR_CAN_VSM_STATE_AS_FINISHED;
    lastStateChangeTime_ms = 0;
    fsm_data()->brakePressureFront_PSI = 0;

    TEST_ASSERT_EQUAL(CMR_CAN_VSM_STATE_AS_EMERGENCY, getNextState(500));
}

void test_glv_as_check_fail(void) {
    vsmStatus.canVSMStatus.internalState = CMR_CAN_VSM_STATE_GLV_ON;
    dim_req()->requestedState = CMR_CAN_AS_READY;
    getASMSState_fake.return_val = 0;

    TEST_ASSERT_EQUAL(CMR_CAN_VSM_STATE_ERROR, getNextState(100));
}

void test_glv_bad_req(void) {
    vsmStatus.canVSMStatus.internalState = CMR_CAN_VSM_STATE_GLV_ON;
    dim_req()->requestedState = CMR_CAN_RTD;

    TEST_ASSERT_EQUAL(CMR_CAN_VSM_STATE_ERROR, getNextState(100));
}

void test_hv_en_bad_req(void) {
    vsmStatus.canVSMStatus.internalState = CMR_CAN_VSM_STATE_HV_EN;
    dim_req()->requestedState = CMR_CAN_AS_READY;

    TEST_ASSERT_EQUAL(CMR_CAN_VSM_STATE_ERROR, getNextState(100));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_glv_as_to_precharge);
    RUN_TEST(test_error_on_hb_fault);
    RUN_TEST(test_error_on_timeout);
    RUN_TEST(test_error_on_latch);
    RUN_TEST(test_as_emerg_on_hb_fault);
    RUN_TEST(test_as_emerg_on_timeout);
    RUN_TEST(test_as_emerg_on_latch);
    RUN_TEST(test_as_emerg_on_res);
    RUN_TEST(test_inv_en_as_clear_fail);
    RUN_TEST(test_as_ready_clear_fail);
    RUN_TEST(test_as_ready_bad_req);
    RUN_TEST(test_as_driving_clear_fail);
    RUN_TEST(test_as_finished_no_brake);
    RUN_TEST(test_glv_as_check_fail);
    RUN_TEST(test_glv_bad_req);
    RUN_TEST(test_hv_en_bad_req);
    return UNITY_END();
}
