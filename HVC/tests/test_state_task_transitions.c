#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "unity.h"
#include "fff.h"

#include "state_task.h"

DEFINE_FFF_GLOBALS;

cmr_canRXMeta_t canRXMeta[CANRX_LEN];
cmr_sensorList_t sensorList;
cmr_adcChannel_t adcChannels[ADC_LEN];

FAKE_VALUE_FUNC(TickType_t, xTaskGetTickCount);
FAKE_VOID_FUNC(vTaskDelayUntil, TickType_t *, TickType_t);

FAKE_VOID_FUNC(cmr_gpioWrite, size_t, int);

FAKE_VALUE_FUNC(volatile void *, getPayload, canRX_t);
FAKE_VALUE_FUNC(int, canTX, cmr_canID_t, const void *, size_t, TickType_t);

FAKE_VALUE_FUNC(cmr_canHVCError_t, checkHVCErrors, cmr_canHVCState_t);
FAKE_VOID_FUNC(clearHVCErrorReg);
FAKE_VALUE_FUNC(cmr_canHVCError_t, getHVCErrorReg);

FAKE_VALUE_FUNC(uint8_t, setRelay, BMS_relay_t, BMS_relay_state_t);
FAKE_VALUE_FUNC(uint8_t, getRelayStatus);
FAKE_VALUE_FUNC(bool, checkRelayPowerFault);

FAKE_VALUE_FUNC(int32_t, getLVmilliamps);
FAKE_VALUE_FUNC(int32_t, getAIRmillivolts);
FAKE_VALUE_FUNC(int32_t, getSafetymillivolts);
FAKE_VALUE_FUNC(int32_t, getHVmillivolts);
FAKE_VALUE_FUNC(int32_t, getHVmilliamps);
FAKE_VALUE_FUNC(int32_t, getHVIvoltage);
FAKE_VALUE_FUNC(int32_t, getHVIcurrent);
FAKE_VALUE_FUNC(int32_t, getHVIvref);
FAKE_VALUE_FUNC(int32_t, getHVmilliamps_avg);

// 1. Helpers that populate/read the fake CAN RX metadata payload array.
static volatile void *getPayload_custom(canRX_t rxMsg) {
    if (rxMsg >= CANRX_LEN) {
        return NULL;
    }

    return (void *)(&canRXMeta[rxMsg].payload);
}

#include "../Src/state_task.c"

static cmr_canHVCCommand_t *hvc_cmd(void) {
    return (cmr_canHVCCommand_t *)(void *)(&canRXMeta[CANRX_HVC_COMMAND].payload);
}

static cmr_canHVBMSPackVoltage_t *pack_volt(void) {
    return (cmr_canHVBMSPackVoltage_t *)(void *)(&canRXMeta[CANRX_HVBMS_PACKVOLT].payload);
}

static void reset_rx(void) {
    for (size_t i = 0; i < CANRX_LEN; i++) {
        for (size_t j = 0; j < sizeof(canRXMeta[i].payload); j++) {
            canRXMeta[i].payload[j] = 0;
        }
    }
}

static void reset_state(void) {
    currentState = CMR_CAN_HVC_STATE_ERROR;
    cellBalancing = false;
    lastPrechargeTime = 0;
}

void setUp(void) {
    RESET_FAKE(xTaskGetTickCount);
    RESET_FAKE(vTaskDelayUntil);
    RESET_FAKE(cmr_gpioWrite);
    RESET_FAKE(getPayload);
    RESET_FAKE(canTX);
    RESET_FAKE(checkHVCErrors);
    RESET_FAKE(clearHVCErrorReg);
    RESET_FAKE(getHVCErrorReg);
    RESET_FAKE(setRelay);
    RESET_FAKE(getRelayStatus);
    RESET_FAKE(checkRelayPowerFault);
    RESET_FAKE(getLVmilliamps);
    RESET_FAKE(getAIRmillivolts);
    RESET_FAKE(getSafetymillivolts);
    RESET_FAKE(getHVmillivolts);
    RESET_FAKE(getHVmilliamps);
    RESET_FAKE(getHVIvoltage);
    RESET_FAKE(getHVIcurrent);
    RESET_FAKE(getHVIvref);
    RESET_FAKE(getHVmilliamps_avg);
    FFF_RESET_HISTORY();

    reset_rx();
    reset_state();

    getPayload_fake.custom_fake = getPayload_custom;
    checkHVCErrors_fake.return_val = CMR_CAN_HVC_ERROR_NONE;
}

void tearDown(void) {
}

// 2. State machine setup helpers.
static void setup_precharge(void) {
    currentState = CMR_CAN_HVC_STATE_DRIVE_PRECHARGE;
    hvc_cmd()->modeRequest = CMR_CAN_HVC_MODE_START;
    lastPrechargeTime = 0;
}

void test_precharge_bad_delta_stays(void) {
    setup_precharge();

    pack_volt()->battVoltage_mV = 420000;
    getHVmillivolts_fake.return_val = 360000;
    xTaskGetTickCount_fake.return_val = 6001;

    TEST_ASSERT_EQUAL(
        CMR_CAN_HVC_STATE_DRIVE_PRECHARGE,
        getNextState(CMR_CAN_HVC_ERROR_NONE)
    );
}

void test_precharge_low_pack_stays(void) {
    setup_precharge();

    pack_volt()->battVoltage_mV = 359999;
    getHVmillivolts_fake.return_val = 359999;
    xTaskGetTickCount_fake.return_val = 6001;

    TEST_ASSERT_EQUAL(
        CMR_CAN_HVC_STATE_DRIVE_PRECHARGE,
        getNextState(CMR_CAN_HVC_ERROR_NONE)
    );
}

void test_precharge_too_early_stays(void) {
    setup_precharge();

    pack_volt()->battVoltage_mV = 400000;
    getHVmillivolts_fake.return_val = 400000;
    xTaskGetTickCount_fake.return_val = 5000;

    TEST_ASSERT_EQUAL(
        CMR_CAN_HVC_STATE_DRIVE_PRECHARGE,
        getNextState(CMR_CAN_HVC_ERROR_NONE)
    );
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_precharge_bad_delta_stays);
    RUN_TEST(test_precharge_low_pack_stays);
    RUN_TEST(test_precharge_too_early_stays);
    return UNITY_END();
}
