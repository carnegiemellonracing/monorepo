/**
 * @file error.c
 * @brief Board-specific error detection.
 *
 * @author Carnegie Mellon Racing
 */

#include <cmr_error.h>          // Interface to implement
#include <string.h>         // memcpy()

#include <CMR/can_types.h>  // CMR CAN types
#include <CMR/tasks.h>      // TickType_t

#include "sensors.h"        // sensorList, cmr_sensorListGetFlags()
#include "can.h"            // Lots of things
#include "motors.h"         // motorLocation_t

/** @brief Inverter error status tracking. Indexed by 'motorLocation_t'. */
static bool amkErrors[MOTOR_LEN] = {
    [MOTOR_FL] = false,
    [MOTOR_FR] = false,
    [MOTOR_RL] = false,
    [MOTOR_RR] = false,
};

/** @brief Inverter timeout tracking. Indexed by 'motorLocation_t'. */
static bool amkTimeouts[MOTOR_LEN] = {
    [MOTOR_FL] = false,
    [MOTOR_FR] = false,
    [MOTOR_RL] = false,
    [MOTOR_RR] = false,
};

// Forward declarations
static void updateErrors(cmr_canError_t *errors, TickType_t lastWakeTime);
static void updateWarnings(cmr_canWarn_t *warnings, TickType_t lastWakeTime);
static void updateAMKTimeouts(TickType_t lastWakeTime);
static void updateAMKErrors(void);

/**
 * @brief Assembles error and warning vectors and copies into heartbeat.
 *
 * @warning Only call this from a FreeRTOS task.
 *
 * @param heartbeat Pointer to destination heartbeat for error and warning vectors.
 * @param lastWakeTime Current time. Pass in from canTX___Hz task.
 *
 * @return None.
 */
void updateErrorsWarnings(cmr_canHeartbeat_t *heartbeat, TickType_t lastWakeTime) {
    cmr_canError_t errors = CMR_CAN_ERROR_NONE;
    cmr_canWarn_t warnings = CMR_CAN_WARN_NONE;

    updateAMKErrors();
    updateAMKTimeouts(lastWakeTime);

    updateErrors(&errors, lastWakeTime);
    updateWarnings(&warnings, lastWakeTime);

    cmr_sensorListGetFlags(&sensorList, &warnings, &errors);

    memcpy(&(heartbeat->error), &errors, sizeof(heartbeat->error));
    memcpy(&(heartbeat->warning), &warnings, sizeof(heartbeat->warning));
}

/**
 * @brief Updates the user-provided error vector.
 *
 * @param errors Pointer to error vector to update.
 * @param lastWakeTime Current time.
 *
 * @return None.
 */
static void updateErrors(cmr_canError_t *errors, TickType_t lastWakeTime) {
    if (errors == NULL) {
        return;
    }

    static cmr_canState_t prevStateVSM = CMR_CAN_ERROR;
    volatile cmr_canHeartbeat_t *heartbeatVSM = canVehicleGetPayload(CANRX_VEH_HEARTBEAT_VSM);

    // Reset AMK timeouts upon transition to HV_EN to prevent immediate timeout error
    if (prevStateVSM == CMR_CAN_GLV_ON && heartbeatVSM->state == CMR_CAN_HV_EN) {
        for (size_t i = 0; i < CANRX_TRAC_LEN; i++) {
            canTractiveRXMeta[i].lastReceived_ms = lastWakeTime;
        }
    }

    // Vehicle CAN timeout errors
    for (size_t i = 0; i < CANRX_VEH_LEN; i++) {
        if (cmr_canRXMetaTimeoutError(&(canVehicleRXMeta[i]), lastWakeTime) < 0) {
        	if(i == 0) {
        		*errors |= canVehicleRXMeta[i].errorFlag;
        	}
            *errors |= canVehicleRXMeta[i].errorFlag;
        }
    }

    // Tractive CAN timeout errors (only occur in HV_EN and RTD when AMK is on)
    if (heartbeatVSM->state == CMR_CAN_HV_EN || heartbeatVSM->state == CMR_CAN_RTD) {
        for (size_t i = 0; i < CANRX_TRAC_LEN; i++) {
            if (cmr_canRXMetaTimeoutError(&(canTractiveRXMeta[i]), lastWakeTime) < 0) {
                *errors |= canTractiveRXMeta[i].errorFlag;
            }
        }
    }

    // ALL inverters either errored or timed out
    if ((amkErrors[MOTOR_FL] || amkTimeouts[MOTOR_FL]) &&
        (amkErrors[MOTOR_FR] || amkTimeouts[MOTOR_FR]) &&
        (amkErrors[MOTOR_RL] || amkTimeouts[MOTOR_RL]) &&
        (amkErrors[MOTOR_RR] || amkTimeouts[MOTOR_RR])) {

        *errors |= CMR_CAN_ERROR_CDC_AMK_ALL;
    }

    prevStateVSM = heartbeatVSM->state;
}

/**
 * @brief Updates the user-provided warning vector.
 *
 * @param warnings Pointer to warning vector to update.
 * @param lastWakeTime Current time.
 *
 * @return None.
 */
static void updateWarnings(cmr_canWarn_t *warnings, TickType_t lastWakeTime) {
    if (warnings == NULL) {
        return;
    }

    // Vehicle CAN timeout warnings
    for (size_t i = 0; i < CANRX_VEH_LEN; i++) {
        if (cmr_canRXMetaTimeoutWarn(&(canVehicleRXMeta[i]), lastWakeTime) < 0) {
            *warnings |= canVehicleRXMeta[i].warnFlag;
        }
    }

    // Tractive CAN timeout warnings
    for (size_t i = 0; i < CANRX_TRAC_LEN; i++) {
        if (cmr_canRXMetaTimeoutWarn(&(canTractiveRXMeta[i]), lastWakeTime) < 0) {
            *warnings |= canTractiveRXMeta[i].warnFlag;
        }
    }

    if (cmr_canRXMetaTimeoutWarn(&(canDaqRXMeta[CANRX_DAQ_MEMORATOR_BROADCAST]), lastWakeTime) < 0) {
        *warnings |= CMR_CAN_WARN_CDC_MEMORATOR_DAQ_TIMEOUT;
    }

    // Individual inverter errors
    if (amkErrors[MOTOR_FL]) {
        *warnings |= CMR_CAN_WARN_CDC_AMK_ERROR;
        *warnings |= CMR_CAN_WARN_CDC_AMK_FL;
    }
    if (amkErrors[MOTOR_FR]) {
        *warnings |= CMR_CAN_WARN_CDC_AMK_ERROR;
        *warnings |= CMR_CAN_WARN_CDC_AMK_FR;
    }
    if (amkErrors[MOTOR_RL]) {
        *warnings |= CMR_CAN_WARN_CDC_AMK_ERROR;
        *warnings |= CMR_CAN_WARN_CDC_AMK_RL;
    }
    if (amkErrors[MOTOR_RR]) {
        *warnings |= CMR_CAN_WARN_CDC_AMK_ERROR;
        *warnings |= CMR_CAN_WARN_CDC_AMK_RR;
    }
}

/**
 * @brief Updates AMK inverter timeout status flags.
 *
 * @param lastWakeTime Current time.
 *
 * @return None.
 */
static void updateAMKTimeouts(TickType_t lastWakeTime) {
    cmr_canRXMeta_t *amkAct1MetaFL = canTractiveGetMeta(CANRX_TRAC_INV_FL_ACT1);
    cmr_canRXMeta_t *amkAct2MetaFL = canTractiveGetMeta(CANRX_TRAC_INV_FL_ACT2);
    cmr_canRXMeta_t *amkAct1MetaFR = canTractiveGetMeta(CANRX_TRAC_INV_FR_ACT1);
    cmr_canRXMeta_t *amkAct2MetaFR = canTractiveGetMeta(CANRX_TRAC_INV_FR_ACT2);
    cmr_canRXMeta_t *amkAct1MetaRL = canTractiveGetMeta(CANRX_TRAC_INV_RL_ACT1);
    cmr_canRXMeta_t *amkAct2MetaRL = canTractiveGetMeta(CANRX_TRAC_INV_RL_ACT2);
    cmr_canRXMeta_t *amkAct1MetaRR = canTractiveGetMeta(CANRX_TRAC_INV_RR_ACT1);
    cmr_canRXMeta_t *amkAct2MetaRR = canTractiveGetMeta(CANRX_TRAC_INV_RR_ACT2);

    // Clear timeouts
    for (size_t i = 0; i < MOTOR_LEN; i++) {
        amkTimeouts[i] = false;
    }

    volatile cmr_canHeartbeat_t *heartbeatVSM = canVehicleGetPayload(CANRX_VEH_HEARTBEAT_VSM);

    // Set timeouts as needed
    if (heartbeatVSM->state == CMR_CAN_HV_EN || heartbeatVSM->state == CMR_CAN_RTD) {
        if (cmr_canRXMetaTimeoutError(amkAct1MetaFL, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(amkAct2MetaFL, lastWakeTime) < 0) {
            amkTimeouts[MOTOR_FL] = true;
        }
        if (cmr_canRXMetaTimeoutError(amkAct1MetaFR, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(amkAct2MetaFR, lastWakeTime) < 0) {
            amkTimeouts[MOTOR_FR] = true;
        }
        if (cmr_canRXMetaTimeoutError(amkAct1MetaRL, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(amkAct2MetaRL, lastWakeTime) < 0) {
            amkTimeouts[MOTOR_RL] = true;
        }
        if (cmr_canRXMetaTimeoutError(amkAct1MetaRR, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(amkAct2MetaRR, lastWakeTime) < 0) {
            amkTimeouts[MOTOR_RR] = true;
        }
    }
}

/**
 * @brief Updates AMK inverter error status flags.
 *
 * @return None.
 */
static void updateAMKErrors(void) {
    volatile cmr_canAMKActualValues1_t *amkAct1FL = canTractiveGetPayload(CANRX_TRAC_INV_FL_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1FR = canTractiveGetPayload(CANRX_TRAC_INV_FR_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1RL = canTractiveGetPayload(CANRX_TRAC_INV_RL_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1RR = canTractiveGetPayload(CANRX_TRAC_INV_RR_ACT1);

    // Clear error statuses
    for (size_t i = 0; i < MOTOR_LEN; i++) {
        amkErrors[i] = false;
    }

    volatile cmr_canHeartbeat_t *heartbeatVSM = canVehicleGetPayload(CANRX_VEH_HEARTBEAT_VSM);

    // Set error statuses as needed
    if (heartbeatVSM->state == CMR_CAN_HV_EN || heartbeatVSM->state == CMR_CAN_RTD) {
        if (amkAct1FL->status_bv & CMR_CAN_AMK_STATUS_ERROR) {
            amkErrors[MOTOR_FL] = true;
        }
        if (amkAct1FR->status_bv & CMR_CAN_AMK_STATUS_ERROR) {
            amkErrors[MOTOR_FR] = true;
        }
        if (amkAct1RL->status_bv & CMR_CAN_AMK_STATUS_ERROR) {
            amkErrors[MOTOR_RL] = true;
        }
        if (amkAct1RR->status_bv & CMR_CAN_AMK_STATUS_ERROR) {
            amkErrors[MOTOR_RR] = true;
        }
    }
}
