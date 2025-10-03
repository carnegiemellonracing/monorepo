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
static bool dtiErrors[MOTOR_LEN] = {
    [MOTOR_FL] = false,
    [MOTOR_FR] = false,
    [MOTOR_RL] = false,
    [MOTOR_RR] = false,
};

/** @brief Inverter timeout tracking. Indexed by 'motorLocation_t'. */
static bool dtiTimeouts[MOTOR_LEN] = {
    [MOTOR_FL] = false,
    [MOTOR_FR] = false,
    [MOTOR_RL] = false,
    [MOTOR_RR] = false,
};

// Forward declarations
static void updateErrors(cmr_canError_t *errors, TickType_t lastWakeTime);
static void updateWarnings(cmr_canWarn_t *warnings, TickType_t lastWakeTime);
static void updateDTITimeouts(TickType_t lastWakeTime);
static void updateDTIErrors(void);

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

    updateDTIErrors();
    updateDTITimeouts(lastWakeTime);

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

    // Reset DTI timeouts upon transition to HV_EN to prevent immediate timeout error
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

    // Tractive CAN timeout errors (only occur in HV_EN and RTD when DTI is on)
    if (heartbeatVSM->state == CMR_CAN_HV_EN || heartbeatVSM->state == CMR_CAN_RTD) {
        for (size_t i = 0; i < CANRX_TRAC_LEN; i++) {
            if (cmr_canRXMetaTimeoutError(&(canTractiveRXMeta[i]), lastWakeTime) < 0) {
                *errors |= canTractiveRXMeta[i].errorFlag;
            }
        }
    }

    // ALL inverters either errored or timed out
    if ((dtiErrors[MOTOR_FL] || dtiTimeouts[MOTOR_FL]) &&
        (dtiErrors[MOTOR_FR] || dtiTimeouts[MOTOR_FR]) &&
        (dtiErrors[MOTOR_RL] || dtiTimeouts[MOTOR_RL]) &&
        (dtiErrors[MOTOR_RR] || dtiTimeouts[MOTOR_RR])) {

        *errors |= CMR_CAN_ERROR_CDC_DTI_ALL;
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
    if (dtiErrors[MOTOR_FL]) {
        *warnings |= CMR_CAN_WARN_CDC_DTI_ERROR;
        *warnings |= CMR_CAN_WARN_CDC_DTI_FL;
    }
    if (dtiErrors[MOTOR_FR]) {
        *warnings |= CMR_CAN_WARN_CDC_DTI_ERROR;
        *warnings |= CMR_CAN_WARN_CDC_DTI_FR;
    }
    if (dtiErrors[MOTOR_RL]) {
        *warnings |= CMR_CAN_WARN_CDC_DTI_ERROR;
        *warnings |= CMR_CAN_WARN_CDC_DTI_RL;
    }
    if (dtiErrors[MOTOR_RR]) {
        *warnings |= CMR_CAN_WARN_CDC_DTI_ERROR;
        *warnings |= CMR_CAN_WARN_CDC_DTI_RR;
    }
}

/**
 * @brief Updates DTI inverter timeout status flags.
 *
 * @param lastWakeTime Current time.
 *
 * @return None.
 */
static void updateDTITimeouts(TickType_t lastWakeTime) {
    cmr_canRXMeta_t *dtiErpmFL = canTractiveGetMeta(CANRX_TRAC_FL_ERPM);
    cmr_canRXMeta_t *dtiCurrentFL = canTractiveGetMeta(CANRX_TRAC_FL_CURRENT);
    cmr_canRXMeta_t *dtiTempFaultFL = canTractiveGetMeta(CANRX_TRAC_FL_TEMPFAULT);
    cmr_canRXMeta_t *dtiIdIqFL = canTractiveGetMeta(CANRX_TRAC_FL_IDIQ);
    cmr_canRXMeta_t *dtiACLimsFL = canTractiveGetMeta(CANRX_TRAC_FL_ACLIMS);
    cmr_canRXMeta_t *dtiDCLimsFL = canTractiveGetMeta(CANRX_TRAC_FL_DCLIMS);
    cmr_canRXMeta_t *dtiControlStatusFL = canTractiveGetMeta(CANRX_TRAC_FL_CONTROL_STATUS);
    cmr_canRXMeta_t *dtiIOStatusFL = canTractiveGetMeta(CANRX_TRAC_FL_IO_STATUS);

    cmr_canRXMeta_t *dtiErpmFR = canTractiveGetMeta(CANRX_TRAC_FR_ERPM);
    cmr_canRXMeta_t *dtiCurrentFR = canTractiveGetMeta(CANRX_TRAC_FR_CURRENT);
    cmr_canRXMeta_t *dtiTempFaultFR = canTractiveGetMeta(CANRX_TRAC_FR_TEMPFAULT);
    cmr_canRXMeta_t *dtiIdIqFR = canTractiveGetMeta(CANRX_TRAC_FR_IDIQ);
    cmr_canRXMeta_t *dtiACLimsFR = canTractiveGetMeta(CANRX_TRAC_FR_ACLIMS);
    cmr_canRXMeta_t *dtiDCLimsFR = canTractiveGetMeta(CANRX_TRAC_FR_DCLIMS);
    cmr_canRXMeta_t *dtiControlStatusFR = canTractiveGetMeta(CANRX_TRAC_FR_CONTROL_STATUS);
    cmr_canRXMeta_t *dtiIOStatusFR = canTractiveGetMeta(CANRX_TRAC_FR_IO_STATUS);

    cmr_canRXMeta_t *dtiErpmRR = canTractiveGetMeta(CANRX_TRAC_RR_ERPM);
    cmr_canRXMeta_t *dtiCurrentRR = canTractiveGetMeta(CANRX_TRAC_RR_CURRENT);
    cmr_canRXMeta_t *dtiTempFaultRR = canTractiveGetMeta(CANRX_TRAC_RR_TEMPFAULT);
    cmr_canRXMeta_t *dtiIdIqRR = canTractiveGetMeta(CANRX_TRAC_RR_IDIQ);
    cmr_canRXMeta_t *dtiACLimsRR = canTractiveGetMeta(CANRX_TRAC_RR_ACLIMS);
    cmr_canRXMeta_t *dtiDCLimsRR = canTractiveGetMeta(CANRX_TRAC_RR_DCLIMS);
    cmr_canRXMeta_t *dtiControlStatusRR = canTractiveGetMeta(CANRX_TRAC_RR_CONTROL_STATUS);
    cmr_canRXMeta_t *dtiIOStatusRR = canTractiveGetMeta(CANRX_TRAC_RR_IO_STATUS);

    cmr_canRXMeta_t *dtiErpmRL = canTractiveGetMeta(CANRX_TRAC_RL_ERPM);
    cmr_canRXMeta_t *dtiCurrentRL = canTractiveGetMeta(CANRX_TRAC_RL_CURRENT);
    cmr_canRXMeta_t *dtiTempFaultRL = canTractiveGetMeta(CANRX_TRAC_RL_TEMPFAULT);
    cmr_canRXMeta_t *dtiIdIqRL = canTractiveGetMeta(CANRX_TRAC_RL_IDIQ);
    cmr_canRXMeta_t *dtiACLimsRL = canTractiveGetMeta(CANRX_TRAC_RL_ACLIMS);
    cmr_canRXMeta_t *dtiDCLimsRL = canTractiveGetMeta(CANRX_TRAC_RL_DCLIMS);
    cmr_canRXMeta_t *dtiControlStatusRL = canTractiveGetMeta(CANRX_TRAC_RL_CONTROL_STATUS);
    cmr_canRXMeta_t *dtiIOStatusRL = canTractiveGetMeta(CANRX_TRAC_RL_IO_STATUS);
    // Clear timeouts
    for (size_t i = 0; i < MOTOR_LEN; i++) {
        dtiTimeouts[i] = false;
    }

    volatile cmr_canHeartbeat_t *heartbeatVSM = canVehicleGetPayload(CANRX_VEH_HEARTBEAT_VSM);

    // Set timeouts as needed
    if (heartbeatVSM->state == CMR_CAN_HV_EN || heartbeatVSM->state == CMR_CAN_RTD) {
        if (cmr_canRXMetaTimeoutError(dtiErpmFL, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiCurrentFL, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiTempFaultFL, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiIdIqFL, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiACLimsFL, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiDCLimsFL, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiControlStatusFL, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiIOStatusFL, lastWakeTime) < 0) {
            dtiTimeouts[MOTOR_FL] = true;
        }
        if (cmr_canRXMetaTimeoutError(dtiErpmFR, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiCurrentFR, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiTempFaultFR, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiIdIqFR, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiACLimsFR, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiDCLimsFR, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiControlStatusFR, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiIOStatusFR, lastWakeTime) < 0) {
            dtiTimeouts[MOTOR_FR] = true;
        }
        if (cmr_canRXMetaTimeoutError(dtiErpmRR, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiCurrentRR, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiTempFaultRR, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiIdIqRR, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiACLimsRR, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiDCLimsRR, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiControlStatusRR, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiIOStatusRR, lastWakeTime) < 0) {
            dtiTimeouts[MOTOR_RR] = true;
        }
        if (cmr_canRXMetaTimeoutError(dtiErpmRL, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiCurrentRL, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiTempFaultRL, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiIdIqRL, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiACLimsRL, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiDCLimsRL, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiControlStatusRL, lastWakeTime) < 0 ||
            cmr_canRXMetaTimeoutError(dtiIOStatusRL, lastWakeTime) < 0) {
            dtiTimeouts[MOTOR_RL] = true;
        }
    }
}

/**
 * @brief Updates DTI inverter error status flags.
 *
 * @return None.
 */
static void updateDTIErrors(void) {
    volatile cmr_canDTI_TX_TempFault_t *dtiTempFaultFL = canTractiveGetPayload(CANRX_TRAC_FL_TEMPFAULT);
    volatile cmr_canDTI_TX_TempFault_t *dtiTempFaultFR = canTractiveGetPayload(CANRX_TRAC_FR_TEMPFAULT);
    volatile cmr_canDTI_TX_TempFault_t *dtiTempFaultRL = canTractiveGetPayload(CANRX_TRAC_RL_TEMPFAULT);
    volatile cmr_canDTI_TX_TempFault_t *dtiTempFaultRR = canTractiveGetPayload(CANRX_TRAC_RR_TEMPFAULT);

    // Clear error statuses
    for (size_t i = 0; i < MOTOR_LEN; i++) {
        dtiErrors[i] = false;
    }

    volatile cmr_canHeartbeat_t *heartbeatVSM = canVehicleGetPayload(CANRX_VEH_HEARTBEAT_VSM);

    // Set error statuses as needed
    if (heartbeatVSM->state == CMR_CAN_HV_EN || 
        heartbeatVSM->state == CMR_CAN_RTD || 
        heartbeatVSM->state == CMR_CAN_AS_READY ||
        heartbeatVSM->state == CMR_CAN_AS_DRIVING) {

        dtiErrors[MOTOR_FL] = !!(dtiTempFaultFL->fault_code);
        dtiErrors[MOTOR_RL] = !!(dtiTempFaultRL->fault_code);
        dtiErrors[MOTOR_FR] = !!(dtiTempFaultFR->fault_code);
        dtiErrors[MOTOR_RR] = !!(dtiTempFaultRR->fault_code);
     }
}
