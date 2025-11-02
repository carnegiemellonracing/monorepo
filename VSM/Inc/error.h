/**
 * @file error.h
 * @brief Vehicle Safety Module error checking.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef ERROR_H
#define ERROR_H

#include <CMR/tasks.h>  // TickType_t, taskENTER_CRITICAL(), taskEXIT_CRITICAL()

#include "state.h"      // vsmStatus_t

/** @brief Error codes. */
typedef enum {
    CANRX_TIMEOUT = 0,
    CANTX_TIMEOUT,
    BADSTATE_CDC,
    BADSTATE_DIM,
    HVC_STATE_ERROR,
    HVC_CLEAR_ERROR,
    HVC_GLV_ON,
    HVC_REQ_PRECHARGE,
    HVC_RUN_BMS,
    HVC_INVERTER_EN,
    HVC_HV_EN,
    HVC_RTD,
    HVC_AS_READY,
    HVC_AS_DRIVING,
    HVC_AS_FINISHED,
    HVC_AS_EMERGENCY,
    VSM_INVALID,
    LATCH_SOFTWARE_ERR,
    LATCH_IMD_ERR,
    LATCH_BSPD_ERR,
    DIMREQ_INVALID,
    INVERTER_COMP,
    INVERTER_ALL,
} vsmErrorCode_t;

extern const uint16_t brakePressureThreshold_PSI;

void updateCurrentErrors(volatile vsmStatus_t *vsmStatus, TickType_t lastWakeTime);
void updateCurrentWarnings(volatile vsmStatus_t *vsmStatus, TickType_t lastWakeTime);
bool invertersPass();

#endif /* ERROR_H */
