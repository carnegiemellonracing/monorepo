/**
 * @file state.h
 * @brief Vehicle Safety Module state machine.
 *
 * @note Full state machine description available at
 * http://cmr-linux.club.cc.cmu.edu/confluence/display/EN/19e+Vehicle+Safety+Module
 *
 * @author Carnegie Mellon Racing
 */

#ifndef STATE_H
#define STATE_H

#include <CMR/can_types.h>  // cmr_canVSMState_t, cmr_canVSMErrors_t for this file (state.h)
                            // and lots of structs/enums for state.c

/** @brief Struct to contain both heartbeat and error matrix errors. */
typedef struct {
    uint16_t heartbeatErrors;          /**< 2B bitvector of errors, sent in heartbeat. */
    uint16_t heartbeatWarnings;        /**< 2B bitvector of warnings, sent in hearbeat.*/
    cmr_canVSMStatus_t canVSMStatus;   /**< 1B Status.
    * Car runs, by and large, on this value. Sent in status message */
    cmr_canVSMLatchedStatus_t canVSMLatchedStatus; /**< 1B Latched status for postmortems. */
    cmr_canState_t dimRequestReject;    /**< @brief Rejected DIM state request. */
} vsmStatus_t;

/** @brief Struct to contain the states of the brake test for autonomous */
typedef enum {
    BRAKE_TEST_NOT_STARTED = 0,
    BRAKE_TEST_RUNNING,
    BRAKE_TEST_PASSED,
    BRAKE_TEST_FAILED
} brakeTestState_t;

extern cmr_canState_t vsmToCANState[];
extern volatile TickType_t lastStateChangeTime_ms;
extern volatile cmr_canHVCMode_t hvcModeRequest;

void stateInit(void);
cmr_canVSMState_t getCurrentState(void);
const vsmStatus_t *getCurrentStatus(void);
uint16_t getCurrentWarnings(void);
void updateErrorsAndWarnings(TickType_t lastWakeTime);

#endif /* STATE_H */
