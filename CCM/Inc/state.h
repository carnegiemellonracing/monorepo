/**
 * @file state.h
 * @brief Charger Control Module state machine.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef STATE_H
#define STATE_H

#include "can.h"

/** @brief Charger Control Modules internal states. */
typedef enum {
    CMR_CCM_STATE_ERROR = 0,        /**< @brief Error state. */
    CMR_CCM_STATE_CLEAR_ERROR,      /**< @brief Clear error state. */
    CMR_CCM_STATE_CLEAR_HVC,        /**< @brief Wait for HVC to enter clear error state */
    CMR_CCM_STATE_IDLE_HVC,         /**< @brief Wait for HVC to enter standby state */
    CMR_CCM_STATE_STANDBY,          /**< @brief Charger control module on. */
    CMR_CCM_STATE_CHARGE_REQ,       /**< @brief Requesting charging mode */
    CMR_CCM_STATE_CHARGE,           /**< @brief Charging. */
    CMR_CCM_STATE_SLOW_CHARGE,      /**< @brief Cell Balancing */
    CMR_CCM_STATE_SHUTDOWN,         /**< @brief Shutdown chargers before HVC shutdown */
    CMR_CCM_STATE_LEN               /**< @brief Number of CCM states. */
} cmr_CCMState_t;

typedef enum {
    CMR_CCM_COMMAND_NONE = 0,
    CMR_CCM_COMMAND_OFF,
    CMR_CCM_COMMAND_SLOW,
    CMR_CCM_COMMAND_FAST,
    CMR_CCM_COMMAND_RESET
} cmr_CCMCommand_t;

extern volatile cmr_CCMState_t state;
extern volatile cmr_canHVCMode_t hvcModeRequest;

void setCommand(cmr_CCMCommand_t command);
void stateInit(void);

#endif
