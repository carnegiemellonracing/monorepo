/**
 * @file can.h
 * @brief Board-specific CAN interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CAN_H
#define CAN_H

#include <stdbool.h>        // bool

#include <CMR/can.h>        // CMR CAN interface
#include <CMR/can_types.h>  // CMR CAN types
#include <CMR/can_ids.h>    // CMR CAN IDs

/**
 * @brief CAN receive metadata indices.
 *
 * @warning New messages MUST be added before `CANRX_LEN`.
 */
typedef enum {
    CANRX_HEARTBEAT_VSM = 0,    /**< @brief VSM heartbeat. */
    CANRX_VSM_STATUS,           /**< @brief VSM status. */
    CANRX_FSM_DATA,             /**< @brief FSM data. */
    CANRX_HVC_MINMAX_TEMPS,     /**< @brief HVC min/max cell temps. */
    CANRX_LEN     /**< @brief Number of periodic CAN messages. */
} canRX_t;

extern cmr_canRXMeta_t canRXMeta[];
extern bool afcMaxCoolingEnabled;
extern cmr_canFanState_t fanState;
extern cmr_canPTCPumpState_t pumpState;

void canInit(void);

int canTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);

#endif /* CAN_H */

