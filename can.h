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

/* ==========================================================
 *
 * CMR_PTC_ID is used to select between PTCp and PTCf code.
 *
 *      0 - indicates the use of the Pump Control Board
 *      1 - indicates the use of the Fan Control Board
 *
 * ==========================================================
 */

#define CMR_PTC_ID 1

/**
 * @brief CAN receive metadata indices.
 *
 * @warning New messages MUST be added before `CANRX_LEN`.
 */
typedef enum {
    CANRX_HEARTBEAT_VSM = 0,    /**< @brief VSM heartbeat. */
    CANRX_VSM_STATUS,           /**< @brief VSM status. */
    CANRX_VSM_SENSORS,          /**< @brief VSM sensors. */
    CANRX_FSM_DATA,             /**< @brief FSM data. */
    CANRX_HVC_MINMAX_TEMPS,     /**< @brief HVC min/max cell temps. */
    CANRX_LEN     /**< @brief Number of periodic CAN messages. */
} canRX_t;

extern cmr_canRXMeta_t canRXMeta[];
extern uint16_t channel_1_State;
extern uint16_t channel_2_State;
extern uint16_t channel_3_State;

void canInit(void);

int canTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);
void *canGetPayload(canRX_t rxMsg);

#endif /* CAN_H */

