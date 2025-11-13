/**
 * @file can.h
 * @brief CAN interface
 * @author Jasmine Li
 */

#ifndef CAN_H_
#define CAN_H_

#include <CMR/can.h>        // CMR CAN interface
#include <CMR/can_types.h>  // CMR CAN types
#include <CMR/can_ids.h>    // CMR CAN IDs

/**
 * @brief CAN receive metadata indices.
 *
 * @warning New messages MUST be added before `CANRX_LEN`.
 */
typedef enum {
    CANRX_HEARTBEAT_VSM = 0,
    CANRX_VSM_STATUS,
    CANRX_LEN
} canRX_t;

void canInit(void);
int canTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);

cmr_canState_t canGetCurrentState(void);

#endif /* CAN_H_ */