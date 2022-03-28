/**
 * @file can.h
 * @brief Board-specific CAN interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CAN_H
#define CAN_H

#include <CMR/can.h>        // CMR CAN interface
#include <CMR/can_types.h>  // CMR CAN types
#include <CMR/can_ids.h>    // CMR CAN IDs
#include "state_task.h"
#include "BMB_task.h"
#include "spi.h"

/**
 * @brief CAN receive metadata indices.
 *
 * @warning New messages MUST be added before `CANRX_LEN`.
 */
typedef enum {
    CANRX_HEARTBEAT_VSM = 0,  /**< @brief VSM heartbeat. */
    CANRX_HVC_COMMAND,
    CANRX_LEN     /**< @brief Number of periodic CAN messages. */
} canRX_t;

extern cmr_canRXMeta_t canRXMeta[];

void canInit(void);

int canTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);

volatile void *getPayload(canRX_t rxMsg);

#endif /* CAN_H */

