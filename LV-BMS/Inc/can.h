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

/**
 * @brief CAN receive metadata indices.
 *
 * @warning New messages MUST be added before `CANRX_LEN`.
 */
typedef enum {
    CANRX_HEARTBEAT_VSM = 0,
    CANRX_LEN,     /**< @brief Number of periodic CAN messages. */
} canRX_t;

extern cmr_canRXMeta_t canRXMeta[];

extern cmr_canHeartbeat_t heartbeat;

void canInit(void);

extern static const uint32_t canTX10Hz_priority;
extern static const TickType_t canTX10Hz_period_ms;

extern static const uint32_t canTX100Hz_priority;

extern static const TickType_t canTX100Hz_period_ms;

extern static const uint32_t canTX1Hz_priority;
extern static const TickType_t canTX1Hz_period_ms;

int canTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);
volatile void *getPayload(canRX_t rxMsg);

#endif /* CAN_H */

