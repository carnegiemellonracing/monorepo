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
    CANRX_HEARTBEAT_VSM = 0,    /**< @brief VSM heartbeat. */
    CANRX_VSM_SENSORS,          /**< @brief VSM sensors data. */
    CANRX_HVC_VOLTAGE,
    CANRX_HVC_TEMPS,
    CANRX_SBG_VELOCITY,
    CANRX_SBG_ACCELERATION,
    CANRX_SBG_GPS_POS,
    CANRX_PTC_TEMPS,
    CANRX_LEN     /**< @brief Number of periodic CAN messages. */
} canRX_t;

/** @brief CAN bus-id enumeration.
 *  @note 0 can be assumed to be the default bus where unspecified. */
typedef enum {
    CMR_CAN_BUS_VEH = 0,        /**< @brief Index of the VEH bus */
    CMR_CAN_BUS_DAQ,            /**< @brief Index of the DAQ bus */
    CMR_CAN_BUS_NUM,            /**< @brief Number of busses in use */
} cmr_canBusID_t;

/** @brief Number of bits in a CAN ID. */
#define CAN_ID_BITS 11

/** @brief "Packed" CAN message. */
typedef struct {
    uint16_t idLen;         /**< @brief ID ([10:0]) and length ([14:11]). */
    uint8_t payload[8];     /**< @brief Payload data. */
} canMsg_t;

extern cmr_canRXMeta_t canRXMeta[];

void canInit(void);
int canTX(
    cmr_canBusID_t bus_id, cmr_canID_t id,
    const void *data, size_t len,
    TickType_t timeout_ms
);

void *getPayload(canRX_t rxMsg);
uint8_t throttleGetPos(void);

#endif /* CAN_H */

