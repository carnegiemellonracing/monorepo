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
    CANRX_HVC_PACK_VOLTAGE,     /**< @brief BMS pack voltage. */
    CANRX_CDC_WHEEL_SPEEDS,     /**< @brief CDC wheel speeds. */
    CANRX_CDC_MOTOR_DATA,       /**< @brief CDC motor data. */
    CANRX_HVC_PACK_TEMPS,       /**< @brief HVC cell temps. */
    CANRX_VSM_STATUS,           /**< @brief VSM status */
    CANRX_CDC_MOTOR_TEMPS,      /**< @brief CDC motor temps */
    CANRX_PTC_COOLING_STATUS,   /**< @brief PTC cooling status. */
    CANRX_HVC_HEARTBEAT,        /**< @brief HVC Error. */
    CANRX_CDC_MOTOR_FAULTS,      /**< @brief CDC Motor Faults */
    CANRX_LEN     /**< @brief Number of periodic CAN messages. */
} canRX_t;

extern cmr_canRXMeta_t canRXMeta[];

void canInit(void);

int canTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);

#endif /* CAN_H */
