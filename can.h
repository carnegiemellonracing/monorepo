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

// Size of text buffer from RAM
#define RAMBUFLEN 1024

/** @brief Text buffer from RAM - used to display messages to driver */
char RAMBUF[RAMBUFLEN];

/**
 * @brief CAN receive metadata indices.
 *
 * @warning New messages MUST be added before `CANRX_LEN`.
 */
typedef enum {
    CANRX_HEARTBEAT_VSM = 0,    /**< @brief VSM heartbeat. */
    CANRX_HVC_PACK_VOLTAGE,     /**< @brief BMS pack voltage. */
    CANRX_AMK_FL_ACT_1,         /**< @brief AMK FL status*/
    CANRX_AMK_FR_ACT_1,         /**< @brief AMK FR status*/
    CANRX_AMK_RL_ACT_1,         /**< @brief AMK BL status*/
    CANRX_AMK_RR_ACT_1,         /**< @brief AMK BR status*/
    CANRX_HVC_PACK_TEMPS,       /**< @brief HVC cell temps. */
    CANRX_VSM_STATUS,           /**< @brief VSM status */
    CANRX_PTCf_LOOP_A_TEMPS,    /**< @brief PTCf Loop A temps */
    CANRX_PTCf_LOOP_B_TEMPS,    /**< @brief PTCf Loop B temps */
    CANRX_PTCp_LOOP_A_TEMPS,    /**< @brief PTCp Loop A temps */
    CANRX_PTCp_LOOP_B_TEMPS,    /**< @brief PTCp Loop B temps */
    CANRX_HVC_HEARTBEAT,        /**< @brief HVC Error. */
    CANRX_CDC_MOTOR_FAULTS,     /**< @brief CDC Motor Faults */
    CANRX_CDL_BROADCAST,        /**< @brief CDL broadcast. */
    CANRX_SBG_STATUS_3,            /**< @brief INS Status 3 */
    CANRX_LEN     /**< @brief Number of periodic CAN messages. */
} canRX_t;

extern cmr_canRXMeta_t canRXMeta[];

void canInit(void);

int canTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);

void ramCallback (cmr_can_t *can, uint16_t canID, const void *data, size_t dataLen);
#endif /* CAN_H */
