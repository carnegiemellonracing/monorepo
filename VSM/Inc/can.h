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
    CANRX_HEARTBEAT_HVC = 0,  /**< @brief HVC heartbeat. */
    CANRX_HEARTBEAT_CDC,      /**< @brief CDC heartbeat. */
    CANRX_HEARTBEAT_DIM,      /**< @brief DIM heartbeat. */
    CANRX_FSM_DATA,           /**< @brief FSM data. */
    CANRX_DIM_REQUEST,        /**< @brief DIM state request. */
    CANRX_INVERTER_1,         /**< @brief Inverter1 status */
    CANRX_INVERTER_2,         /**< @brief Inverter2 status */
    CANRX_INVERTER_3,         /**< @brief Inverter3 status */
    CANRX_INVERTER_4,         /**< @brief Inverter4 status */
    CANRX_LEN     /**< @brief Number of periodic CAN messages. */
} canRX_t;

//extern volatile TickType_t lastStateChangeTime;
extern cmr_canRXMeta_t canRXMeta[];
extern const cmr_canVSMErrorSource_t vsmErrorSourceFlags[];

void canInit(void);
int canTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);
void *getPayload(canRX_t rxMsg);
cmr_canState_t getModuleState(canRX_t module);

#endif /* CAN_H */

