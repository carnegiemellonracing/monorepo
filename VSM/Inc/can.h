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
    CANRX_FSM_SWANGLE,
    CANRX_DIM_REQUEST,        /**< @brief DIM state request. */
    CANRX_RES,                /**< @brief RES */
    CANRX_AS_PRESSURE_READING,/**< @brief Autonomous Pressure Readings */
    CANRX_ASMS_STATE,
    CANRX_FL_TEMPFAULT,       /**< @brief Front Left Inverter Fault */
    CANRX_FR_TEMPFAULT,       /**< @brief Front Right Inverter Fault */
    CANRX_RL_TEMPFAULT,       /**< @brief Rear Left Inverter Fault */
    CANRX_RR_TEMPFAULT,       /**< @brief Rear Right Inverter Fault */
    CANRX_FL_IO_STATUS,       /**< @brief Front Left Inverter Status */
    CANRX_FR_IO_STATUS,       /**< @brief Front Right Inverter Status */
    CANRX_RL_IO_STATUS,       /**< @brief Rear Left Inverter Status */
    CANRX_RR_IO_STATUS,       /**< @brief Rear Right Inverter Status */
    CANRX_FL_ERPM,
    CANRX_FR_ERPM,
    CANRX_RL_ERPM,
    CANRX_RR_ERPM,
    CANRX_LEN     /**< @brief Number of periodic CAN messages. */
} canRX_t;

//extern volatile TickType_t lastStateChangeTime;
extern cmr_canRXMeta_t canRXMeta[];
extern const cmr_canVSMErrorSource_t vsmErrorSourceFlags[];

void canInit(void);
int canTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);
void *getPayload(canRX_t rxMsg);
cmr_canState_t getModuleState(canRX_t module);
uint8_t getASMSState(void);
void sendFirstError(uint8_t error_code);
void resetError();

#endif /* CAN_H */

