/**
 * @file can.h
 * @brief Board-specific CAN interface.
 * @author Carnegie Mellon Racing
**/


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
    CANRX_HEARTBEAT_DAQ = 0,  /**< @brief VSM heartbeat. */
    CANRX_LEN_DAQ     /**< @brief Number of periodic CAN messages. */
} canRX_DAQ_t;

typedef enum {
    CANRX_HEARTBEAT_VSM = 0,  /**< @brief VSM heartbeat. */
	CANRX_HVC_COMMAND,
	CANRX_CCM_HEARTBEAT,
    CANRX_LEN_VEH     /**< @brief Number of periodic CAN messages. */
} canRX_VEH_t;

typedef enum {
    CANRX_HEARTBEAT_INV = 0,  /**< @brief VSM heartbeat. */
    CANRX_LEN_TRAC    /**< @brief Number of periodic CAN messages. */
} canRX_TRAC_t;

typedef enum {
    CANRX_MAX_VOLTAGE_HIGH = 0,
    CANRX_MAX_VOLTAGE_LOW,
    CANRX_MAX_CURRENT_HIGH,
    CANRX_MAX_CURRENT_LOW,
    CANRX_CHARGER_DISABLE,
    CANRX_ENABLE_HEATING
} canRX_canDilongCommand_t;

extern cmr_canRXMeta_t canRXMeta_DAQ[];
extern cmr_canRXMeta_t canRXMeta_VEH[];
extern cmr_canRXMeta_t canRXMeta_TRAC[];
extern cmr_canRXMeta_t canRXMeta_canDilongCommand[];

// message modifying function
void setHVCHeartbeat(uint8_t state, uint8_t mode);
void setCCMCommand(uint8_t mode);

void canInit(void);

int canTX_DAQ(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);
int canTX_VEH(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);
int canTX_TRAC(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);
int canTX_Dilong(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);

void *getPayloadDAQ(canRX_DAQ_t rxMsg);
void *getPayloadVEH(canRX_VEH_t rxMsg);
void *getPayloadTRAC(canRX_TRAC_t rxMsg);


#endif /* CAN_H */
