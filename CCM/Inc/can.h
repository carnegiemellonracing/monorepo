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
#include "state.h"

/**
 * @brief CAN receive metadata indices.
 *
 * @warning New messages MUST be added before `CANRX_LEN`.
 */
typedef enum {
    CANRX_HVC_HEARTBEAT,
    CANRX_HVC_PACK_VOLTAGE,
    CANRX_HVC_CELL_TEMPS,
    CANRX_HVC_CELL_VOLTAGE,
    CANRX_CCM_COMMAND,
    CANRX_VEHICLE_LEN     /**< @brief Number of periodic CAN messages. */
} canVehicleRX_t;

typedef enum {
    CANRX_CHARGER_ONE_STATE = 0,
    CANRX_CHARGER_ONE_LEN     /**< @brief Number of periodic CAN messages. */
} canChargerOneRX_t;

typedef enum {
    CANRX_CHARGER_TWO_STATE = 0,
    CANRX_CHARGER_TWO_LEN     /**< @brief Number of periodic CAN messages. */
} canChargerTwoRX_t;

extern cmr_canRXMeta_t canVehicleRXMeta[];
extern cmr_canRXMeta_t canChargerOneRXMeta[];
extern cmr_canRXMeta_t canChargerTwoRXMeta[];


void canInit(void);

int canVehicleTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);
int canChargerOneTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);
int canChargerTwoTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);
void *canGetPayload(canRX_t rxMsg);

#endif /* CAN_H */

