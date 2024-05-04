/**
 * @file can.h
 * @brief Board-specific CAN interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CAN_H
#define CAN_H

#include <stdbool.h>        // bool

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
    CANRX_VSM_STATUS,           /**< @brief VSM status. */
    CANRX_VSM_SENSORS,          /**< @brief VSM sensors. */
    CANRX_FSM_DATA,             /**< @brief FSM data. */
    CANRX_HVC_MINMAX_TEMPS,     /**< @brief HVC min/max cell temps. */
    CANRX_INV1_STATUS,          /**< @brief Inverter 1 temp. */
    CANRX_INV2_STATUS,          /**< @brief Inverter 2 temp. */
    CANRX_INV3_STATUS,          /**< @brief Inverter 3 temp. */
    CANRX_INV4_STATUS,          /**< @brief Inverter 4 temp. */
    CANRX_LEN     /**< @brief Number of periodic CAN messages. */
} canRX_t;

extern cmr_canRXMeta_t canRXMeta[];

extern cmr_canHeartbeat_t heartbeat;

extern uint16_t fan_1_State;
extern uint16_t fan_2_State;
extern uint16_t pump_1_State;
extern uint16_t pump_2_State;



void canInit(void);

int canTX(cmr_canID_t id, const void *data, size_t len, TickType_t timeout);
void *canGetPayload(canRX_t rxMsg);

/**
 * @brief Represents parameters for thermistor logarithmic transfer function
 *
 * temp = a + b * ln(adc_raw_value)
 *
 * a -> intercept
 * b -> slope
 */
typedef struct {
    float slope;                 /**< @brief */
    float intercept;             /**< @brief */
} thermParams_t;


#endif /* CAN_H */

