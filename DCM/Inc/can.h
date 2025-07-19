/**
 * @file can.h
 * @brief Board-specific CAN interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CAN_H
#define CAN_H

#include <CMR/fdcan.h>        // CMR CAN interface
#include <CMR/can_types.h>  // CMR CAN types
#include <CMR/can_ids.h>    // CMR CAN IDs

/**
 * @brief Vehicle CAN receive metadata indices.
 *
 * @warning New messages MUST be added before `CANRX_VEH_LEN`.
 */
typedef enum {
    CANRX_VEH_HEARTBEAT_VSM = 0,    /**< @brief VSM heartbeat. */
    CANRX_VSM_STATUS,
    CANRX_VEH_DATA_FSM,             /**< @brief FSM data. */
    CANRX_VEH_SWANGLE_FSM,          /**< @brief VSM status */
    CANRX_VEH_REQUEST_DIM,          /**< @brief DIM state and gear request. */
    CANRX_VEH_VOLTAGE_HVC,          /**< @brief HVC pack voltage. */
    CANRX_VEH_CURRENT_HVC,          /**< @brief HVC pack current. */
    CANRX_VEH_DIM_ACTION_BUTTON,    /**< @brief DIM action button. */
    CANRX_VEH_PACK_CELL_VOLTAGE,    /**< @brief Min/Max Cell voltage*/
    CANRX_VEH_PACK_CELL_TEMP,       /**< @brief Min/Max Cell temp*/
    CANRX_VEH_VSM_SENSORS,          /**< @brief VSM Sensors */
	CANRX_RTC_SET,
	CANRX_HVI_SENSE,
    CANRX_VEH_MOVELLA_STATUS,
    CANRX_VEH_MOVELLA_QUATERNION,
    CANRX_VEH_MOVELLA_IMU_EULER_ANGLES,
    CANRX_VEH_MOVELLA_IMU_GYRO,
    CANRX_VEH_MOVELLA_IMU_ACCEL,
    CANRX_VEH_MOVELLA_VELOCITY,
    CANRX_VEH_LEN                   /**< @brief Number of periodic CAN messages. */
} canVehicleRX_t;

/**
 * @brief Tractive CAN receive metadata indices.
 *
 * @warning New messages MUST be added before `CANRX_TRAC_LEN`.
 */
typedef enum {
    CANRX_TRAC_INV_FL_ACT1 = 0, /**< @brief Front left inverter actual values 1. */
    CANRX_TRAC_INV_FL_ACT2,     /**< @brief Front left inverter actual values 2. */
    CANRX_TRAC_INV_FR_ACT1,     /**< @brief Front right inverter actual values 1. */
    CANRX_TRAC_INV_FR_ACT2,     /**< @brief Front right inverter actual values 2. */
    CANRX_TRAC_INV_RL_ACT1,     /**< @brief Rear left inverter actual values 1. */
    CANRX_TRAC_INV_RL_ACT2,     /**< @brief Rear left inverter actual values 2. */
    CANRX_TRAC_INV_RR_ACT1,     /**< @brief Rear right inverter actual values 1. */
    CANRX_TRAC_INV_RR_ACT2,     /**< @brief Rear right inverter actual values 2. */
    CANRX_TRAC_HVI_SENSE,       /**< @brief High voltage, current, and power sense in inverters. */
    CANRX_TRAC_LEN              /**< @brief Number of periodic CAN messages. */
} canTractiveRX_t;

/**
 * @brief DAQ CAN receive metadata indices.
 *
 * @warning New messages MUST be added before `CAN_AUX_RX_LEN`.
 */
typedef enum {
    CANRX_DAQ_MOVELLA_STATUS = 0,
    CANRX_DAQ_MOVELLA_QUATERNION,
    CANRX_DAQ_MOVELLA_IMU_EULER_ANGLES,
    CANRX_DAQ_MOVELLA_IMU_GYRO,
    CANRX_DAQ_MOVELLA_IMU_ACCEL,
    CANRX_DAQ_MOVELLA_VELOCITY,
    CANRX_DAQ_SBG_STATUS_3,     /**< @brief SBG Status containing solution info. */
    CANRX_DAQ_SBG_POS,          /**< @brief EKF Position. */
    CANRX_DAQ_SBG_VEL,          /**< @brief EKF Velocity. */
    CANRX_DAQ_SBG_ORIENT,       /**< @brief EKF Orientation. */
    CANRX_DAQ_SBG_IMU_ACCEL,    /**< @brief IMU Acceleration. */
    CANRX_DAQ_SBG_IMU_GYRO,     /**< @brief IMU Gyro rate. */
    CANRX_DAQ_SBG_BODY_VEL,     /**< @brief Body Velocity. */
    CANRX_DAQ_LOAD_FL,          /**< @brief front left load cell/newtons. */
    CANRX_DAQ_LOAD_FR,          /**< @brief front right load cell/newtons. */
    CANRX_DAQ_LOAD_RL,          /**< @brief rear left load cell/newtons. */
    CANRX_DAQ_LOAD_RR,          /**< @brief rear right load cell/newtons. */
	CANRX_DAQ_SBG_SLIPANGLE,    /**< @brief Slip Angle Radians 10^4. */
    CANRX_DAQ_LINPOTS_LEFTS,    /**< @brief front left load cell/newtons. */
    CANRX_DAQ_LINPOTS_RIGHTS,   /**< @brief front right load cell/newtons. */
    CANRX_DAQ_MEMORATOR_BROADCAST,
    CANRX_DAQ_LEN               /**< @brief Number of periodic CAN messages. */
} canDaqRX_t;

/** @brief CAN bus-id enumeration.
 *  @note 0 can be assumed to be the default bus where unspecified. */
typedef enum {
    CMR_CAN_BUS_VEH = 0,        /**< @brief Index of the VEH bus */
    CMR_CAN_BUS_DAQ,            /**< @brief Index of the DAQ bus */
    CMR_CAN_BUS_TRAC,           /**< @brief Index of the TRAC bus */
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
extern cmr_canRXMeta_t canVehicleRXMeta[CANRX_VEH_LEN];
extern cmr_canRXMeta_t canTractiveRXMeta[CANRX_TRAC_LEN];
extern cmr_canRXMeta_t canDaqRXMeta[CANRX_DAQ_LEN];

extern uint16_t fan_1_State;
extern uint16_t fan_2_State;
extern uint16_t pump_1_State;
extern uint16_t pump_2_State;

void canInit(void);
int canTX(
    cmr_canBusID_t bus, cmr_canID_t id,
    const void *data, size_t len,
    TickType_t timeout
);

volatile void *canVehicleGetPayload(canVehicleRX_t msg);
volatile void *canTractiveGetPayload(canTractiveRX_t msg);
volatile void *canDAQGetPayload(canDaqRX_t msg);
cmr_canRXMeta_t *canVehicleGetMeta(canVehicleRX_t msg);
cmr_canRXMeta_t *canTractiveGetMeta(canTractiveRX_t msg);
cmr_canRXMeta_t *canDAQGetMeta(canDaqRX_t msg);

float canEmdHvVoltage();
float canEmdHvCurrent();

#endif /* CAN_H */

