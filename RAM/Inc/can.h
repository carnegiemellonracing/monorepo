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
 * @brief Vehicle CAN receive metadata indices.
 *
 * @warning New messages MUST be added before `CANRX_VEH_LEN`.
 */
typedef enum {
    CANRX_VEH_HEARTBEAT_VSM = 0,    /**< @brief VSM heartbeat. */
    CANRX_VEH_DATA_FSM,             /**< @brief FSM data. */
    CANRX_VEH_SWANGLE_FSM,
    CANRX_VEH_REQUEST_DIM,          /**< @brief DIM state and gear request. */
    CANRX_VEH_VOLTAGE_HVC,          /**< @brief HVC pack voltage. */
    CANRX_VEH_CURRENT_HVC,          /**< @brief HVC pack current. */
    CANRX_VEH_DIM_ACTION_BUTTON,    /**< @brief DIM action button. */
    CANRX_VEH_PACK_CELL_VOLTAGE,    /**< @brief Min/Max Cell voltage*/
    CANRX_VEH_PACK_CELL_TEMP,       /**< @brief Min/Max Cell temp*/
    CANRX_VEH_EMD_MEASURE,          /**< @brief EMD Measurement */
    CANRX_VEH_VSM_SENSORS,          /**< @brief VSM Sensors */
	CANRX_RTC_SET,
    CANRX_VEH_LEN                   /**< @brief Number of periodic CAN messages. */
} canVehicleRX_t;

/**
 * @brief Tractive CAN receive metadata indices.
 *
 * @warning New messages MUST be added before `CANRX_TRAC_LEN`.
 */
typedef enum {
   CANRX_TRAC_FL_ERPM = 0,
    CANRX_TRAC_FL_CURRENT,
    CANRX_TRAC_FL_TEMPFAULT,
    CANRX_TRAC_FL_IDIQ,
    CANRX_TRAC_FL_ACLIMS,
    CANRX_TRAC_FL_DCLIMS,
    CANRX_TRAC_FL_CONTROL_STATUS,
    CANRX_TRAC_FL_IO_STATUS,

    CANRX_TRAC_FR_ERPM,
    CANRX_TRAC_FR_CURRENT,
    CANRX_TRAC_FR_TEMPFAULT,
    CANRX_TRAC_FR_IDIQ,
    CANRX_TRAC_FR_ACLIMS,
    CANRX_TRAC_FR_DCLIMS,
    CANRX_TRAC_FR_CONTROL_STATUS,
    CANRX_TRAC_FR_IO_STATUS,

    CANRX_TRAC_RR_ERPM,
    CANRX_TRAC_RR_CURRENT,
    CANRX_TRAC_RR_TEMPFAULT,
    CANRX_TRAC_RR_IDIQ,
    CANRX_TRAC_RR_ACLIMS,
    CANRX_TRAC_RR_DCLIMS,
    CANRX_TRAC_RR_CONTROL_STATUS,
    CANRX_TRAC_RR_IO_STATUS,

    CANRX_TRAC_RL_ERPM,
    CANRX_TRAC_RL_CURRENT,
    CANRX_TRAC_RL_TEMPFAULT,
    CANRX_TRAC_RL_IDIQ,
    CANRX_TRAC_RL_ACLIMS,
    CANRX_TRAC_RL_DCLIMS,
    CANRX_TRAC_RL_CONTROL_STATUS,
    CANRX_TRAC_RL_IO_STATUS,
    CANRX_TRAC_LEN              /**< @brief Number of periodic CAN messages. */
} canTractiveRX_t;

/**
 * @brief DAQ CAN receive metadata indices.
 *
 * @warning New messages MUST be added before `CAN_AUX_RX_LEN`.
 */
typedef enum {
    CANRX_DAQ_SBG_STATUS_3 = 0, /**< @brief SBG Status containing solution info. */
    CANRX_DAQ_SBG_POS,          /**< @brief EKF Position. */
    CANRX_DAQ_SBG_VEL,          /**< @brief EKF Velocity. */
    CANRX_DAQ_SBG_ORIENT,       /**< @brief EKF Orientation. */
    CANRX_DAQ_SBG_IMU_ACCEL,    /**< @brief IMU Acceleration. */
    CANRX_DAQ_SBG_IMU_GYRO,     /**< @brief IMU Gyro rate. */
    CANRX_DAQ_SBG_BODY_VEL,     /**< @brief Body Velocity. */
    CANRX_DAQ_EMD_MEASURE,      /**< @brief EMD HV volts/amps. */
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
	CMR_CAN_BUS_TRAC,			/**< @brief Index of the TRAC bus */
    CMR_CAN_BUS_NUM,            /**< @brief Number of buses in use */
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

//void *getPayload(canRX_t rxMsg);
uint8_t throttleGetPos(void);

#endif /* CAN_H */
