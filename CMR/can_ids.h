/**
 * @file can_ids.h
 * @brief Shared CAN ID definitions.
 *
 * @see http://cmr-linux.club.cc.cmu.edu/confluence/display/EN/CAN+ID+Structure
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_CAN_IDS_H
#define CMR_CAN_IDS_H

/** @brief The RMS motor controller allows a configurable offset for
 *  all CAN message addresses. Ours is set to this value.
 */
#define CMR_CANID_RMS_OFFSET    0x3A0

/** @brief CAN IDs. */
typedef enum {
    CMR_CANID_HEARTBEAT_VSM = 0x100,    /**< @brief VSM heartbeat. */
    CMR_CANID_HEARTBEAT_HVC = 0x101,    /**< @brief HVC heartbeat. */
    CMR_CANID_HEARTBEAT_CDC = 0x102,    /**< @brief CDC heartbeat. */
    CMR_CANID_HEARTBEAT_FSM = 0x103,    /**< @brief FSM heartbeat. */
    CMR_CANID_HEARTBEAT_PTC = 0x104,    /**< @brief PTC heartbeat. */
    CMR_CANID_HEARTBEAT_DIM = 0x105,    /**< @brief DIM heartbeat. */
    CMR_CANID_HEARTBEAT_AFC0 = 0x106,   /**< @brief AFC0 heartbeat. */
    CMR_CANID_HEARTBEAT_AFC1 = 0x107,   /**< @brief AFC1 heartbeat. */
    CMR_CANID_HEARTBEAT_APC = 0x108,    /**< @brief APC heartbeat. */
    CMR_CANID_HEARTBEAT_TOM = 0x109,    /**< @brief TOM heartbeat. */
    CMR_CANID_HEARTBEAT_COM = 0x10A,    /**< @brief COM heartbeat. */

    CMR_CANID_VSM_STATUS = 0x110,               /**< @brief VSM status. */
    CMR_CANID_VSM_SENSORS = 0x200,              /**< @brief VSM sensor data. */
    CMR_CANID_VSM_LATCHED_STATUS = 0x510,       /**< @brief VSM latched status. */
    CMR_CANID_VSM_POWER_DIAGNOSTICS = 0x530,    /**< @brief VSM power diagnostics. */

    CMR_CANID_HVC_COMMAND = 0x130,              /**< @brief HVC command, sent by VSM. */
    CMR_CANID_HVC_PACK_VOLTAGE = 0x301,         /**< @brief HVC pack voltage. */
    CMR_CANID_HVC_MINMAX_CELL_TEMPS = 0x311,    /**< @brief HVC pack min and max cell temps. */

    CMR_CANID_CDC_WHEEL_SPEEDS = 0x132,         /**< @brief CDC wheel speeds. */
    CMR_CANID_CDC_SOLENOID_PTC = 0x142,         /**< @brief CDC brake solenoid command. */
    CMR_CANID_CDC_MOTOR_DATA = 0x152,           /**< @brief CDC motor data. */
    CMR_CANID_CDC_MOTOR_FAULTS = 0x502,         /**< @brief CDC motor faults. */
    CMR_CANID_CDC_MOTOR_TEMPS = 0x512,          /**< @brief CDC motor temperatures. */

    CMR_CANID_FSM_DATA = 0x133,                 /**< @brief FSM data. */
    CMR_CANID_FSM_PEDALS_ADC = 0x533,           /**< @brief FSM raw pedal positions. */
    CMR_CANID_FSM_SENSORS_ADC = 0x543,          /**< @brief FSM raw sensors. */
    CMR_CANID_FSM_POWER_DIAGNOSTICS = 0x553,    /**< @brief FSM power diagnostics. */

    CMR_CANID_PTC_COOLING_STATUS = 0x234,       /**< @brief PTC cooling status. */
    CMR_CANID_PTC_AFC_CONTROL = 0x254,          /**< @brief VSM AFC control message. */
    CMR_CANID_PTC_VOLTAGE_DIAGNOSTICS = 0x534,  /**< @brief PTC voltage diagnostics. */
    CMR_CANID_PTC_CURRENT_DIAGNOSTICS = 0x544,  /**< @brief PTC current diagnostics. */

    CMR_CANID_DIM_REQUEST = 0x235,              /**< @brief DIM state/gear request. */
    CMR_CANID_DIM_POWER_DIAGNOSTICS = 0x535,    /**< @brief DIM power diagnostics. */

    CMR_CANID_AFC0_FAN_STATUS = 0x236,          /**< @brief AFC 0 fan status. */
    CMR_CANID_AFC0_DRIVER_TEMPS = 0x536,        /**< @brief AFC 0 temperatures. */
    CMR_CANID_AFC0_POWER_DIAGNOSTICS = 0x546,   /**< @brief AFC 0 power diagnostics. */

    CMR_CANID_AFC1_FAN_STATUS = 0x237,          /**< @brief AFC 1 fan status. */
    CMR_CANID_AFC1_DRIVER_TEMPS = 0x537,        /**< @brief AFC 1 temperatures. */
    CMR_CANID_AFC1_POWER_DIAGNOSTICS = 0x547,   /**< @brief AFC 1 power diagnostics. */

    CMR_CANID_RMS_TEMPA = 0x000 + CMR_CANID_RMS_OFFSET,         /**< @brief RMS temp set A. */
    CMR_CANID_RMS_TEMPB = 0x001 + CMR_CANID_RMS_OFFSET,         /**< @brief RMS temp set B. */
    CMR_CANID_RMS_TEMPC = 0x002 + CMR_CANID_RMS_OFFSET,         /**< @brief RMS temp set C. */
    CMR_CANID_RMS_MOTOR_POS = 0x005 + CMR_CANID_RMS_OFFSET,     /**< @brief RMS motor position. */
    CMR_CANID_RMS_FAULTS = 0x00B + CMR_CANID_RMS_OFFSET,        /**< @brief RMS faults (pg 23). */
    CMR_CANID_RMS_TORQUE_DIAG = 0x00C + CMR_CANID_RMS_OFFSET,   /**< @brief RMS torque diagnostic data. */
    CMR_CANID_RMS_CURRENT_INFO = 0x006 + CMR_CANID_RMS_OFFSET,  /**< @brief RMS current info. */
    CMR_CANID_RMS_VOLTAGE_INFO = 0x007 + CMR_CANID_RMS_OFFSET,  /**< @brief RMS voltage info. */
    CMR_CANID_RMS_COMMAND = 0x020 + CMR_CANID_RMS_OFFSET,       /**< @brief RMS command. */
    CMR_CANID_RMS_PARAM_REQ = 0x021 + CMR_CANID_RMS_OFFSET,     /**< @brief RMS parameter request. */
    CMR_CANID_RMS_PARAM_RES = 0x022 + CMR_CANID_RMS_OFFSET      /**< @brief RMS parameter response. */
} cmr_canID_t;

#endif /* CMR_CAN_IDS_H */
