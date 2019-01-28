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
    CMR_CANID_HEARTBEAT_TOM = 0x108,    /**< @brief TOM heartbeat. */
    CMR_CANID_HEARTBEAT_COM = 0x109,    /**< @brief COM heartbeat. */

    CMR_CANID_FSM_DATA = 0x133,         /**< @brief FSM data. */
    CMR_CANID_FSM_PEDALS_ADC = 0x533,   /**< @brief FSM raw pedal positions. */
    CMR_CANID_FSM_SENSORS_ADC = 0x543,  /**< @brief FSM raw sensors. */
    /** @brief FSM power diagnostics. */
    CMR_CANID_FSM_POWER_DIAGNOSTICS = 0x553,

    CMR_CANID_PTC_COOLING_STATUS = 0x234,   /**< @brief PTC cooling status. */
    /** @brief PTC voltage diagnostics. */
    CMR_CANID_PTC_VOLTAGE_DIAGNOSTICS = 0x534,
    /** @brief PTC current diagnostics. */
    CMR_CANID_PTC_CURRENT_DIAGNOSTICS = 0x544,

    CMR_CANID_DIM_REQUEST = 0x235,  /**< @brief DIM state/gear request. */
    /** @brief DIM power diagnostics. */
    CMR_CANID_DIM_POWER_DIAGNOSTICS = 0x535,

    CMR_CANID_AFC0_FAN_STATUS = 0x236,      /**< @brief AFC #0 fan status. */
    CMR_CANID_AFC0_DRIVER_TEMPS = 0x536,    /**< @brief AFC #0 temperatures. */
    /** @brief AFC #0 power diagnostics. */
    CMR_CANID_AFC0_POWER_DIAGNOSTICS = 0x546,

    CMR_CANID_AFC1_FAN_STATUS = 0x237,      /**< @brief AFC #1 fan status. */
    CMR_CANID_AFC1_DRIVER_TEMPS = 0x537,    /**< @brief AFC #1 temperatures. */
    /** @brief AFC #1 power diagnostics. */
    CMR_CANID_AFC1_POWER_DIAGNOSTICS = 0x547
} cmr_canID_t;

#endif /* CMR_CAN_IDS_H */

