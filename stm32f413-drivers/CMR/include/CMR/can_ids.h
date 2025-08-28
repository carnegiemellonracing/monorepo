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
#define NUM_CONFIG_PACKETS 4 

/** @brief CAN IDs. */
typedef enum {
    CMR_CANID_HEARTBEAT_VSM = 0x100,    /**< @brief VSM heartbeat. */
    CMR_CANID_HEARTBEAT_HVC = 0x101,    /**< @brief HVC heartbeat. */
    CMR_CANID_HEARTBEAT_CDC = 0x102,    /**< @brief CDC heartbeat. */
    CMR_CANID_HEARTBEAT_FSM = 0x103,    /**< @brief FSM heartbeat. */
    CMR_CANID_HEARTBEAT_DIM = 0x104,    /**< @brief DIM heartbeat. */
    CMR_CANID_HEARTBEAT_PTC = 0x105,    /**< @brief PTC heatbeart. */
    CMR_CANID_HEARTBEAT_HVI = 0x106,    /**< @brief HVI heatbeart. */
    CMR_CANID_HEARTBEAT_LV_BMS = 0x107, /**< @brief LV-BMS heatbeart. */
    CMR_CANID_HEARTBEAT_MEMORATOR = 0x109,      /**< @brief Memorator heartbeat.*/

    CMR_CANID_VSM_STATUS = 0x110,               /**< @brief VSM status. */
    CMR_CANID_VSM_SENSORS = 0x200,              /**< @brief VSM sensor data. */
    CMR_CANID_VSM_LATCHED_STATUS = 0x510,       /**< @brief VSM latched status. */
    CMR_CANID_VSM_POWER_DIAGNOSTICS = 0x530,    /**< @brief VSM power diagnostics. */

    CMR_CANID_HVC_COMMAND = 0x130,              /**< @brief HVC command, sent by VSM. */
    CMR_CANID_HVC_PACK_VOLTAGE = 0x301,         /**< @brief HVC pack voltage. */
    CMR_CANID_HVC_MINMAX_CELL_VOLTAGE = 0x310,  /**< @brief HVC pack min and max cell voltages*/
    CMR_CANID_HVC_MINMAX_CELL_TEMPS = 0x311,    /**< @brief HVC pack min and max cell temps. */
    CMR_CANID_HVC_PACK_CURRENT = 0x302,         /**< @brief HVC pack current. */

    CMR_CANID_CDC_WHEEL_SPEEDS = 0x132,         /**< @brief CDC (19e) wheel speeds. */
    CMR_CANID_CDC_SOLENOID_PTC = 0x142,         /**< @brief CDC (19e) brake solenoid command. */
    CMR_CANID_CDC_MOTOR_DATA = 0x152,           /**< @brief CDC (19e) motor data. */
    CMR_CANID_CDC_MOTOR_PHASES = 0x172,         /**< @brief CDC (19e) motor phase currents. */
    CMR_CANID_CDC_IMU_ACCEL = 0x162,            /**< @brief CDC (19e) IMU accelerations. */
    CMR_CANID_CDC_MOTOR_FAULTS = 0x502,         /**< @brief CDC (19e) motor faults. */
    CMR_CANID_CDC_MOTOR_TEMPS = 0x512,          /**< @brief CDC (19e) motor temperatures. */

    CMR_CANID_CDC_WHEEL_SPEED_FEEDBACK = 0x232,     /**< @brief CDC (20e) actual wheel speeds. */
    CMR_CANID_CDC_WHEEL_SPEED_SETPOINT = 0x252,     /**< @brief CDC (20e) setpoint wheel speeds. */
    CMR_CANID_CDC_WHEEL_TORQUE_FEEDBACK = 0x262,    /**< @brief CDC (20e) actual wheel torques. */
    CMR_CANID_CDC_WHEEL_TORQUE_SETPOINT = 0x272,    /**< @brief CDC (20e) setpoint wheel torques. */
    CMR_CANID_CDC_POSE_POSITION = 0x282,            /**< @brief CDC (20e) lat/lon real car position. */
    CMR_CANID_CDC_POSE_ORIENTATION = 0x292,         /**< @brief CDC (20e) roll/pitch/yaw real car position. */
    CMR_CANID_CDC_POSE_VELOCITY = 0x2A2,            /**< @brief CDC (20e) real car velocity. */
    CMR_CANID_CDC_POSE_ACCELERATION = 0x2B2,        /**< @brief CDC (20e) real car acceleration. */

    CMR_CANID_CDC_COG_VELOCITY = 0x2C2,        /**< @brief CDC (20e) real car acceleration. */
    CMR_CANID_CDC_FRONT_VELOCITY = 0x2D2,        /**< @brief CDC (20e) real car acceleration. */
    CMR_CANID_CDC_REAR_VELOCITY = 0x2E2,        /**< @brief CDC (20e) real car acceleration. */

    CMR_CANID_CDC_POWER_SENSE = 0x305,
    CMR_CANID_CDC_RTC_DATA_OUT = 0x6A2,             /**< @brief CDC RTC data. */
    CMR_CANID_CDC_RTC_DATA_IN = 0x6B2,              /**< @brief CDC RTC data. */
    CMR_CANID_CDC_ODOMETER = 0x6C2,                 /**< @brief CDC Odometer data. */
    CMR_CANID_CDC_CONTROLS_STATUS = 0x6D2,          /**< @brief CDC controls status data. */
    CMR_CANID_CDC_POWER_UPDATE = 0x6E2,             /**< @brief DAQ Live to CDC - changing power limit. */
    CMR_CANID_CDC_COULOMB_COUNTING = 0x6E3,
    CMR_CANID_CDC_POWER_LOG = 0x6E4,             /**< @brief DAQ Live to CDC - changing power limit. */

    CMR_CANID_FSM_DATA = 0x133,                 /**< @brief FSM data. */
    CMR_CANID_CELL_BALANCE_ENABLE = 0x134,
    CMR_CANID_FSM_SWANGLE = 0x135,
    CMR_CANID_FSM_PEDALS_ADC = 0x533,           /**< @brief FSM raw pedal positions. */
    CMR_CANID_FSM_SENSORS_ADC = 0x543,          /**< @brief FSM raw sensors. */
    CMR_CANID_FSM_POWER_DIAGNOSTICS = 0x553,    /**< @brief FSM power diagnostics. */
    CMR_CANID_SS_STATUS = 0x554,               /**< @brief Safety Circuit status. */

    CMR_CANID_PTC_LOOP_TEMPS_A = 0x224,        /**< @brief (fan board) cooling loop temps. */
    CMR_CANID_PTC_LOOP_TEMPS_B = 0x234,        /**< @brief (fan board) cooling loop temps. */
    CMR_CANID_PTC_LOOP_TEMPS_C = 0x244,        /**< @brief (fan board) cooling loop temps. */
    CMR_CANID_PTC_FANS_PUMPS_STATUS = 0x314,   /**< @brief (fan board) fans status */
    CMR_CANID_PTC_POWER_DIAGNOSTICS = 0x534,   /**< @brief (fan board) power diagnostics. */

    CMR_CANID_DIM_REQUEST = 0x235,              /**< @brief DIM state/gear request. */
    CMR_CANID_DIM_POWER_DIAGNOSTICS = 0x535,    /**< @brief DIM power diagnostics. */
    CMR_CANID_DIM_TEXT_WRITE = 0x525,           /**< @brief DIM write command for sending text. */
    CMR_CANID_DIM_ACTIONS = 0x515,              /**< @brief DIM action buttons pressed status and regen percentage. */
    CMR_CANID_DIM_ACKNOWLEDGE = 0x545,              /**< @brief DIM action buttons pressed status and regen percentage. */

    /** @Note: The following CAN IDs are used for the dim-cdc driver configuration system.
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
 = 0x546,
     * If you wish to modify these, you must maintain the invariant that the number of config
     * packets is correct and the config packets are in the correct order with dim config packets
     * first and then cdc config packets in ascending order. This is imperative to maintaining
     * code modularity :)
    */
    CMR_CANID_DIM_CONFIG0_DRV0 = 0x600,         /**< @brief DIM config request */
    CMR_CANID_DIM_CONFIG1_DRV0 = 0x601,
    CMR_CANID_DIM_CONFIG2_DRV0 = 0x602,
    CMR_CANID_DIM_CONFIG3_DRV0 = 0x603,
    CMR_CANID_CDC_CONFIG0_DRV0 = 0x604,
    CMR_CANID_CDC_CONFIG1_DRV0 = 0x605,
    CMR_CANID_CDC_CONFIG2_DRV0 = 0x606,
    CMR_CANID_CDC_CONFIG3_DRV0 = 0x607,
    CMR_CANID_DIM_CONFIG0_DRV1 = 0x608,
    CMR_CANID_DIM_CONFIG1_DRV1 = 0x609,
    CMR_CANID_DIM_CONFIG2_DRV1 = 0x60a,
    CMR_CANID_DIM_CONFIG3_DRV1 = 0x60b,
    CMR_CANID_CDC_CONFIG0_DRV1 = 0x60c,
    CMR_CANID_CDC_CONFIG1_DRV1 = 0x60d,
    CMR_CANID_CDC_CONFIG2_DRV1 = 0x60e,
    CMR_CANID_CDC_CONFIG3_DRV1 = 0x60f,
    CMR_CANID_DIM_CONFIG0_DRV2 = 0x610,
    CMR_CANID_DIM_CONFIG1_DRV2 = 0x611,
    CMR_CANID_DIM_CONFIG2_DRV2 = 0x612,
    CMR_CANID_DIM_CONFIG3_DRV2 = 0x613,
    CMR_CANID_CDC_CONFIG0_DRV2 = 0x614,
    CMR_CANID_CDC_CONFIG1_DRV2 = 0x615,
    CMR_CANID_CDC_CONFIG2_DRV2 = 0x616,
    CMR_CANID_CDC_CONFIG3_DRV2 = 0x617,
    CMR_CANID_DIM_CONFIG0_DRV3 = 0x618,
    CMR_CANID_DIM_CONFIG1_DRV3 = 0x619,
    CMR_CANID_DIM_CONFIG2_DRV3 = 0x61a,
    CMR_CANID_DIM_CONFIG3_DRV3 = 0x61b,
    CMR_CANID_CDC_CONFIG0_DRV3 = 0x61c,
    CMR_CANID_CDC_CONFIG1_DRV3 = 0x61d,
    CMR_CANID_CDC_CONFIG2_DRV3 = 0x61e,
    CMR_CANID_CDC_CONFIG3_DRV3 = 0x61f,


    CMR_CANID_AFC0_FAN_STATUS = 0x236,          /**< @brief AFC 0 fan status. */
    CMR_CANID_AFC0_DRIVER_TEMPS = 0x536,        /**< @brief AFC 0 temperatures. */
    CMR_CANID_AFC0_POWER_DIAGNOSTICS = 0x546,   /**< @brief AFC 0 power diagnostics. */

    CMR_CANID_AFC1_FAN_STATUS = 0x237,          /**< @brief AFC 1 fan status. */
    CMR_CANID_AFC1_DRIVER_TEMPS = 0x537,        /**< @brief AFC 1 temperatures. */
    CMR_CANID_AFC1_POWER_DIAGNOSTICS = 0x547,   /**< @brief AFC 1 power diagnostics. */

    CMR_CANID_DRS_STATE = 0x52C,                /**< @brief DRS state values.*/
	CMR_CANID_SF_STATE = 0x52D,				/**< @brief Safety Filter state. */
    CMR_CANID_MOTORPOWER_STATE = 0x52E,				/**< @brief Motor Power state. */

    // FL
    CMR_CANID_AMK_FL_ACT_1 = 0x288,              /**< @brief AMK Inverter 4 actual values 1.*/
    CMR_CANID_AMK_FL_ACT_2 = 0x28A,              /**< @brief AMK Inverter 4 actual values 2.*/
    CMR_CANID_AMK_FL_SETPOINTS = 0x189,

    CMR_CANID_AMK_FR_ACT_1 = 0x284,              /**< @brief AMK Inverter 2 actual values 1.*/
    CMR_CANID_AMK_FR_ACT_2 = 0x286,              /**< @brief AMK Inverter 2 actual values 2.*/
    CMR_CANID_AMK_FR_SETPOINTS = 0x185,          /**< @brief AMK Inverter 2 setpoints.*/
    
    // RL
    CMR_CANID_AMK_RL_ACT_1 = 0x287,              /**< @brief AMK Inverter 3 actual values 1.*/
    CMR_CANID_AMK_RL_ACT_2 = 0x289,              /**< @brief AMK Inverter 3 actual values 2.*/
    CMR_CANID_AMK_RL_SETPOINTS = 0x188,          /**< @brief AMK Inverter 3 setpoints.*/
    
    // RR
    CMR_CANID_AMK_RR_ACT_1 = 0x283,              /**< @brief AMK Inverter 1 actual values 1.*/
    CMR_CANID_AMK_RR_ACT_2 = 0x285,              /**< @brief AMK Inverter 1 actual values 2.*/
    CMR_CANID_AMK_RR_SETPOINTS = 0x184,          /**< @brief AMK Inverter 1 setpoints.*/

    CMR_CANID_RMS_TEMPA = 0x3a0,
    CMR_CANID_RMS_TEMPB = 0x3a1,
    CMR_CANID_RMS_TEMPC = 0x3a2,
    CMR_CANID_RMS_MOTOR_POS = 0x3a5,
    CMR_CANID_RMS_FAULTS = 0x3ab,
    CMR_CANID_RMS_TORQUE_DIAG = 0x3ac,
    CMR_CANID_RMS_CURRENT_INFO = 0x3a6,
    CMR_CANID_RMS_VOLTAGE_INFO = 0x3a7,
    CMR_CANID_RMS_COMMAND = 0x3c0,
    CMR_CANID_RMS_PARAM_REQ = 0x3c1,
    CMR_CANID_RMS_PARAM_RES = 0x3c2,


    // BMS CAN Structs
    CMR_CANID_HVC_MIN_MAX_CELL_VOLTAGE = 0x310,                  /**< @brief HVC Max/Min Cell Voltage. */
    CMR_CANID_HVC_MIN_MAX_CELL_TEMPERATURE = 0x311,              /**< @brief HVC Max/Min Cell Temperature. */
    CMR_CANID_HVC_LOW_VOLTAGE = 0x303,                           /**< @brief HVC Low Voltage. */
    CMR_CANID_HVC_BMB_STATUS_ERRORS = 0x304,
    CMR_CANID_HVC_BRUSA_MSG = 0x528,                             /**< @brief HVC Brusa message. */

    CMR_CANID_HVC_BMB_0_STATUS_VOLTAGE = 0x380,                    /**< @brief HVC BMB Voltage: Message ID is BMB number << 1*/
    CMR_CANID_HVC_BMB_1_STATUS_VOLTAGE = 0x382,
    CMR_CANID_HVC_BMB_2_STATUS_VOLTAGE = 0x384,
    CMR_CANID_HVC_BMB_3_STATUS_VOLTAGE = 0x386,
    CMR_CANID_HVC_BMB_4_STATUS_VOLTAGE = 0x388,
    CMR_CANID_HVC_BMB_5_STATUS_VOLTAGE = 0x38A,
    CMR_CANID_HVC_BMB_6_STATUS_VOLTAGE = 0x38C,
    CMR_CANID_HVC_BMB_7_STATUS_VOLTAGE = 0x38E,
    CMR_CANID_HVC_BMB_8_STATUS_VOLTAGE = 0x390,
    CMR_CANID_HVC_BMB_9_STATUS_VOLTAGE = 0x392,
    CMR_CANID_HVC_BMB_10_STATUS_VOLTAGE = 0x394,
    CMR_CANID_HVC_BMB_11_STATUS_VOLTAGE = 0x396,
    CMR_CANID_HVC_BMB_12_STATUS_VOLTAGE = 0x398,
    CMR_CANID_HVC_BMB_13_STATUS_VOLTAGE = 0x39A,
    CMR_CANID_HVC_BMB_14_STATUS_VOLTAGE = 0x39C,
    CMR_CANID_HVC_BMB_15_STATUS_VOLTAGE = 0x39E,
    CMR_CANID_HVC_BMB_0_STATUS_TEMP = 0x381,                       /**< @brief HVC BMB Temperature: Message ID is BMB number << 1 + 1*/
    CMR_CANID_HVC_BMB_1_STATUS_TEMP = 0x383,
    CMR_CANID_HVC_BMB_2_STATUS_TEMP = 0x385,
    CMR_CANID_HVC_BMB_3_STATUS_TEMP = 0x387,
    CMR_CANID_HVC_BMB_4_STATUS_TEMP = 0x389,
    CMR_CANID_HVC_BMB_5_STATUS_TEMP = 0x38B,
    CMR_CANID_HVC_BMB_6_STATUS_TEMP = 0x38D,
    CMR_CANID_HVC_BMB_7_STATUS_TEMP = 0x38F,
    CMR_CANID_HVC_BMB_8_STATUS_TEMP = 0x391,
    CMR_CANID_HVC_BMB_9_STATUS_TEMP = 0x393,
    CMR_CANID_HVC_BMB_10_STATUS_TEMP = 0x395,
    CMR_CANID_HVC_BMB_11_STATUS_TEMP = 0x397,
    CMR_CANID_HVC_BMB_12_STATUS_TEMP = 0x399,
    CMR_CANID_HVC_BMB_13_STATUS_TEMP = 0x39B,
    CMR_CANID_HVC_BMB_14_STATUS_TEMP = 0x39D,
    CMR_CANID_HVC_BMB_15_STATUS_TEMP = 0x39F,

    CMR_CANID_SBG_STATUS_1 = 0x700,             /**< @brief SBG_ECAN_LOG_STATUS_01 */
    CMR_CANID_SBG_STATUS_2 = 0x701,             /**< @brief SBG_ECAN_LOG_STATUS_02 */
    CMR_CANID_SBG_STATUS_3 = 0x702,             /**< @brief SBG_ECAN_LOG_STATUS_03 */
    CMR_CANID_SBG_EKF_ORIENT = 0x722,           /**< @brief SBG_ECAN_LOG_EKF_EULER */
    CMR_CANID_SBG_EKF_POS = 0x724,              /**< @brief SBG_ECAN_LOG_EKF_POS */
    CMR_CANID_SBG_EKF_VEL = 0x727,              /**< @brief SBG_ECAN_LOG_EKF_VEL */
    CMR_CANID_SBG_IMU_ACCEL = 0x711,            /**< @brief SBG_ECAN_LOG_IMU_ACCEL */
    CMR_CANID_SBG_IMU_GYRO = 0x712,             /**< @brief SBG_ECAN_LOG_IMU_GYRO */
    CMR_CANID_SBG_BODY_VEL = 0x729,             /**< @brief SBG_ECAN_MSG_EKF_VEL_BODY */
    CMR_CANID_SBG_AUTOMOTIVE = 0x72A,           /**< @brief SBG_ECAN_MSG_AUTO_TRACK_SLIP_CURV */

    CMR_CANID_MOVELLA_STATUS = 0x772,
    CMR_CANID_MOVELLA_QUATERNION = 0x774,
    CMR_CANID_MOVELLA_IMU_GYRO = 0x776,
    CMR_CANID_MOVELLA_IMU_ACCEL = 0x77E,
    CMR_CANID_MOVELLA_VELOCITY = 0x77D,

    CMR_CANID_EMD_STATUS = 0x400,               /**< @brief EMD status. */
    CMR_CANID_EMD_MEASUREMENT_RETX = 0x401,     /**< @brief EMD measurement for HV voltage/current. */
    CMR_CANID_EMD_MEASUREMENT = 0x402,          /**< @brief EMD measurement for HV voltage/current. */

    CMR_IZZIE_LOADCELL = 0x7F0,                 /**< @brief IZZIE Amp load data. */
    CMR_CANID_CONTROLS_DEBUG_GLOBAl = 0x7E0,    /**< @brief control algo testing data. */
    CMR_CANID_CONTROLS_DEBUG_FR = 0x7E1,        /**< @brief control algo testing data. */
    CMR_CANID_CONTROLS_DEBUG_FL = 0x7E2,        /**< @brief control algo testing data. */
    CMR_CANID_CONTROLS_DEBUG_RR = 0x7E3,        /**< @brief control algo testing data. */
    CMR_CANID_CONTROLS_DEBUG_RL = 0x7E4,        /**< @brief control algo testing data. */
    CMR_CANID_CONTROLS_PID_IO = 0x7E5,        /**< @brief control algo testing data. */

    CMR_CANID_CONTROLS_SOLVER_INPUTS = 0x7E6,
    CMR_CANID_CONTROLS_SOLVER_OUTPUTS = 0x7F0,
    CMR_CANID_CONTROLS_SOLVER_SETTINGS = 0x7EE,
    CMR_CANID_CONTROLS_SOLVER_AUX = 0x7FF,

    CMR_CANID_LOADCELL_FL = 0x7D3,
    CMR_CANID_LOADCELL_FR = 0x7D0,
    CMR_CANID_LOADCELL_RL = 0x7D1,
    CMR_CANID_LOADCELL_RR = 0x7D2,

    CMR_CANID_FRONT_SLIP_RATIOS = 0x7E8,
    CMR_CANID_REAR_SLIP_RATIOS = 0x7E9,
    CMR_CANID_FRONT_WHL_SETPOINTS = 0x7EA,
    CMR_CANID_REAR_WHL_SETPOINTS = 0x7EB,
    CMR_CANID_FRONT_WHL_VELS = 0x7EC,
    CMR_CANID_REAR_WHL_VELS = 0x7ED,

    CMR_CANID_TEST_ID=0x777,

	CMR_CANID_DRS_CONTROLS = 0x29C, 				/**< @brief DRS Motor Controls. */

    CMR_CANID_DAQ_0_LOADCELL = 0x650,           /**< @brief Load cell data for DAQ Board 0. */
    CMR_CANID_DAQ_0_THERMISTOR = 0x651,
    CMR_CANID_DAQ_0_DEBUG = 0x652,
    CMR_CANID_DAQ_1_LOADCELL = 0x653,
    CMR_CANID_DAQ_1_THERMISTOR = 0x654,
    CMR_CANID_DAQ_1_DEBUG = 0x655,
    CMR_CANID_DAQ_2_LOADCELL = 0x656,
    CMR_CANID_DAQ_2_THERMISTOR = 0x657,
    CMR_CANID_DAQ_2_DEBUG = 0x658,
    CMR_CANID_DAQ_3_LOADCELL = 0x659,
    CMR_CANID_DAQ_3_THERMISTOR = 0x65a,
    CMR_CANID_DAQ_3_DEBUG = 0x65b,

    CMR_CANID_VSM_GIT = 0x7F1,
	CMR_CANID_HVC_GIT = 0x7f2,
	CMR_CANID_PTC_GIT = 0x7f3,
	CMR_CANID_CDC_GIT = 0x7f4,
	CMR_CANID_DIM_GIT = 0x7f5,
	CMR_CANID_RAM_GIT = 0x7f6,

    CAN_ID_LV_BMS_CELL_VOLTAGE_1_3 = 0x7F7,
    CAN_ID_LV_BMS_CELL_VOLTAGE_4_6 = 0x7f8,
    CAN_ID_LV_BMS_CELL_TEMP_1_4 = 0x7f9,
    CAN_ID_LV_BMS_CELL_TEMP_5_8 = 0x7fa,
    CAN_ID_LV_BMS_CELL_OVERVOLTAGE = 0x7fb,
    CAN_ID_LV_BMS_CELL_OVERTEMP = 0x7fc,
    CAN_ID_LV_BMS_BUS_VOLTAGE = 0x7fd,
    CAN_ID_LV_BMS_CURRENT = 0x7fe,
} cmr_canID_t;

#endif /* CMR_CAN_IDS_H */