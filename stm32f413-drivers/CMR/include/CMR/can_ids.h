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
     * If you wish to modify these, you must maintain the invariant that the number of config
     * packets is correct and the config packets are in the correct order with dim config packets
     * first and then cdc config packets in ascending order. This is imperative to maintaining
     * code modularity :)
    */
    CMR_CANID_DIM_CONFIG0_DRV0 = 0x600,         /**< @brief DIM config request */
    CMR_CANID_DIM_CONFIG1_DRV0,                 /**< @brief DIM config request */
    CMR_CANID_DIM_CONFIG2_DRV0,                 /**< @brief DIM config request */
    CMR_CANID_DIM_CONFIG3_DRV0,                 /**< @brief DIM config request */
    CMR_CANID_CDC_CONFIG0_DRV0,                 /**< @brief CDC config request */
    CMR_CANID_CDC_CONFIG1_DRV0,                 /**< @brief CDC config request */
    CMR_CANID_CDC_CONFIG2_DRV0,                 /**< @brief CDC config request */
    CMR_CANID_CDC_CONFIG3_DRV0,                 /**< @brief CDC config request */
    CMR_CANID_DIM_CONFIG0_DRV1,                 /**< @brief DIM config request */
    CMR_CANID_DIM_CONFIG1_DRV1,                 /**< @brief DIM config request */
    CMR_CANID_DIM_CONFIG2_DRV1,                 /**< @brief DIM config request */
    CMR_CANID_DIM_CONFIG3_DRV1,                 /**< @brief DIM config request */
    CMR_CANID_CDC_CONFIG0_DRV1,                 /**< @brief CDC config request */
    CMR_CANID_CDC_CONFIG1_DRV1,                 /**< @brief CDC config request */
    CMR_CANID_CDC_CONFIG2_DRV1,                 /**< @brief CDC config request */
    CMR_CANID_CDC_CONFIG3_DRV1,                 /**< @brief CDC config request */
    CMR_CANID_DIM_CONFIG0_DRV2,                 /**< @brief DIM config request */
    CMR_CANID_DIM_CONFIG1_DRV2,                 /**< @brief DIM config request */
    CMR_CANID_DIM_CONFIG2_DRV2,                 /**< @brief DIM config request */
    CMR_CANID_DIM_CONFIG3_DRV2,                 /**< @brief DIM config request */
    CMR_CANID_CDC_CONFIG0_DRV2,                 /**< @brief CDC config request */
    CMR_CANID_CDC_CONFIG1_DRV2,                 /**< @brief CDC config request */
    CMR_CANID_CDC_CONFIG2_DRV2,                 /**< @brief CDC config request */
    CMR_CANID_CDC_CONFIG3_DRV2,                 /**< @brief CDC config request */
    CMR_CANID_DIM_CONFIG0_DRV3,                 /**< @brief DIM config request */
    CMR_CANID_DIM_CONFIG1_DRV3,                 /**< @brief DIM config request */
    CMR_CANID_DIM_CONFIG2_DRV3,                 /**< @brief DIM config request */
    CMR_CANID_DIM_CONFIG3_DRV3,                 /**< @brief DIM config request */
    CMR_CANID_CDC_CONFIG0_DRV3,                 /**< @brief CDC config request */
    CMR_CANID_CDC_CONFIG1_DRV3,                 /**< @brief CDC config request */
    CMR_CANID_CDC_CONFIG2_DRV3,                 /**< @brief CDC config request */
    CMR_CANID_CDC_CONFIG3_DRV3,                 /**< @brief CDC config request */

    CMR_CANID_AFC0_FAN_STATUS = 0x236,          /**< @brief AFC 0 fan status. */
    CMR_CANID_AFC0_DRIVER_TEMPS = 0x536,        /**< @brief AFC 0 temperatures. */
    CMR_CANID_AFC0_POWER_DIAGNOSTICS = 0x546,   /**< @brief AFC 0 power diagnostics. */

    CMR_CANID_AFC1_FAN_STATUS = 0x237,          /**< @brief AFC 1 fan status. */
    CMR_CANID_AFC1_DRIVER_TEMPS = 0x537,        /**< @brief AFC 1 temperatures. */
    CMR_CANID_AFC1_POWER_DIAGNOSTICS = 0x547,   /**< @brief AFC 1 power diagnostics. */

    CMR_CANID_DRS_STATE = 0x52C,                /**< @brief DRS state values.*/
	CMR_CANID_SF_STATE = 0x52D,				    /**< @brief Safety Filter state. */
    CMR_CANID_MOTORPOWER_STATE = 0x52E,		    /**< @brief Motor Power state. */

    /** CAN IDs for the Front Left (FL) Motor Controller.
     *  Node ID Detailed to be 0x00. 
     */
    CMR_CANID_DTI_FL_CONTROL_STATUS = 0x1F0,    /**< @brief FL Control mode, Target Iq, motor position, isMotorStill */
    CMR_CANID_DTI_FL_ERPM = 0x200,              /**< @brief FL ERPM, Duty, Input Voltage */  
    CMR_CANID_DTI_FL_CURRENT = 0x210,           /**< @brief FL AC Current, DC Current */
    CMR_CANID_DTI_FL_TEMPFAULT = 0x220,         /**< @brief FL Controller Temp., Motor Temp., Fault code */
    CMR_CANID_DTI_FL_IDIQ = 0x230,              /**< @brief FL Id, Iq values */
    CMR_CANID_DTI_FL_IO_STATUS = 0x240,         /**< @brief FL Throttle signal, Brake signal, Digital I/Os, Drive enable, Limit status bits */
    CMR_CANID_DTI_FL_ACLIMS = 0x250,            /**< @brief FL Cfg max AC, avail max AC, cfg min AC, avail min AC */
    CMR_CANID_DTI_FL_DCLIMS = 0x260,            /**< @brief FL Cfg max DC, avail max DC, cfg min DC, avail min DC */
    CMR_CANID_DTI_FL_VELOCITY = 0x030,          /**< @brief FL Velocity (rpm). */
    CMR_CANID_DTI_FL_TORLIMPOS = 0x080,         /**< @brief FL Positive torque limit */
    CMR_CANID_DTI_FL_TORLIMNEG = 0x090,         /**< @brief FL Negative torque limit */
    CMR_CANID_DTI_FL_TORQUE = 0x010,            /**< @brief FL Torque */
    CMR_CANID_DTI_FL_DRIVE_EN = 0x0C0,          /**< @brief FL Drive Enable Message */

    /** CAN IDs for the Front Right Motor Controller.
     *  Node ID Detailed to be 0x01. 
     */
    CMR_CANID_DTI_FR_CONTROL_STATUS = 0x1F1,    /**< @brief FR Control mode, Target Iq, motor position, isMotorStill */
    CMR_CANID_DTI_FR_ERPM = 0x201,              /**< @brief FR ERPM, Duty, Input Voltage */  
    CMR_CANID_DTI_FR_CURRENT = 0x211,           /**< @brief FR AC Current, DC Current */
    CMR_CANID_DTI_FR_TEMPFAULT = 0x221,         /**< @brief FR Controller Temp., Motor Temp., Fault code */
    CMR_CANID_DTI_FR_IDIQ = 0x231,              /**< @brief FR Id, Iq values */
    CMR_CANID_DTI_FR_IO_STATUS = 0x241,         /**< @brief FR Throttle signal, Brake signal, Digital I/Os, Drive enable, Limit status bits */
    CMR_CANID_DTI_FR_ACLIMS = 0x251,            /**< @brief FR Cfg max AC, avail max AC, cfg min AC, avail min AC */
    CMR_CANID_DTI_FR_DCLIMS = 0x261,            /**< @brief FR Cfg max DC, avail max DC, cfg min DC, avail min DC */
    CMR_CANID_DTI_FR_VELOCITY = 0x031,          /**< @brief FR Velocity (rpm). */
    CMR_CANID_DTI_FR_TORLIMPOS = 0x081,         /**< @brief FR Positive torque limit */
    CMR_CANID_DTI_FR_TORLIMNEG = 0x091,         /**< @brief FR Negative torque limit */
    CMR_CANID_DTI_FR_TORQUE = 0x011,            /**< @brief FR Torque */
    CMR_CANID_DTI_FR_DRIVE_EN = 0x0C1,          /**< @brief FR Drive Enable Message */

    /** CAN IDs for the Rear Right Motor Controller.
     *  Node ID Detailed to be 0x02. 
     */
    CMR_CANID_DTI_RR_CONTROL_STATUS = 0x1F2,    /**< @brief RR Control mode, Target Iq, motor position, isMotorStill */
    CMR_CANID_DTI_RR_ERPM = 0x202,              /**< @brief RR ERPM, Duty, Input Voltage */  
    CMR_CANID_DTI_RR_CURRENT = 0x212,           /**< @brief RR AC Current, DC Current */
    CMR_CANID_DTI_RR_TEMPFAULT = 0x222,         /**< @brief RR Controller Temp., Motor Temp., Fault code */
    CMR_CANID_DTI_RR_IDIQ = 0x232,              /**< @brief RR Id, Iq values */
    CMR_CANID_DTI_RR_IO_STATUS = 0x242,         /**< @brief RR Throttle signal, Brake signal, Digital I/Os, Drive enable, Limit status bits */
    CMR_CANID_DTI_RR_ACLIMS = 0x252,            /**< @brief RR Cfg max AC, avail max AC, cfg min AC, avail min AC */
    CMR_CANID_DTI_RR_DCLIMS = 0x262,            /**< @brief RR Cfg max DC, avail max DC, cfg min DC, avail min DC */
    CMR_CANID_DTI_RR_VELOCITY = 0x032,          /**< @brief RR Velocity (rpm). */
    CMR_CANID_DTI_RR_TORLIMPOS = 0x082,         /**< @brief RR Positive torque limit */
    CMR_CANID_DTI_RR_TORLIMNEG = 0x092,         /**< @brief RR Negative torque limit */
    CMR_CANID_DTI_RR_TORQUE = 0x012,            /**< @brief RR Torque */
    CMR_CANID_DTI_RR_DRIVE_EN = 0x0C2,          /**< @brief RR Drive Enable Message */

    /** CAN IDs for the Rear Left Motor Controller.
     *  Node ID Detailed to be 0x03. 
     */
    CMR_CANID_DTI_RL_CONTROL_STATUS = 0x1F3,    /**< @brief RL Control mode, Target Iq, motor position, isMotorStill */
    CMR_CANID_DTI_RL_ERPM = 0x203,              /**< @brief RL ERPM, Duty, Input Voltage */  
    CMR_CANID_DTI_RL_CURRENT = 0x213,           /**< @brief RL AC Current, DC Current */
    CMR_CANID_DTI_RL_TEMPFAULT = 0x223,         /**< @brief RL Controller Temp., Motor Temp., Fault code */
    CMR_CANID_DTI_RL_IDIQ = 0x233,              /**< @brief RL Id, Iq values */
    CMR_CANID_DTI_RL_IO_STATUS = 0x243,         /**< @brief RL Throttle signal, Brake signal, Digital I/Os, Drive enable, Limit status bits */
    CMR_CANID_DTI_RL_ACLIMS = 0x253,            /**< @brief RL Cfg max AC, avail max AC, cfg min AC, avail min AC */
    CMR_CANID_DTI_RL_DCLIMS = 0x263,            /**< @brief RL Cfg max DC, avail max DC, cfg min DC, avail min DC */
    CMR_CANID_DTI_RL_VELOCITY = 0x033,          /**< @brief RL Velocity (rpm). */
    CMR_CANID_DTI_RL_TORLIMPOS = 0x083,         /**< @brief RL Positive torque limit */
    CMR_CANID_DTI_RL_TORLIMNEG = 0x093,         /**< @brief RL Negative torque limit */
    CMR_CANID_DTI_RL_TORQUE = 0x013,            /**< @brief RL Torque */
    CMR_CANID_DTI_RL_DRIVE_EN = 0x0C3,          /**< @brief RL Drive Enable Message */

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
    CMR_CANID_RMS_PARAM_RES = 0x022 + CMR_CANID_RMS_OFFSET,     /**< @brief RMS parameter response. */

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
    CMR_CANID_DAQ_0_THERMISTOR,                 /**< @brief Thermistor data for DAQ Board 0. */
    CMR_CANID_DAQ_0_DEBUG,                      /**< @brief Load cell amplifier debug data for DAQ Board 0. */
    CMR_CANID_DAQ_1_LOADCELL,                   /**< @brief Load cell data for DAQ Board 1. */
    CMR_CANID_DAQ_1_THERMISTOR,                 /**< @brief Thermistor data for DAQ Board 1. */
    CMR_CANID_DAQ_1_DEBUG,                      /**< @brief Load cell amplifier debug data for DAQ Board 1. */
    CMR_CANID_DAQ_2_LOADCELL,                   /**< @brief Load cell data for DAQ Board 2. */
    CMR_CANID_DAQ_2_THERMISTOR,                 /**< @brief Thermistor data for DAQ Board 2. */
    CMR_CANID_DAQ_2_DEBUG,                      /**< @brief Load cell amplifier debug data for DAQ Board 2. */
    CMR_CANID_DAQ_3_LOADCELL,                   /**< @brief Load cell data for DAQ Board 3. */
    CMR_CANID_DAQ_3_THERMISTOR,                 /**< @brief Thermistor data for DAQ Board 3. */
    CMR_CANID_DAQ_3_DEBUG,                      /**< @brief Load cell amplifier debug data for DAQ Board 3. */

    CMR_CANID_VSM_GIT = 0x7F1,
	CMR_CANID_HVC_GIT,
	CMR_CANID_PTC_GIT,
	CMR_CANID_CDC_GIT,
	CMR_CANID_DIM_GIT,
	CMR_CANID_RAM_GIT,

    CAN_ID_LV_BMS_CELL_VOLTAGE_1_3 = 0x7F7,
    CAN_ID_LV_BMS_CELL_VOLTAGE_4_6,
    CAN_ID_LV_BMS_CELL_TEMP_1_4,
    CAN_ID_LV_BMS_CELL_TEMP_5_8,
    CAN_ID_LV_BMS_CELL_OVERVOLTAGE,
    CAN_ID_LV_BMS_CELL_OVERTEMP,
    CAN_ID_LV_BMS_BUS_VOLTAGE,
    CAN_ID_LV_BMS_CURRENT,
} cmr_canID_t;

#endif /* CMR_CAN_IDS_H */