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
//#define CMR_CANID_RMS_OFFSET    0x3A0
#define NUM_CONFIG_PACKETS 4 
#define CONTROLLER_ID 120

/** @brief CAN IDs. */
typedef enum {
    CMR_CANID_HEARTBEAT_VSM = 0x100,    /**< @brief VSM heartbeat. */
    CMR_CANID_HEARTBEAT_HVC = 0x101,    /**< @brief HVC heartbeat. */
    CMR_CANID_HEARTBEAT_DCM = 0x102,    /**< @brief DCM heartbeat. */
    CMR_CANID_HEARTBEAT_DIM = 0x103,    /**< @brief DIM heartbeat. */
    CMR_CANID_HEARTBEAT_DIM = 0x104,    /**< @brief DIM heartbeat. */
    CMR_CANID_HEARTBEAT_DCM = 0x105,    /**< @brief DCM heatbeart. */
    CMR_CANID_HEARTBEAT_LV_BMS = 0x107, /**< @brief LV-BMS heatbeart. */
    CMR_CANID_HEARTBEAT_MEMORATOR = 0x109,      /**< @brief Memorator heartbeat.*/
    CMR_CANID_TSAB_STATUS = 0x10A,
    CMR_CANID_DSMS_STATUS = 0x194,
    CMR_CANID_HEARTBEAT_HV_BMS = 0x108, 

    CMR_CANID_VSM_STATUS = 0x110,               /**< @brief VSM status. */
    CMR_CANID_VSM_SENSORS = 0x200,              /**< @brief VSM sensor data. */
    CMR_CANID_VSM_LATCHED_STATUS = 0x510,       /**< @brief VSM latched status. */
    CMR_CANID_VSM_POWER_DIAGNOSTICS = 0x530,    /**< @brief VSM power diagnostics. */
    CMR_CANID_VSM_FIRST_ERROR = 0x10b,          /**< @brief VSM first error state */

    CMR_CANID_HVC_COMMAND = 0x130,              /**< @brief HVC command, sent by VSM. */
    CMR_CANID_HVBMS_PACK_VOLTAGE = 0x301,         /**< @brief HVC pack voltage. */
    CMR_CANID_HVC_MINMAX_CELL_VOLTAGE = 0x310,  /**< @brief HVC pack min and max cell voltages*/
    CMR_CANID_HVC_MINMAX_CELL_TEMPS = 0x311,    /**< @brief HVC pack min and max cell temps. */
    CMR_CANID_HVC_PACK_CURRENT = 0x302,         /**< @brief HVC pack current. */

    CMR_CANID_DCM_WHEEL_SPEED_FEEDBACK = 0x232,     /**< @brief DCM (20e) actual wheel speeds. */
    CMR_CANID_DCM_WHEEL_SPEED_SETPOINT = 0x252,     /**< @brief DCM (20e) setpoint wheel speeds. */
    CMR_CANID_DCM_WHEEL_TORQUE_FEEDBACK = 0x262,    /**< @brief DCM (20e) actual wheel torques. */
    CMR_CANID_DCM_WHEEL_TORQUE_SETPOINT = 0x272,    /**< @brief DCM (20e) setpoint wheel torques. */
    CMR_CANID_DCM_POSE_POSITION = 0x282,            /**< @brief DCM (20e) lat/lon real car position. */
    CMR_CANID_DCM_POSE_ORIENTATION = 0x292,         /**< @brief DCM (20e) roll/pitch/yaw real car position. */
    CMR_CANID_DCM_POSE_VELOCITY = 0x2A2,            /**< @brief DCM (20e) real car velocity. */
    CMR_CANID_DCM_POSE_ACCELERATION = 0x2B2,        /**< @brief DCM (20e) real car acceleration. */

    CMR_CANID_DCM_COG_VELOCITY = 0x2C2,        /**< @brief DCM (20e) real car acceleration. */
    CMR_CANID_DCM_FRONT_VELOCITY = 0x2D2,        /**< @brief DCM (20e) real car acceleration. */
    CMR_CANID_DCM_REAR_VELOCITY = 0x2E2,        /**< @brief DCM (20e) real car acceleration. */

    CMR_CANID_DCM_POWER_SENSE = 0x305,
    CMR_CANID_DCM_RTC_DATA_OUT = 0x6A2,             /**< @brief DCM RTC data. */
    CMR_CANID_DCM_RTC_DATA_IN = 0x6B2,              /**< @brief DCM RTC data. */
    CMR_CANID_DCM_ODOMETER = 0x6C0,                 /**< @brief DCM Odometer data. */
    CMR_CANID_DCM_CONTROLS_STATUS = 0x6c1,          /**< @brief DCM controls status data. */
    CMR_CANID_DCM_POWER_UPDATE = 0x6c2,             /**< @brief DAQ Live to DCM - changing power limit. */
    CMR_CANID_DCM_COULOMB_COUNTING = 0x6c3,
    CMR_CANID_DCM_POWER_LOG = 0x6c4,             /**< @brief DAQ Live to DCM - changing power limit. */

    CMR_CANID_DIM_DATA = 0x133,                 /**< @brief DIM data. */
    CMR_CANID_CELL_BALANCE_ENABLE = 0x134,
    CMR_CANID_DIM_SWANGLE = 0x135,
    CMR_CANID_DIM_PEDALS_ADC = 0x533,           /**< @brief DIM raw pedal positions. */
    CMR_CANID_DIM_SENSORS_ADC = 0x543,          /**< @brief DIM raw sensors. */
    CMR_CANID_DIM_POWER_DIAGNOSTICS = 0x553,    /**< @brief DIM power diagnostics. */
    CMR_CANID_SS_STATUS = 0x554,               /**< @brief Safety Circuit status. */

    CMR_CANID_DCM_LOOP_TEMPS_A = 0x224,        /**< @brief (fan board) cooling loop temps. */
    CMR_CANID_DCM_LOOP_TEMPS_B = 0x234,        /**< @brief (fan board) cooling loop temps. */
    CMR_CANID_DCM_LOOP_TEMPS_C = 0x244,        /**< @brief (fan board) cooling loop temps. */
    CMR_CANID_DCM_FANS_PUMPS_STATUS = 0x314,   /**< @brief (fan board) fans status */
    CMR_CANID_DCM_POWER_DIAGNOSTICS = 0x534,   /**< @brief (fan board) power diagnostics. */

    CMR_CANID_DIM_REQUEST = 0x235,              /**< @brief DIM state/gear request. */
    CMR_CANID_DIM_POWER_DIAGNOSTICS = 0x535,    /**< @brief DIM power diagnostics. */
    CMR_CANID_DIM_TEXT_WRITE = 0x525,           /**< @brief DIM write command for sending text. */
    CMR_CANID_DIM_ACTIONS = 0x515,              /**< @brief DIM action buttons pressed status and regen percentage. */
    CMR_CANID_DIM_ACKNOWLEDGE = 0x545,              /**< @brief DIM action buttons pressed status and regen percentage. */

    /** @Note: The following CAN IDs are used for the dim-DCM driver configuration system.
     * If you wish to modify these, you must maintain the invariant that the number of config
     * packets is correct and the config packets are in the correct order with dim config packets
     * first and then DCM config packets in ascending order. This is imperative to maintaining
     * code modularity :)
    */
    CMR_CANID_DIM_CONFIG0_DRV0 = 0x600,         /**< @brief DIM config request */
    CMR_CANID_DIM_CONFIG1_DRV0 = 0x601,
    CMR_CANID_DIM_CONFIG2_DRV0 = 0x602,
    CMR_CANID_DIM_CONFIG3_DRV0 = 0x603,
    CMR_CANID_DCM_CONFIG0_DRV0 = 0x604,
    CMR_CANID_DCM_CONFIG1_DRV0 = 0x605,
    CMR_CANID_DCM_CONFIG2_DRV0 = 0x606,
    CMR_CANID_DCM_CONFIG3_DRV0 = 0x607,
    CMR_CANID_DIM_CONFIG0_DRV1 = 0x608,
    CMR_CANID_DIM_CONFIG1_DRV1 = 0x609,
    CMR_CANID_DIM_CONFIG2_DRV1 = 0x60a,
    CMR_CANID_DIM_CONFIG3_DRV1 = 0x60b,
    CMR_CANID_DCM_CONFIG0_DRV1 = 0x60c,
    CMR_CANID_DCM_CONFIG1_DRV1 = 0x60d,
    CMR_CANID_DCM_CONFIG2_DRV1 = 0x60e,
    CMR_CANID_DCM_CONFIG3_DRV1 = 0x60f,
    CMR_CANID_DIM_CONFIG0_DRV2 = 0x610,
    CMR_CANID_DIM_CONFIG1_DRV2 = 0x611,
    CMR_CANID_DIM_CONFIG2_DRV2 = 0x612,
    CMR_CANID_DIM_CONFIG3_DRV2 = 0x613,
    CMR_CANID_DCM_CONFIG0_DRV2 = 0x614,
    CMR_CANID_DCM_CONFIG1_DRV2 = 0x615,
    CMR_CANID_DCM_CONFIG2_DRV2 = 0x616,
    CMR_CANID_DCM_CONFIG3_DRV2 = 0x617,
    CMR_CANID_DIM_CONFIG0_DRV3 = 0x618,
    CMR_CANID_DIM_CONFIG1_DRV3 = 0x619,
    CMR_CANID_DIM_CONFIG2_DRV3 = 0x61a,
    CMR_CANID_DIM_CONFIG3_DRV3 = 0x61b,
    CMR_CANID_DCM_CONFIG0_DRV3 = 0x61c,
    CMR_CANID_DCM_CONFIG1_DRV3 = 0x61d,
    CMR_CANID_DCM_CONFIG2_DRV3 = 0x61e,
    CMR_CANID_DCM_CONFIG3_DRV3 = 0x61f,


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
    CMR_CANID_HVBMS_MIN_MAX_CELL_VOLTAGE = 0x312,                  /**< @brief HVC Max/Min Cell Voltage. */
    CMR_CANID_HVBMS_MIN_MAX_CELL_TEMPERATURE = 0x313,              /**< @brief HVC Max/Min Cell Temperature. */
    CMR_CANID_HVC_LOW_VOLTAGE = 0x303,                           /**< @brief HVC Low Voltage. */
    CMR_CANID_HVC_BMB_STATUS_ERRORS = 0x304,
    CMR_CANID_HV_SENSORS = 0x306, 
    CMR_CANID_HVC_BRUSA_MSG = 0x528,                             /**< @brief HVC Brusa message. */

    CMR_CANID_HVBMS_BMB_0_STATUS_VOLTAGE = 0x380,                    /**< @brief HVC BMB Voltage: Message ID is BMB number << 1*/
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
    CMR_CANID_HVBMS_BMB_0_STATUS_TEMP = 0x381,                       /**< @brief HVC BMB Temperature: Message ID is BMB number << 1 + 1*/
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

    CMR_CANID_SBG_STATUS_1 = 0x6ce,             /**< @brief SBG_ECAN_LOG_STATUS_01 */
    CMR_CANID_SBG_STATUS_2 = 0x6cf,             /**< @brief SBG_ECAN_LOG_STATUS_02 */
    CMR_CANID_SBG_STATUS_3 = 0x6d0,             /**< @brief SBG_ECAN_LOG_STATUS_03 */
    CMR_CANID_SBG_EKF_ORIENT = 0x6d3,           /**< @brief SBG_ECAN_LOG_EKF_EULER */
    CMR_CANID_SBG_EKF_POS = 0x6d4,              /**< @brief SBG_ECAN_LOG_EKF_POS */
    CMR_CANID_SBG_EKF_VEL = 0x6d5,              /**< @brief SBG_ECAN_LOG_EKF_VEL */
    CMR_CANID_SBG_IMU_ACCEL = 0x6d1,            /**< @brief SBG_ECAN_LOG_IMU_ACCEL */
    CMR_CANID_SBG_IMU_GYRO = 0x6d2,             /**< @brief SBG_ECAN_LOG_IMU_GYRO */
    CMR_CANID_SBG_BODY_VEL = 0x6d6,             /**< @brief SBG_ECAN_MSG_EKF_VEL_BODY */
    CMR_CANID_SBG_AUTOMOTIVE = 0x6d7,           /**< @brief SBG_ECAN_MSG_AUTO_TRACK_SLIP_CURV */

    CMR_CANID_MOVELLA_STATUS = 0x6d8,
    CMR_CANID_MOVELLA_QUATERNION = 0x6d9,
    CMR_CANID_MOVELLA_IMU_GYRO = 0x6da,
    CMR_CANID_MOVELLA_IMU_ACCEL = 0x6dd,
    CMR_CANID_MOVELLA_VELOCITY = 0x6dc,

    CMR_CANID_EMD_STATUS = 0x400,               /**< @brief EMD status. */
    CMR_CANID_EMD_MEASUREMENT_RETX = 0x401,     /**< @brief EMD measurement for HV voltage/current. */
    CMR_CANID_EMD_MEASUREMENT = 0x402,          /**< @brief EMD measurement for HV voltage/current. */

    CMR_IZZIE_LOADCELL = 0x6F1,                 /**< @brief IZZIE Amp load data. */
    CMR_CANID_CONTROLS_DEBUG_GLOBAl = 0x6E2,    /**< @brief control algo testing data. */
    CMR_CANID_CONTROLS_DEBUG_FR = 0x6E3,        /**< @brief control algo testing data. */
    CMR_CANID_CONTROLS_DEBUG_FL = 0x6E4,        /**< @brief control algo testing data. */
    CMR_CANID_CONTROLS_DEBUG_RR = 0x6E5,        /**< @brief control algo testing data. */
    CMR_CANID_CONTROLS_DEBUG_RL = 0x6E6,        /**< @brief control algo testing data. */
    CMR_CANID_CONTROLS_PID_IO = 0x6E7,        /**< @brief control algo testing data. */

    CMR_CANID_CONTROLS_SOLVER_INPUTS = 0x6E8,
    CMR_CANID_CONTROLS_SOLVER_OUTPUTS = 0x6F0,
    CMR_CANID_CONTROLS_SOLVER_SETTINGS = 0x6EF,
    CMR_CANID_CONTROLS_SOLVER_AUX = 0x6FF,

    CMR_CANID_LOADCELL_FL = 0x6e1,
    CMR_CANID_LOADCELL_FR = 0x6de,
    CMR_CANID_LOADCELL_RL = 0x6df,
    CMR_CANID_LOADCELL_RR = 0x6e0,

    CMR_CANID_FRONT_SLIP_RATIOS = 0x6E9,
    CMR_CANID_REAR_SLIP_RATIOS = 0x6Ea,
    CMR_CANID_FRONT_WHL_SETPOINTS = 0x6Eb,
    CMR_CANID_REAR_WHL_SETPOINTS = 0x6Ec,
    CMR_CANID_FRONT_WHL_VELS = 0x6Ed,
    CMR_CANID_REAR_WHL_VELS = 0x6Ee,

    CMR_CANID_TEST_ID=0x6db,

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

    CMR_CANID_VSM_GIT = 0x6F2,
	CMR_CANID_HVC_GIT = 0x6f3,
	CMR_CANID_DCM_GIT = 0x6f4,
	CMR_CANID_DIM_GIT = 0x6f5,
	CMR_CANID_RAM_GIT = 0x6f6,

    //AS
    CMR_CANID_DS_RES = 0x182,                   /**< @brief Autonomous RES (remote E-stop)*/
    CMR_CANID_AUTONOMOUS_ACTION = 0x190,        /**< @brief Autonomous Action*/
    CMR_CANID_DS_PRESSURE_READINGS = 0x191,     /**< @brief Autonomous Pressure Readings for Tank and EBS*/
    CMR_CANID_DS_RACK_DISPLACMENT = 0x192,      /**< @brief Rack Displacement Mesurment*/
    CMR_CANID_DSMS_STATE = 0x193,


    CMR_CANID_HVBMS_BMB_0_CELL_VOLTAGES_0_3 = 0x700,
    CMR_CANID_HVBMS_BMB_1_CELL_VOLTAGES_0_3 = 0x701,
    CMR_CANID_HVBMS_BMB_2_CELL_VOLTAGES_0_3 = 0x702,
    CMR_CANID_HVBMS_BMB_3_CELL_VOLTAGES_0_3 = 0x703,
    CMR_CANID_HVBMS_BMB_4_CELL_VOLTAGES_0_3 = 0x704,
    CMR_CANID_HVBMS_BMB_5_CELL_VOLTAGES_0_3 = 0x705,
    CMR_CANID_HVBMS_BMB_6_CELL_VOLTAGES_0_3 = 0x706,
    CMR_CANID_HVBMS_BMB_7_CELL_VOLTAGES_0_3 = 0x707,
    CMR_CANID_HVBMS_BMB_8_CELL_VOLTAGES_0_3 = 0x708,
    CMR_CANID_HVBMS_BMB_9_CELL_VOLTAGES_0_3 = 0x709,
    CMR_CANID_HVBMS_BMB_10_CELL_VOLTAGES_0_3 = 0x70a,
    CMR_CANID_HVBMS_BMB_11_CELL_VOLTAGES_0_3 = 0x70b,
    CMR_CANID_HVBMS_BMB_12_CELL_VOLTAGES_0_3 = 0x70c,
    CMR_CANID_HVBMS_BMB_13_CELL_VOLTAGES_0_3 = 0x70d,
    CMR_CANID_HVBMS_BMB_14_CELL_VOLTAGES_0_3 = 0x70e,
    CMR_CANID_HVBMS_BMB_15_CELL_VOLTAGES_0_3 = 0x70f,

    CMR_CANID_HVBMS_BMB_0_CELL_VOLTAGES_4_7 = 0x710, 
    CMR_CANID_HVBMS_BMB_1_CELL_VOLTAGES_4_7 = 0x711,
    CMR_CANID_HVBMS_BMB_2_CELL_VOLTAGES_4_7 = 0x712,
    CMR_CANID_HVBMS_BMB_3_CELL_VOLTAGES_4_7 = 0x713,
    CMR_CANID_HVBMS_BMB_4_CELL_VOLTAGES_4_7 = 0x714,
    CMR_CANID_HVBMS_BMB_5_CELL_VOLTAGES_4_7 = 0x715,
    CMR_CANID_HVBMS_BMB_6_CELL_VOLTAGES_4_7 = 0x716,
    CMR_CANID_HVBMS_BMB_7_CELL_VOLTAGES_4_7 = 0x717,
    CMR_CANID_HVBMS_BMB_8_CELL_VOLTAGES_4_7 = 0x718,
    CMR_CANID_HVBMS_BMB_9_CELL_VOLTAGES_4_7 = 0x719,
    CMR_CANID_HVBMS_BMB_10_CELL_VOLTAGES_4_7 = 0x71a,
    CMR_CANID_HVBMS_BMB_11_CELL_VOLTAGES_4_7 = 0x71b,
    CMR_CANID_HVBMS_BMB_12_CELL_VOLTAGES_4_7 = 0x71c,
    CMR_CANID_HVBMS_BMB_13_CELL_VOLTAGES_4_7 = 0x71d,
    CMR_CANID_HVBMS_BMB_14_CELL_VOLTAGES_4_7 = 0x71e,
    CMR_CANID_HVBMS_BMB_15_CELL_VOLTAGES_4_7 = 0x71f,

    CMR_CANID_HVBMS_BMB_0_CELL_VOLTAGES_8_11 = 0x720, 
    CMR_CANID_HVBMS_BMB_1_CELL_VOLTAGES_8_11 = 0x721,
    CMR_CANID_HVBMS_BMB_2_CELL_VOLTAGES_8_11 = 0x722,
    CMR_CANID_HVBMS_BMB_3_CELL_VOLTAGES_8_11 = 0x723,
    CMR_CANID_HVBMS_BMB_4_CELL_VOLTAGES_8_11 = 0x724,
    CMR_CANID_HVBMS_BMB_5_CELL_VOLTAGES_8_11 = 0x725,
    CMR_CANID_HVBMS_BMB_6_CELL_VOLTAGES_8_11 = 0x726,
    CMR_CANID_HVBMS_BMB_7_CELL_VOLTAGES_8_11 = 0x727,
    CMR_CANID_HVBMS_BMB_8_CELL_VOLTAGES_8_11 = 0x728,
    CMR_CANID_HVBMS_BMB_9_CELL_VOLTAGES_8_11 = 0x729,
    CMR_CANID_HVBMS_BMB_10_CELL_VOLTAGES_8_11 = 0x72a,
    CMR_CANID_HVBMS_BMB_11_CELL_VOLTAGES_8_11 = 0x72b,
    CMR_CANID_HVBMS_BMB_12_CELL_VOLTAGES_8_11 = 0x72c,
    CMR_CANID_HVBMS_BMB_13_CELL_VOLTAGES_8_11 = 0x72d,
    CMR_CANID_HVBMS_BMB_14_CELL_VOLTAGES_8_11 = 0x72e,
    CMR_CANID_HVBMS_BMB_15_CELL_VOLTAGES_8_11 = 0x72f,

    CMR_CANID_HVBMS_BMB_0_CELL_VOLTAGES_12_14 = 0x730, 
    CMR_CANID_HVBMS_BMB_1_CELL_VOLTAGES_12_14 = 0x731,
    CMR_CANID_HVBMS_BMB_2_CELL_VOLTAGES_12_14 = 0x732,
    CMR_CANID_HVBMS_BMB_3_CELL_VOLTAGES_12_14 = 0x733,
    CMR_CANID_HVBMS_BMB_4_CELL_VOLTAGES_12_14 = 0x734,
    CMR_CANID_HVBMS_BMB_5_CELL_VOLTAGES_12_14 = 0x735,
    CMR_CANID_HVBMS_BMB_6_CELL_VOLTAGES_12_14 = 0x736,
    CMR_CANID_HVBMS_BMB_7_CELL_VOLTAGES_12_14 = 0x737,
    CMR_CANID_HVBMS_BMB_8_CELL_VOLTAGES_12_14 = 0x738,
    CMR_CANID_HVBMS_BMB_9_CELL_VOLTAGES_12_14 = 0x739,
    CMR_CANID_HVBMS_BMB_10_CELL_VOLTAGES_12_14 = 0x73a,
    CMR_CANID_HVBMS_BMB_11_CELL_VOLTAGES_12_14 = 0x73b,
    CMR_CANID_HVBMS_BMB_12_CELL_VOLTAGES_12_14 = 0x73c,
    CMR_CANID_HVBMS_BMB_13_CELL_VOLTAGES_12_14 = 0x73d,
    CMR_CANID_HVBMS_BMB_14_CELL_VOLTAGES_12_14 = 0x73e,
    CMR_CANID_HVBMS_BMB_15_CELL_VOLTAGES_12_14 = 0x73f,

    CMR_CANID_HVBMS_BMB_0_CELL_TEMPS_0_3 = 0x740, 
    CMR_CANID_HVBMS_BMB_1_CELL_TEMPS_0_3 = 0x741,
    CMR_CANID_HVBMS_BMB_2_CELL_TEMPS_0_3 = 0x742,
    CMR_CANID_HVBMS_BMB_3_CELL_TEMPS_0_3 = 0x743,
    CMR_CANID_HVBMS_BMB_4_CELL_TEMPS_0_3 = 0x744,
    CMR_CANID_HVBMS_BMB_5_CELL_TEMPS_0_3 = 0x745,
    CMR_CANID_HVBMS_BMB_6_CELL_TEMPS_0_3 = 0x746,
    CMR_CANID_HVBMS_BMB_7_CELL_TEMPS_0_3 = 0x747,
    CMR_CANID_HVBMS_BMB_8_CELL_TEMPS_0_3 = 0x748,
    CMR_CANID_HVBMS_BMB_9_CELL_TEMPS_0_3 = 0x749,
    CMR_CANID_HVBMS_BMB_10_CELL_TEMPS_0_3 = 0x74a,
    CMR_CANID_HVBMS_BMB_11_CELL_TEMPS_0_3 = 0x74b,
    CMR_CANID_HVBMS_BMB_12_CELL_TEMPS_0_3 = 0x74c,
    CMR_CANID_HVBMS_BMB_13_CELL_TEMPS_0_3 = 0x74d,
    CMR_CANID_HVBMS_BMB_14_CELL_TEMPS_0_3 = 0x74e,
    CMR_CANID_HVBMS_BMB_15_CELL_TEMPS_0_3 = 0x74f,

    CMR_CANID_HVBMS_BMB_0_CELL_TEMPS_4_7 = 0x750, 
    CMR_CANID_HVBMS_BMB_1_CELL_TEMPS_4_7 = 0x751,
    CMR_CANID_HVBMS_BMB_2_CELL_TEMPS_4_7 = 0x752,
    CMR_CANID_HVBMS_BMB_3_CELL_TEMPS_4_7 = 0x753,
    CMR_CANID_HVBMS_BMB_4_CELL_TEMPS_4_7 = 0x754,
    CMR_CANID_HVBMS_BMB_5_CELL_TEMPS_4_7 = 0x755,
    CMR_CANID_HVBMS_BMB_6_CELL_TEMPS_4_7 = 0x756,
    CMR_CANID_HVBMS_BMB_7_CELL_TEMPS_4_7 = 0x757,
    CMR_CANID_HVBMS_BMB_8_CELL_TEMPS_4_7 = 0x758,
    CMR_CANID_HVBMS_BMB_9_CELL_TEMPS_4_7 = 0x759,
    CMR_CANID_HVBMS_BMB_10_CELL_TEMPS_4_7 = 0x75a,
    CMR_CANID_HVBMS_BMB_11_CELL_TEMPS_4_7 = 0x75b,
    CMR_CANID_HVBMS_BMB_12_CELL_TEMPS_4_7 = 0x75c,
    CMR_CANID_HVBMS_BMB_13_CELL_TEMPS_4_7 = 0x75d,
    CMR_CANID_HVBMS_BMB_14_CELL_TEMPS_4_7 = 0x75e,
    CMR_CANID_HVBMS_BMB_15_CELL_TEMPS_4_7 = 0x75f,

    CMR_CANID_HVBMS_BMB_0_CELL_TEMPS_8_11 = 0x760, 
    CMR_CANID_HVBMS_BMB_1_CELL_TEMPS_8_11 = 0x761,
    CMR_CANID_HVBMS_BMB_2_CELL_TEMPS_8_11 = 0x762,
    CMR_CANID_HVBMS_BMB_3_CELL_TEMPS_8_11 = 0x763,
    CMR_CANID_HVBMS_BMB_4_CELL_TEMPS_8_11 = 0x764,
    CMR_CANID_HVBMS_BMB_5_CELL_TEMPS_8_11 = 0x765,
    CMR_CANID_HVBMS_BMB_6_CELL_TEMPS_8_11 = 0x766,
    CMR_CANID_HVBMS_BMB_7_CELL_TEMPS_8_11 = 0x767,
    CMR_CANID_HVBMS_BMB_8_CELL_TEMPS_8_11 = 0x768,
    CMR_CANID_HVBMS_BMB_9_CELL_TEMPS_8_11 = 0x769,
    CMR_CANID_HVBMS_BMB_10_CELL_TEMPS_8_11 = 0x76a,
    CMR_CANID_HVBMS_BMB_11_CELL_TEMPS_8_11 = 0x76b,
    CMR_CANID_HVBMS_BMB_12_CELL_TEMPS_8_11 = 0x76c,
    CMR_CANID_HVBMS_BMB_13_CELL_TEMPS_8_11 = 0x76d,
    CMR_CANID_HVBMS_BMB_14_CELL_TEMPS_8_11 = 0x76e,
    CMR_CANID_HVBMS_BMB_15_CELL_TEMPS_8_11 = 0x76f,

    CMR_CANID_HVBMS_BMB_0_CELL_TEMPS_12_14 = 0x770, 
    CMR_CANID_HVBMS_BMB_1_CELL_TEMPS_12_14 = 0x771,
    CMR_CANID_HVBMS_BMB_2_CELL_TEMPS_12_14 = 0x772,
    CMR_CANID_HVBMS_BMB_3_CELL_TEMPS_12_14 = 0x773,
    CMR_CANID_HVBMS_BMB_4_CELL_TEMPS_12_14 = 0x774,
    CMR_CANID_HVBMS_BMB_5_CELL_TEMPS_12_14 = 0x775,
    CMR_CANID_HVBMS_BMB_6_CELL_TEMPS_12_14 = 0x776,
    CMR_CANID_HVBMS_BMB_7_CELL_TEMPS_12_14 = 0x777,
    CMR_CANID_HVBMS_BMB_8_CELL_TEMPS_12_14 = 0x778,
    CMR_CANID_HVBMS_BMB_9_CELL_TEMPS_12_14 = 0x779,
    CMR_CANID_HVBMS_BMB_10_CELL_TEMPS_12_14 = 0x77a,
    CMR_CANID_HVBMS_BMB_11_CELL_TEMPS_12_14 = 0x77b,
    CMR_CANID_HVBMS_BMB_12_CELL_TEMPS_12_14 = 0x77c,
    CMR_CANID_HVBMS_BMB_13_CELL_TEMPS_12_14 = 0x77d,
    CMR_CANID_HVBMS_BMB_14_CELL_TEMPS_12_14 = 0x77e,
    CMR_CANID_HVBMS_BMB_15_CELL_TEMPS_12_14 = 0x77f,
    CMR_CANID_LVBMS_CELL_VOLTAGE_1_4 = 0x6F7,
    CMR_CANID_LVBMS_CELL_VOLTAGE_5_7 = 0x6f8,
    CMR_CANID_LVBMS_CELL_TEMP_1_4 = 0x6f9,
    CMR_CANID_LVBMS_CELL_TEMP_5_7 = 0x6fa,
    CMR_CANID_LVBMS_CELL_OVERVOLTAGE = 0x6fb,
    CMR_CANID_LVBMS_CELL_OVERTEMP = 0x6fc,
    CMR_CANID_LVBMS_BUS_VOLTAGE = 0x6fd,
    CMR_CANID_LVBMS_CURRENT = 0x6fe,
} cmr_canID_t;

typedef enum {
    CMR_CANID_EXTENDED_CUBEMARS_SET_DUTY = 0 << 8 | CONTROLLER_ID,
    CMR_CANID_EXTENDED_CUBEMARS_SET_CURRENT = 1 << 8 | CONTROLLER_ID,
    CMR_CANID_EXTENDED_CUBEMARS_SET_CURRENT_BRAKE = 2 << 8 | CONTROLLER_ID,
    CMR_CANID_EXTENDED_CUBEMARS_SET_RPM = 3 << 8 | CONTROLLER_ID,
    CMR_CANID_EXTENDED_CUBEMARS_SET_POS = 4 << 8 | CONTROLLER_ID,
    CMR_CANID_EXTENDED_CUBEMARS_SET_ORIGIN_HERE = 5 << 8 | CONTROLLER_ID,
    CMR_CANID_EXTENDED_CUBEMARS_SET_POS_SPD = 6 << 8 | CONTROLLER_ID,
    CMR_CANID_EXTENDED_CUBEMARS_DATA = 0x000029FF
} cmr_canExtendedID_t;

#endif /* CMR_CAN_IDS_H */