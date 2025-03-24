/**
 * @file can_types.h
 * @brief Shared CAN type definitions.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_CAN_TYPES_H
#define CMR_CAN_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// ------------------------------------------------------------------------------------------------
// Common enums and structs

/** @brief Node states. */
typedef enum {
    CMR_CAN_UNKNOWN = 0,    /**< @brief Current state unknown. */
    CMR_CAN_GLV_ON,         /**< @brief Grounded low voltage on. */
    CMR_CAN_HV_EN,          /**< @brief High voltage enabled. */
    CMR_CAN_RTD,            /**< @brief Ready to drive. */
    CMR_CAN_ERROR,          /**< @brief Error has occurred. */
    CMR_CAN_CLEAR_ERROR     /**< @brief Request to clear error. */
} cmr_canState_t;

/** @brief Standard CAN heartbeat. */
typedef struct {
    uint8_t state;          /**< @brief Board state. */
    uint8_t error[2];       /**< @brief Error matrix. */
    uint8_t warning[2];     /**< @brief Warning matrix. */
} cmr_canHeartbeat_t;

/** @brief Heartbeat error matrix bit fields. */
typedef enum {
    CMR_CAN_ERROR_NONE = 0,     /**< @brief No errors. */

    /** @brief No VSM heartbeat received for 50 ms. */
    CMR_CAN_ERROR_VSM_TIMEOUT = (1 << 0),

    /**
     * @brief Reception period for at least one message from another module
     * has surpassed its error threshold.
     */
    CMR_CAN_ERROR_VSM_MODULE_TIMEOUT = (1 << 15),
    /** @brief At least one module is in the wrong state. */
    CMR_CAN_ERROR_VSM_MODULE_STATE = (1 << 14),
    /** @brief At least one of the error latches is active. */
    CMR_CAN_ERROR_VSM_LATCHED_ERROR = (1 << 13),
    /** @brief VSM DCDC fault signal. */
    CMR_CAN_ERROR_VSM_DCDC_FAULT = (1 << 12),
    /** @brief VSM hall effect sensor out-of-range. */
    CMR_CAN_ERROR_VSM_HALL_EFFECT = (1 << 11),
    /** @brief VSM brake pressure sensor out-of-range. */
    CMR_CAN_ERROR_VSM_BPRES = (1 << 10),

    /** @brief AFC fan current out-of-range. */
    CMR_CAN_ERROR_AFC_FANS_CURRENT = (1 << 15),
    /** @brief AFC driver IC #1 temperature out-of-range. */
    CMR_CAN_ERROR_AFC_DRIVER1_TEMP = (1 << 14),
    /** @brief AFC driver IC #2 temperature out-of-range. */
    CMR_CAN_ERROR_AFC_DRIVER2_TEMP = (1 << 13),
    /** @brief AFC driver IC #3 temperature out-of-range. */
    CMR_CAN_ERROR_AFC_DRIVER3_TEMP = (1 << 12),
    /** @brief AFC DCDC #1 temperature out-of-range. */
    CMR_CAN_ERROR_AFC_DCDC1_TEMP = (1 << 11),
    /** @brief AFC DCDC #2 temperature out-of-range. */
    CMR_CAN_ERROR_AFC_DCDC2_TEMP = (1 << 10),

    /** @brief
 fan current out-of-range. */
    CMR_CAN_ERROR_PTC_FAN_CURRENT = (1 << 15),
    /** @brief
 fan/pump driver IC temperature out-of-range. */
    CMR_CAN_ERROR_PTC_DRIVERS_TEMP = (1 << 14),
    /** @brief
 water temperature out-of-range. */
    CMR_CAN_ERROR_PTC_WATER_TEMP = (1 << 13),
    //power errors(shunt resistor), water over heating errors, oil overheatin errors
    //no oil overheating errors cuz going into uprights
    // temperature
    // pump always on 35 c
    // pump turn on at 53 start turning on and 56 turning at 100
    // fan turn on at 56 starting 58 turn it to max

    /** @brief CDC All motor controllers have errored or timed out. */
    CMR_CAN_ERROR_CDC_AMK_ALL = (1 << 15)
} cmr_canError_t;

/** @brief Heartbeat warning matrix bit fields. */
typedef enum {
    CMR_CAN_WARN_NONE = 0,  /**< @brief No warnings. */

    /** @brief No VSM heartbeat received for 25 ms. */
    CMR_CAN_WARN_VSM_TIMEOUT = (1 << 0),
    /** @brief Low-voltage bus voltage out-of-range. */
    CMR_CAN_WARN_BUS_VOLTAGE = (1 << 1),
    /** @brief Low-voltage bus current out-of-range. */
    CMR_CAN_WARN_BUS_CURRENT = (1 << 2),

    // TODO: Consolidate
    /** @brief VSM hasn't received HVC heartbeat for 25 ms. */
    CMR_CAN_WARN_VSM_HVC_TIMEOUT = (1 << 14),
    /** @brief VSM hasn't received CDC heartbeat for 25 ms. */
    CMR_CAN_WARN_VSM_CDC_TIMEOUT = (1 << 13),
    /** @brief VSM hasn't received FSM heartbeat for 25 ms. */
    CMR_CAN_WARN_VSM_FSM_TIMEOUT = (1 << 12),
    /** @brief VSM hasn't received DIM heartbeat for 25 ms. */
    CMR_CAN_WARN_VSM_DIM_TIMEOUT = (1 << 11),
    /** @brief VSM hasn't received PTCf heartbeat for 25 ms. */
    CMR_CAN_WARN_VSM_PTC_TIMEOUT = (1 << 10),
    /** @brief VSM hasn't received APC heartbeat for 25 ms. */
    CMR_CAN_WARN_VSM_APC_TIMEOUT = (1 << 8),
    /** @brief VSM is rejecting DIM state request. */
    CMR_CAN_WARN_VSM_DIM_REQ_NAK = (1 << 7),

    /** @brief FSM throttle position implausibility (L/R difference > 10%). */
    CMR_CAN_WARN_FSM_TPOS_IMPLAUSIBLE = (1 << 15),
    /** @brief FSM brake pedal plausibility fault. */
    CMR_CAN_WARN_FSM_BPP = (1 << 14),
    /** @brief FSM right throttle position out-of-range. */
    CMR_CAN_WARN_FSM_TPOS_R = (1 << 13),
    /** @brief FSM left throttle position out-of-range. */
    CMR_CAN_WARN_FSM_TPOS_L = (1 << 12),
    /** @brief FSM brake pedal position out-of-range. */
    CMR_CAN_WARN_FSM_BPOS = (1 << 11),
    /** @brief FSM brake pressure sensor out-of-range. */
    CMR_CAN_WARN_FSM_BPRES = (1 << 10),
    /** @brief FSM steering wheel angle out-of-range. */
    CMR_CAN_WARN_FSM_SWANGLE = (1 << 9),

    /** @brief Safety circuit module tripped */
    CMR_CAN_WARN_FSM_SS_MODULE = (1 << 3),
    /** @brief Safety circuit cockpit tripped */
    CMR_CAN_WARN_FSM_SS_COCKPIT = (1 << 4),
    /** @brief Safety circuit FR hub tripped */
    CMR_CAN_WARN_FSM_SS_FRHUB = (1 << 5),
    /** @brief Safety circuit inertia tripped */
    CMR_CAN_WARN_FSM_SS_INERTIA = (1 << 6),
    /** @brief Safety circuit FL hub tripped */
    CMR_CAN_WARN_FSM_SS_FLHUB = (1 << 7),
    /** @brief Safety circuit bots tripped */
    CMR_CAN_WARN_FSM_SS_BOTS = (1 << 8),

    /** @brief CDC Front left motor controller is warning source. */
    CMR_CAN_WARN_CDC_AMK_FL = (1 << 15),
    /** @brief CDC Front right motor controller is warning source. */
    CMR_CAN_WARN_CDC_AMK_FR = (1 << 14),
    /** @brief CDC Rear left motor controller is warning source. */
    CMR_CAN_WARN_CDC_AMK_RL = (1 << 13),
    /** @brief CDC Rear right motor controller is warning source. */
    CMR_CAN_WARN_CDC_AMK_RR = (1 << 12),
    /** @brief CDC Motor controller has an error. */
    CMR_CAN_WARN_CDC_AMK_ERROR = (1 << 11),
    /** @brief CDC Motor controller has timed out. */
    CMR_CAN_WARN_CDC_AMK_TIMEOUT = (1 << 10),
    CMR_CAN_WARN_CDC_MEMORATOR_DAQ_TIMEOUT = (1 << 9)
} cmr_canWarn_t;

/** @brief Represents the car's current driving mode (gear). */
typedef enum {
    CMR_CAN_GEAR_UNKNOWN = 0,   /**< @brief Unknown Gear State */
    CMR_CAN_GEAR_SLOW,          /**< @brief Slow mode */
    CMR_CAN_GEAR_FAST,          /**< @brief Fast simple mode */
    CMR_CAN_GEAR_ENDURANCE,     /**< @brief Endurance-event mode */
    CMR_CAN_GEAR_AUTOX,         /**< @brief Autocross-event mode */
    CMR_CAN_GEAR_SKIDPAD,       /**< @brief Skidpad-event mode */
    CMR_CAN_GEAR_ACCEL,         /**< @brief Acceleration-event mode */
    CMR_CAN_GEAR_TEST,          /**< @brief Test mode (for experimentation) */
    CMR_CAN_GEAR_REVERSE,       /**< @brief Reverse mode */
    CMR_CAN_GEAR_LEN
} cmr_canGear_t;

/** @brief Represents the car's current DRS mode (). */
typedef enum {
    CMR_CAN_DRSM_QUIET = 0,
    CMR_CAN_DRSM_CLOSED,
    CMR_CAN_DRSM_OPEN,
    CMR_CAN_DRSM_TOGGLE,
    CMR_CAN_DRSM_HOLD,
    CMR_CAN_DRSM_AUTO,
    CMR_CAN_DRSM_LEN,
    CMR_CAN_DRSM_UNKNOWN
} cmr_canDrsMode_t;

// ------------------------------------------------------------------------------------------------
// Vehicle Safety Module

/** @brief Vehicle Safety Module internal states. */
typedef enum {
    CMR_CAN_VSM_STATE_ERROR = 0,        /**< @brief Error state. */
    CMR_CAN_VSM_STATE_CLEAR_ERROR,      /**< @brief Clear error state. */
    CMR_CAN_VSM_STATE_GLV_ON,           /**< @brief Grounded Low Voltage system on. */
    CMR_CAN_VSM_STATE_REQ_PRECHARGE,    /**< @brief Request accumulator isolation relay precharge. */
    CMR_CAN_VSM_STATE_RUN_BMS,          /**< @brief Run Battery Management System. */
    CMR_CAN_VSM_STATE_DCDC_EN,          /**< @brief Enable DCDC converters. */
    CMR_CAN_VSM_STATE_INVERTER_EN,      /**< #brief Enable inverter logic power. */
    CMR_CAN_VSM_STATE_HV_EN,            /**< @brief Enable high voltage system. */
    CMR_CAN_VSM_STATE_RTD,              /**< @brief Ready to drive. */
    CMR_CAN_VSM_STATE_COOLING_OFF,      /**< @brief Disable powertrain cooling system. */
    CMR_CAN_VSM_STATE_DCDC_OFF,         /**< @brief Disable DCDC converters. */
    CMR_CAN_VSM_STATE_LEN               /**< @brief Number of VSM states. */
} cmr_canVSMState_t;

/** @brief Bit definitions for timeoutMatrix and badStateMatrix in cmr_canVSMErrors_t. */
typedef enum {
    /** @brief No modules have timed out. */
    CMR_CAN_VSM_ERROR_SOURCE_NONE = 0,
    /** @brief At least one High Voltage Controller message has timed out. */
    CMR_CAN_VSM_ERROR_SOURCE_HVC = (1 << 6),
    /** @brief At least one Central Dynamics Controller message has timed out. */
    CMR_CAN_VSM_ERROR_SOURCE_CDC = (1 << 5),
    /** @brief At least one Front Sensor Module message has timed out. */
    CMR_CAN_VSM_ERROR_SOURCE_FSM = (1 << 4),
    /** @brief At least one Driver Interface Module message has timed out. */
    CMR_CAN_VSM_ERROR_SOURCE_DIM = (1 << 3),
    /** @brief At least one
 message has timed out. */
    CMR_CAN_VSM_ERROR_SOURCE_PTC = (1 << 2),
    /** @brief At least one Auxiliary Power Controller message has timed out. */
    CMR_CAN_VSM_ERROR_SOURCE_APC = (1 << 0)
} cmr_canVSMErrorSource_t;

/** @brief Bit definitions for latchMatrix in cmr_canVSMErrors_t. */
typedef enum {
    /** @brief No error latches are active. */
    CMR_CAN_VSM_LATCH_NONE = 0,
    /** @brief Software error latch is active. */
    CMR_CAN_VSM_LATCH_SOFTWARE = (1 << 2),
    /** @brief AMS errors use software error latch. */
    CMR_CAN_VSM_LATCH_AMS = CMR_CAN_VSM_LATCH_SOFTWARE,
    /** @brief IMD error latch is active. */
    CMR_CAN_VSM_LATCH_IMD = (1 << 1),
    /** @brief BSPD error latch is active. */
    CMR_CAN_VSM_LATCH_BSPD = (1 << 0),
} cmr_canVSMLatch_t;

/** @brief Vehicle Safety Module state and error status. */
typedef struct {
    uint8_t internalState;  /**< @brief VSM internal state. See cmr_canVSMState_t. */
    /**
     * @brief Matrix of modules for which at least one message exceeded its error timeout.
     * Bits defined by cmr_canVSMErrorSource_t.
     */
    uint8_t moduleTimeoutMatrix;
    /**
     * @brief Matrix of modules that are in the wrong state.
     * Bits defined by cmr_canVSMErrorSource_t.
     */
    uint8_t badStateMatrix;
    /** @brief Matrix of active error latches. Bits defined by cmr_canVSMLatch_t. */
    uint8_t latchMatrix;
} cmr_canVSMStatus_t;

/** @brief Vehicle Safety Module sensor data. */
/** @brief Vehicle Safety Module sensor data. */
typedef struct {
    uint16_t brakePressureRear_PSI;     /**< @brief Rear brake pressure (pounds-per-square-inch). */
    int16_t hallEffect_cA;     /**< @brief Hall effect current (centi-Amps). */
    uint8_t safetyIn_dV;        /**< @brief Safety circuit input voltage (deci-Volts). */
    uint8_t safetyOut_dV;       /**< @brief Safety circuit output voltage (deci-Volts). */
} cmr_canVSMSensors_t;

/** @brief Vehicle Safety Module latched error status. */
typedef struct {
    /**
     * @brief Matrix of modules for which at least one message exceeded its error timeout.
     * Bits defined by cmr_canVSMErrorSource_t.
     */
    uint8_t moduleTimeoutMatrix;
    /**
     * @brief Matrix of modules that are in the wrong state.
     * Bits defined by cmr_canVSMErrorSource_t.
     */
    uint8_t badStateMatrix;
    /** @brief Matrix of active error latches. Bits defined by cmr_canVSMLatch_t. */
    uint8_t latchMatrix;
} cmr_canVSMLatchedStatus_t;

/** @brief Vehicle Safety Module power diagnostics. */
typedef struct {
    uint16_t busVoltage_mV;     /**< @brief Low-voltage bus voltage (mV). */
    uint16_t busCurrent_mA;     /**< @brief Low-voltage bus current (mA). */
} cmr_canVSMPowerDiagnostics_t;

// ------------------------------------------------------------------------------------------------
// High Voltage Controller

/** @brief CMR High Voltage Controller modes. */
typedef enum {
    CMR_CAN_HVC_MODE_ERROR  = 0,        /**< @brief Error mode. */
    CMR_CAN_HVC_MODE_IDLE   = (1 << 0), /**< @brief Idle mode. */
    CMR_CAN_HVC_MODE_START  = (1 << 1), /**< @brief Start mode to go into run or charge. */
    CMR_CAN_HVC_MODE_RUN    = (1 << 2), /**< @brief Run mode for driving. */
    CMR_CAN_HVC_MODE_CHARGE = (1 << 3)  /**< @brief Charge mode. */
} cmr_canHVCMode_t;

/** @brief CMR High Voltage Controller internal states. */
typedef enum {

    // Error states

    /** @brief Error state. */
    CMR_CAN_HVC_STATE_ERROR = 0x00,
    /** @brief State for clearing errors. */
    CMR_CAN_HVC_STATE_CLEAR_ERROR = 0x0B,
    /** @brief Unknown state. */
    CMR_CAN_HVC_STATE_UNKNOWN = 0x0C,

    // General states

    /** @brief High voltage rails discharging after opening AIRs. */
    CMR_CAN_HVC_STATE_DISCHARGE = 0x01,
    /** @brief AIRs open, vehicle idle in GLV_ON state. */
    CMR_CAN_HVC_STATE_STANDBY = 0x02,

    // Drive states

    /** @brief Precharge high voltage rails in preparation for driving. */
    CMR_CAN_HVC_STATE_DRIVE_PRECHARGE = 0x03,
    /** @brief AIRs closed, ready to go to drive state upon VSM requesting run mode. */
    CMR_CAN_HVC_STATE_DRIVE_PRECHARGE_COMPLETE = 0x04,
    /** @brief AIRs closed and vehicle ready to drive. */
    CMR_CAN_HVC_STATE_DRIVE = 0x05,

    // Charge states

    /** @brief Precharge high voltage rails in preparation for charging. */
    CMR_CAN_HVC_STATE_CHARGE_PRECHARGE = 0x06,
    /** @brief AIRs closed, ready to go to proper charge state upon requesting charge mode. */
    CMR_CAN_HVC_STATE_CHARGE_PRECHARGE_COMPLETE = 0x07,
    /** @brief Trickle charge for severely depleted cells. */
    CMR_CAN_HVC_STATE_CHARGE_TRICKLE = 0x08,
    /** @brief Constant current for majority of charging. */
    CMR_CAN_HVC_STATE_CHARGE_CONSTANT_CURRENT = 0x09,
    /** @brief Constant voltage for topping off charge. */
    CMR_CAN_HVC_STATE_CHARGE_CONSTANT_VOLTAGE = 0x0A
} cmr_canHVCState_t;

/** @brief High Voltage Controller error bit vector definitions. */
typedef enum {
    CMR_CAN_HVC_ERROR_NONE = 0x0000,    /**< @brief No errors detected. */

    // Pack errors
    CMR_CAN_HVC_ERROR_PACK_UNDERVOLT   = 0x0001,    /**< @brief Pack voltage too low. */
    CMR_CAN_HVC_ERROR_PACK_OVERVOLT    = 0x0002,    /**< @brief Pack voltage too high. */
    CMR_CAN_HVC_ERROR_PACK_OVERCURRENT = 0x0008,    /**< @brief Pack current too high. */

    // Cell errors
    CMR_CAN_HVC_ERROR_CELL_UNDERVOLT = 0x0010,  /**< @brief At least one cell is undervoltage. */
    CMR_CAN_HVC_ERROR_CELL_OVERVOLT  = 0x0020,  /**< @brief At least one cell is overvoltage. */
    CMR_CAN_HVC_ERROR_CELL_OVERTEMP  = 0x0040,  /**< @brief At least one cell has overheated. */
    CMR_CAN_HVC_ERROR_BMB_FAULT      = 0x0080,  /**< @brief At least one BMB has faulted. */

    // Communication errors
    CMR_CAN_HVC_ERROR_BMB_TIMEOUT = 0x0100, /**< @brief BMB has timed out. */
    CMR_CAN_HVC_ERROR_CAN_TIMEOUT = 0x0200, /**< @brief HVC command timed out. */

    // Other errors
    CMR_CAN_HVC_ERROR_RELAY        = 0x1000,    /**< @brief Fault with AIRs. */
    CMR_CAN_HVC_ERROR_LV_UNDERVOLT = 0x2000,    /**< @brief Shutdown circuit/AIR voltage too low. */
} cmr_canHVCError_t;

/** @brief High Voltage Controller heartbeat (does not follow universal structure). */
typedef struct {
    uint16_t errorStatus;   /**< @brief Current HVC errors. See cmr_canHVCError_t. */
    uint8_t hvcMode;        /**< @brief Current HVC operating mode. See cmr_canHVCMode_t. */
    uint8_t hvcState;       /**< @brief Current internal HVC state. See cmr_canHVCState_t. */
    uint8_t relayStatus;    /**< @brief Status of AIRs. See cmr_canHVCRelayStatus_t. */
    uint8_t uptime_s;       /**< @brief HVC uptime in seconds. */
} cmr_canHVCHeartbeat_t;

/** @brief High Voltage Controller command. */
typedef struct {
    uint8_t modeRequest;    /**< @brief HVC operating mode request. See cmr_canHVCMode_t. */
} cmr_canHVCCommand_t;

/** @brief High Voltage Controller Balance Command. */
typedef struct {
    bool balanceRequest;    /**< @brief HVC balance command. */
    uint16_t threshold;  /**< @brief Voltage threshold to stop balancing at */
} cmr_canHVCBalanceCommand_t;

/** @brief High Voltage Controller pack voltages. */
typedef struct {
    int32_t battVoltage_mV;    /**< @brief Voltage measured across battery. */
    int32_t hvVoltage_mV;      /**< @brief Voltage outside accumulator. */
} cmr_canHVCPackVoltage_t;

/** @brief High Voltage Controller pack overall min and max cell temperatures. */
typedef struct {
    uint16_t minCellTemp_dC;    /**< @brief Pack min cell temp in dC (tenth of degree C). */
    uint16_t maxCellTemp_dC;    /**< @brief Pack max cell temp in dC (tenth of degree C). */
    uint8_t minTempBMBIndex;    /**< @brief BMB index of coldest cell. */
    uint8_t minTempCellIndex;   /**< @brief Index of coldest cell. */
    uint8_t maxTempBMBIndex;    /**< @brief BMB index of hottest cell. */
    uint8_t maxTempCellIndex;   /**< @brief Index of hottest cell. */
} cmr_canHVCPackMinMaxCellTemps_t;

/** @brief High Voltage Controller pack overall min and max cell voltages. */
typedef struct {
    uint16_t minCellVoltage_mV; /**< @brief Min BMB cell voltage (mV). */
    uint16_t maxCellVoltage_mV; /**< @brief Max BMB cell voltage (mV). */
    uint8_t minCellVoltBMB;     /**< @brief */
    uint8_t minVoltIndex;       /**< @brief Min BMB cell voltage index. */
    uint8_t maxCellVoltBMB;     /**< @brief */
    uint8_t maxVoltIndex;       /**< @brief Max BMB cell voltage index. */
} cmr_canHVCPackMinMaxCellVolages_t;

/** @brief High Voltage Controller pack currents. */
typedef struct {
    int32_t instantCurrent_mA;  /**< @brief Instantaneous current measurement. */
    int32_t avgCurrent_mA;      /**< @brief (Not working) rolling average of current. */
} cmr_canHVCPackCurrent_t;

/** @brief High Voltage Controller BMB errors. */
typedef struct {
    uint8_t BMB1_2_Errs;  /**< @brief Errors for BMB1&2 (BMB1 = higher 4 bits). */
    uint8_t BMB3_4_Errs;  /**< @brief Errors for BMB3&4 (BMB3 = higher 4 bits). */
    uint8_t BMB5_6_Errs;  /**< @brief Errors for BMB5&6 (BMB5 = higher 4 bits). */
    uint8_t BMB7_8_Errs;  /**< @brief Errors for BMB7&8 (BMB7 = higher 4 bits). */
    uint8_t BMB9_10_Errs;  /**< @brief Errors for BMB9&10 (BMB9 = higher 4 bits). */
    uint8_t BMB11_12_Errs;  /**< @brief Errors for BMB11&12 (BMB11 = higher 4 bits). */
    uint8_t BMB13_14_Errs;  /**< @brief Errors for BMB13&14 (BMB13 = higher 4 bits). */
    uint8_t BMB15_16_Errs;  /**< @brief Errors for BMB15&16 (BMB15 = higher 4 bits). */
} cmr_canHVCBMBErrors_t;

//HV_I Sense Board CAN Types
typedef struct {
    int16_t packCurrent_dA;
    uint16_t packVoltage_cV;
    int32_t packPower_W;
} cmr_canHVIHeartbeat_t;

//Power Sense Board CAN Types
typedef struct {
    int16_t packCurrent_dA;
    uint16_t packVoltage_cV;
    int32_t packPower_W;
} cmr_canPowerSense_t;

// ------------------------------------------------------------------------------------------------
// Central Dynamics Controller (19e)

/** @brief Central Dynamics Controller DRS states. */
typedef struct {
    uint8_t state;          /**< @brief DRS current control state (open or closed position). */
    uint8_t angle;          /**< @brief DRS setpoint angle for its current state (debug info). */
    uint8_t pwm_left;       /**< @brief PWM of the left  DRS servo (debug info). */
    uint8_t pwm_right;      /**< @brief PWM of the right DRS servo (debug info). */
} cmr_canCDCDRSStates_t;
typedef enum {
  CMR_CAN_DRS_STATE_CLOSED = 0,
  CMR_CAN_DRS_STATE_OPEN,
  CMR_CAN_DRS_STATE_OTHER
} cmr_canCDCDRSStateEnum_t;

/** @brief Central Dynamics Controller */
typedef struct {
    float odometer_km;      /**< @brief Odometer in km*/
} cmr_canCDCOdometer_t;

typedef struct {
    uint8_t tcOn;
    uint8_t yrcOn;
} cmr_canCDCControlsStatus_t;

/** @brief New power limit from DAQ live during endurance. */
typedef struct {
    uint8_t powerLimit_kW;
} cmr_canCDCPowerLimit_t;

typedef struct {
    float power_limit_W;
} cmr_canCDCPowerLimitLog_t;

/** @brief Central Dynamics Controller Safety Filter states. */
typedef struct {
	float power_limit_max_violation_W;  /**< @brief the maximum amount in W the power hard-limit is violated, expect 0.0 */
	uint8_t longest_power_violation_ms; /**< @brief counts the number of clock cycles when power is over the hard limit, expect <2*/
    uint8_t over_voltage_count;         /**< @brief incremented when pack voltage exceeds 590 */
    uint8_t under_voltage_count;        /**< @brief incremented when pack voltage under 365 */
    uint8_t over_temp_count;            /**<@brief incremented when pack temperature exceeds the hard limit, expect 0>*/
} cmr_canCDCSafetyFilterStates_t;

typedef struct {
    uint16_t motor_power_FL;
    uint16_t motor_power_FR;
    uint16_t motor_power_RL;
    uint16_t motor_power_RR;
} cmr_canCDCMotorPower_t;

typedef struct {
	float KCoulombs;
} cmr_canCDCKiloCoulombs_t;

// ------------------------------------------------------------------------------------------------
// Central Dynamics Controller (20e)

/** @brief CDC wheel speeds (used for setpoint and actual). */
typedef struct {
    int16_t frontLeft_rpm;  /**< @brief Wheel speed on 20e (rpm * 10). */
    int16_t frontRight_rpm; /**< @brief Wheel speed on 20e (rpm * 10). */
    int16_t rearLeft_rpm;   /**< @brief Wheel speed on 20e (rpm * 10). */
    int16_t rearRight_rpm;  /**< @brief Wheel speed on 20e (rpm * 10). */
} cmr_canCDCWheelVelocity_t;

typedef struct {
    int16_t frontLeft_Nm;   /**< @brief Wheel torque on 20e (Nm * 10). */
    int16_t frontRight_Nm;  /**< @brief Wheel speed on 20e (Nm * 10). */
    int16_t rearLeft_Nm;    /**< @brief Wheel speed on 20e (Nm * 10). */
    int16_t rearRight_Nm;   /**< @brief Wheel speed on 20e (Nm * 10). */
} cmr_canCDCWheelTorque_t;

typedef struct {
    float latitude_deg;     /**< @brief Position of car on earth. */
    float longitude_deg;    /**< @brief Position of car on earth. */
} cmr_canCDCPosePosition_t;

typedef struct {
    int16_t roll_deg;       /**< @brief Roll of the car (deg * 10). */
    int16_t pitch_deg;      /**< @brief Pitch of the car (deg * 10). */
    int16_t yaw_deg;        /**< @brief Yaw of the car (deg * 10). */
    int16_t velocity_deg;   /**< @brief Velocity vector of the car (deg * 10). */
} cmr_canCDCPoseOrientation_t;

typedef struct {
    int16_t longitudinalVel_mps;    /**< @brief Velocity of the car in the forward direction (m/s * 100). */
    int16_t lateralVel_mps;         /**< @brief Velocity of the car in the right direction (m/s * 100). */
    int16_t verticalVel_mps;        /**< @brief Velocity of the car in the down direction (m/s * 100). */
} cmr_canCDCPoseVelocity_t;

// ------------------------------------------------------------------------------------------------
// Driver Interface Module

/** @brief Driver Interface Module state/gear request. */
typedef struct {
    uint8_t requestedState;     /**< @brief Requested state. */
    uint8_t requestedGear;      /**< @brief Requested gear. */
    uint8_t requestedDrsMode;   /**< @brief Requested DRS mode. */
    uint8_t requestedDriver;    /**< @brief Requested Driver for Config Screen. */
} cmr_canDIMRequest_t;

/** @brief Driver Interface Module power diagnostics. */
typedef struct {
    uint16_t busVoltage_mV;     /**< @brief Low-voltage bus voltage (mV). */
    uint16_t busCurrent_mA;     /**< @brief Low-voltage bus current (mA). */
} cmr_canDIMPowerDiagnostics_t;

/** @brief Driver Interface Module text write command. This is
 *  used in conjunction with the RAM to facilite remote text
 *  writing to the driver's display.
*/
typedef struct {
    uint8_t address;            /**< @brief Buffer index for text. */
    uint8_t data[4];            /**< @brief Data to write. */
} cmr_canDIMTextWrite_t;

typedef struct {
    uint8_t buttons;                 /**< @brief Button states packed into an uint8_t. {drs,0,1,2,up,down,left,right}*/
    uint8_t rotaryPos;
    uint8_t switchValues;
    uint8_t regenPercent;            /**< @brief Integer percentage for regen. */
    uint8_t paddle;            /**< @brief Between 0 and 255 for paddle pos*/
    uint8_t LRUDButtons;     /**< @brief LRUD Button States, packed into an uint8_t*/
} cmr_canDIMActions_t;

/** @brief DIM sends message to acknowledge radio message
 * CDC rebroadcasts to DAQ Live.
*/
typedef struct {
    uint8_t acknowledge;
} cmr_canDIMAck_t;

// DIM Config Screen data
/** @brief Driver Interface Module config screen data. */

// these are all generic types. To modify what values are stored,
// modify the config_screen_helper.h file instead
typedef struct {
    uint8_t config_val_1;
    uint8_t config_val_2;
    uint8_t config_val_3;
    uint8_t config_val_4;
} cmr_canDIMCDCconfig_t;


// ------------------------------------------------------------------------------------------------
// Front Sensor Module

/** @brief Front Sensor Module data. */
typedef struct {
    uint8_t torqueRequested;            /**< @brief Torque requested (0-255). */
    uint8_t throttlePosition;           /**< @brief Throttle position (0-255). */
    uint8_t brakePressureFront_PSI;     /**< @brief Front brake pressure. */
    uint8_t brakePedalPosition;         /**< @brief Brake pedal position (0-255). */

    /** @brief Steering wheel angle (-180 to 180 degrees). */
    int16_t steeringWheelAngle_millideg;
} cmr_canFSMData_t;

/** @brief Front Sensor Module raw pedal positions. */
typedef struct {
    uint16_t throttleLeftADC;   /**< @brief Raw left throttle value. */
    uint16_t throttleRightADC;  /**< @brief Raw right throttle value. */
    uint16_t brakePedalADC;     /**< @brief Raw brake pedal value. */
} cmr_canFSMPedalsADC_t;

/** @brief Front Sensor Module raw sensors. */
typedef struct {
    uint16_t brakePressureFrontADC;     /**< @brief Raw brake pressure value. */
    uint16_t steeringWheelAngleADC;     /**< @brief Raw steering wheel value. */
} cmr_canFSMSensorsADC_t;

/** @brief Front Sensor Module power diagnostics. */
typedef struct {
    uint16_t busVoltage_mV;     /**< @brief Low-voltage bus voltage (mV). */
    uint16_t busCurrent_mA;     /**< @brief Low-voltage bus current (mA). */
} cmr_canFSMPowerDiagnostics_t;

// ------------------------------------------------------------------------------------------------
// Powertrain Thermal Controller

/** @brief Standard Git committed State. */
typedef struct {
	uint32_t commitHash;
	uint8_t dirtyFlash;
} cmr_canGitFlashStatus;

// ------------------------------------------------------------------------------------------------
// AMK Motor controller definitions.

/** @brief AMK motor controller status bits. */
typedef enum {
    CMR_CAN_AMK_STATUS_SYSTEM_READY = (1 << 8),     /**< @brief System ready. */
    CMR_CAN_AMK_STATUS_ERROR        = (1 << 9),     /**< @brief Error is present. */
    CMR_CAN_AMK_STATUS_WARNING      = (1 << 10),    /**< @brief Warning is present. */
    CMR_CAN_AMK_STATUS_HV_EN_ACK    = (1 << 11),    /**< @brief HV enabled acknowledgement. */
    CMR_CAN_AMK_STATUS_HV_EN        = (1 << 12),    /**< @brief HV enabled. */
    CMR_CAN_AMK_STATUS_INV_EN_ACK   = (1 << 13),    /**< @brief Inverter enabled acknowledgement. */
    CMR_CAN_AMK_STATUS_INV_EN       = (1 << 14),    /**< @brief Inverter enabled. */
    CMR_CAN_AMK_STATUS_DERATING_EN  = (1 << 15)     /**< @brief Protective torque derating enabled. */
} cmr_canAMKStatus_t;

/** @brief AMK motor controller control bits. */
typedef enum {
    CMR_CAN_AMK_CTRL_INV_ON     = (1 << 8),     /**< @brief Inverter on command. */
    CMR_CAN_AMK_CTRL_HV_EN      = (1 << 9),     /**< @brief HV enable command. */
    CMR_CAN_AMK_CTRL_INV_EN     = (1 << 10),    /**< @brief Inverter enable command. */
    CMR_CAN_AMK_CTRL_ERR_RESET  = (1 << 11)     /**< @brief Inverter error reset command. */
} cmr_canAMKControl_t;

/** @brief AMK motor controller status and velocity. */
typedef struct {
    uint16_t status_bv;         /**< @brief Status bit vector. See cmr_canAMKStatus_t. */
    int16_t velocity_rpm;       /**< @brief Motor velocity (RPM). */
    int16_t torqueCurrent_raw;  /**< @brief Raw value for torque producing current. */
    int16_t magCurrent_raw;     /**< @brief Raw value for magnetizing current. */
} cmr_canAMKActualValues1_t;

/** @brief AMK motor controller temperatures and error code. */
typedef struct {
    int16_t motorTemp_dC;       /**< @brief Motor temperature in dC (0.1 C). */
    int16_t coldPlateTemp_dC;   /**< @brief Cold plate temperature in dC (0.1 C). */
    uint16_t errorCode;         /**< @brief Inverter error code. */
    int16_t igbtTemp_dC;        /**< @brief IGBT temperature in dC (0.1 C). */
} cmr_canAMKActualValues2_t;

/** @brief AMK motor controller command message. */
typedef struct {
    uint16_t control_bv;        /**< @brief Control bit vector. See cmr_canAMKControl_t. */
    int16_t velocity_rpm;       /**< @brief Velocity setpoint (RPM). */
    int16_t torqueLimPos_dpcnt; /**< @brief Positive torque limit in 0.1% of 9.8 Nm (nominal torque). */
    int16_t torqueLimNeg_dpcnt; /**< @brief Negative torque limit in 0.1% of 9.8 Nm (nominal torque). */
} cmr_canAMKSetpoints_t;

// ------------------------------------------------------------------------------------------------
// Battery Management System

typedef struct {
    int32_t instantCurrent_mA; /**< @brief Instant Current (mA). */
    int32_t averageCurrent_mA; /**< @brief Average Current (mA). */
} cmr_canBMSPackCurrent_t;

typedef struct {
	uint8_t maxVoltIndex;        /**< @brief Max BMB cell voltage index. */
    uint8_t minVoltIndex;        /**< @brief Min BMB cell voltage index. */
    uint16_t maxCellVoltage_mV;  /**< @brief Max BMB cell voltage (mV). */
    uint16_t minCellVoltage_mV;  /**< @brief Min BMB cell voltage (mV). */
} cmr_canBMSBMBStatusVoltage_t;

typedef struct {
	uint8_t maxTempIndex;        /**< @brief Max BMB cell temp index. */
	uint8_t minTempIndex;        /**< @brief Min BMB cell temp index. */
    int16_t maxCellTemp_C;       /**< @brief Max BMB cell temp (C). */
    int16_t minCellTemp_C;       /**< @brief Min BMB cell temp (C). */
} cmr_canBMSBMBStatusTemp_t;

typedef struct {
	uint16_t minCellVoltage_mV;  /**< @brief Min pack cell voltage (mV). */
	uint16_t maxCellVoltage_mV;  /**< @brief Max pack cell voltage (mV). */
	uint8_t minVoltageBMBNum;    /**< @brief Min pack cell voltage BMB number. */
	uint8_t minVoltageCellNum;   /**< @brief Min pack cell voltage cell number. */
	uint8_t maxVoltageBMBNum;    /**< @brief Max pack cell voltage BMB number. */
	uint8_t maxVoltageCellNum;   /**< @brief Max pack cell voltage cell number. */
} cmr_canBMSMinMaxCellVoltage_t;

typedef struct {
    uint16_t minCellTemp_C;      /**< @brief Min pack cell temp (C). */
    uint16_t maxCellTemp_C;      /**< @brief Max pack cell temp (C). */
    uint8_t minTempBMBNum;       /**< @brief Min pack cell temp BMB number. */
    uint8_t minTempCellNum;      /**< @brief Min pack cell temp cell number. */
    uint8_t maxTempBMBNum;       /**< @brief Max pack cell temp BMB number. */
    uint8_t maxTempCellNum;      /**< @brief Max pack cell temp cell number. */
} cmr_canBMSMinMaxCellTemperature_t;

typedef struct {
    uint8_t vbatt_mV;       /**< @brief LV battery voltage (mV). */
    uint8_t vAIR_mV;        /**< @brief AIR voltage (mV). */
    uint8_t safety_mV;       /**< @brief Safety circuit voltage (mA). */
	uint8_t iDCDC_mA;       /**< @brief DCDC current (mA). */
} cmr_canBMSLowVoltage_t;

// BRUSA Charger Structs
// packed because BRUSA charger expects 7 bytes DLC
typedef struct __attribute__((__packed__)) {
    uint8_t enableVector;
    uint16_t maxMainsCurrent;
    uint16_t requestedVoltage;
    uint16_t requestedCurrent;
} cmr_canBRUSAChargerControl_t;

// SBG Systems INS definitions.

/** @brief SBG INS 'GENERAL_STATUS' bits. */
typedef enum {
    CMR_CAN_SBG_GENERAL_MAIN_POWER_OK   = (1 << 0),    /**< @brief Main power supply is OK. */
    CMR_CAN_SBG_GENERAL_IMU_POWER_OK    = (1 << 1),    /**< @brief IMU power supply is OK. */
    CMR_CAN_SBG_GENERAL_GPS_POWER_OK    = (1 << 2),    /**< @brief GPS power supply is OK. */
    CMR_CAN_SBG_GENERAL_SETTINGS_OK     = (1 << 3),    /**< @brief Settings were correctly loaded */
    CMR_CAN_SBG_GENERAL_TEMPERATURE_OK  = (1 << 4),    /**< @brief Temperature is within specified limits. */
    CMR_CAN_SBG_GENERAL_DATALOGGER_OK   = (1 << 5),    /**< @brief Data-logger is working correctly. */
    CMR_CAN_SBG_GENERAL_CPU_OK          = (1 << 6),    /**< @brief CPU headroom is good. */
} cmr_canSBGGeneralStatus_t;

/** @brief SBG INS 'SOLUTION_STATUS' bits. */
typedef enum {
    CMR_CAN_SBG_SOL_ATTITUDE_VALID  = (1 << 4),    /**< @brief Attitude data is reliable (Roll/Pitch error < 0.5°). */
    CMR_CAN_SBG_SOL_HEADING_VALID   = (1 << 5),    /**< @brief Heading data is reliable (Heading error < 1°). */
    CMR_CAN_SBG_SOL_VELOCITY_VALID  = (1 << 6),    /**< @brief Velocity data is reliable (velocity error < 1.5 m/s). */
    CMR_CAN_SBG_SOL_POSITION_VALID  = (1 << 7),    /**< @brief Position data is reliable (Position error < 10m). */
    CMR_CAN_SBG_SOL_VERT_REF_USED   = (1 << 8),    /**< @brief Vertical reference is used in solution (data used and valid since 3s). */
    CMR_CAN_SBG_SOL_MAG_REF_USED    = (1 << 9),    /**< @brief Magnetometer is used in solution (data used and valid since 3s). */
    CMR_CAN_SBG_SOL_GPS1_VEL_USED   = (1 << 10),   /**< @brief GPS velocity is used in solution (data used and valid since 3s). */
    CMR_CAN_SBG_SOL_GPS1_POS_USED   = (1 << 11),   /**< @brief GPS Position is used in solution (data used and valid since 3s). */
    CMR_CAN_SBG_SOL_GPS1_HDT_USED   = (1 << 13),   /**< @brief GPS True Heading is used in solution (data used and valid since 3s). */
    CMR_CAN_SBG_SOL_GPS2_VEL_USED   = (1 << 14),   /**< @brief GPS2 velocity is used in solution (data used and valid since 3s). */
    CMR_CAN_SBG_SOL_GPS2_POS_USED   = (1 << 15),   /**< @brief GPS2 Position is used in solution (data used and valid since 3s). */
    CMR_CAN_SBG_SOL_GPS2_HDT_USED   = (1 << 17),   /**< @brief GPS2 True Heading is used in solution (data used and valid since 3s). */
    CMR_CAN_SBG_SOL_ODO_USED        = (1 << 18),   /**< @brief Odometer is used in solution (data used and valid since 3s). */
    CMR_CAN_SBG_SOL_DVL_BT_USED     = (1 << 19),   /**< @brief DVL Bottom Tracking is used in solution (data used and valid since 3s). */
    CMR_CAN_SBG_SOL_DVL_WT_USED     = (1 << 20),   /**< @brief DVL Water Layer is used in solution (data used and valid since 3s). */
    CMR_CAN_SBG_SOL_USBL_USED       = (1 << 24),   /**< @brief USBL / LBL is used in solution (data used and valid since 3s). */
    CMR_CAN_SBG_SOL_PRESSURE_USED   = (1 << 25),   /**< @brief Pressure is used in solution (data used and valid since 3s). */
    CMR_CAN_SBG_SOL_ZUPT_USED       = (1 << 26),   /**< @brief ZUPT is used in solution (data used and valid since 3s). */
    CMR_CAN_SBG_SOL_ALIGN_VALID     = (1 << 27),   /**< @brief Sensor alignment and calibration parameters are valid. */
} cmr_canSBGSolutionStatus_t;

/** @brief SBG INS 'SOLUTION_STATUS' solution mode (first 4 bits) values. */
typedef enum {
    CMR_CAN_SBG_SOL_MODE_UNINITIALIZED = 0,     /**< @brief The Kalman filter is not initialized and the returned data are all invalid. */
    CMR_CAN_SBG_SOL_MODE_VERTICAL_GYRO = 1,     /**< @brief The Kalman filter only rely on a vertical reference to compute roll and
                                                            pitch angles. Heading and navigation data drift freely. */
    CMR_CAN_SBG_SOL_MODE_AHRS          = 2,     /**< @brief A heading reference is available, the Kalman filter provides full orientation
                                                            but navigation data drift freely. */
    CMR_CAN_SBG_SOL_MODE_NAV_VELOCITY  = 3,     /**< @brief The Kalman filter computes orientation and velocity. Position is freely
                                                            integrated from velocity estimation. */
    CMR_CAN_SBG_SOL_MODE_NAV_POSITION  = 4,     /**< @brief Nominal mode, the Kalman filter computes all parameters
                                                            (attitude, velocity, position). Absolute position is provided. */
} cmr_canSBGSolutionStatusMode_t;

/** @brief SBG Systems Status (part 1). */
typedef struct {
    uint32_t timestamp;         /**< @brief Timestamp in microseconds. */
    uint16_t general_status;    /**< @brief General status bit vector. */
    uint16_t clock_status;      /**< @brief Clock status bit vector. */
} cmr_canSBGStatus1_t;

/** @brief SBG Systems Status (part 2). */
typedef struct {
    uint32_t com_status;        /**< @brief Com status bit vector. */
    uint32_t aiding_status;     /**< @brief Aiding status bit vector. */
} cmr_canSBGStatus2_t;

/** @brief SBG Systems Status (part 3). */
typedef struct {
    uint32_t solution_status;   /**< @brief Solution status bit vector. */
    uint16_t heave_status;      /**< @brief Heave status bit vector. */
} cmr_canSBGStatus3_t;

/** @brief SBG Systems EKF Position. */
typedef struct {
    int32_t latitude;           /**< @brief Latitude (Degrees times 10^7). */
    int32_t longitude;          /**< @brief Longitude (Degrees times 10^7). */
} cmr_canSBGEKFPosition_t;

/** @brief SBG Systems EKF Euler Orientation. */
typedef struct {
    int16_t roll;               /**< @brief Car Roll (radians times 10^4). */
    int16_t pitch;              /**< @brief Car Pitch (radians times 10^4). */
    int16_t yaw;                /**< @brief Car Yaw (radians times 10^4). */
} cmr_canSBGEKFOrient_t;

/** @brief SBG Systems EKF Velocity. */
typedef struct {
    int16_t velocity_n;         /**< @brief Velocity in North Direction (m/s times 100). */
    int16_t velocity_e;         /**< @brief Velocity in East Direction (m/s times 100). */
    int16_t velocity_d;         /**< @brief Velocity in Down Direction (m/s times 100). */
} cmr_canSBGEKFVelocity_t;

/** @brief SBG Systems Body Velocity. */
typedef struct {
    int16_t velocity_forward;     /**< @brief Velocity in Car Forward Direction (m/s times 100). */
    int16_t velocity_right;       /**< @brief Velocity in Car Right Direction (m/s times 100). */
    int16_t velocity_down;        /**< @brief Velocity in Car Down Direction (m/s times 100). */
} cmr_canSBGBodyVelocity_t;

/** @brief SBG Systems IMU Acceleration. */
typedef struct {
    int16_t accel_x_mps2;         /**< @brief Acceleration in Car Forward Direction (m/s^2 times 100). */
    int16_t accel_y_mps2;         /**< @brief Acceleration in Car Right Direction (m/s^2 times 100). */
    int16_t accel_z_mps2;         /**< @brief Acceleration in Car Down Direction (m/s^2 times 100). */
} cmr_canSBGIMUAcceleration_t;

/** @brief SBG Systems IMU Gyro. */
typedef struct {
    int16_t gyro_x_rads;        /**< @brief Roll rate around the Car Forward Direction (rad/s times 1000). */
    int16_t gyro_y_rads;        /**< @brief Roll rate around the Car Right Direction (rad/s times 1000). */
    int16_t gyro_z_rads;        /**< @brief Roll rate around the Car Down Direction (rad/s times 1000). */
} cmr_canSBGIMUGyro_t;

/** @brief SBG Systems automotive data. */
typedef struct {
    int16_t angle_track_rad;        /**< @brief Track course angle/direction of travel (rad times 10^4). */
    int16_t angle_slip_rad;         /**< @brief Vehicle slip angle (rad times 10^4). */
    uint16_t curvature_radius_m;    /**< @brief Curvature radius based on down rotation rate (meters times 10^2). */
    uint8_t status;                 /**< @brief Status bitmasks as AUTO_STATUS definition. */
} cmr_canSBGAutomotive_t;

// Endianness hell.
typedef struct {
    uint8_t msb;
    uint8_t lsb;
} big_endian_16_t;

typedef struct {
    big_endian_16_t q0;
    big_endian_16_t q1;
    big_endian_16_t q2;
    big_endian_16_t q3;
} cmr_canMovellaQuaternion_t;

typedef struct {
    big_endian_16_t yaw;
    big_endian_16_t pitch;
    big_endian_16_t roll;
} cmr_canMovellaEulerAngles_t;

typedef struct {
    big_endian_16_t gyro_x;
    big_endian_16_t gyro_y;
    big_endian_16_t gyro_z;
} cmr_canMovellaIMUGyro_t;

typedef struct {
    big_endian_16_t accel_x;
    big_endian_16_t accel_y;
    big_endian_16_t accel_z;
} cmr_canMovellaIMUAccel_t;

typedef struct {
    big_endian_16_t vel_x;
    big_endian_16_t vel_y;
    big_endian_16_t vel_z;
} cmr_canMovellaVelocity_t;

typedef struct {
    int16_t cog_x;
    int16_t cog_y;
    float slip_angle;
} cmr_canCOGVelocity_t;

typedef struct {
    int16_t fl_x;
    int16_t fl_y;
    int16_t fr_x;
    int16_t fr_y;
} cmr_canFrontWheelVelocity_t;

typedef struct {
    int16_t rl_x;
    int16_t rl_y;
    int16_t rr_x;
    int16_t rr_y;
} cmr_canRearWheelVelocity_t;

// ------------------------------------------------------------------------------------------------
// IZZIE Racing sensors

/** @brief IZZIE Racing loadcell sensors. Big Endian*/
typedef struct {
    int16_t delta_voltage;        /**< @brief differential voltage in the wheatstone bridge */
    int16_t calibrated_output_f;  /**< @brief force output from the loadcell. */
    int16_t internal_temp;        /**< @brief amp's internal temp */
    int16_t external_temp;        /**< @brief amp's external temp */
} cmr_canIzzie_loadcell_raw_t;


/** @brief IZZIE Racing loadcell sensors. */
typedef struct {
    int16_t delta_voltage;        /**< @brief differential voltage in the wheatstone bridge */
    int16_t calibrated_output_f;  /**< @brief force output from the loadcell. */
    int16_t internal_temp;        /**< @brief amp's internal temp */
    int16_t external_temp;        /**< @brief amp's external temp */
} cmr_canIzzie_loadcell_calibrated_t;

// ------------------------------------------------------------------------------------------------
// Controls algo debugging struct

typedef struct {
    int16_t controls_elapsed_time;
    int16_t controls_sbg_speed_mps;
    int16_t controls_target_velocity;
    int16_t controls_target_accel;
} cmr_can_controls_debug_global_t;

typedef struct {
    int16_t controls_current_slip_FR;
    int16_t controls_slip_correction_active_FR;
    int16_t controls_wheel_speed_mps_actual_FR;
    int16_t controls_wheel_speed_mps_target_FR;
} cmr_can_controls_debug_FR_t;

typedef struct {
    int16_t controls_current_slip_FL;
    int16_t controls_slip_correction_active_FL;
    int16_t controls_wheel_speed_mps_actual_FL;
    int16_t controls_wheel_speed_mps_target_FL;
} cmr_can_controls_debug_FL_t;

typedef struct {
    int16_t controls_current_slip_RR;
    int16_t controls_slip_correction_active_RR;
    int16_t controls_wheel_speed_mps_actual_RR;
    int16_t controls_wheel_speed_mps_target_RR;
} cmr_can_controls_debug_RR_t;

typedef struct {
    int16_t controls_current_slip_RL;
    int16_t controls_slip_correction_active_RL;
    int16_t controls_wheel_speed_mps_actual_RL;
    int16_t controls_wheel_speed_mps_target_RL;
} cmr_can_controls_debug_RL_t;

typedef struct {
    int16_t controls_current_yaw_rate;
    int16_t controls_target_yaw_rate;
    int16_t controls_bias;
    int16_t controls_pid;
} cmr_can_controls_pid_debug_t;

typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t date;
    uint8_t month;
    uint8_t year;
    uint8_t err; /* 1 in error state and 0 otherwise */
} cmr_can_rtc_data_t;

typedef struct
{
    float slipRatio_FL;
    float slipRatio_FR;
} cmr_can_front_slip_ratio_data_t;

typedef struct
{
    float slipRatio_RL;
    float slipRatio_RR;
} cmr_can_rear_slip_ratio_data_t;

typedef struct
{
    float omega_FL;
    float omega_FR;
} cmr_can_front_whl_speed_setpoint_t;

typedef struct
{
    float omega_RL;
    float omega_RR;
} cmr_can_rear_whl_speed_setpoint_t;

typedef struct
{
    float v_whl_fl;
    float v_whl_fr;
} cmr_can_front_whl_velocity_t;

typedef struct
{
    float v_whl_rl;
    float v_whl_rr;
} cmr_can_rear_whl_velocity_t;

typedef struct
{
    float moment_req_Nm;
    float lin_accel_Nm;
} cmr_can_solver_inputs_t;

typedef struct
{
    int16_t combined_normalized_throttle;
    bool allow_regen;
    uint8_t placeholder[5];
} cmr_can_solver_aux_t;

typedef struct {
    uint16_t k_lin;
    uint16_t k_yaw;
    uint16_t k_tie;
} cmr_can_solver_settings_t;

// ------------------------------------------------------------------------------------------------
// SAE Provided EMD definitions

typedef struct {
    int32_t current;    /**< @brief Current (amps * 2^16). */
    int32_t voltage;    /**< @brief Voltage (volts * 2^16). */
} cmr_canEMDMeasurements_t;

// ------------------------------------------------------------------------------------------------
// DAQ Modules

typedef uint8_t cmr_canDAQTest_t; /** @brief DAQ Test type. MSB is to start/stop bits, rest are test id **/

typedef struct {
    uint16_t linpot_front_mm;       /**< @brief Front damper length in mm */
    uint16_t linpot_front_adc;      /**< @brief Front linpot ADC Value */
    uint16_t linpot_rear_mm;        /**< @brief Rear damper length in mm */
    uint16_t linpot_rear_adc;       /**< @brief Rear linpot ADC Value */
} cmr_canDAQLinpot_t;

typedef struct {
    int16_t differential_voltage_uv;
    int16_t force_output_N;
    int16_t internal_temp;
    int16_t external_temp;
} cmr_canIZZELoadCell_t;

typedef struct {
    uint8_t state;
} cmr_canMemoratorHeartbeat_t;

typedef struct {
	uint32_t test_id;
} cmr_canTestID_t;

#endif /* CMR_CAN_TYPES_H */