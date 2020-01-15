/**
 * @file can_types.h
 * @brief Shared CAN type definitions.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_CAN_TYPES_H
#define CMR_CAN_TYPES_H

#include <stdint.h>

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

/** @brief Fan states. */
typedef enum {
    CMR_CAN_FAN_OFF,    /**< @brief Fan turned off. */
    CMR_CAN_FAN_LOW,    /**< @brief Fan at low speed. */
    CMR_CAN_FAN_HIGH    /**< @brief Fan at high speed. */
} cmr_canFanState_t;

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

    /** @brief PTC fan current out-of-range. */
    CMR_CAN_ERROR_PTC_FAN_CURRENT = (1 << 15),
    /** @brief PTC fan/pump driver IC temperature out-of-range. */
    CMR_CAN_ERROR_PTC_DRIVERS_TEMP = (1 << 14),
    /** @brief PTC water temperature out-of-range. */
    CMR_CAN_ERROR_PTC_WATER_TEMP = (1 << 13)
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

    /** @brief VSM hasn't received HVC heartbeat for 25 ms. */
    CMR_CAN_WARN_VSM_HVC_TIMEOUT = (1 << 15),
    /** @brief VSM hasn't received CDC heartbeat for 25 ms. */
    CMR_CAN_WARN_VSM_CDC_TIMEOUT = (1 << 14),
    /** @brief VSM hasn't received FSM heartbeat for 25 ms. */
    CMR_CAN_WARN_VSM_FSM_TIMEOUT = (1 << 13),
    /** @brief VSM hasn't received PTC heartbeat for 25 ms. */
    CMR_CAN_WARN_VSM_PTC_TIMEOUT = (1 << 12),
    /** @brief VSM hasn't received DIM heartbeat for 25 ms. */
    CMR_CAN_WARN_VSM_DIM_TIMEOUT = (1 << 11),
    /** @brief VSM hasn't received AFC 0 heartbeat for 25 ms. */
    CMR_CAN_WARN_VSM_AFC0_TIMEOUT = (1 << 10),
    /** @brief VSM hasn't received AFC 1 heartbeat for 25 ms. */
    CMR_CAN_WARN_VSM_AFC1_TIMEOUT = (1 << 9),
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
    CMR_CAN_WARN_FSM_SWANGLE = (1 << 9)
} cmr_canWarn_t;

/** @brief Represents the car's current driving mode (gear). */
typedef enum {
    CMR_CAN_GEAR_UNKNOWN = 0,   /**< @brief Unknown Gear State */
    CMR_CAN_GEAR_REVERSE,       /**< @brief Reverse mode */
    CMR_CAN_GEAR_SLOW,          /**< @brief Slow mode */
    CMR_CAN_GEAR_FAST,          /**< @brief Fast simple mode */
    CMR_CAN_GEAR_ENDURANCE,     /**< @brief Endurance-event mode */
    CMR_CAN_GEAR_AUTOX,         /**< @brief Autocross-event mode */
    CMR_CAN_GEAR_SKIDPAD,       /**< @brief Skidpad-event mode */
    CMR_CAN_GEAR_ACCEL,         /**< @brief Acceleration-event mode */
    CMR_CAN_GEAR_TEST,          /**< @brief Test mode (for experimentation) */
    CMR_CAN_GEAR_LEN
} cmr_canGear_t;

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
    CMR_CAN_VSM_ERROR_SOURCE_HVC = (1 << 7),
    /** @brief At least one Central Dynamics Controller message has timed out. */
    CMR_CAN_VSM_ERROR_SOURCE_CDC = (1 << 6),
    /** @brief At least one Front Sensor Module message has timed out. */
    CMR_CAN_VSM_ERROR_SOURCE_FSM = (1 << 5),
    /** @brief At least one Powertrain Thermal Controller message has timed out. */
    CMR_CAN_VSM_ERROR_SOURCE_PTC = (1 << 4),
    /** @brief At least one Driver Interface Module message has timed out. */
    CMR_CAN_VSM_ERROR_SOURCE_DIM = (1 << 3),
    /** @brief At least one Accumulator Fan Controller 0 message has timed out. */
    CMR_CAN_VSM_ERROR_SOURCE_AFC0 = (1 << 2),
    /** @brief At least one Accumulator Fan Controller 1 message has timed out. */
    CMR_CAN_VSM_ERROR_SOURCE_AFC1 = (1 << 1),
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
typedef struct {
    uint16_t brakePressureRear_PSI;     /**< @brief Rear brake pressure (pounds-per-square-inch). */
    uint16_t hallEffect_dA;     /**< @brief Hall effect current (deci-Amps). */
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

/** @brief High Voltage Controller relay status bit vector definitions. */
typedef enum {
    CMR_CAN_HVC_RELAY_STATUS_DISCHARGE_CLOSED   = (1 << 0), /**<@ brief Discharge relay closed. */
    CMR_CAN_HVC_RELAY_STATUS_PRECHARGE_CLOSED   = (1 << 1), /**<@ brief Precharge relay closed. */
    CMR_CAN_HVC_RELAY_STATUS_AIR_NEG_CLOSED     = (1 << 2), /**<@ brief Negative AIR closed. */
    CMR_CAN_HVC_RELAY_STATUS_AIR_POS_CLOSED     = (1 << 3), /**<@ brief Positive AIR closed. */
    CMR_CAN_HVC_RELAY_STATUS_DISCHARGE_ERROR    = (1 << 4), /**<@ brief Discharge error. */
    CMR_CAN_HVC_RELAY_STATUS_PRECHARGE_ERROR    = (1 << 5), /**<@ brief Precharge error. */
    CMR_CAN_HVC_RELAY_STATUS_AIR_NEG_ERROR      = (1 << 6), /**<@ brief Negative AIR error. */
    CMR_CAN_HVC_RELAY_STATUS_AIR_POS_ERROR      = (1 << 7), /**<@ brief Positive AIR error. */
} cmr_canHVCRelayStatus_t;

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

/** @brief High Voltage Controller pack voltages. */
typedef struct {
    int32_t battVoltage;    /**< @brief voltage measured across battery. */
    int32_t hvVoltage;      /**< @brief voltage outside accumulator. */
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

// ------------------------------------------------------------------------------------------------
// Accumulator Fan Controller

/** @brief Accumulator Fan Controller fan status. */
typedef struct {
    uint8_t acFanState;    /**< @brief Accumulator fan states. */
    uint8_t dcdcFanState;   /**< @brief DCDC fan state. */
} cmr_canAFCFanStatus_t;

/** @brief Accumulator Fan Controller temperatures. */
typedef struct {
    uint8_t driverTemp_C[3];    /**< @brief Driver IC temperatures (C). */
    uint8_t dcdcTemp_C[2];      /**< @brief DCDC temperatures (C). */
} cmr_canAFCDriverTemps_t;

/** @brief Accumulator Fan Controller power diagnostics. */
typedef struct {
    uint16_t busVoltage_mV;     /**< @brief Low-voltage bus voltage (mV). */
    uint16_t busCurrent_mA;     /**< @brief Low-voltage bus current (mA). */
    uint16_t fansCurrent_mA;    /**< @brief Total fans current (mA). */
} cmr_canAFCPowerDiagnostics_t;

// ------------------------------------------------------------------------------------------------
// Central Dynamics Controller

/** @brief Central Dynamics Controller wheel speeds. */
typedef struct {
    uint16_t frontLeft;     /**< @brief Front left wheel speed (RPM). */
    uint16_t frontRight;    /**< @brief Front right wheel speed (RPM). */
    uint16_t backLeft;      /**< @brief Back left wheel speed (RPM). */
    uint16_t backRight;     /**< @brief Back right wheel speed (RPM). */
} cmr_canCDCWheelSpeeds_t;

/** @brief Central Dynamics Controller PTC brake solenoid command. */
typedef struct {
    uint8_t solenoidEnable;     /**< @brief Enable the solenoid (disable the brakes). */
} cmr_canCDCSolenoidPTC_t;

/** @brief Central Dynamics Controller motor data. */
typedef struct {
    int16_t torque_dNm;     /**< @brief Commanded torque (deci-Newton-meters). */
    int16_t speed_rpm;      /**< @brief Motor speed (RPM). */
    int16_t current_dA;     /**< @brief DC bus current (deci-Amps). */
    int16_t voltage_dV;     /**< @brief DC bus voltage (deci-Volts). */
} cmr_canCDCMotorData_t;

/** @brief Central Dynamics Controller motor temperatures. */
typedef struct {
    int16_t motorTemp_dC;           /**< @brief Motor temperature (deci-Celsius). */
    int16_t mcMaxInternalTemp_dC;   /**< @brief Max measured internal MC temperature (deci-Celsius). */
} cmr_canCDCMotorTemps_t;

/** @brief Central Dynamics Controller motor faults. */
typedef struct {
    uint32_t post;  /**< @brief Power-on-self-test faults. */
    uint32_t run;   /**< @brief Run faults. */
} cmr_canCDCMotorFaults_t;

/** @brief Central Dynamics Controller motor phase currents. */
typedef struct {
    int16_t phaseA; /**< @brief Current in the phase A cable (Amps * 10). */
    int16_t phaseB; /**< @brief Current in the phase B cable (Amps * 10). */
    int16_t phaseC; /**< @brief Current in the phase C cable (Amps * 10). */
} cmr_canCDCMotorPhaseCurrents_t;

/** @brief Central Dynamics Controller IMU accelerations. */
typedef struct {
    int16_t longitudinal;   /**< @brief Longitudinal Acceleration where full scale is +/- 2g (positive Forward). */
    int16_t lateral;        /**< @brief Lateral Acceleration where full scale is +/- 2g (positive Left). */
    int16_t vertical;       /**< @brief Vertical Acceleration where full scale is +/- 2g (positive Down). */
} cmr_canCDCIMUAcceleration_t;

// ------------------------------------------------------------------------------------------------
// Driver Interface Module

/** @brief Driver Interface Module state/gear request. */
typedef struct {
    uint8_t requestedState;     /**< @brief Requested state. */
    uint8_t requestedGear;      /**< @brief Requested gear. */
} cmr_canDIMRequest_t;

/** @brief Driver Interface Module power diagnostics. */
typedef struct {
    uint16_t busVoltage_mV;     /**< @brief Low-voltage bus voltage (mV). */
    uint16_t busCurrent_mA;     /**< @brief Low-voltage bus current (mA). */
} cmr_canDIMPowerDiagnostics_t;

// ------------------------------------------------------------------------------------------------
// Front Sensor Module

/** @brief Front Sensor Module data. */
typedef struct {
    uint8_t torqueRequested;            /**< @brief Torque requested (0-255). */
    uint8_t throttlePosition;           /**< @brief Throttle position (0-255). */
    uint8_t brakePressureFront_PSI;     /**< @brief Front brake pressure. */
    uint8_t brakePedalPosition_pcnt;    /**< @brief Brake pedal position (0-100). */

    /** @brief Steering wheel angle (-180 to 180 degrees). */
    int16_t steeringWheelAngle_deg;
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

/** @brief Water cooling pump state enumeration. */
typedef enum {
    CMR_CAN_PTC_PUMP_STATE_OFF = 0, /**< @brief Pump disabled. */
    CMR_CAN_PTC_PUMP_STATE_ON       /**< @brief Pump enabled. */
} cmr_canPTCPumpState_t;

/** @brief Powertrain Thermal Controller cooling status. */
typedef struct {
    uint8_t fanState;             /**< @brief Radiator fan state. */
    uint8_t pumpState;            /**< @brief Radiator water pump state. */
    uint16_t preRadiatorTemp_dC;  /**< @brief Pre-radiator water temperature (.1 C). */
    uint16_t postRadiatorTemp_dC; /**< @brief Post-radiator water temperature (.1 C). */
} cmr_canPTCCoolingStatus_t;

/** @brief Powertrain Thermal Controller voltage diagnostics. */
typedef struct {
    uint16_t logicVoltage_mV;   /**< @brief Logic voltage (mV). */
    uint16_t loadVoltage_mV;    /**< @brief Load voltage (mV). */
} cmr_canPTCVoltageDiagnostics_t;

/** @brief Powertrain Thermal Controller current diagnostics. */
typedef struct {
    uint16_t logicCurrent_mA;   /**< @brief Logic current (mA). */
    uint16_t loadCurrent_mA;    /**< @brief Load current (mA). */
    uint16_t fanCurrent_mA;     /**< @brief Fan current (mA). */
} cmr_canPTCCurrentDiagnostics_t;

/** @brief Powertrain Thermal Controller accumulator fan duty cycles. */
typedef struct {
    uint8_t acFansDuty_pcnt;    /**< @brief Accumulator fan duty cycle. */
    uint8_t dcdcFanDuty_pcnt;   /**< @brief DCDC fan duty cycle. */
} cmr_canPTCAFCControl_t;

// ------------------------------------------------------------------------------------------------
// AMK Motor controller definitions.

/** @brief AMK motor controller status bits. */
typedef enum {
    CMR_CAN_AMK_STATUS_SYSTEM_READY = (1 << 8), /**< @brief System ready. */
    CMR_CAN_AMK_STATUS_ERROR = (1 << 9),        /**< @brief Error is present. */
    CMR_CAN_AMK_STATUS_WARNING = (1 << 10),     /**< @brief Warning is present. */
    CMR_CAN_AMK_STATUS_HV_EN_ACK = (1 << 11),   /**< @brief HV enabled acknowledgement. */
    CMR_CAN_AMK_STATUS_HV_EN = (1 << 12),       /**< @brief HV enabled. */
    CMR_CAN_AMK_STATUS_INV_EN_ACK = (1 << 13),  /**< @brief Inverter enabled acknowledgement. */
    CMR_CAN_AMK_STATUS_INV_EN = (1 << 14),      /**< @brief Inverter enabled. */
    CMR_CAN_AMK_STATUS_DERATING_EN = (1 << 15)  /**< @brief Protective torque derating enabled. */
} cmr_canAMKStatus_t;

/** @brief AMK motor controller control bits. */
typedef enum {
    CMR_CAN_AMK_CTRL_INV_ON = (1 << 8),     /**< @brief Inverter on command. */
    CMR_CAN_AMK_CTRL_HV_EN = (1 << 9),      /**< @brief HV enable command. */
    CMR_CAN_AMK_CTRL_INV_EN = (1 << 10),    /**< @brief Inverter enable command. */
    CMR_CAN_AMK_CTRL_ERR_RESET = (1 << 11)  /**< @brief Inverter error reset command. */
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

#endif /* CMR_CAN_TYPES_H */

