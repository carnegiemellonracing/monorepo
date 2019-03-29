/**
 * @file can_types.h
 * @brief Shared CAN type definitions.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_CAN_TYPES_H
#define CMR_CAN_TYPES_H

#include <stdint.h>

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
    CMR_CAN_FAN_OFF,
    CMR_CAN_FAN_LOW,
    CMR_CAN_FAN_HIGH
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

typedef enum {
    GEAR_UNKNOWN = 0,   /**< @brief Unknown Gear State */
    GEAR_REVERSE,       /**< @brief Reverse mode */
    GEAR_SLOW,          /**< @brief Slow mode */
    GEAR_FAST,          /**< @brief Fast simple mode */
    GEAR_ENDURANCE,     /**< @brief Endurance-event mode */
    GEAR_AUTOX,         /**< @brief Autocross-event mode */
    GEAR_SKIDPAD,       /**< @brief Skidpad-event mode */
    GEAR_ACCEL,         /**< @brief Acceleration-event mode */
    GEAR_TEST,          /**< @brief Test mode (for experimentation) */
    GEAR_LEN
} cmr_canGear_t;

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

/** @brief Front Sensor Module data. */
typedef struct {
    uint8_t torqueRequested_pcnt;    /**< @brief Torque requested (0-100). */
    uint8_t throttlePosition_pcnt;   /**< @brief Throttle position (0-100). */
    uint8_t brakePressureFront_PSI;  /**< @brief Front brake pressure. */
    uint8_t brakePedalPosition_pcnt; /**< @brief Brake pedal position (0-100). */

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

/** @brief Powertrain Thermal Controller cooling status. */
typedef struct {
    uint8_t fanState;   /**< @brief Radiator fan state. */
    uint8_t pumpState;  /**< @brief Radiator water pump state. */

    /** @brief Pre-radiator water temperature (C). */
    uint16_t preRadiatorTemp_C;

    /**< @brief Post-radiator water temperature (C). */
    uint16_t postRadiatorTemp_C;
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

typedef struct {
    uint8_t acFansDrive;    /**< @brief Accumulator fan drive signal. */
    uint8_t dcdcFanDrive;   /**< @brief DCDC fan drive signal. */
} cmr_canPTCAFCControl_t;

//
// Rinehart Motor Controller Definitions
//

/** @brief Drive motor with parameters. */
typedef struct {
    int16_t torqueCommand; /**< @brief Torque in N.m. times 10. */
    int16_t speedCommand; /**< @brief Angular velocity in RPM. */
    /** @brief 0 -> CW, 1 -> CCW viewed looking at shaft side. */
    uint8_t directionCommand; 
    /** @brief bit 0 = inverter enable, bit 1 = discharge enable. */	
    uint8_t inverterEnableDischargeSpeedMode;	
    /** @brief Set 0 for no limit override (torque in N.m. times 10). */
    uint16_t torqueLimitCommand; 
} cmr_canRMSCommand_t;

/** @brief Faults report from motor controller (see pg 23). */
typedef struct {
    uint16_t postFaultLo; /**< @brief See "RMS CAN Protocol" pg 23. */
    uint16_t postFaultHi; /**< @brief See "RMS CAN Protocol" pg 23. */
    uint16_t runFaultLo; /**< @brief See "RMS CAN Protocol" pg 23. */
    uint16_t runFaultHi; /**< @brief See "RMS CAN Protocol" pg 23. */
} cmr_canRMSFaults_t;

/** @brief Motor controller temperatures (set A). Temp in degC times 10. */
typedef struct {
    int16_t moduleATemp; /**< @brief Temp in internal module A (degC times 10). */
    int16_t moduleBTemp; /**< @brief Temp in internal module B (degC times 10). */
    int16_t moduleCTemp; /**< @brief Temp in internal module C (degC times 10). */
    int16_t gateDriverBoardTemp; /**< @brief Temp of gate driver (degC times 10). */
} cmr_canRMSTempA_t;

/** @brief Motor controller temperatures (set B). Temp in degC times 10. */
typedef struct {
    int16_t controlBoardTemp; /**< @brief Control board temp (degC times 10). */
    int16_t RTD1Temp; /**< @brief RTD input 1 temp (degC times 10). */
    int16_t RTD2Temp; /**< @brief RTD input 2 temp (degC times 10). */
    int16_t RTD3Temp; /**< @brief RTD input 3 temp (degC times 10). */
} cmr_canRMSTempB_t;

/** @brief Motor controller temperatures (set C). Temp in degC times 10. */
typedef struct {
    int16_t RTD4Temp; /**< @brief RTD input 4 temp (degC times 10). */
    int16_t RTD5Temp; /**< @brief RTD input 5 temp (degC times 10). */
    int16_t motorTemp; /**< @brief Motor temp (degC times 10). */
    int16_t torqueShudder; /**< @brief Torque (N.m. times 10). */
} cmr_canRMSTempC_t;

/** @brief Motor position information. */
typedef struct {
    int16_t angle; /**< @brief Angle (deg times 10) of motor as read by resolver. */
    int16_t speed; /**< @brief Speed (RPM) of motor. */
    int16_t frequency; /**< @brief Inverter frequency (Hz times 10). */
    int16_t resolverAngle; /**< @brief Resolver angle for calibration. */
} cmr_canRMSMotorPosition_t;

/** @brief Motor controller measured currents. Current in amps times 10.*/
typedef struct {
    int16_t phaseA; /**< @brief Current in phase A cable. */
    int16_t phaseB; /**< @brief Current in phase B cable. */
    int16_t phaseC; /**< @brief Current in phase C cable. */
    int16_t bus; /**< @brief DC bus current. */
} cmr_canRMSCurrents_t;

/** @brief Motor controller measured voltages. Voltage in volts times 10.*/
typedef struct {
    int16_t bus; /**< @brief DC bus voltage. */
    int16_t output; /**< @brief Output voltage as peak line-neutral volts. */
    int16_t phaseAB; /**< @brief Vab when disabled, Vd when enabled. */
    int16_t phaseBC; /**< @brief Vbc when disabled, Vq when enabled. */
} cmr_canRMSVoltages_t;

/** @brief Motor controller torque diagnostics. */
typedef struct {
    /** @brief Setpoint torque (torque in N.m. times 10). */
    int16_t commandedTorque; 
    /** @brief Measured torque produced (torque in N.m. times 10). */
    int16_t torqueFeedback; 
    uint32_t powerOnTimer;
} cmr_canRMSTorqueDiag_t;

#endif /* CMR_CAN_TYPES_H */

