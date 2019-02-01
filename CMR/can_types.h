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
    CMR_CAN_GLV_ON,         /**< @brief Grounded low voltage on. */
    CMR_CAN_HV_EN,          /**< @brief High voltage enabled. */
    CMR_CAN_RTD,            /**< @brief Ready to drive. */
    CMR_CAN_ERROR,          /**< @brief Error has occurred. */
    CMR_CAN_CLEAR_ERROR,    /**< @brief Request to clear error. */
    CMR_CAN_UNKNOWN         /**< @brief Current state unknown. */
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
    /** @brief FSM brake pressure sensor out-of-range. */
    CMR_CAN_WARN_FSM_BPRES = (1 << 11),
    /** @brief FSM steering wheel angle out-of-range. */
    CMR_CAN_WARN_FSM_SWANGLE = (1 << 10)
} cmr_canWarn_t;

/** @brief Accumulator Fan Controller fan status. */
typedef struct {
    uint8_t acFanState[6];    /**< @brief Accumulator fan states. */
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
    uint8_t torqueRequested;    /**< @brief Torque requested (0-100). */
    uint8_t throttlePosition;   /**< @brief Throttle position (0-100). */
    uint8_t brakePressureFront;     /**< @brief Front brake pressure. */
    uint8_t brakePedalPosition;     /**< @brief Brake pedal position (0-100). */

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
    uint16_t auxiliaryADC;              /**< @brief Auxiliary value. */
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

#endif /* CMR_CAN_TYPES_H */

