/**
 * @file safety_filter.c
 * @brief including relevant functions from motors.c to test safety filter
 *
 * @author Carnegie Mellon Racing- Ayush Garg and Anna Paek
 */

// ------------------------------------------------------------------------------------------------
// Includes

#include "motors.h"         // Interface to implement
#include "motors_helper.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <limits.h>
#include <CMR/can_types.h>  // CMR CAN types
#include <CMR/config_screen_helper.h>
#include <CMR/fir_filter.h>
#include "controls_23e.h"
#include "drs_controls.h"
#include "servo.h"
#include "can.h"
#include "daq.h"
#include "safety_filter.h"

// ------------------------------------------------------------------------------------------------
// Constants

/** @brief Max amount of power the car should draw
 *  @warning Safety margins MUST BE STRICTLY POSITIVE
 */
float power_upper_limit_W = 80000.0f; //michigan power limit
float power_safety_margin_W = 14000.0f;

/** @brief Max temperature of the hottest cell
 *  @warning Safety margins MUST BE STRICTLY POSITIVE
 */
static const float temperature_safety_margin_C = 5.0f;
static const float temperature_upper_limit_C = 59.0f;

/** @brief Max/Min voltage of the whole pack
 *  @warning Safety margins MUST BE STRICTLY POSITIVE
 */
static const float pack_voltage_upper_limit_V = 590.0f;
static const float pack_voltage_lower_limit_V = 340.0f; // only used in last-line-of-defense checks
static const float pack_voltage_safety_margin_V = 30.0f;

/** @brief Max/Min voltage of the worst cell (the cell with the highest DCIR)
 *  @warning Safety margins MUST BE STRICTLY POSITIVE
 */
static const float cell_voltage_upper_limit_V = 4.35f;
static const float cell_voltage_lower_limit_V = 2.5f;
static const float cell_voltage_safety_margin_V = 0.2f;

/** @brief DCIR of the worst cell (the cell with the highest DCIR)
 *  @note Only used for preemptive limiting
 */
static const float worst_dcir_Ohms = 13.2f * 1e-3f; // 13.2 miliohms

/** @brief Power efficiency of the inverter, only used for preemptive limiting */
static const float inverter_efficiency = 0.98f; // the rated efficiency of the inverter

/** @brief Power efficiency of the motor, only used for preemptive limiting */
static const float motor_efficiency = 0.9086f; // the highest efficiency in the motor's efficiency table

/** @brief This value is multiplied to total efficiency for the purposes of preemptive limiting */
static const float efficiency_multiplier = 0.9f;

/** @brief This value is added to the sum of abs(omega) for the purposes of preemptive limiting
 *  @warning Must be NON-NEGATIVE
*/
static const float wheel_speed_sum_offset = 0.0f; // set to 0 to obtain a tighter lower bound on power

/** @brief Threshold of the break pressure for it to be considered engaged
 *  @warning Also used by the safety filter in motors.c
*/
const float braking_threshold_psi = 40.0f;

/** @brief This value is added to braking_threshold_psi when checking whether or not breaks are engaged */
static const float braking_threshold_offset_psi = 10.0f; // increase threshold by 10 PSI for drivers that leave their foot on the break pedal

/** @brief Threshold of the vehicle speed below which regen-braking is not allowed */
static const float regen_speed_threshold_kmph = 0.0f;



// ********* FIR Filters *********

/** @brief Torque multiplier filter length  */
#define TORQUE_MULTIPLIER_FILTER_LEN 6

/** @brief Torque multiplier filter coefficients */
static const float torque_multiplier_filter_coefficients[TORQUE_MULTIPLIER_FILTER_LEN] = {
    0.04042214435,
    0.1635863605,
    0.2959914951,
    0.2959914951,
    0.1635863605,
    0.04042214435
};

/** @brief Torque multiplier filter buffers */
static float accel_torque_multiplier_filter_buf[TORQUE_MULTIPLIER_FILTER_LEN];
static float regen_torque_multiplier_filter_buf[TORQUE_MULTIPLIER_FILTER_LEN];

/** @brief Torque multiplier filter states */
static cmr_fir_filter_state_t accel_torque_multiplier_filter_state;
static cmr_fir_filter_state_t regen_torque_multiplier_filter_state;


// ------------------------------------------------------------------------------------------------
// Globals

// ********* SF CAN Messages *********

static cmr_canCDCSafetyFilterStates_t sf_state = {
    .power_limit_max_violation_W = 0.0f,
    .longest_power_violation_ms = 0,
    .over_voltage_count = 0,
    .under_voltage_count = 0,
    .over_temp_count = 0
};

static cmr_canCDCMotorPower_t motorPower_state = {
    .motor_power_FL = 0,
    .motor_power_FR = 0,
    .motor_power_RL = 0,
    .motor_power_RR = 0
};

// ------------------------------------------------------------------------------------------------
// Private functions

/**
 * @brief Convert normalized violation to a linear falloff factor
 *
 * @param normalized_violation violation divided by safety margin, MUST BE NON-NEGATIVE
 * @return falloff factor, between 0 and 1 (inclusive)
 */
static float getLinearFalloffFactor (
    float normalized_violation
) {
    normalized_violation = fmaxf(normalized_violation, 0.0f); // ensures normalized_violation >= 0, handles INF and NAN
    return fmaxf(1.0f - normalized_violation, 0.0f);
}

/**
 * @brief Convert normalized violation to a quadratic falloff factor
 *
 * @param normalized_violation violation divided by safety margin, MUST BE NON-NEGATIVE
 * @return falloff factor, between 0 and 1 (inclusive)
 */
static float getQuadraticFalloffFactor (
    float normalized_violation
) {
    normalized_violation = fmaxf(normalized_violation, 0.0f); // ensures normalized_violation >= 0, handles INF and NAN
    const float normalized_violation_squared = normalized_violation * normalized_violation; // quadratic falloff
    return fmaxf(1.0f - normalized_violation_squared, 0.0f);
}

/**
 * @brief A generic function to calculate falloff factor by an upper limit
 *
 * @param measured_value the measurement of a value to be limited
 * @param hard_upper_limit the hard upper limit above which the falloff factor is 0
 * @param safety_margin the range in which quadratic falloff happens, MUST BE STRICTLY POSITIVE
 */
static float getFalloffFactorByUpperLimit (
    float measured_value,
    float hard_upper_limit,
    float safety_margin
) {
    safety_margin = fmaxf(safety_margin, 0.0f); // ensures safety_margin >= 0; note that safety_margin should be > 0
    const float soft_upper_limit = hard_upper_limit - safety_margin;
    const float violation = fmaxf(measured_value - soft_upper_limit, 0.0f); // ensures violation >= 0
    const float normalized_violation = violation / safety_margin; // INF or NAN if safety_margin == 0
    return getQuadraticFalloffFactor(normalized_violation);
}

/**
 * @brief A generic function to calculate falloff factor by a lower limit
 *
 * @param measured_value the measurement of a value to be limited
 * @param hard_lower_limit the hard lower limit below which the falloff factor is 0
 * @param safety_margin the range in which quadratic falloff happens, MUST BE STRICTLY POSITIVE
 */
static float getFalloffFactorByLowerLimit (
    float measured_value,
    float hard_lower_limit,
    float safety_margin
) {
    safety_margin = fmaxf(safety_margin, 0.0f); // ensures safety_margin >= 0; note that safety_margin should be > 0
    const float soft_lower_limit = hard_lower_limit + safety_margin;
    const float violation = fmaxf(soft_lower_limit - measured_value, 0.0f); // ensures violation >= 0
    const float normalized_violation = violation / safety_margin; // INF or NAN if safety_margin == 0
    return getQuadraticFalloffFactor(normalized_violation);
}

/**
 * @brief Calculates the falloff factor by pack power
 * @warning This is CRITICAL for rule-compliance, as it imposes the upper limit on pack power
 *
 * @param measured_pack_power_W the measured power of the pack
 */
static float getPackPowerFalloffFactor (
    float measured_pack_power_W
) {
    return getFalloffFactorByUpperLimit(measured_pack_power_W, power_upper_limit_W, power_safety_margin_W);
}

/**
 * @brief Calculates the falloff factor by pack voltage rise
 * @warning This is CRITICAL for rule-compliance, as it imposes the upper limit on pack voltage
 *
 * @param measured_pack_voltage_V the measured voltage of the pack
 */
static float getPackVoltageRiseFalloffFactor (
    float measured_pack_voltage_V
) {
    return getFalloffFactorByUpperLimit(measured_pack_voltage_V, pack_voltage_upper_limit_V, pack_voltage_safety_margin_V);
}

/**
 * @brief Calculates the falloff factor by cell voltage rise
 * @note This is not critical for rule-compliance, but is useful for protecting the cells
 *
 * @param measured_max_cell_voltage_V the measured voltage of cell with the highest voltage
 */
static float getCellVoltageRiseFalloffFactor (
    float measured_max_cell_voltage_V
) {
    return getFalloffFactorByUpperLimit(measured_max_cell_voltage_V, cell_voltage_upper_limit_V, cell_voltage_safety_margin_V);
}

/**
 * @brief Calculates the falloff factor by cell voltage drop
 * @note This is not critical for rule-compliance, but is useful for protecting the cells
 *
 * @param measured_min_cell_voltage_V the measured voltage of the cell with the lowest voltage
 */
static float getCellVoltageDropFalloffFactor (
    float measured_min_cell_voltage_V
) {
    return getFalloffFactorByLowerLimit(measured_min_cell_voltage_V, cell_voltage_lower_limit_V, cell_voltage_safety_margin_V);
}

/**
 * @brief Calculates the falloff factor by cell temperature
 * @warning This is CRITICAL for rule-compliance, as it imposes the upper limit on cell temperature
 */
static float getTemperatureFalloffFactor() {
    volatile cmr_canHVCPackMinMaxCellTemps_t *cellTemps = canVehicleGetPayload(CANRX_VEH_PACK_CELL_TEMP);
    const float maxCellTemp_C = ((float)(cellTemps->maxCellTemp_dC)) * 1e-1f;
    return getFalloffFactorByUpperLimit(maxCellTemp_C, temperature_upper_limit_C, temperature_safety_margin_C);
}

static float getMaxVoltageDrop (
    float measured_min_cell_voltage_V,
    float measured_current_through_worst_cell_A
) {
    // Compensate for the voltage across the parasitic resistor of the worst cell
    const float zero_current_voltage_of_worst_cell = measured_min_cell_voltage_V + measured_current_through_worst_cell_A * worst_dcir_Ohms;
    return fmaxf(zero_current_voltage_of_worst_cell - (cell_voltage_lower_limit_V + cell_voltage_safety_margin_V), 0.0f); // always non-negative
    // When we draw current from the AC, its voltage drops. We don't want the voltage to be too low, so we need to upper-limit the voltage drop
}

static float getMinVoltageDrop (
    float measured_max_cell_voltage_V,
    float measured_current_through_worst_cell_A
) {
    // Compensate for the voltage across the parasitic resistor of the worst cell
    const float zero_current_voltage_of_worst_cell = measured_max_cell_voltage_V + measured_current_through_worst_cell_A * worst_dcir_Ohms;
    return fminf(zero_current_voltage_of_worst_cell - (cell_voltage_upper_limit_V - cell_voltage_safety_margin_V), 0.0f); // always non-positive
    // When we pump current into the AC, its voltage rises. We don't want the voltage to be too high, so we need to lower-limit the voltage drop (negative voltage rise)
}

static float getMaxPowerFromVoltageDrop (
    float max_voltage_drop_V,
    float pack_voltage_V
) {
    return fminf(power_upper_limit_W - power_safety_margin_W, pack_voltage_V * max_voltage_drop_V / worst_dcir_Ohms);
    // For voltage to drop by max_voltage_drop_V, the current would be max_voltage_drop_V / worst_dcir_Ohms
    // Therefore, max power should be pack_voltage_V * max_voltage_drop_V / worst_dcir_Ohms
    // Also, we don't want max power to be above power_upper_limit_W - power_safety_margin_W
}

static float getMinPowerFromVoltageDrop (
    float min_voltage_drop_V,
    float pack_voltage_V
){
    return pack_voltage_V * (min_voltage_drop_V / worst_dcir_Ohms);
    // For voltage to drop by min_voltage_drop_V, the current would be min_voltage_drop_V / worst_dcir_Ohms
    // Therefore, min power (max negative power) should be pack_voltage_V * min_voltage_drop_V / worst_dcir_Ohms
}



// ------------------------------------------------------------------------------------------------
// Public functions


/** @brief Initialize FIR filters for retroactive limiting */
void initRetroactiveLimitFilters() {
    cmr_fir_filter_init (
        &accel_torque_multiplier_filter_state,
        accel_torque_multiplier_filter_buf,
        torque_multiplier_filter_coefficients,
        TORQUE_MULTIPLIER_FILTER_LEN,
        1.0f
    );

    cmr_fir_filter_init (
        &regen_torque_multiplier_filter_state,
        regen_torque_multiplier_filter_buf,
        torque_multiplier_filter_coefficients,
        TORQUE_MULTIPLIER_FILTER_LEN,
        1.0f
    );
}

/** @brief Reset FIR filters for retroactive limiting */
void resetRetroactiveLimitFilters() {
    cmr_fir_filter_reset(&accel_torque_multiplier_filter_state, 1.0f);
    cmr_fir_filter_reset(&regen_torque_multiplier_filter_state, 1.0f);
}


/**
 * @brief Calculates torque upper- and lower-limits for preemptive limiting.
 *
 * @return The torque upper- and lower-limits for a motor, which applies to every motor.
 */
cmr_torque_limit_t getPreemptiveTorqueLimits
() {
    const float max_power_W = power_upper_limit_W - power_safety_margin_W;
    const float measured_total_motor_speed = getTotalMotorSpeed_radps() + wheel_speed_sum_offset;

    // legacy cell-voltage-based preemptive limiting
//    const float max_voltage_drop_V = getMaxVoltageDrop(min_cell_voltage_V, pack_current_A);
//    const float min_voltage_drop_V = getMinVoltageDrop(max_cell_voltage_V, pack_current_A);
//    const float max_power_W = getMaxPowerFromVoltageDrop(max_voltage_drop_V, pack_voltage_V);
//    const float min_power_W = getMinPowerFromVoltageDrop(min_voltage_drop_V, pack_voltage_V);

    cmr_torque_limit_t torque_limits;
    const float efficiency = inverter_efficiency * motor_efficiency * efficiency_multiplier;
    torque_limits.max_torque = max_power_W * efficiency / measured_total_motor_speed;
    torque_limits.min_torque = -maxTorque_Nm;

    // bound preemptive limits by the motor's output range
    torque_limits.max_torque = fminf(torque_limits.max_torque, maxTorque_Nm);
    torque_limits.min_torque = fmaxf(torque_limits.min_torque, -maxTorque_Nm);

    return torque_limits;
}

/** @brief Returns whether or not the throttle is considered to be inactive */
static bool throttleNeutral() {
    volatile cmr_canFSMData_t *dataFSM = canVehicleGetPayload(CANRX_VEH_DATA_FSM);
    // Using torqueRequested instead of throttlePosition because FSM performs some checks
    return dataFSM->torqueRequested == 0;
}

/** @brief Returns whether or not the brakes is considered to be active */
static bool mechanicalBrakesEngaged() {
    volatile cmr_canVSMSensors_t *vsmSensor = canVehicleGetPayload(CANRX_VEH_VSM_SENSORS);
    // volatile cmr_canFSMData_t *dataFSM = canVehicleGetPayload(CANRX_VEH_DATA_FSM);

    const float brakepsi = (float)(vsmSensor->brakePressureRear_PSI);
    return brakepsi >= braking_threshold_psi + braking_threshold_offset_psi;
}

/** @brief Returns whether or not the car if fast enough to regen-break */
static bool fastEnoughToRegen() {
    const float vehicle_speed_low_mps = motorSpeedToWheelLinearSpeed_mps(getMinMotorSpeed_radps());
    const float vehicle_speed_low_kmph = vehicle_speed_low_mps * 3.6f; // 1m/s = 3.6km/h
    return vehicle_speed_low_kmph > regen_speed_threshold_kmph;
}

/**
 * @brief Sets both positive and negative torque limits for all motors with over/undervolt protection.
 * @note The Safety Filter (SF) is implemented here.
 * @warning This function should be called at a rate close to 200Hz due to its use of FIR filters
 *
 * @param torquesPos_Nm Max torques: upper-bounds the torque SF sends to each motor. NULL will be treated as zeros. MUST BE NON-NEGATIVE!
 * @param torquesNeg_Nm Min torques: lower-bounds the torque SF sends to each motor. NULL will be treated as zeros. MUST BE NON-POSITIVE!
 * @note The SF may decide to send any torque within the limits specified by torqueLimPosDist_Nm and torqueLimNegDist_Nm.
 */
void setTorqueLimsProtected (
    const cmr_torqueDistributionNm_t *torquesPos_Nm,
    const cmr_torqueDistributionNm_t *torquesNeg_Nm
) {
    // ********* Local Parameters *********

    /** @brief Limit torque preemptively based on motor and AC models
     *  @warning Preemptive limiting is not yest thoroughly validated
     */
    static const bool apply_preemptive_limits = true;

    // ********* Time Keeping *********

    const TickType_t current_time = xTaskGetTickCount();
    static TickType_t last_power_non_violation_time = 0; // for measuring limit violation durations

    // ********* Preemptive Limiting *********

    const float pack_voltage_V = getPackVoltage();
    const float pack_current_A = getPackCurrent();
    const float pack_power_W = pack_voltage_V * pack_current_A;

    const float max_cell_voltage_V = getMaxCellVoltage();
    const float min_cell_voltage_V = getMinCellVoltage();
    cmr_torque_limit_t preemptive_torque_limits = getPreemptiveTorqueLimits();

    // ********* Retroactive Limiting *********

    // limit power
    const float falloff_factor_by_pack_power = getPackPowerFalloffFactor(pack_power_W);
    if (falloff_factor_by_pack_power == 0.0f) { // measured power over hard limit
        const float power_limit_violation_W = pack_current_A * pack_voltage_V - power_upper_limit_W;
        sf_state.power_limit_max_violation_W = fmaxf(sf_state.power_limit_max_violation_W, power_limit_violation_W);

        // update the duration of the longest power hard limit violation
        if (last_power_non_violation_time != 0 && current_time >= last_power_non_violation_time) { // last_power_non_violation_time is valid
            TickType_t power_violation_duration = current_time - last_power_non_violation_time;
            sf_state.longest_power_violation_ms = (uint8_t)min(power_violation_duration, 255); // fit into uint8_t
        }
    }
    else { // measured power not over hard limit
        last_power_non_violation_time = current_time;
    }

    // limit pack voltage
    const float falloff_factor_by_pack_voltage_rise = getPackVoltageRiseFalloffFactor(pack_voltage_V);

    // limit cell voltage
    const float falloff_factor_by_cell_voltage_rise = getCellVoltageRiseFalloffFactor(max_cell_voltage_V);
    const float falloff_factor_by_cell_voltage_drop = getCellVoltageDropFalloffFactor(min_cell_voltage_V);

    // limit temperature
    const float falloff_factor_by_temperature = getTemperatureFalloffFactor();

    // compute torque multipliers
    const float accel_torque_multiplier = falloff_factor_by_pack_power * falloff_factor_by_cell_voltage_drop ;// falloff_factor_by_temperature;
    const float regen_torque_multiplier = falloff_factor_by_cell_voltage_rise;// * falloff_factor_by_temperature;

    float filtered_accel_torque_multiplier = accel_torque_multiplier;
    filtered_accel_torque_multiplier = cmr_fir_filter_update(&accel_torque_multiplier_filter_state, accel_torque_multiplier);
    filtered_accel_torque_multiplier = fmaxf(filtered_accel_torque_multiplier, 0.0f); // ensures filtered_accel_torque_multiplier >= 0
//    if(accel_torque_multiplier == 1.0f){
//    	filtered_accel_torque_multiplier = 1.0f;
//    }
    filtered_accel_torque_multiplier = fminf(filtered_accel_torque_multiplier, 1.0f); // ensures filtered_accel_torque_multiplier <= 1

    float filtered_regen_torque_multiplier = regen_torque_multiplier;
    filtered_regen_torque_multiplier = cmr_fir_filter_update(&regen_torque_multiplier_filter_state, regen_torque_multiplier);
    filtered_regen_torque_multiplier = fmaxf(filtered_regen_torque_multiplier, 0.0f); // ensures filtered_regen_torque_multiplier >= 0

//   if(regen_torque_multiplier == 1.0f){
//    	filte red_regen_torque_multiplier = 1.0f;
//    }
    filtered_regen_torque_multiplier = fminf(filtered_regen_torque_multiplier, 1.0f); // ensures filtered_regen_torque_multiplier <= 1

    // apply limits
    float all_wheels_final_max_torque = maxTorque_Nm;
    float all_wheels_final_min_torque = -maxTorque_Nm;

    if (apply_preemptive_limits) {
        all_wheels_final_max_torque = fminf(all_wheels_final_max_torque, preemptive_torque_limits.max_torque);
        all_wheels_final_min_torque = fmaxf(all_wheels_final_min_torque, preemptive_torque_limits.min_torque);
    }

    all_wheels_final_max_torque *= filtered_accel_torque_multiplier;
    all_wheels_final_min_torque *= filtered_regen_torque_multiplier;

    // set positive torque to 0 if either throttle is neutral or mechanical breaks are engaged
    if (throttleNeutral() || mechanicalBrakesEngaged()) {
        all_wheels_final_max_torque = 0.0f;
    }

    // set negative torque to 0 if the vehicle is not fast enough for regen-braking
    //if (!fastEnoughToRegen()) {
    //    all_wheels_final_min_torque = 0.0f;
    //}

    // ********* Applying Limits *********

    for (size_t motor = 0; motor < MOTOR_LEN; motor++) {
        /** @note When limits are active, different wheels may spin at different speeds on jacks.
         * This is likely due to differences in friction, as no issues that can cause this are found yet.
        */

        // ********* Handle Requested Torque *********

        const float requested_max_torque = fmaxf(getTorqueNmByIndex(torquesPos_Nm, motor), 0.0f); // ensures requested_max_torque >= 0
        const float requested_min_torque = fminf(getTorqueNmByIndex(torquesNeg_Nm, motor), 0.0f); // ensures requested_min_torque <= 0

        float final_max_torque = fminf(all_wheels_final_max_torque, requested_max_torque);
        float final_min_torque = fmaxf(all_wheels_final_min_torque, requested_min_torque);

        // ********* Prevent Reverse *********

        const float omega = getMotorSpeed_radps(motor);

        // overrides final max and min torque if omega is negative
        if (omega < 0.0f) {
            // in regen because omega is negative, so re-calculate max torque based on regen_torque_multiplier
            final_max_torque = fminf(requested_max_torque, maxTorque_Nm) * regen_torque_multiplier;
            final_min_torque = 0.0f; // if wheel speed is negative, min torque limit is 0 because we don't want wheel speed to become more negative
        }

        // ********* Last-Line-of-Defense Safety Checks *********

        final_max_torque = fmaxf(final_max_torque, 0.0f); // ensures final_max_torque >= 0
        final_max_torque = fminf(final_max_torque, maxTorque_Nm); // ensure final_max_torque <= 21Nm
        final_min_torque = fminf(final_min_torque, 0.0f); // ensures final_min_torque <= 0
        final_min_torque = fmaxf(final_min_torque, -maxTorque_Nm); // ensure final_min_torque >= -21Nm

        if (pack_power_W > power_upper_limit_W) {
            final_max_torque = 0.0f; // redundant with retroactive limiting, but keep for extra safety
        }

        if (pack_current_A > 0.0f || final_max_torque > 0.0f) { // discharging
            if (pack_voltage_V < pack_voltage_lower_limit_V) {
                sf_state.under_voltage_count++;
                final_max_torque = 0.0f; // redundant with retroactive limiting, but keep for extra safety
            }
        }

        if (pack_current_A < 0.0f || final_min_torque < 0.0f) { // chargine
            if (pack_voltage_V > pack_voltage_upper_limit_V) {
                sf_state.over_voltage_count++;
                final_min_torque = 0.0f; // redundant with retroactive limiting, but keep for extra safety
            }
        }

        volatile cmr_canHVCPackMinMaxCellTemps_t *cellTemps = canVehicleGetPayload(CANRX_VEH_PACK_CELL_TEMP);
//        const float maxCellTemp_C = ((float)(cellTemps->maxCellTemp_dC)) * 1e-1f;
//        if (maxCellTemp_C > temperature_upper_limit_C) {
//            sf_state.over_temp_count++;
//            final_max_torque = 0.0f; // redundant with retroactive limiting, but keep for extra safety
//        }

        setTorqueLimsUnprotected(motor, final_max_torque, final_min_torque); // apply torque limits
    }

    // send motor powers over CAN for debugging
    motorPower_state.motor_power_FL = falloff_factor_by_cell_voltage_rise;//getMotorPower(MOTOR_FL, pack_voltage_V);
    motorPower_state.motor_power_FR = falloff_factor_by_cell_voltage_drop;//getMotorPower(MOTOR_FR, pack_voltage_V);
    motorPower_state.motor_power_RL = falloff_factor_by_temperature; //getMotorPower(MOTOR_RL, pack_voltage_V);
    motorPower_state.motor_power_RR = falloff_factor_by_pack_power; getMotorPower(MOTOR_RR, pack_voltage_V);
}

const cmr_canCDCSafetyFilterStates_t *getSafetyFilterInfo(){
    return (const cmr_canCDCSafetyFilterStates_t*) &sf_state;
}

const cmr_canCDCMotorPower_t *getMotorPowerInfo(){
    return (const cmr_canCDCMotorPower_t*) &motorPower_state;
}

void setPowerLimit(uint8_t limit) {
	power_upper_limit_W = limit*1000.0f;
	power_safety_margin_W = power_upper_limit_W*0.17f;
}
