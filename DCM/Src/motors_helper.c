/**
 * @file motors_helper.c
 * @brief AMK quad-inverter helper.
 *
 * @author Carnegie Mellon Racing
 */

#include "motors_helper.h"
#include "constants.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include "can.h"
#include "controls_helper.h"

// ------------------------------------------------------------------------------------------------
// Constants

/** @brief AMK Torque command increment (Newton-meters). */
const float torqueIncrement_Nm = 0.0098f;

/** @brief Constant factor for transforming motor RPM to induced voltage.
 *
 *  @details See motor manual page 37
 */
const float rpm_to_mV_factor = 0.026587214972f;

/**
 * @brief Converts floating point torque into AMK format
 *        (0.1% increments of 9.8 Nm).
 *
 * @param torque_Nm Torque as a floating point value.
 *
 * @return Torque in AMK format.
 */
int16_t convertNmToAMKTorque (float torque_Nm) {
    // clamp torque to the motor's output range
    torque_Nm = fminf(maxTorque_Nm, torque_Nm);
    torque_Nm = fmaxf(-maxTorque_Nm, torque_Nm);

    return (int16_t) (torque_Nm / torqueIncrement_Nm);
}
/**
 * @brief Retrieve actual values 1 (ERPM, duty cycle, voltage) for DTI inverter
 */
volatile cmr_canDTIActualValues1_t *getDTIActualValues1(motorLocation_t motor) {
    switch (motor) {
        case MOTOR_FL:
            return canTractiveGetPayload(CMR_CANID_DTI_FL_ACT_1);
        case MOTOR_FR:
            return canTractiveGetPayload(CMR_CANID_DTI_FR_ACT_1);
        case MOTOR_RL:
            return canTractiveGetPayload(CMR_CANID_DTI_RL_ACT_1);
        case MOTOR_RR:
            return canTractiveGetPayload(CMR_CANID_DTI_RR_ACT_1);
        default:
            return NULL;
    }
}

/**
 * @brief Retrieve actual values 2 (AC and DC current) for DTI inverter
 */
volatile cmr_canDTIActualValues2_t *getDTIActualValues2(motorLocation_t motor) {
    switch (motor) {
        case MOTOR_FL:
            return canTractiveGetPayload(CMR_CANID_DTI_FL_ACT_2);
        case MOTOR_FR:
            return canTractiveGetPayload(CMR_CANID_DTI_FR_ACT_2);
        case MOTOR_RL:
            return canTractiveGetPayload(CMR_CANID_DTI_RL_ACT_2);
        case MOTOR_RR:
            return canTractiveGetPayload(CMR_CANID_DTI_RR_ACT_2);
        default:
            return NULL;
    }
}

/**
 * @brief Retrieve actual values 3 (temperatures, fault code) for DTI inverter
 */
volatile cmr_canDTIActualValues3_t *getDTIActualValues3(motorLocation_t motor) {
    switch (motor) {
        case MOTOR_FL:
            return canTractiveGetPayload(CMR_CANID_DTI_FL_ACT_3);
        case MOTOR_FR:
            return canTractiveGetPayload(CMR_CANID_DTI_FR_ACT_3);
        case MOTOR_RL:
            return canTractiveGetPayload(CMR_CANID_DTI_RL_ACT_3);
        case MOTOR_RR:
            return canTractiveGetPayload(CMR_CANID_DTI_RR_ACT_3);
        default:
            return NULL;
    }
}

/**
 * @brief Retrieve actual values 4 (Id, Iq) for DTI inverter
 */
volatile cmr_canDTIActualValues4_t *getDTIActualValues4(motorLocation_t motor) {
    switch (motor) {
        case MOTOR_FL:
            return canTractiveGetPayload(CMR_CANID_DTI_FL_ACT_4);
        case MOTOR_FR:
            return canTractiveGetPayload(CMR_CANID_DTI_FR_ACT_4);
        case MOTOR_RL:
            return canTractiveGetPayload(CMR_CANID_DTI_RL_ACT_4);
        case MOTOR_RR:
            return canTractiveGetPayload(CMR_CANID_DTI_RR_ACT_4);
        default:
            return NULL;
    }
}

/**
 * @brief Check if motor's actual values are fresh
 *
 * @param motor Which motor to retrieve value for.
 */
bool isMotorDataValid(motorLocation_t motor) {
    switch (motor) {
        case MOTOR_FL:
            return cmr_canRXMetaTimeoutWarn(&canTractiveRXMeta[CANRX_TRAC_INV_FL_ACT1], xTaskGetTickCount()) >= 0;
        case MOTOR_FR:
            return cmr_canRXMetaTimeoutWarn(&canTractiveRXMeta[CANRX_TRAC_INV_FR_ACT1], xTaskGetTickCount()) >= 0;
        case MOTOR_RL:
            return cmr_canRXMetaTimeoutWarn(&canTractiveRXMeta[CANRX_TRAC_INV_RL_ACT1], xTaskGetTickCount()) >= 0;
        case MOTOR_RR:
            return cmr_canRXMetaTimeoutWarn(&canTractiveRXMeta[CANRX_TRAC_INV_RR_ACT1], xTaskGetTickCount()) >= 0;
        default:
            return false;
    }
}

/** @brief Convert rpm to rad/s*/
float rpmToRadps(float rpm) {
    static const float radps_per_rpm = 2.0f * (float)M_PI / 60.0f;
    return rpm * radps_per_rpm;
}

/** @brief Convert motor speed (in rad/s) to wheel linear speed (in m/s) */
float motorSpeedToWheelLinearSpeed_mps(float motor_speed_radps) {
    return motor_speed_radps / gear_ratio * effective_wheel_rad_m;
}

/**
 * @brief Retrieve RPM for motor.
 *
 * @param motor Which motor to retrieve value for.
 */
int16_t getMotorSpeed_rpm(motorLocation_t motor) {
    return getMotorActualValues1(motor)->velocity_rpm;
}

/**
 * @brief Retrieve speed in rad/s for motor.
 *
 * @param motor Which motor to retrieve value for.
 */
float getMotorSpeed_radps(motorLocation_t motor) {
    return rpmToRadps((float)(getMotorSpeed_rpm(motor)));
}

/** @brief Returns the sum of motor speeds in rad/s, ignoring negative speeds */
float getTotalMotorSpeed_radps() {
    float total_speed_radps = 0.0f;
    for (size_t motor = 0; motor < MOTOR_LEN; motor++){
        total_speed_radps += fmaxf(getMotorSpeed_radps(motor), 0.0f); // ignore negative motor speeds
    }
    return total_speed_radps;
}

/** @brief Returns the sum of motor speeds in rpm, ignoring negative speeds */
float getTotalMotorSpeed_rpm() {
    float total_speed_rpm = 0.0f;
    for (size_t motor = 0; motor < MOTOR_LEN; motor++){
        total_speed_rpm += fmaxf(getMotorSpeed_rpm(motor), 0.0f); // ignore negative motor speeds
    }
    return total_speed_rpm;
}

/** @brief Returns the minimum motor speed in rad/s, ignoring negative speeds */
float getMinMotorSpeed_radps() {
    float min_speed_radps = INFINITY;
    for (size_t motor = 0; motor < MOTOR_LEN; motor++){
        min_speed_radps = fminf(min_speed_radps, getMotorSpeed_radps(motor));
    }
    return min_speed_radps;
}

/**
 * @brief Returns the power in watts of a given motor.
 * Uses inverter-reported magnetization current and torque current to calculate total current,
 * which is multiplied with pack voltage to calculate power
 * @bug This is incorrect because the output voltage of the inverters, that is the voltage across the motors, is not the AC voltage
 *
 * @param motor the ID of the motor
 * @param pack_voltage_V the pack voltage
 */
float getMotorPower(motorLocation_t motor, float pack_voltage_V) {
    static const float RAW_CURRENT_TO_A = 0.00654297f;

    const float torque_current_A = (float)(getMotorActualValues1(motor)->torqueCurrent_raw) * RAW_CURRENT_TO_A;
    const float magnetization_current_A = (float)(getMotorActualValues1(motor)->magCurrent_raw) * RAW_CURRENT_TO_A;

    const float currentMagnitude_A = hypotf(torque_current_A, magnetization_current_A);
    return currentMagnitude_A * pack_voltage_V;
    /** @bug This is incorrect because the output voltage of the inverters, that is the voltage across the motors, is not the AC voltage */
}

/** @brief Get a motor load from cmr_loadDistribution_t */
float getLoadByIndex(const cmr_loadDistribution_t *loads, size_t motor) {
    if (loads == NULL) {
        return 1.0f;
    }
    float load = 0.0f;
    switch (motor) {
        case MOTOR_FL:
            load = loads->fl;
            break;
        case MOTOR_FR:
            load = loads->fr;
            break;
        case MOTOR_RL:
            load = loads->rl;
            break;
        case MOTOR_RR:
            load = loads->rr;
            break;
        default:
            load = 0.0f;
            break;
    }
    return fmaxf(load, 0.0f);
}

/** @brief Get a torque from cmr_torqueDistributionNm_t */
float getTorqueNmByIndex(const cmr_torqueDistributionNm_t *torques_Nm, size_t motor) {
    if (torques_Nm == NULL) {
        return 0.0f;
    }
    switch (motor) {
        case MOTOR_FL:
            return torques_Nm->fl;
        case MOTOR_FR:
            return torques_Nm->fr;
        case MOTOR_RL:
            return torques_Nm->rl;
        case MOTOR_RR:
            return torques_Nm->rr;
        default:
            return 0.0f;
    }
}

/** @brief get the max torque in cmr_torqueDistributionNm_t */
float getTorqueNmMax(const cmr_torqueDistributionNm_t *torques_Nm) {
    if (torques_Nm == NULL) {
        return 0.0f;
    }
    return fmaxf(fmaxf(torques_Nm->fl, torques_Nm->fr), fmaxf(torques_Nm->rl, torques_Nm->rr));
}

/** @brief get the min torque in cmr_torqueDistributionNm_t */
float getTorqueNmMin(const cmr_torqueDistributionNm_t *torques_Nm) {
    if (torques_Nm == NULL) {
        return 0.0f;
    }
    return fminf(fminf(torques_Nm->fl, torques_Nm->fr), fminf(torques_Nm->rl, torques_Nm->rr));
}

/**
 * @brief Given a motor RPM, return the maximum-power regenerative torque possible
 *
 * @param motor Which motor to retrieve value for.
 */
float getMotorRegenerativeCapacity(int32_t rpm) {
    static const float RPM_TO_LOWER_LIMIT = -0.02621878;

    if(rpm <= 0) {
        return 0.0f;
    }
    
    float limit = rpm * RPM_TO_LOWER_LIMIT;
    if (limit < -maxTorque_continuous_stall_Nm) {
        limit = -maxTorque_continuous_stall_Nm;
    }

    if (overVoltProtection()) {
        return limit;
    }

    return 0.0f;
}

/**
 * @brief returns true if AC voltage is under ~590
 *
 * Need to test
 */
bool overVoltProtection() {
    // TODO: Use closed loop control to determine upper voltage
    const int32_t upperVoltageLimit_V = 570;

    volatile cmr_canHVCPackVoltage_t *HVCPackVoltage = canVehicleGetPayload(CANRX_VEH_VOLTAGE_HVC);
    float battVoltage_V = ((float) HVCPackVoltage->battVoltage_mV) / 1000.f;

    int32_t voltage_V = battVoltage_V;

    // If HVI Sense has timed out, use batt voltage
    if (cmr_canRXMetaTimeoutError(&canTractiveRXMeta[CANRX_TRAC_HVI_SENSE], xTaskGetTickCount()) == 0) {
        // Otherwise, use HV Voltage
        volatile cmr_canHVIHeartbeat_t *HVISense = canTractiveGetPayload(CANRX_TRAC_HVI_SENSE);
        float hvVoltage_V = ((float) HVISense->packVoltage_cV) / 100.f;
        voltage_V = (int32_t) fmaxf(battVoltage_V, hvVoltage_V);
    }

    // Returns false voltage is above threshold
    if (voltage_V >= upperVoltageLimit_V) {
        return false;
    }
    return true;
}

int *setVelocity(cmr_canAMKSetpoints_t *motor_setpoints){
    return &(motor_setpoints->velocity_rpm);
} 

int *setTorqueLimPos(cmr_canAMKSetpoints_t *motor_setpoints){
    return &motor_setpoints->torqueLimPos_dpcnt;
}

int *setTorqueLimNeg(cmr_canAMKSetpoints_t *motor_setpoints){
    return &motor_setpoints->torqueLimNeg_dpcnt;
}

