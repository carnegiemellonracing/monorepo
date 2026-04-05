/**
 * @file motors_helper.c
 * @brief DTI quad-inverter helper.
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

// Macros
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)));
    
/**
 * @brief Retrieve ERPM, duty cycle, voltage for DTI inverter
 */
int32_t getDTIErpm(motorLocation_t motor) {
    switch (motor) {
        cmr_canDTI_TX_Erpm_t *dtiERPM;
        case MOTOR_FL:
            dtiERPM = canTractiveGetPayload(CANRX_TRAC_FL_ERPM);
            return big_endian_to_int32(&(dtiERPM->erpm));
        case MOTOR_FR:
            dtiERPM = canTractiveGetPayload(CANRX_TRAC_FR_ERPM);
            return big_endian_to_int32(&(dtiERPM->erpm));
        case MOTOR_RL:
            dtiERPM = canTractiveGetPayload(CANRX_TRAC_RL_ERPM);
            return big_endian_to_int32(&(dtiERPM->erpm));
        case MOTOR_RR:
            dtiERPM = canTractiveGetPayload(CANRX_TRAC_RR_ERPM);
            return big_endian_to_int32(&(dtiERPM->erpm));
        default:
            return NULL;
    }
}

/**
 * @brief Retrieve AC and DC current for DTI inverter
 */
volatile cmr_canDTI_TX_Current_t *getDTICurrent(motorLocation_t motor) {
    switch (motor) {
        case MOTOR_FL:
            return canTractiveGetPayload(CANRX_TRAC_FL_CURRENT);
        case MOTOR_FR:
            return canTractiveGetPayload(CANRX_TRAC_FR_CURRENT);
        case MOTOR_RL:
            return canTractiveGetPayload(CANRX_TRAC_RL_CURRENT);
        case MOTOR_RR:
            return canTractiveGetPayload(CANRX_TRAC_RR_CURRENT);
        default:
            return NULL;
    }
}

/**
 * @brief Retrieve controller/motor temperatures and fault code for DTI inverter
 */
volatile cmr_canDTI_TX_TempFault_t *getDTITempFault(motorLocation_t motor) {
    switch (motor) {
        case MOTOR_FL:
            return canTractiveGetPayload(CANRX_TRAC_FL_TEMPFAULT);
        case MOTOR_FR:
            return canTractiveGetPayload(CANRX_TRAC_FR_TEMPFAULT);
        case MOTOR_RL:
            return canTractiveGetPayload(CANRX_TRAC_RL_TEMPFAULT);
        case MOTOR_RR:
            return canTractiveGetPayload(CANRX_TRAC_RR_TEMPFAULT);
        default:
            return NULL;
    }
}

/**
 * @brief Retrieve Id, Iq values for DTI inverter
 */
volatile cmr_canDTI_TX_IdIq_t *getDTIIdIq(motorLocation_t motor) {
    switch (motor) {
        case MOTOR_FL:
            return canTractiveGetPayload(CANRX_TRAC_FL_IDIQ);
        case MOTOR_FR:
            return canTractiveGetPayload(CANRX_TRAC_FR_IDIQ);
        case MOTOR_RL:
            return canTractiveGetPayload(CANRX_TRAC_RL_IDIQ);
        case MOTOR_RR:
            return canTractiveGetPayload(CANRX_TRAC_RR_IDIQ);
        default:
            return NULL;
    }
}

/**
 * @brief Retrieve configured and available AC current limits for DTI inverter
 */
volatile cmr_canDTI_TX_ACLimits_t *getDTIACLimits(motorLocation_t motor) {
    switch (motor) {
        case MOTOR_FL:
            return canTractiveGetPayload(CANRX_TRAC_FL_ACLIMS);
        case MOTOR_FR:
            return canTractiveGetPayload(CANRX_TRAC_FR_ACLIMS);
        case MOTOR_RL:
            return canTractiveGetPayload(CANRX_TRAC_RL_ACLIMS);
        case MOTOR_RR:
            return canTractiveGetPayload(CANRX_TRAC_RR_ACLIMS);
        default:
            return NULL;
    }
}

/**
 * @brief Retrieve configured and available DC current limits for DTI inverter
 */
volatile cmr_canDTI_TX_DCLimits_t *getDTIDCLimits(motorLocation_t motor) {
    switch (motor) {
        case MOTOR_FL:
            return canTractiveGetPayload(CANRX_TRAC_FL_DCLIMS);
        case MOTOR_FR:
            return canTractiveGetPayload(CANRX_TRAC_FR_DCLIMS);
        case MOTOR_RL:
            return canTractiveGetPayload(CANRX_TRAC_RL_DCLIMS);
        case MOTOR_RR:
            return canTractiveGetPayload(CANRX_TRAC_RR_DCLIMS);
        default:
            return NULL;
    }
}

/**
 * @brief Retrieve control mode, target Iq, motor position, and still flag for DTI inverter
 */
volatile cmr_canDTI_TX_ControlStatus_t *getDTIControlStatus(motorLocation_t motor) {
    switch (motor) {
        case MOTOR_FL:
            return canTractiveGetPayload(CANRX_TRAC_FL_CONTROL_STATUS);
        case MOTOR_FR:
            return canTractiveGetPayload(CANRX_TRAC_FR_CONTROL_STATUS);
        case MOTOR_RL:
            return canTractiveGetPayload(CANRX_TRAC_RL_CONTROL_STATUS);
        case MOTOR_RR:
            return canTractiveGetPayload(CANRX_TRAC_RR_CONTROL_STATUS);
        default:
            return NULL;
    }
}

/**
 * @brief Retrieve throttle, brake, digital I/Os, drive enable, limit status for DTI inverter
 */
volatile cmr_canDTI_TX_IOStatus_t *getDTIIOStatus(motorLocation_t motor) {
    switch (motor) {
        case MOTOR_FL:
            return canTractiveGetPayload(CANRX_TRAC_FL_IO_STATUS);
        case MOTOR_FR:
            return canTractiveGetPayload(CANRX_TRAC_FR_IO_STATUS);
        case MOTOR_RL:
            return canTractiveGetPayload(CANRX_TRAC_RL_IO_STATUS);
        case MOTOR_RR:
            return canTractiveGetPayload(CANRX_TRAC_RR_IO_STATUS);
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
            return cmr_canRXMetaTimeoutWarn(&canTractiveRXMeta[CANRX_TRAC_FL_ERPM], xTaskGetTickCount()) >= 0;
        case MOTOR_FR:
            return cmr_canRXMetaTimeoutWarn(&canTractiveRXMeta[CANRX_TRAC_FR_ERPM], xTaskGetTickCount()) >= 0;
        case MOTOR_RL:
            return cmr_canRXMetaTimeoutWarn(&canTractiveRXMeta[CANRX_TRAC_RL_ERPM], xTaskGetTickCount()) >= 0;
        case MOTOR_RR:
            return cmr_canRXMetaTimeoutWarn(&canTractiveRXMeta[CANRX_TRAC_RR_ERPM], xTaskGetTickCount()) >= 0;
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
    return getDTIErpm(motor) / pole_pairs;
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
 *
 * @param motor the ID of the motor
 */
// float getMotorPower(motorLocation_t motor) {

//     const float dc_current_A = (float)(getDTICurrent(motor)->dc_current_dA) * 0.1;
//     const float voltage = (float)(getDTIErpm(motor)->input_voltage_V);
//     return dc_current_A * voltage;
// }

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
        volatile cmr_canHVSense_t *HVISense = canTractiveGetPayload(CANRX_TRAC_HVI_SENSE);
        float hvVoltage_V = ((float) HVISense->packVoltage_cV) / 100.f;
        voltage_V = (int32_t) fmaxf(battVoltage_V, hvVoltage_V);
    }

    // Returns false voltage is above threshold
    if (voltage_V >= upperVoltageLimit_V) {
        return false;
    }
    return true;
}

/**
 * @brief Performs linear interpolation between two points.
 *
 * @param torqueLower Lower bound torque value from the LUT.
 * @param torqueUpper Upper bound torque value from the LUT.
 * @param currLower Current corresponding to torqueLower.
 * @param currUpper Current corresponding to torqueUpper.
 * @param torque The torque value for which we want to estimate the current.
 * @return Interpolated current corresponding to the input torque (in deciAmps).
 */
int16_t transferFn(float torqueLower, float torqueUpper, 
                   int16_t currLower, int16_t currUpper, float torque){
  return (currUpper - currLower) * (torque - torqueLower) / 
         (torqueUpper - torqueLower) + currLower;
}

/**
 * @brief Converts a desired motor torque to the corresponding AC current using a LUT.
 *
 * @param torque_mNm Desired torque in mNm.
 * @return Corresponding AC current in deciAmps.
 */

int16_t torqueToCurrent(float torque_mNm){ 
    int16_t sign = 1;

    float torqueLower, torqueUpper;
    int16_t currLower, currUpper;

    float torque_Nm = torque_mNm / 1000.0f;

    /* Handle negative torque, convert to positive and preserve the sign */ 
    if (torque_Nm < 0) {
        sign = -1;
        torque_Nm = -torque_Nm; 
    }

    // temporary linear torque-current scaling
    torque_Nm = CLAMP(torque_Nm, 0.0f, maxTorque_Nm);
    int16_t current_dA = (int)(torque_Nm * current_torque_slope * 10.0f);
    return sign * current_dA;

    /* Finding the LUT intervals containing requested torque */
    int i;
    for (i = 0; i < DTI_TORQUE_CURRENT_LUT_LEN; i++) {
        if (torque_Nm <= DTI_torque_current_LUT[i].torque_Nm) {
            break;
        }
    }

    int16_t current;
    /* Edge case where torque is at torque maximum */
    if (i == DTI_TORQUE_CURRENT_LUT_LEN - 1) { 
        current = DTI_torque_current_LUT[i].current_Arms;
    } else {
        /* Else set bounds for interpoldation between LUT indices i and i+1 */
        torqueLower = DTI_torque_current_LUT[i].torque_Nm;
        torqueUpper = DTI_torque_current_LUT[i + 1].torque_Nm;
        currLower = DTI_torque_current_LUT[i].current_Arms;
        currUpper = DTI_torque_current_LUT[i + 1].current_Arms;
        current =  transferFn(torqueLower, torqueUpper, 
                                  currLower, currUpper, 
                                  torque_Nm); 
    }

    /* Perform linear interpolation, apply sign, and scale */
    return sign * 10 * current;
}

/**
 * @brief Converts measured  AC current (in deciAmps) to the corresponding torque using the LUT.
 *
 * @param current_dA Measured motor AC current in deciAmps.
 * @return Corresponding torque in mNm.
 */
float currentToTorque(int16_t current_dA){ 
    int16_t sign = 1;
    float torqueLower, torqueUpper;
    int16_t currLower, currUpper;

    if (current_dA < 0) {
        sign = -1;
        current_dA = -current_dA; 
    }

    /* Scale deciAmps back to Arms for LUT */
    float current_Arms = current_dA / 10.0f;

    current_Arms = CLAMP(current_Arms, DTI_torque_current_LUT[0].current_Arms,
                                   DTI_torque_current_LUT[DTI_TORQUE_CURRENT_LUT_LEN].current_Arms);

    int i;
    for (i = 0; i < DTI_TORQUE_CURRENT_LUT_LEN + 1; i++) {
        if (current_Arms <= DTI_torque_current_LUT[i].current_Arms) {
            break;
        }
    }

    if (i == DTI_TORQUE_CURRENT_LUT_LEN) { 
        currLower = DTI_torque_current_LUT[i - 1].current_Arms;
        currUpper = DTI_torque_current_LUT[i].current_Arms;
        torqueLower = DTI_torque_current_LUT[i - 1].torque_Nm;
        torqueUpper = DTI_torque_current_LUT[i].torque_Nm;
    } else {
        currLower = DTI_torque_current_LUT[i].current_Arms;
        currUpper = DTI_torque_current_LUT[i + 1].current_Arms;
        torqueLower = DTI_torque_current_LUT[i].torque_Nm;
        torqueUpper = DTI_torque_current_LUT[i + 1].torque_Nm;
    }

    return sign * 1000 * transferFn(currLower, currUpper, torqueLower, torqueUpper, current_Arms); 
}
