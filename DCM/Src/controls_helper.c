#include "constants.h"

#include "controls_helper.h"
#include "lut_3d.h"
#include "safety_filter.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "motors.h"

/** @brief  min brake pressure for starting to apply regen */
const uint16_t brake_pressure_start = 50;

/** @brief  min paddle pressure for starting to apply regen */
const uint8_t paddle_pressure_start = 30;

// ------------------------------------------------------------------------------------------------
// Functions

/** @brief returns the pack voltage */
float getPackVoltage() {
    volatile cmr_canHVCPackVoltage_t *voltages = canVehicleGetPayload(CANRX_VEH_VOLTAGE_HVC);
    const float batt_voltage_V = ((float)(voltages->battVoltage_mV)) * 1e-3f; // convert to volts

    volatile cmr_canHVIHeartbeat_t *HVISense = canVehicleGetPayload(CANRX_TRAC_HVI_SENSE);
    const float hv_voltage_V = ((float)(HVISense->packVoltage_cV)) * 1e-2f; // convert to volts

    // If HVI Sense hasn't timed out, use it. Otherwise, use batt voltage
    float measured_voltage_V = batt_voltage_V;
    // if (cmr_canRXMetaTimeoutError(&canTractiveRXMeta[CANRX_TRAC_HVI_SENSE], xTaskGetTickCount()) == 0) {
    //     measured_voltage_V = hv_voltage_V;
    // }

    return measured_voltage_V;
}

/** @brief returns the pack current */
float getPackCurrent() {
//    volatile cmr_canVSMSensors_t *vsmSensor = canVehicleGetPayload(CANRX_VEH_VSM_SENSORS);
//    return ((float)(vsmSensor->hallEffect_cA)) * 1e-2f; // convert to amps
	volatile cmr_canHVIHeartbeat_t *HVISense = canVehicleGetPayload(CANRX_HVI_SENSE);
	return (((float)(HVISense->packCurrent_dA)) * 1e-1f); // convert to amps
}

/** @brief returns the pack power measured by HVISense */
float getHVISensePackPower() {
    volatile cmr_canHVIHeartbeat_t *HVISense = canTractiveGetPayload(CANRX_HVI_SENSE);
    const int32_t pack_power_mW = ((int32_t)(HVISense->packCurrent_dA)) * ((int32_t)(HVISense->packVoltage_cV));
    return ((float)pack_power_mW) * 1e-3f; // convert to watts
}

/** @brief returns the voltage of the cell that has the highest voltage */
float getMaxCellVoltage() {
    volatile cmr_canHVCPackMinMaxCellVolages_t *minMaxCellVoltages = canVehicleGetPayload(CANRX_VEH_PACK_CELL_VOLTAGE);
    return minMaxCellVoltages->maxCellVoltage_mV * 1e-3f; // convert to volts
}

/** @brief returns the voltage of the cell that has the lowest voltage */
float getMinCellVoltage() {
    volatile cmr_canHVCPackMinMaxCellVolages_t *minMaxCellVoltages = canVehicleGetPayload(CANRX_VEH_PACK_CELL_VOLTAGE);
    return minMaxCellVoltages->minCellVoltage_mV * 1e-3f; // convert to volts
}

/**
 * @brief Convert steering wheel angle to steering angle (the orientation of the front wheels)
 */
float swAngleMillidegToSteeringAngleRad(int32_t swAngle_millideg) {
    float steering_angle_deg = ((float)swAngle_millideg); // convert steering wheel angle into steering angle SIKE BITCHED ALREADY GOT STEERING ANGLE
    float steering_angle_rad = steering_angle_deg * 0.001f * M_PI / 180.0f;
    return  steering_angle_rad; // convert to rads
}

bool setPaddleRegen(uint8_t *throttlePos_u8, uint16_t brakePressurePsi_u8, int32_t avgMotorSpeed_RPM, uint8_t paddle_pressure, uint8_t paddle_regen_strength) {
    if (paddle_pressure < paddle_pressure_start) return false;

    float paddle_request = ((float)(paddle_pressure - paddle_pressure_start)) * (((float) paddle_regen_strength) / (100.0f));
    float pedal_request = ((float)(*throttlePos_u8)) - paddle_request; // delta b/w throttle and regen

    if (pedal_request >= 0.0f) {
        *throttlePos_u8 = (uint8_t) pedal_request;
        return false;
    }

    // Regen due to paddles
    *throttlePos_u8 = 0; // avoid negative - wheel going backwards


    float reqTorque = maxFastTorque_Nm * pedal_request / ((float)(UINT8_MAX));
    float recuperative_limit = getMotorRegenerativeCapacity(avgMotorSpeed_RPM);

    // Requested recuperation that is less than the maximum-power regen point possible
    if (reqTorque > recuperative_limit) {
        setTorqueLimsAllProtected(0.0f, reqTorque);
        setVelocityInt16All(0);
    }
    // Requested recuperation is even more negative than the limit i.e. "more than possible negative torque allowed"
    else {
        setTorqueLimsAllProtected(0.0f, recuperative_limit);
        setVelocityInt16All(0);
    }
    return true;
}

static float test_local() {
	return 42.42f;
}

bool setParallelRegen(uint8_t throttlePos_u8, uint16_t brakePressurePsi_u8, int32_t avgMotorSpeed_RPM) {

    // return if regen is not needed
    if (brakePressurePsi_u8 < braking_threshold_psi) {
        setVelocityInt16All(0);
        return false;
    }

    bool ret_val = true;

    // DIM requested regen_force_multiplier
    uint8_t regen_force_multiplier_int8 = 80;//0;

    // process the max regen force requested:
    float regen_force_multiplier_f = (float)regen_force_multiplier_int8 / 100.0f;

    // clamping multiplier from DIM
    if (regen_force_multiplier_f > 1.0) {
        regen_force_multiplier_f = 1.0;
    }

    // process regen button max force:
    // float dim_regen_buttons_f = (float)dim_regen_buttons / 100.0f;

    if(ret_val == false) {
        setVelocityInt16All(0);
        return false;
    }
    if (regen_force_multiplier_int8 == 0) {
        setVelocityInt16All(0);
        return false;
    }

    // this will overflow as brake pressure exceeds max regen pressure, that's why there is a clamp below

    // get brake kappa for each motor and then run the following if checks against maxfasttorque and the capping max regen force
    float regenTorque_FL = getBrakeMaxTorque_mNm(MOTOR_FL, brakePressurePsi_u8) * 0.001;
    float regenTorque_FR = getBrakeMaxTorque_mNm(MOTOR_FR, brakePressurePsi_u8) * 0.001;
    float regenTorque_RL = getBrakeMaxTorque_mNm(MOTOR_RL, brakePressurePsi_u8) * 0.001;
    float regenTorque_RR = getBrakeMaxTorque_mNm(MOTOR_RR, brakePressurePsi_u8) * 0.001;

    cmr_torqueDistributionNm_t neg_torques = {
        .fl = regenTorque_FL,
        .fr = regenTorque_FR,
        .rl = regenTorque_RL,
        .rr = regenTorque_RR,
    };
    setTorqueLimsProtected(NULL, &neg_torques);
    setVelocityInt16All(0);

    return true;
}
