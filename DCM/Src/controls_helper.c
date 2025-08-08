#include "constants.h"

#include "controls_helper.h"
#include "lut_3d.h"
#include "../optimizer/optimizer.h"
#include "controls.h"
#include "lut.h"
#include "safety_filter.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "motors.h"

/** @brief  min brake pressure for starting to apply regen */
const uint16_t brake_pressure_start = 50;

/** @brief  min paddle pressure for starting to apply regen */
const uint8_t paddle_pressure_start = 30;

/** @brief the ratio between swangle (steering wheel angle) and steering angle
 *  @todo steering angle should be linear to the readings of the swangle sensor, before the nonlinearity correction
*/
const float swangle_per_steering_angle = (112.0f - (-84.0f)) / 40.0f;

// ------------------------------------------------------------------------------------------------
// Functions

/**
 * @brief returns requested moment Nm given torques and steering_angle_deg
 * @param torque_req four torques
 * @param steering_angle_deg steering anlge
*/
float Mz_calc(cmr_torqueDistributionNm_t *torque_req, float steering_angle_deg)
{
    //TODO degrees or radians?? I think should be radians
	float sangle = sin(steering_angle_deg);
	float cangle = cos(steering_angle_deg);

    //calculates Mz Nm from four torques
	return (gear_ratio / effective_wheel_rad_m)
            * (torque_req->fl * chassis_a * sangle
            + torque_req->fl * (chassis_w_f * 0.5f)  * cangle
            + torque_req->fr * chassis_b * sangle
            - torque_req->fr * (chassis_w_f * 0.5f) * cangle
            + torque_req->rl * (chassis_w_r * 0.5f)
            - torque_req->rr * (chassis_w_r * 0.5f));
}

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

float get_load_cell_angle_rad(canDaqRX_t loadIndex) {
    switch (loadIndex) {
        case CANRX_DAQ_LOAD_FL:
        case CANRX_DAQ_LOAD_FR:
            return DAQ_PUSHROD_ANGLE_FR / 180.0f * PI;
        case CANRX_DAQ_LOAD_RL:
        case CANRX_DAQ_LOAD_RR:
            return DAQ_PUSHROD_ANGLE_RR / 180.0f * PI;
        default:
            return 0.0f;
    }
}

/**
 * @brief Return downforce given motor location
 */
float get_downforce(canDaqRX_t loadIndex, bool use_true_downforce) {
    float downforce_N;
    bool not_timeout = cmr_canRXMetaTimeoutWarn(&canDaqRXMeta[loadIndex],
                                                xTaskGetTickCount()) == 0;
    if (use_true_downforce && not_timeout) {
        volatile cmr_canIZZELoadCell_t *downforcePayload =
            (volatile cmr_canIZZELoadCell_t *)canDAQGetPayload(loadIndex);
        float angle = get_load_cell_angle_rad(loadIndex);
        volatile int16_t raw = parse_int16(&downforcePayload->force_output_N);
        downforce_N = (float)raw * 0.1f * sinf(angle);
    } else {
        downforce_N = (float) car_mass_kg * G * 0.25f;
        // downforce car mass * g * 1/4 every motor
    }
    return downforce_N;
}

// OPTIMAL CONTROL HELPERS
void get_tractive_capabilities(float *cap_fl, float *cap_fr, float *cap_rl, float *cap_rr, bool use_true_downforce) {
    
    const float corner_weight_Nm = 80.0f;
    *cap_fl = lut_get_max_Fx_kappa(0.0f, get_downforce(CANRX_DAQ_LOAD_FL, use_true_downforce) + corner_weight_Nm).Fx;
    *cap_fr = lut_get_max_Fx_kappa(0.0f, get_downforce(CANRX_DAQ_LOAD_FR, use_true_downforce) + corner_weight_Nm).Fx;
    *cap_rl = lut_get_max_Fx_kappa(0.0f, get_downforce(CANRX_DAQ_LOAD_RL, use_true_downforce) + corner_weight_Nm).Fx;
    *cap_rr = lut_get_max_Fx_kappa(0.0f, get_downforce(CANRX_DAQ_LOAD_RR, use_true_downforce) + corner_weight_Nm).Fx;
}

// converting force to torque linearly ignoring rolling resistance + inefficiencies
void compute_torque_limits(float cap_fl, float cap_fr, float cap_rl, float cap_rr,
    float *lim_fl, float *lim_fr, float *lim_rl, float *lim_rr) {

        const float r = effective_wheel_rad_m;
        const float g = gear_ratio;
        static const float motor_resistance = 0.5f;
        
        *lim_fl = fminf(cap_fl * r / g + motor_resistance, maxTorque_continuous_stall_Nm);
        *lim_fr = fminf(cap_fr * r / g + motor_resistance, maxTorque_continuous_stall_Nm);
        *lim_rl = fminf(cap_rl * r / g + motor_resistance, maxTorque_continuous_stall_Nm);
        *lim_rr = fminf(cap_rr * r / g + motor_resistance, maxTorque_continuous_stall_Nm);
}

// preparing optimizer state
void load_optimizer_state(optimizer_state_t *state, float normalized_throttle, int32_t swAngle_millideg_FL, int32_t swAngle_millideg_FR, float lim_fl, float lim_fr,
    float lim_rl, float lim_rr, bool allow_regen) {

    int32_t swAngle_millideg = (swAngle_millideg_FL + swAngle_millideg_FR) / 2;

    float vel_fl = getMotorSpeed_radps(MOTOR_FL);
    float vel_fr = getMotorSpeed_radps(MOTOR_FR);
    float vel_rl = getMotorSpeed_radps(MOTOR_RL);
    float vel_rr = getMotorSpeed_radps(MOTOR_RR);

    state->power_limit = getPowerLimit_W();
    state->omegas[0] = vel_fl;
    state->omegas[1] = vel_fr;
    state->omegas[2] = vel_rl;
    state->omegas[3] = vel_rr;

    const float resist = 0.5f;
    const float throttle_accel = maxTorque_continuous_stall_Nm * MOTOR_LEN * gear_ratio / effective_wheel_rad_m / car_mass_kg;
    state->areq = normalized_throttle * throttle_accel;

    float mreq = -getYawRateControlLeftRightBias(swAngle_millideg);
    state->mreq = mreq;
    state->theta_left = -swAngleMillidegToSteeringAngleRad(swAngle_millideg_FL);
    state->theta_right = -swAngleMillidegToSteeringAngleRad(swAngle_millideg_FR);

    float regen[4];
    for (int i = 0; i < 4; i++) {
        regen[i] = getMotorRegenerativeCapacity(getMotorSpeed_rpm(i));
    }

    for (int i = 0; i < 4; i++) {
        float torque_limit = (&lim_fl)[i];
        float regen_limit = regen[i];

        float lower = allow_regen ? fmaxf(-torque_limit + resist, regen_limit) : 0.0f;

        state->variable_profile[i].lower = lower;
        state->variable_profile[i].upper = torque_limit;
    }
}


// REGEN CODE FOLLOWS

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
