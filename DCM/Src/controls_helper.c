#include "controls_helper.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "motors.h"

/** @brief Maximum motor torque
 * motor datasheet: "maximum torque 'Mmax': 21 Nm"
*/
const float maxTorque_Nm = 21.0f;

/** @brief Maximum motor speed
 * motor datasheet: "no-load-speed 'No': 18617"
 * motor datasheet: "mech. speed limit 'Nmax': 20000"
 * AMK racing kit summary p.16: "Speed setpoint values are limited to 30000 rpm"
*/
const int16_t maxSpeed_rpm = 20000;

/** @brief Maximum motor torque in slow gear */
const float maxSlowTorque_Nm = 5.0f;

/** @brief Maximum motor speed in slow gear. Roughly 10 MPH.
 *  @details WolframAlpha query: (convert 10mph to meters/second) /
 *                               (((convert 18 inches to meters) / rev) * (pi revolutions / rev)) *
 *                               (60 seconds / 1 minute) * (15 revolutions / rev)
 */
const int16_t maxSlowSpeed_rpm = 1500;

/** @brief Maximum motor torque in medium gear */
const float maxMediumTorque_Nm = 10.5f;

/** @brief Maximum motor speed in medium gear. Roughly 20m/s */
const int16_t maxMediumSpeed_rpm = 13000;

/** @brief Maximum motor torque in fast gear */
const float maxFastTorque_Nm = 21.0f;

/** @brief Maximum motor speed in fast gear */
const int16_t maxFastSpeed_rpm = maxSpeed_rpm;

/** @brief horizontal distance between the center of the front wheels and the origin of the SBG frame in meters */
const float chassis_a = 0.775f;

/** @brief horizontal distance between the center of the rear wheels and the origin of the SBG frame in meters*/
const float chassis_b = 0.775f;

/** @brief horizontal distance between the front wheels in meters */
const float chassis_w_f = 0.625; //Confirm with CAD people

/** @brief horizontal distance between the rear wheels in meters */
const float chassis_w_r = 0.625; //Confirm with CAD people

/** @brief  min brake pressure for starting to apply regen */
const uint8_t brake_pressure_start = 50;

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
	return (GEAR_RATIO / EFFECTIVE_WHEEL_RAD_M) 
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
    if (cmr_canRXMetaTimeoutError(&canTractiveRXMeta[CANRX_TRAC_HVI_SENSE], xTaskGetTickCount()) == 0) {
        measured_voltage_V = hv_voltage_V;
    }

    return measured_voltage_V;
}

/** @brief returns the pack current */
float getPackCurrent() {
//    volatile cmr_canVSMSensors_t *vsmSensor = canVehicleGetPayload(CANRX_VEH_VSM_SENSORS);
//    return ((float)(vsmSensor->hallEffect_cA)) * 1e-2f; // convert to amps
	volatile cmr_canHVIHeartbeat_t *HVISense = canVehicleGetPayload(CANRX_HVI_SENSE);
	return ((float)(HVISense->packCurrent_dA)) * 1e-1f; // convert to amps
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
float swAngleDegToSteeringAngleRad(int16_t swAngle_deg) {
    const float steering_angle_deg = ((float)swAngle_deg); // convert steering wheel angle into steering angle SIKE BITCHED ALREADY GOT STEERING ANGLE
    return steering_angle_deg * M_PI / 180.0f; // convert to rads
}

/**
 * @brief Determine whether or not the SBG sensor's velocity readings could be trusted
 * 
 * @param ignore_valid_bit Whether or not to ignore the sensor's reported validity of its velocity readings
 */
bool canTrustSBGVelocity(bool ignore_valid_bit) {
    const volatile cmr_canSBGStatus3_t *sbg_status = canDAQGetPayload(CANRX_DAQ_SBG_STATUS_3);
    const uint32_t sbg_soln_status = sbg_status->solution_status;
    const uint32_t sbg_vel_valid = sbg_soln_status & CMR_CAN_SBG_SOL_VELOCITY_VALID;
    const bool sbg_timeout = cmr_canRXMetaTimeoutError(&canDaqRXMeta[CANRX_DAQ_SBG_STATUS_3], xTaskGetTickCount()) != 0;

    return !sbg_timeout && (sbg_vel_valid || ignore_valid_bit);
}

// returns if regen was activated. Updates throttlePos_u8 in paddle regen
bool setRegen(uint8_t *throttlePos_u8, uint8_t brakePressurePsi_u8, int32_t avgMotorSpeed_RPM){
    // get the regen type
    uint8_t pedal_regen_strength = 0;
    //bool retval1 = getProcessedValue(&pedal_regen_strength, PEDAL_REGEN_STRENGTH_INDEX, unsigned_integer);

    uint8_t paddle_regen_strength = 100;//0;
    //bool retval2 = getProcessedValue(&paddle_regen_strength, PADDLE_MAX_REGEN_INDEX, unsigned_integer);
//
//    if (!retval1 || !retval2) {
//        return false;
//    }

    uint8_t leftPaddle = ((volatile cmr_canDIMActions_t *) canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON))->paddleLeft;
    uint8_t rightPaddle = ((volatile cmr_canDIMActions_t *) canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON))->paddleRight;
    uint8_t paddle_pressure = max(leftPaddle, rightPaddle);

    if (pedal_regen_strength > 0 && brakePressurePsi_u8 >= brake_pressure_start) {
        return setParallelRegen(throttlePos_u8, brakePressurePsi_u8, avgMotorSpeed_RPM);
    } else if (paddle_regen_strength > 0 && paddle_pressure >= paddle_pressure_start) {
        return setPaddleRegen(throttlePos_u8, brakePressurePsi_u8, avgMotorSpeed_RPM, paddle_pressure, paddle_regen_strength);
    }
    return false;
}

/**
 * @brief returns a negative torque limit in Nm after paddle regen request,
 *        or changes value of requested throttle
*/
float getRegenTorqueReq(uint8_t *throttlePos_u8, uint8_t brakePressurePsi_u8){
    // get the regen type
    uint8_t pedal_regen_strength = 0;
    bool retval1 = getProcessedValue(&pedal_regen_strength, PEDAL_REGEN_STRENGTH_INDEX, unsigned_integer);

    uint8_t paddle_regen_strength = 100;//0;
    //bool retval2 = getProcessedValue(&paddle_regen_strength, PADDLE_MAX_REGEN_INDEX, unsigned_integer);

//    if (!retval1 || !retval2) {
//        return 0.0f;
//    }

    uint8_t leftPaddle = ((volatile cmr_canDIMActions_t *) canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON))->paddleLeft;
    uint8_t rightPaddle = ((volatile cmr_canDIMActions_t *) canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON))->paddleRight;
    uint8_t paddle_pressure = max(leftPaddle, rightPaddle);

    if (paddle_regen_strength > 0 && paddle_pressure >= paddle_pressure_start) {

        float paddle_request = ((float)(paddle_pressure - paddle_pressure_start)) * (((float) paddle_regen_strength) / (100.0f));
        float pedal_request = ((float)(*throttlePos_u8)) - paddle_request;
    
        if (pedal_request >= 0.0f) {
            *throttlePos_u8 = (uint8_t) pedal_request;
            return 0.0; 
        }

        // Regen due to paddles
        *throttlePos_u8 = 0;

        float reqTorque = maxFastTorque_Nm * pedal_request / ((float)(UINT8_MAX));
        float avgMotorSpeed_RPM = getTotalMotorSpeed_rpm()* 0.25f;
        float recuperative_limit = getMotorRegenerativeCapacity(avgMotorSpeed_RPM);
        return fmaxf(reqTorque, recuperative_limit);
    }
    return 0.0f;
}

bool setPaddleRegen(uint8_t *throttlePos_u8, uint8_t brakePressurePsi_u8, int32_t avgMotorSpeed_RPM, uint8_t paddle_pressure, uint8_t paddle_regen_strength) {
    if (paddle_pressure < paddle_pressure_start) return false;

    float paddle_request = ((float)(paddle_pressure - paddle_pressure_start)) * (((float) paddle_regen_strength) / (100.0f));
    float pedal_request = ((float)(*throttlePos_u8)) - paddle_request;
    
    if (pedal_request >= 0.0f) {
        *throttlePos_u8 = (uint8_t) pedal_request;
        return false; 
    }

    // Regen due to paddles
    *throttlePos_u8 = 0;

    float reqTorque = maxFastTorque_Nm * pedal_request / ((float)(UINT8_MAX));
    float recuperative_limit = -21.0f;//getMotorRegenerativeCapacity(avgMotorSpeed_RPM);
    
    // Requested recuperation that is less than the maximum-power regen point possible
    if (reqTorque > recuperative_limit) {
        setTorqueLimsAllProtected(0.0f, reqTorque);
        setVelocityInt16All(0);
    } 
    // Requested recuperation is even more negative than the limit
    else {
        setTorqueLimsAllProtected(0.0f, recuperative_limit);
        setVelocityInt16All(0);
    }
    return true;
}

bool setParallelRegen(uint8_t throttlePos_u8, uint8_t brakePressurePsi_u8, int32_t avgMotorSpeed_RPM) {
    
    // return if regen is not needed
    if (brakePressurePsi_u8 < brake_pressure_start) return false;

    bool ret_val = true;

    uint8_t regen_force_multiplier_int8 = 80;//0;
    ret_val &= getProcessedValue(&regen_force_multiplier_int8, PEDAL_REGEN_STRENGTH_INDEX, unsigned_integer); 


    // regen button unused due to endurance tuning throwing off drivers
    // uint8_t dim_regen_buttons = !((volatile cmr_canDIMActions_t *) canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON))->regenPercent;

    // process the max regen force requested:
    float regen_force_multiplier_f = (float)regen_force_multiplier_int8 / 100.0f;

    // process regen button max force:
    // float dim_regen_buttons_f = (float)dim_regen_buttons / 100.0f;

    if(ret_val == false) return false;
    if (regen_force_multiplier_int8 == 0) return false;

    // ******* END DIM CONFIG SETTINGS *********


    // get regen limit
    float recuperative_limit = getMotorRegenerativeCapacity(avgMotorSpeed_RPM);
    // this will overflow as brake pressure exceeds max regen pressure, that's why there is a clamp below
    float regenTorque = maxFastTorque_Nm * regen_force_multiplier_f * ((float)brakePressurePsi_u8 - (float)brake_pressure_start);

    if (regenTorque > maxFastTorque_Nm) {
        regenTorque = maxFastTorque_Nm;
    }

    // cap max regen force
    if (regenTorque > regen_force_multiplier_f * maxFastTorque_Nm) {
        regenTorque = regen_force_multiplier_f * maxFastTorque_Nm;
    }
    

    // we want negative torque
    regenTorque = regenTorque * -1.00f ;

    if (regenTorque > recuperative_limit) {
        setTorqueLimsAllProtected(0.0f, regenTorque);
        setVelocityInt16All(0);
    }

    // Requested recuperation is even more negative than the limit
    else {
        setTorqueLimsAllProtected(0.0f, recuperative_limit);
        setVelocityInt16All(0);
    }
    return true;
}
