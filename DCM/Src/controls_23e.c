/**
 * @file controls.c
 * @brief Vehicle control loops.
 *
 * @author Carnegie Mellon Racing
 */

// ------------------------------------------------------------------------------------------------
// Includes

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include "controls_23e.h"
#include "motors.h"
#include "lut_3d.h"
#include "../cvxgen/solver.h"
#include "cvxgen_interface.h"
#include "safety_filter.h"

#define PI 3.1415926535897932384626


// ------------------------------------------------------------------------------------------------
// Globals

/* @brief cvxgen solver max iterations*/
static int max_iters = 25;

/* @brief cvxgen solver max time IN MS*/
static int max_time = 4;

/** @brief Yaw rate control kp */
volatile cmr_can_controls_pid_debug_t yrcDebug;
static float yrc_kp;

/** @brief CAN data for traction control */
volatile cmr_can_front_slip_ratio_data_t frontSlipRatios;
volatile cmr_can_rear_slip_ratio_data_t rearSlipRatios;
volatile cmr_can_front_whl_speed_setpoint_t frontWhlSetpoints;
volatile cmr_can_rear_whl_speed_setpoint_t rearWhlSetpoints;
volatile cmr_can_front_whl_velocity_t frontWhlVelocities;
volatile cmr_can_rear_whl_velocity_t rearWhlVelocities;
volatile cmr_can_front_whl_slip_angle_t frontWhlSlipAngles;

/** @brief FF Launch Control Start Tick and Button Released  **/
static TickType_t startTickCount;
static bool launchControlButtonPressed;
static bool launchControlActive;

/** @brief CAN data for CVXGEN*/
volatile cmr_can_CVXGEN_info_t solverInfo;
volatile cmr_canCDCWheelTorque_t solverTorques;
volatile cmr_can_CVXGEN_counter_t nonConvergenceCounter;

/* @brief For testing only; false = use calculated downforce */
volatile bool use_true_downforce = false;

/** @brief total distance traveled */
extern volatile float odometer_km;

/** @brief whether or not TC and YRC are enabled */
static volatile cmr_canCDCControlsStatus_t controlsStatus = {
    .tcOn = (uint8_t)false,
    .yrcOn = (uint8_t)false
};

volatile cmr_canCDCKiloCoulombs_t coulombCounting;

static float getYawRateControlLeftRightBias(int16_t swAngle_deg);

/** @brief Coulomb counting info **/
static TickType_t previousTickCount;

// ------------------------------------------------------------------------------------------------
// Function implementations

/** @brief initialize yaw rate control */
static void initYawRateControl() {
    // read yrc_kp from DIM
    yrc_kp = 0.0f;
    getProcessedValue(&yrc_kp, YRC_KP_INDEX, float_1_decimal);

    yrc_kp = yrc_kp*100.0f;
    //yrcDebug = getPidDebug();
    yrcDebug.controls_pid = yrc_kp;
    // set yrc_ki to 0 because we don't aim to eliminate steady-state error
    //REMOVE const float yrc_ki = 0.0f;

    // disable derivate separation because no significant derivative kick was observed
    // steering angle and the car's yaw seem to have similar timescales
    //REMOVE const bool enable_derivative_separation = false;
}

/** @brief initialize controls */
void initControls() {
    initYawRateControl();
    cvxgen_init(max_iters, max_time);
    nonConvergenceCounter.non_convergence_count = 0;
    startTickCount = xTaskGetTickCount();
	launchControlButtonPressed = false;
	launchControlActive = false;
	coulombCounting.KCoulombs = 0.0f;
}

/** @brief update controlsStatus to be displayed on DIM */
void setControlsStatus(cmr_canGear_t gear) {
    switch (gear) {
        case CMR_CAN_GEAR_SLOW:
            controlsStatus.tcOn = (uint8_t)false;
            controlsStatus.yrcOn = (uint8_t)false;
            break;
        case CMR_CAN_GEAR_FAST:
            controlsStatus.tcOn = (uint8_t)false;
            controlsStatus.yrcOn = (uint8_t)false;
            break;
        case CMR_CAN_GEAR_ENDURANCE:
            controlsStatus.tcOn = (uint8_t)false;
            controlsStatus.yrcOn = (uint8_t)false;
            break;
        case CMR_CAN_GEAR_AUTOX:
            controlsStatus.tcOn = (uint8_t)true;
            controlsStatus.yrcOn = (uint8_t)true;
            break;
        case CMR_CAN_GEAR_SKIDPAD:
            controlsStatus.tcOn = (uint8_t)false;
            controlsStatus.yrcOn = (uint8_t)true;
            break;
        case CMR_CAN_GEAR_ACCEL:
            controlsStatus.tcOn = (uint8_t)true;
            controlsStatus.yrcOn = (uint8_t)false;
            break;
        case CMR_CAN_GEAR_TEST:
            controlsStatus.tcOn = (uint8_t)true;
            controlsStatus.yrcOn = (uint8_t)true;
            break;
        case CMR_CAN_GEAR_REVERSE:
            controlsStatus.tcOn = (uint8_t)false;
            controlsStatus.yrcOn = (uint8_t)false;
            break;
        default:
            controlsStatus.tcOn = (uint8_t)false;
            controlsStatus.yrcOn = (uint8_t)false;
            break;
    }
}

/** @brief get the a read-only pointer to controlsStatus */
const volatile cmr_canCDCControlsStatus_t *getControlsStatus() {
    return (const cmr_canCDCControlsStatus_t*) &controlsStatus;
}

/**
 * @brief Runs control loops and sets motor torque limits and velocity targets accordingly.
 *
 * @param gear Which gear the vehicle is in.
 * @param throttlePos_u8 Throttle position, 0-255.
 * @param brakePos_u8 Brake position, 0-255.
 * @param swAngle_deg Steering wheel angle in degrees. Zero-centered, right turn positive.
 * @param battVoltage_mV Accumulator voltage in millivolts.
 * @param battCurrent_mA Accumulator current in milliamps.
 * @param blank_command Additional signal that forces the motor commands to zero vel. and zero torque
 */
void runControls (
    cmr_canGear_t gear,
    uint8_t throttlePos_u8,
    uint8_t brakePos_u8,
    uint8_t brakePressurePsi_u8,
    int16_t swAngle_deg,
    int32_t battVoltage_mV,
    int32_t battCurrent_mA,
    bool blank_command
) {
    integrateCurrent();
    if (blank_command) {
        setTorqueLimsAllProtected(0.0f, 0.0f);
        setVelocityInt16All(0);
        return;
    }

    volatile cmr_canAMKActualValues1_t *amkAct1FL = canTractiveGetPayload(CANRX_TRAC_INV_FL_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1FR = canTractiveGetPayload(CANRX_TRAC_INV_FR_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1RL = canTractiveGetPayload(CANRX_TRAC_INV_RL_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1RR = canTractiveGetPayload(CANRX_TRAC_INV_RR_ACT1);

    const int32_t avgMotorSpeed_RPM = (
        + (int32_t)(amkAct1FL->velocity_rpm)
        + (int32_t)(amkAct1FR->velocity_rpm)
        + (int32_t)(amkAct1RL->velocity_rpm)
        + (int32_t)(amkAct1RR->velocity_rpm)
    ) / MOTOR_LEN;

    // Update odometer
    /* Wheel Speed to Vehicle Speed Conversion
    *      (x rotations / 1min) * (16" * PI) *  (2.54*10^-5km/inch)
    *      (1min / 60sec) * (1sec/1000ms) * (5ms period) * (1/13.93 gear ratio)
    *      = x * 7.6378514861 × 10^-9 */
    odometer_km += ((float)avgMotorSpeed_RPM) * 7.6378514861e-9;
    /** @todo check floating point granularity for potential issues with adding small numbers repeatedly to large numbers */

    switch (gear) {
        case CMR_CAN_GEAR_SLOW: {
            setSlowTorque(throttlePos_u8);
            break;
        }
        case CMR_CAN_GEAR_FAST: {
            setFastTorque(throttlePos_u8);
            break;
        }
        case CMR_CAN_GEAR_ENDURANCE: {
            const bool useCVXGEN = false;
            const bool enableRegen = true;
            const bool timingTest = false;
            const bool assumeNoTurn = true;
            const bool clampbyside = true; // LOL-TC
            if(useCVXGEN){
                if(!setCVXGENSolver(throttlePos_u8, brakePressurePsi_u8, swAngle_deg, enableRegen, timingTest, assumeNoTurn))
                	setEnduranceTestTorque(avgMotorSpeed_RPM, throttlePos_u8, brakePos_u8,
                	               swAngle_deg, battVoltage_mV, battCurrent_mA, brakePressurePsi_u8, clampbyside);
            }else{
                setEnduranceTestTorque(avgMotorSpeed_RPM, throttlePos_u8, brakePos_u8,
               swAngle_deg, battVoltage_mV, battCurrent_mA, brakePressurePsi_u8, clampbyside);
            }
            break;
        }
        case CMR_CAN_GEAR_AUTOX: {
            // const bool assumeNoTurn = true; // TC is not allowed to behave left-right asymmetrically due to the lack of testing
            // const bool ignoreYawRate = false; // TC takes yaw rate into account to prevent the vehicle from stopping unintendedly when turning at low speeds
            // const bool allowRegen = true; // regen-braking is allowed to protect the AC by keeping charge level high
            // const float critical_speed_mps = 5.0f; // using a high value to prevent the vehicle from stopping unintendedly when turning at low speeds
            const bool clampbyside = true;
            setYawRateControl(throttlePos_u8, brakePressurePsi_u8, swAngle_deg, clampbyside);
            //setYawRateAndTractionControl(throttlePos_u8, brakePressurePsi_u8, swAngle_deg, assumeNoTurn, ignoreYawRate, allowRegen, critical_speed_mps);
            break;
        }
        case CMR_CAN_GEAR_SKIDPAD: {
            const bool clampbyside = false;
            setYawRateControl(throttlePos_u8, brakePressurePsi_u8, swAngle_deg, clampbyside);
            break;
        }
        case CMR_CAN_GEAR_ACCEL: {
            const bool assumeNoTurn = true; // TC is not allowed to behave left-right asymmetrically because it's meaningless in accel
            const bool ignoreYawRate = true;  // TC ignores yaw rate because it's meaningless in accel
            const bool allowRegen = false; // regen-braking is not allowed because it's meaningless in accel
            const float critical_speed_mps = 0.0f; // using a low value to prevent excessive wheel spin at low speeds
            const float leftRightBias_Nm = 0.0f; // YRC is not enabled for accel, so there should be no left-right torque bias
            setLaunchControl(throttlePos_u8, brakePressurePsi_u8, swAngle_deg, leftRightBias_Nm, assumeNoTurn, ignoreYawRate, allowRegen, critical_speed_mps);
            break;
        }
        case CMR_CAN_GEAR_TEST: {
            const bool clampbyside = false;
            setYawRateControl(throttlePos_u8, brakePressurePsi_u8, swAngle_deg, clampbyside);
            setTorqueLimsUnprotected(MOTOR_FL, 0.0f, 0.0f);
            setTorqueLimsUnprotected(MOTOR_FR, 0.0f, 0.0f);
            // const bool enableRegen = true;
            // const bool timingTest = false;
            // const bool assumeNoTurn = true;
            // if(!setCVXGENSolver(throttlePos_u8, brakePressurePsi_u8, swAngle_deg, enableRegen, timingTest, assumeNoTurn))
            //     setFastTorque(throttlePos_u8);
            break;
        }
        case CMR_CAN_GEAR_REVERSE: {
            // for rule-compliance, the car shouldn't reverse
            setTorqueLimsAllProtected(0.0f, 0.0f);
            setVelocityInt16All(0);
            break;
        }
        default: {
            setTorqueLimsAllProtected(0.0f, 0.0f);
            setVelocityInt16All(0);
            break;
        }
    }
}

/**
 * @brief updates kC CAN message
 */
void integrateCurrent() {
    const cmr_canRXMeta_t *timeoutMsg = canVehicleGetMeta(CANRX_HVI_SENSE);
    if(cmr_canRXMetaTimeoutError(timeoutMsg, xTaskGetTickCount()) == (-1))
        return;

	if(coulombCounting.KCoulombs == 0.0f){
		previousTickCount = xTaskGetTickCount();
        coulombCounting.KCoulombs = 0.001f;
	}else{
        const float packCurrent_kA = getPackCurrent()*0.001f;
        const TickType_t currentTick = xTaskGetTickCount();
        coulombCounting.KCoulombs += ((currentTick-previousTickCount)*0.001f)*packCurrent_kA;
        previousTickCount = currentTick;
    }
}

/**
 * @brief Sets motor torques and velocities according to speed limit for slow gear.
 *
 * @param throttlePos_u8 Throttle position, 0-255.
 */
void setSlowTorque (
    uint8_t throttlePos_u8
) {
    const float reqTorque = maxSlowTorque_Nm * (float)(throttlePos_u8) / (float)(UINT8_MAX);
     setTorqueLimsAllProtected(reqTorque, 0.0f);

    // setTorqueLimsUnprotected(MOTOR_FL, reqTorque, 0.0f);
    // setTorqueLimsUnprotected(MOTOR_FR, reqTorque, 0.0f);
    // setTorqueLimsUnprotected(MOTOR_RR, reqTorque, 0.0f);
    // setTorqueLimsUnprotected(MOTOR_RL, reqTorque, 0.0f);

    // Testing motors one by one
//    motorLocation_t active_motor = MOTOR_FR;
//    for(int i = 0; i < MOTOR_LEN; i++) {
//    	setTorqueLimsUnprotected(i, reqTorque, 0.0f);
//    }
//
    setVelocityInt16All(maxSlowSpeed_rpm);
}

/**
 * @brief Calculates and sets motor torques and velocities for fast gear.
 *
 * @param throttlePos_u8 Throttle position, 0-255.
 */
void setFastTorque (
    uint8_t throttlePos_u8
) {
    const float reqTorque = maxFastTorque_Nm * (float)(throttlePos_u8) / (float)(UINT8_MAX);
    setTorqueLimsAllProtected(reqTorque, 0.0f);
//    setTorqueLimsUnprotected(MOTOR_FL, reqTorque, 0.0f);
//    setTorqueLimsUnprotected(MOTOR_FR, reqTorque, 0.0f);
//    setTorqueLimsUnprotected(MOTOR_RR, reqTorque, 0.0f);
//    setTorqueLimsUnprotected(MOTOR_RL, reqTorque, 0.0f);
    setVelocityInt16All(maxFastSpeed_rpm);
}

/**
 * @brief Calculates the relative speeds between each wheel and the ground
 *
 * @param steering_ang_fl Steering angle of the front-left wheel
 * @param steering_ang_fr Steering angle of the front-right wheel
 * @param longitudinal_velocity_mps The longitudinal velocity of the vehicle
 * @param lateral_velocity_mps The lateral velocity of the vehicle
 * @param yaw_rate_radps_sae The yaw rate of the vehicle in SAE coordinates (a right turn is positive)
*/
static void update_whl_vels_and_angles (
    float steering_ang_fl,
    float steering_ang_fr,
    float longitudinal_velocity_mps,
    float lateral_velocity_mps,
    float yaw_rate_radps_sae
) {
    // project vehicle velocities onto each wheel
    const float V_whl_flx = + cosf(steering_ang_fl) * (longitudinal_velocity_mps    + yaw_rate_radps_sae * chassis_w_f)
                            + sinf(steering_ang_fl) * (lateral_velocity_mps         + yaw_rate_radps_sae * chassis_a);
    const float V_whl_fly = - sinf(steering_ang_fl) * (longitudinal_velocity_mps    + yaw_rate_radps_sae * chassis_w_f)
                            + cosf(steering_ang_fl) * (lateral_velocity_mps         + yaw_rate_radps_sae * chassis_a);
    const float V_whl_frx = + cosf(steering_ang_fr) * (longitudinal_velocity_mps    - yaw_rate_radps_sae * chassis_w_f)
                            + sinf(steering_ang_fr) * (lateral_velocity_mps         + yaw_rate_radps_sae * chassis_a);
    const float V_whl_fry = - sinf(steering_ang_fr) * (longitudinal_velocity_mps    - yaw_rate_radps_sae * chassis_w_f)
                            + cosf(steering_ang_fr) * (lateral_velocity_mps         + yaw_rate_radps_sae * chassis_a);
    const float V_whl_rlx = longitudinal_velocity_mps   + yaw_rate_radps_sae * chassis_w_r;
    const float V_whl_rly = lateral_velocity_mps        - yaw_rate_radps_sae * chassis_b;
    const float V_whl_rrx = longitudinal_velocity_mps   - yaw_rate_radps_sae * chassis_w_r;
    const float V_whl_rry = lateral_velocity_mps        - yaw_rate_radps_sae * chassis_b;

    // take the magnitude of wheel velocities
    frontWhlVelocities.v_whl_fl =   hypotf(V_whl_flx, V_whl_fly);
    frontWhlVelocities.v_whl_fr =   hypotf(V_whl_frx, V_whl_fry);
    rearWhlVelocities.v_whl_rl =    hypotf(V_whl_rlx, V_whl_rly);
    rearWhlVelocities.v_whl_rr =    hypotf(V_whl_rrx, V_whl_rry);

    frontWhlSlipAngles.angle_whl_fl = atan2(V_whl_flx, V_whl_fly) - steering_ang_fl;
    frontWhlSlipAngles.angle_whl_fr = atan2(V_whl_frx, V_whl_fry) - steering_ang_fr;

}

/**
 * @brief Calculates the wheel speed setpoints
 *
 * @param slip_ratio_* The slip ratio of a wheel
 * @param steering_ang_fl Steering angle of the front-left wheel
 * @param steering_ang_fr Steering angle of the front-right wheel
 * @param longitudinal_velocity_mps The longitudinal velocity of the vehicle
 * @param lateral_velocity_mps The lateral velocity of the vehicle
 * @param yaw_rate_radps_sae The yaw rate of the vehicle in SAE coordinates (a right turn is positive)
 * @param critical_speed_mps The value added to the linear speed of each wheel
*/
static void update_whl_speed_setpoint (
    float slip_ratio_fl,
    float slip_ratio_fr,
    float slip_ratio_rl,
    float slip_ratio_rr,
    float steering_ang_fl,
    float steering_ang_fr,
    float longitudinal_velocity_mps,
    float lateral_velocity_mps,
    float yaw_rate_radps_sae,
    float critical_speed_mps
) {
    update_whl_vels_and_angles(steering_ang_fl, steering_ang_fr, longitudinal_velocity_mps, lateral_velocity_mps, yaw_rate_radps_sae);

    // ensure that critical_speed_mps is nonnegative
    critical_speed_mps = fmaxf(critical_speed_mps, 0.0f);

    // ensure that the linear velocity of each wheel is nonnegative
    const float V_fl = fmaxf(frontWhlVelocities.v_whl_fl, 0.0f);
    const float V_fr = fmaxf(frontWhlVelocities.v_whl_fr, 0.0f);
    const float V_rl = fmaxf(rearWhlVelocities.v_whl_rl, 0.0f);
    const float V_rr = fmaxf(rearWhlVelocities.v_whl_rr, 0.0f);

    // use the definition of slip ratio to calculate wheel speeds
    frontWhlSetpoints.omega_FL = ((V_fl + critical_speed_mps) * slip_ratio_fl + V_fl) / EFFECTIVE_WHEEL_RAD_M;
    frontWhlSetpoints.omega_FR = ((V_fr + critical_speed_mps) * slip_ratio_fr + V_fr) / EFFECTIVE_WHEEL_RAD_M;
    rearWhlSetpoints.omega_RL = ((V_rl + critical_speed_mps) * slip_ratio_rl + V_rl) / EFFECTIVE_WHEEL_RAD_M;
    rearWhlSetpoints.omega_RR = ((V_rr + critical_speed_mps) * slip_ratio_rr + V_rr) / EFFECTIVE_WHEEL_RAD_M;
}

/**
 * @brief Use the CVXGEN library to solve
 *
 * @param throttlePos_u8 Throttle position, 0-255
 * @param brakePressurePsi_u8 Brake Pressure PSI
 * @param swAngle_deg Steering wheel angle, IGNORED if assumeNoTurn is true
*/
bool setCVXGENSolver(
    uint8_t throttlePos_u8,
    uint8_t brakePressurePsi_u8,
    int16_t swAngle_deg,
    bool enable_Regen,
    bool timing_Test,
    bool assumeNoTurn
) {

    if (brakePressurePsi_u8 >= braking_threshold_psi) { // breaking
        setTorqueLimsAllProtected(0.0f, 0.0f);
        setVelocityInt16All(0);
        return; // skip the rest of YRC
    }

    float torque_min_Nm = 0.0f;
    if(enable_Regen){
    	torque_min_Nm = getRegenTorqueReq(&throttlePos_u8, braking_threshold_psi);
    }else{
    	torque_min_Nm = 0.0f;
    }

    // get requested torque from four tires
    const float throttle_torque_req_Nm = maxFastTorque_Nm * ((float)(throttlePos_u8) / (float)(UINT8_MAX));
    const float T_REQ_Nm = throttle_torque_req_Nm * 4.0f;
    const float bias_Nm = getYawRateControlLeftRightBias(swAngle_deg);
    const float left_torques_Nm = throttle_torque_req_Nm - bias_Nm;
    const float right_torques_Nm = throttle_torque_req_Nm + bias_Nm;

    const float steering_angle_rad = (swAngleDegToSteeringAngleRad(swAngle_deg));
    //const float steering_angle_deg = steering_angle_rad * 180.0f / PI;

    cmr_torqueDistributionNm_t torques_Req_Nm = {.fl = left_torques_Nm, .fr = right_torques_Nm
                                                , .rl = left_torques_Nm, .rr = right_torques_Nm};

    const float M_REQ_Nm = Mz_calc(&torques_Req_Nm, steering_angle_rad);

    // Default to previous YRC determined torques
    cmr_torqueDistributionNm_t result_torques_Nm = {.fl = left_torques_Nm, .fr = right_torques_Nm
            , .rl = left_torques_Nm, .rr = right_torques_Nm};

    torques_Req_Nm.fl = (getKappaFxGlobalMax(MOTOR_FL, throttlePos_u8, assumeNoTurn).Fx)*EFFECTIVE_WHEEL_RAD_M;
    torques_Req_Nm.fr = (getKappaFxGlobalMax(MOTOR_FR, throttlePos_u8, assumeNoTurn).Fx)*EFFECTIVE_WHEEL_RAD_M;
    torques_Req_Nm.rl = (getKappaFxGlobalMax(MOTOR_RL, throttlePos_u8, assumeNoTurn).Fx)*EFFECTIVE_WHEEL_RAD_M;
    torques_Req_Nm.rr = (getKappaFxGlobalMax(MOTOR_RR, throttlePos_u8, assumeNoTurn).Fx)*EFFECTIVE_WHEEL_RAD_M;

    const bool converged = load_data(steering_angle_rad, T_REQ_Nm, M_REQ_Nm, torque_min_Nm,
            &torques_Req_Nm, &result_torques_Nm);

    //CAN message
    if(!converged)
        nonConvergenceCounter.non_convergence_count += 1;

    solverTorques.frontLeft_Nm = (int16_t)result_torques_Nm.fl;
    solverTorques.frontRight_Nm = (int16_t)result_torques_Nm.fr;
    solverTorques.rearLeft_Nm = (int16_t)result_torques_Nm.rl;
    solverTorques.rearRight_Nm = (int16_t)result_torques_Nm.rr;

    solverInfo.moment_req_Nm = M_REQ_Nm;
    solverInfo.lin_accel_Nm = T_REQ_Nm;

    const float battVoltage_mV =  getPackVoltage();
    const float battCurrent_mA = getPackCurrent();

    if(!timing_Test) {
    	if(!converged){
    		return false;
//    		setEnduranceTestTorque(avgMotorSpeed_RPM, throttlePos_u8, brakePos_u8,
//    		               swAngle_deg, battVoltage_mV, battCurrent_mA, brakePressurePsi_u8);
    	}else{
            // set velocities
            float FL_vel = (result_torques_Nm.fl > 0.0f) ? maxFastSpeed_rpm : 0.0f;
            float FR_vel = (result_torques_Nm.fr > 0.0f) ? maxFastSpeed_rpm : 0.0f;
            float RL_vel = (result_torques_Nm.rl > 0.0f) ? maxFastSpeed_rpm : 0.0f;
            float RR_vel = (result_torques_Nm.rr > 0.0f) ? maxFastSpeed_rpm : 0.0f;

            setVelocityFloat(MOTOR_FL, FL_vel); // Converts rad/s to rpm
            setVelocityFloat(MOTOR_FR, FR_vel);
            setVelocityFloat(MOTOR_RL, RL_vel);
            setVelocityFloat(MOTOR_RR, RR_vel);

            //set torques
            setTorqueLimsProtected(&result_torques_Nm, &result_torques_Nm);
            return true;
    	}

    } else {
        float FL_vel = 0.0f;
        float FR_vel = 0.0f;
        float RL_vel = 0.0f;
        float RR_vel = 0.0f;

        setVelocityFloat(MOTOR_FL, FL_vel); // Converts rad/s to rpm
        setVelocityFloat(MOTOR_FR, FR_vel);
        setVelocityFloat(MOTOR_RL, RL_vel);
        setVelocityFloat(MOTOR_RR, RR_vel);

        //set torques
        setTorqueLimsAllProtected(0.0f, 0.0f);
    	//setFastTorque(throttlePos_u8);
    }


    //TODO still need to update wheelspeeds, make sure all other CAN messages are sent.
}

/**
* @brief velocity schedule -- target vehicle velocity given elapsed accel time
*
* @param t_sec elapsed accel time (sec)
*
* @return the target vehicle velocity (m/s)
*/
static float getFFScheduleVelocity(float t_sec) {

   float tMax = 6.4f; // limit time for safety reasons
   float scheduleVelocity_mps = 0.0f;

   if (t_sec < 0.0f) {
       scheduleVelocity_mps = 0.0f;
   } else if(t_sec < tMax) {
//	uint8_t pedal_regen_strength = 0; This line times out mysteriously
    float pedal_regen_strength = 0;
   	getProcessedValue(&pedal_regen_strength, YRC_KP_INDEX, float_1_decimal);

   	float scheduleVelocity_mps2 = ((float) pedal_regen_strength) * 0.8f + 6.f;
   	float startingVel_mps = 0.0;
   	getProcessedValue(&startingVel_mps, K_LIN_INDEX, float_1_decimal);
       scheduleVelocity_mps = (scheduleVelocity_mps2 * t_sec) + (startingVel_mps);
           // 2023 Michigan EV fastest accel - 3.645s -> 11.29m/s^2 linear accel
           // 2023 Michigan EV CMR's accel -> memorator data -> 8.63m/s^2 before
   } else {
       scheduleVelocity_mps = 0.0f;
   }

   return scheduleVelocity_mps;
}



/**
* @brief Launch control code. feedforward and uses LUT
*/
void setLaunchControl(
	uint8_t throttlePos_u8,
	uint8_t brakePressurePsi_u8,
	int16_t swAngle_deg, /** IGNORED if assumeNoTurn is true */
	float leftRightBias_Nm, /** IGNORED UNLESS traction_control_mode (defined in the function) is TC_MODE_TORQUE */
	bool assumeNoTurn,
	bool ignoreYawRate,
	bool allowRegen,
	float critical_speed_mps
){
	static const float launch_control_speed_threshold_mps = 0.05f;
	//launch control
	float test = 0.0f;
   	getProcessedValue(&test, YRC_KP_INDEX, float_1_decimal);
	bool inhibit_throttle = false;
	const float nonnegative_odometer_velocity_mps = motorSpeedToWheelLinearSpeed_mps(getTotalMotorSpeed_radps() * 0.25f);
	if (nonnegative_odometer_velocity_mps < launch_control_speed_threshold_mps) { // odometer velocity is below the launch control threshold
		const bool action1_button_pressed = (((volatile cmr_canDIMActions_t *)(canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON)))->buttons) & BUTTON_ACT;
		inhibit_throttle = action1_button_pressed; // inhibit throttle if action1 is pressed

		if (inhibit_throttle) {
			launchControlButtonPressed = true;
		}

		if(launchControlButtonPressed & !inhibit_throttle) {

			if(!launchControlActive)
			{
				startTickCount = xTaskGetTickCount();
				launchControlActive = true;
			}
		}
	}
	     // launch control
//	     bool inhibit_throttle = false;
//	     const float nonnegative_odometer_velocity_mps = motorSpeedToWheelLinearSpeed_mps(getTotalMotorSpeed_radps() * 0.25f);
//	     if ((nonnegative_odometer_velocity_mps < launch_control_speed_threshold_mps)) { // odometer velocity is below the launch control threshold
//	        const bool action1_button_pressed = (((volatile cmr_canDIMActions_t *)(canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON)))->buttons) & BUTTON_ACT;
//	        inhibit_throttle = action1_button_pressed; // inhibit throttle if action1 is pressed
//	        if(!action1_button_pressed){
//	            if(!launchControlActive)
//	                startTickCount = xTaskGetTickCount();
//	            launchControlActive = true;
//	        }else
//	            launchControlActive = (startTickCount != 0) ? false: true;
//	     }
	cmr_torqueDistributionNm_t neg_torques_Nm = {.fl = 0.0f, .fr = 0.0f, .rl = 0.0f, .rr = 0.0f};
	 if (brakePressurePsi_u8 < braking_threshold_psi && throttlePos_u8 > 0 && !inhibit_throttle && launchControlActive) { // not braking and throttle is not neutral or inhibited
	         TickType_t tick = xTaskGetTickCount();
	         float seconds = (float)(tick - startTickCount) * (0.001f); //convert Tick Count to Seconds


	         //debugging
	         if (seconds > 0.5) {
	         	uint8_t x = 0;
	         }

	         float scheduledBodyVel_mps = getFFScheduleVelocity(seconds);

	         float rearWhlVelocity_RL_mps = motorSpeedToWheelLinearSpeed_mps(getMotorSpeed_radps(MOTOR_RL));
	         float rearWhlVelocity_RR_mps = motorSpeedToWheelLinearSpeed_mps(getMotorSpeed_radps(MOTOR_RR));
//	         frontSlipRatios.slipRatio_FL = getKappaFxGlobalMax(MOTOR_FL, throttlePos_u8, assumeNoTurn).kappa;
//			 frontSlipRatios.slipRatio_FR = getKappaFxGlobalMax(MOTOR_FR, throttlePos_u8, assumeNoTurn).kappa;
//			 rearSlipRatios.slipRatio_RL = getKappaFxGlobalMax(MOTOR_RL, throttlePos_u8, assumeNoTurn).kappa;
//			 rearSlipRatios.slipRatio_RR = getKappaFxGlobalMax(MOTOR_RR, throttlePos_u8, assumeNoTurn).kappa;

//			 frontWhlSetpoints.omega_FL = ((scheduledBodyVel_mps + critical_speed_mps) * frontSlipRatios.slipRatio_FL + scheduledBodyVel_mps) / EFFECTIVE_WHEEL_RAD_M;
//			 frontWhlSetpoints.omega_FR = ((scheduledBodyVel_mps + critical_speed_mps) * frontSlipRatios.slipRatio_FR + scheduledBodyVel_mps) / EFFECTIVE_WHEEL_RAD_M;
//			 rearWhlSetpoints.omega_RL = ((scheduledBodyVel_mps + critical_speed_mps) * rearSlipRatios.slipRatio_RL + scheduledBodyVel_mps) / EFFECTIVE_WHEEL_RAD_M;
//			 rearWhlSetpoints.omega_RR = ((scheduledBodyVel_mps + critical_speed_mps) * rearSlipRatios.slipRatio_RR + scheduledBodyVel_mps) / EFFECTIVE_WHEEL_RAD_M;

			 // set velocities
//			setVelocityFloat(MOTOR_FL, frontWhlSetpoints.omega_FL * GEAR_RATIO * 60.0f / (2.0f * M_PI)); // Converts rad/s to rpm
//			setVelocityFloat(MOTOR_FR, frontWhlSetpoints.omega_FR * GEAR_RATIO * 60.0f / (2.0f * M_PI));
//			setVelocityFloat(MOTOR_RL, rearWhlSetpoints.omega_RL * GEAR_RATIO * 60.0f / (2.0f * M_PI));
//			setVelocityFloat(MOTOR_RR, rearWhlSetpoints.omega_RR * GEAR_RATIO * 60.0f / (2.0f * M_PI));

	        float wheelVelocityRPM = maxFastSpeed_rpm;//GEAR_RATIO * 60.0f * scheduledBodyVel_mps / (2 * M_PI * EFFECTIVE_WHEEL_RAD_M);
	        float rearWhlAvgVelocity_mps = (rearWhlVelocity_RL_mps + rearWhlVelocity_RR_mps) / 2.0f;
	        float frontWhlTargetVelocity_mps = rearWhlAvgVelocity_mps * 1.3f;
	        float frontWheelVelocityRPM = GEAR_RATIO * 60.0f * frontWhlTargetVelocity_mps / (2 * M_PI * EFFECTIVE_WHEEL_RAD_M);

			setVelocityFloat(MOTOR_FL, frontWheelVelocityRPM); // Converts rad/s to rpm
			setVelocityFloat(MOTOR_FR, frontWheelVelocityRPM);
			setVelocityFloat(MOTOR_RL, wheelVelocityRPM);
			setVelocityFloat(MOTOR_RR, wheelVelocityRPM);

			const float reqTorque = maxFastTorque_Nm * (float)(throttlePos_u8) / (float)(UINT8_MAX);
			cmr_torqueDistributionNm_t pos_torques_Nm = {.fl = reqTorque, .fr = reqTorque, .rl = reqTorque, .rr = reqTorque};

			setTorqueLimsProtected(&pos_torques_Nm, &neg_torques_Nm);
//			setTorqueLimsUnprotected(MOTOR_FL, reqTorque, 0);
//			setTorqueLimsUnprotected(MOTOR_FR, reqTorque, 0);
//			setTorqueLimsUnprotected(MOTOR_RL, reqTorque, 0);
//			setTorqueLimsUnprotected(MOTOR_RR, reqTorque, 0);
			return;

	  }else{
		 setTorqueLimsProtected(&neg_torques_Nm, &neg_torques_Nm);
		 return;
	  }

}

/**
 * @brief Set wheelspeed setpoints and torque limits based on the traction control algorithm
 *
 * @param throttlePos_u8 Throttle position, 0-255
 * @param brakePressurePsi_u8 Brake Pressure PSI
 * @param swAngle_deg Steering wheel angle, IGNORED if assumeNoTurn is true
 * @param leftRightBias_Nm positive values increase torque output on the right side and decreases that on the left side,
 *                         IGNORED UNLESS traction_control_mode (defined in the function) is TC_MODE_TORQUE
 * @param assumeNoTurn Forces the behavior of TC to be left-right symmetric, but doesn't affect how yaw rate influences slip ratios
 * @param ignoreYawRate Ignores yaw rate when calculating slip ratios
 * @param allowRegen Allow regenerative breaking
 * @param critical_speed_mps A NONNEGATIVE value that is added to the speed of each wheel when calculating wheelspeed setpoint
 *                           This ensures that wheelspeed setpoint is strictly positive when the vehicle is stationary.
 *                           If this value is too low, the vehicle might not respont to throttle when stationary.
 *                           If this value is too high, there might be excessive wheelspin.
 */
void setTractionControl (
    uint8_t throttlePos_u8,
    uint8_t brakePressurePsi_u8,
    int16_t swAngle_deg, /** IGNORED if assumeNoTurn is true */
    float leftRightBias_Nm, /** IGNORED UNLESS traction_control_mode (defined in the function) is TC_MODE_TORQUE */
    bool assumeNoTurn,
    bool ignoreYawRate,
    bool allowRegen,
    float critical_speed_mps
) {
    // ********* Local Parameters *********

    typedef enum {
        TC_MODE_TORQUE,          /** @brief Map throttle and left-right bias to motor torque, APPLIES leftRightBias_Nm */
        TC_MODE_FX_GLOBAL_MAX,   /** @brief Map max throttle to the global max Fx in the LUT, IGNORES leftRightBias_Nm  */
        TC_MODE_FX_LOCAL_MAX     /** @brief Map max throttle to the max Fx available at the current state, IGNORES leftRightBias_Nm  */
    } traction_control_mode_t;
    static const traction_control_mode_t traction_control_mode = TC_MODE_TORQUE;

    /** @brief Trust SBG velocities even if SBG reports that they're invalid
     *  @warning If set to false, TC will fall back to Fast Mode when SBG velocities are invalid
     *  @note We might want to set this to false for comp but true for testing and data collection
     */
    static const bool trust_sbg_vels_when_invalid = true;

    /** @brief Ease in torque based on throttle position, ignored if traction_control_mode is TC_MODE_TORQUE */
    static const bool ease_in_torque = false;

    /** @brief Torque saturation point, IGNORED if ease_in_torque is false or traction_control_mode is TC_MODE_TORQUE
     *  @note For example, 0.25 will raise the torque limit to max when throttle is more than 25%
     */
    static const float ease_in_torque_saturation_point = 0.25f;

    /** @brief The speed threshold above which the launch button is ignored
     *  @note Compared against wheelspeed, so this behavior doesn't depend on SBG data
     */
    static const float launch_control_speed_threshold_mps = 0.05f;

    // ********* Steering Angle *********

    if (assumeNoTurn) {
        swAngle_deg = 0;
    }
    const float steering_angle_rad = swAngleDegToSteeringAngleRad(swAngle_deg);

    // ********* SBG Data: Vehicle Velocity and Yaw Rate *********

    const volatile cmr_canSBGBodyVelocity_t *body_vels = canDAQGetPayload(CANRX_DAQ_SBG_BODY_VEL);
    const volatile cmr_canSBGIMUGyro_t *body_gyro = canDAQGetPayload(CANRX_DAQ_SBG_IMU_GYRO);

    if (!canTrustSBGVelocity(trust_sbg_vels_when_invalid)) { // SBG velocity can't be trusted
        // fall back to fast mode
        setFastTorque(throttlePos_u8); // set torque and velocity setpoints as if we're in fast mode

        // set wheelspeed setpoints to NAN
        frontWhlSetpoints.omega_FL = NAN;
        frontWhlSetpoints.omega_FR = NAN;
        rearWhlSetpoints.omega_RL = NAN;
        rearWhlSetpoints.omega_RR = NAN;

        // set slip ratio setpoints to NAN
        frontSlipRatios.slipRatio_FL = NAN;
        frontSlipRatios.slipRatio_FR = NAN;
        rearSlipRatios.slipRatio_RL = NAN;
        rearSlipRatios.slipRatio_RR = NAN;

        return; // skip the rest of TC
    }

    const float forward_velocity_nonnegative_mps = fmax(((float)(body_vels->velocity_forward)) * 1e-2f, 0.0f); // velocity_forward is in (m/s times 100)
    const float right_velocity_mps = ((float)(body_vels->velocity_right)) * 1e-2f; // velocity_right is in (m/s times 100)
    const float yaw_rate_radps_sae = ignoreYawRate ? 0.0f : ((float)(body_gyro->gyro_z_rads)) * 1e-3f; // gyro_z_rads is in (rad/s times 1000)

    // ********* Wheelspeed and Torque Setpoints *********

    // clear wheelspeed setpoints
    frontWhlSetpoints.omega_FL = 0.0f;
    frontWhlSetpoints.omega_FR = 0.0f;
    rearWhlSetpoints.omega_RL = 0.0f;
    rearWhlSetpoints.omega_RR = 0.0f;

    // clear slip ratios setpoints
    frontSlipRatios.slipRatio_FL = 0.0f;
    frontSlipRatios.slipRatio_FR = 0.0f;
    rearSlipRatios.slipRatio_RL = 0.0f;
    rearSlipRatios.slipRatio_RR = 0.0f;

    cmr_torqueDistributionNm_t pos_torques_Nm = {.fl = 0.0f, .fr = 0.0f, .rl = 0.0f, .rr = 0.0f};
    cmr_torqueDistributionNm_t neg_torques_Nm = {.fl = 0.0f, .fr = 0.0f, .rl = 0.0f, .rr = 0.0f};

    // launch control
    bool inhibit_throttle = false;
    const float nonnegative_odometer_velocity_mps = motorSpeedToWheelLinearSpeed_mps(getTotalMotorSpeed_radps() * 0.25f);
    if (nonnegative_odometer_velocity_mps < launch_control_speed_threshold_mps) { // odometer velocity is below the launch control threshold
        const bool action1_button_pressed = (((volatile cmr_canDIMActions_t *)(canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON)))->buttons) & BUTTON_ACT;
        inhibit_throttle = action1_button_pressed; // inhibit throttle if action1 is pressed
    }

    if (brakePressurePsi_u8 < braking_threshold_psi && throttlePos_u8 > 0 && !inhibit_throttle) { // not breaking and throttle is not neutral or inhibited
        // calculate slip ratio setpoints
        switch (traction_control_mode) {
            default: // default to torque-mapped mode if traction_control_mode is not valid
            case TC_MODE_TORQUE: { // torque-mapped mode, retrieve the local max slip ratio setpoints
                frontSlipRatios.slipRatio_FL = getMaxKappaCurrentState(MOTOR_FL, assumeNoTurn);
                frontSlipRatios.slipRatio_FR = getMaxKappaCurrentState(MOTOR_FR, assumeNoTurn);
                rearSlipRatios.slipRatio_RL = getMaxKappaCurrentState(MOTOR_RL, assumeNoTurn);
                rearSlipRatios.slipRatio_RR = getMaxKappaCurrentState(MOTOR_RR, assumeNoTurn);
            } break;

            case TC_MODE_FX_GLOBAL_MAX: { // maps max throttle to the global max Fx of the LUT
                frontSlipRatios.slipRatio_FL = getKappaFxGlobalMax(MOTOR_FL, throttlePos_u8, assumeNoTurn).kappa;
                frontSlipRatios.slipRatio_FR = getKappaFxGlobalMax(MOTOR_FR, throttlePos_u8, assumeNoTurn).kappa;
                rearSlipRatios.slipRatio_RL = getKappaFxGlobalMax(MOTOR_RL, throttlePos_u8, assumeNoTurn).kappa;
                rearSlipRatios.slipRatio_RR = getKappaFxGlobalMax(MOTOR_RR, throttlePos_u8, assumeNoTurn).kappa;
            } break;

            case TC_MODE_FX_LOCAL_MAX: { // maps max throttle to the max Fx available at the current state
                float traction_fl = getTraction(MOTOR_FL, throttlePos_u8, ASSUME_NO_TURN);
                float traction_fr = getTraction(MOTOR_FR, throttlePos_u8, ASSUME_NO_TURN);
                float traction_rl = getTraction(MOTOR_RL, throttlePos_u8, ASSUME_NO_TURN);
                float traction_rr = getTraction(MOTOR_RR, throttlePos_u8, ASSUME_NO_TURN);

                if (assumeNoTurn) { // ignore lateral load transfer
                    const float min_traction_front = fminf(traction_fl, traction_fr);
                    const float min_traction_back = fminf(traction_rl, traction_rr);
                    traction_fl = min_traction_front;
                    traction_fr = min_traction_front;
                    traction_rl = min_traction_back;
                    traction_rr = min_traction_back;
                }

                frontSlipRatios.slipRatio_FL = getKappaByFx(MOTOR_FL, throttlePos_u8, traction_fl, ASSUME_NO_TURN);
                frontSlipRatios.slipRatio_FR = getKappaByFx(MOTOR_FR, throttlePos_u8, traction_fr, ASSUME_NO_TURN);
                rearSlipRatios.slipRatio_RL = getKappaByFx(MOTOR_RL, throttlePos_u8, traction_rl, ASSUME_NO_TURN);
                rearSlipRatios.slipRatio_RR = getKappaByFx(MOTOR_RR, throttlePos_u8, traction_rr, ASSUME_NO_TURN);
            } break;
        }

        // write wheelspeed setpoints to frontWhlSetpoints and rearWhlSetpoints
        update_whl_speed_setpoint (
            frontSlipRatios.slipRatio_FL, frontSlipRatios.slipRatio_FR, rearSlipRatios.slipRatio_RL, rearSlipRatios.slipRatio_RR,
            steering_angle_rad, steering_angle_rad,
            forward_velocity_nonnegative_mps, right_velocity_mps, yaw_rate_radps_sae,
            critical_speed_mps
        );

        // calculate torque limits
        // leave negative torques at 0 if not breaking for better robustness against sensor noise
        // set positive torques according to throttle position and left-right bias (only in torque-mapped mode)
        switch (traction_control_mode) {
            default: // default to torque-mapped mode if traction_control_mode is not valid
            case TC_MODE_TORQUE: { // torque-mapped mode, retrieve the local max slip ratio setpoints
                const float throttle_pos_torque_Nm = maxFastTorque_Nm * (((float)throttlePos_u8) / ((float)(UINT8_MAX)));
                leftRightBias_Nm = fminf(leftRightBias_Nm, throttle_pos_torque_Nm); // ensures throttle_pos_torque_Nm <= throttle_pos_torque_Nm
                leftRightBias_Nm = fmaxf(leftRightBias_Nm, -throttle_pos_torque_Nm); // ensures throttle_pos_torque_Nm >= -throttle_pos_torque_Nm
                const float left_pos_torque_Nm = fmaxf(throttle_pos_torque_Nm - leftRightBias_Nm, 0.0f);
                const float right_pos_torque_Nm = fmaxf(throttle_pos_torque_Nm + leftRightBias_Nm, 0.0f);
                pos_torques_Nm.fl = left_pos_torque_Nm;
                pos_torques_Nm.fr = right_pos_torque_Nm;
                pos_torques_Nm.rl = left_pos_torque_Nm;
                pos_torques_Nm.rr = right_pos_torque_Nm;
            } break;

            case TC_MODE_FX_GLOBAL_MAX: // maps max throttle to the global max Fx of the LUT
            case TC_MODE_FX_LOCAL_MAX: { // maps max throttle to the max Fx available at the current state
                float throttle_pos_torque_Nm = maxFastTorque_Nm;
                if (ease_in_torque) { // ease in torque based on throttle position
                    throttle_pos_torque_Nm *= ((float)throttlePos_u8) / ((float)(UINT8_MAX)) / ease_in_torque_saturation_point;
                    throttle_pos_torque_Nm = fminf(throttle_pos_torque_Nm, maxFastTorque_Nm); // saturate at maxFastTorque_Nm
                }
                pos_torques_Nm.fl = throttle_pos_torque_Nm;
                pos_torques_Nm.fr = throttle_pos_torque_Nm;
                pos_torques_Nm.rl = throttle_pos_torque_Nm;
                pos_torques_Nm.rr = throttle_pos_torque_Nm;
            } break;
        }
    } else if (allowRegen && brakePressurePsi_u8 >= braking_threshold_psi) { // regen-breaking
        // calculate slip ratios for breaking
        frontSlipRatios.slipRatio_FL = getBrakeKappa(MOTOR_FL, brakePressurePsi_u8, braking_threshold_psi);
        frontSlipRatios.slipRatio_FR = getBrakeKappa(MOTOR_FR, brakePressurePsi_u8, braking_threshold_psi);
        rearSlipRatios.slipRatio_RL = getBrakeKappa(MOTOR_RL, brakePressurePsi_u8, braking_threshold_psi);
        rearSlipRatios.slipRatio_RR = getBrakeKappa(MOTOR_RR, brakePressurePsi_u8, braking_threshold_psi);

        // write wheelspeed setpoints to frontWhlSetpoints and rearWhlSetpoints
        update_whl_speed_setpoint (
            frontSlipRatios.slipRatio_FL, frontSlipRatios.slipRatio_FR, rearSlipRatios.slipRatio_RL, rearSlipRatios.slipRatio_RR,
            steering_angle_rad, steering_angle_rad,
            forward_velocity_nonnegative_mps, right_velocity_mps, yaw_rate_radps_sae,
            critical_speed_mps
        );

        // calculate torque limits
        // leave positive torques at 0 when breaking to avoid going against the mechanical breaks
        // set negative torques according to left-right bias
        const float brake_neg_torque_Nm = -maxFastTorque_Nm; /** @todo allow configuration of regen torque via DIM */
        leftRightBias_Nm = fminf(leftRightBias_Nm, -brake_neg_torque_Nm); // ensures leftRightBias_Nm <= -brake_neg_torque_Nm
        leftRightBias_Nm = fmaxf(leftRightBias_Nm, brake_neg_torque_Nm); // ensures leftRightBias_Nm >= brake_neg_torque_Nm
        const float left_neg_torque_Nm = fminf(brake_neg_torque_Nm - leftRightBias_Nm, 0.0f);
        const float right_neg_torque_Nm = fminf(brake_neg_torque_Nm + leftRightBias_Nm, 0.0f);
        neg_torques_Nm.fl = left_neg_torque_Nm;
        neg_torques_Nm.fr = right_neg_torque_Nm;
        neg_torques_Nm.rl = left_neg_torque_Nm;
        neg_torques_Nm.rr = right_neg_torque_Nm;
    } else { // neutral throttle and breaks
        // clear wheelspeed setpoints
        frontWhlSetpoints.omega_FL = 0.0f;
        frontWhlSetpoints.omega_FR = 0.0f;
        rearWhlSetpoints.omega_RL = 0.0f;
        rearWhlSetpoints.omega_RR = 0.0f;
        // leave positive and negative torques at 0
    }

    // set velocities
    setVelocityFloat(MOTOR_FL, frontWhlSetpoints.omega_FL * GEAR_RATIO * 60.0f / (2.0f * M_PI)); // Converts rad/s to rpm
    setVelocityFloat(MOTOR_FR, frontWhlSetpoints.omega_FR * GEAR_RATIO * 60.0f / (2.0f * M_PI));
    setVelocityFloat(MOTOR_RL, rearWhlSetpoints.omega_RL * GEAR_RATIO * 60.0f / (2.0f * M_PI));
    setVelocityFloat(MOTOR_RR, rearWhlSetpoints.omega_RR * GEAR_RATIO * 60.0f / (2.0f * M_PI));

    // set torques
    setTorqueLimsProtected(&pos_torques_Nm, &neg_torques_Nm);
}

/**
 * @brief Calculate the control action (left-right torque bias) of the yaw rate controller
 * @param swAngle_deg Steering wheel angle
 */
static float getYawRateControlLeftRightBias(int16_t swAngle_deg) {
    // ********* Local Parameters *********

    /** @brief Trust SBG velocities even if SBG reports that they're invalid
     *  @warning If set to false, TC will fall back to Fast Mode when SBG velocities are invalid
     *  @note We might want to set this to false for comp but true for testing and data collection
     */
    static const bool trust_sbg_vels_when_invalid = false;

    /** @brief Natural, uncontrolled, steady-state understeer gradient
     *  @note The formula is:
     *        (W_f/C_af - W_r/C_ar) / g
     *        although we measure this value experimentally
     *  @note Some literature suggest that this value could be tuned for different steering responses:
     *        "Therefore, by tuning K_ug, we can impose a neutral, understeering or oversteering behavior" (Kissai et. al., 2020, p.5)
     *  @note Increasing this value decreases yaw rate setpoint, especially when linear velocity is high
     */
    static const float natural_understeer_gradient = 0.011465f; //rad/g

    // get sensor data
    const volatile cmr_canSBGBodyVelocity_t *body_vels = canDAQGetPayload(CANRX_DAQ_SBG_BODY_VEL);
    const volatile cmr_canSBGIMUGyro_t *body_gyro = canDAQGetPayload(CANRX_DAQ_SBG_IMU_GYRO);
    const float forward_velocity_nonnegative_mps = fmaxf(((float)(body_vels->velocity_forward)) * 1e-2f, 0.0f); // velocity_forward is in (m/s times 100)
    const float yaw_rate_radps_sae = ((float)(body_gyro->gyro_z_rads)) * 1e-3f; // gyro_z_rads is in (rad/s times 1000)

    const float steering_angle_rad = swAngleDegToSteeringAngleRad(swAngle_deg);
    const float distance_between_axles_m = chassis_a + chassis_b;
    const float yaw_rate_setpoint_radps = steering_angle_rad * forward_velocity_nonnegative_mps /
        (distance_between_axles_m + forward_velocity_nonnegative_mps * forward_velocity_nonnegative_mps * natural_understeer_gradient);
    yrcDebug.controls_target_yaw_rate = yaw_rate_setpoint_radps;
    yrcDebug.controls_current_yaw_rate = yaw_rate_radps_sae;
    if (!canTrustSBGVelocity(trust_sbg_vels_when_invalid)) {
        yrcDebug.controls_pid = -1.0f; // SBG velocity can't be trusted
        return 0.0f;
    }
    yrcDebug.controls_pid = yrc_kp;
    const float left_right_bias = yrc_kp * (yaw_rate_radps_sae - yaw_rate_setpoint_radps);

    return left_right_bias;
}


/**
 * @brief Set wheelspeed setpoints and torque limits based on the yaw rate control algorithm
 *
 * @param throttlePos_u8 Throttle position, 0-255
 * @param brakePressurePsi_u8 Brake Pressure PSI
 * @param swAngle_deg Steering wheel angle, IGNORED if assumeNoTurn is true
 */
void setYawRateControl (
    uint8_t throttlePos_u8,
    uint8_t brakePressurePsi_u8,
    int16_t swAngle_deg,
    bool clampbyside
) {
    if (brakePressurePsi_u8 >= braking_threshold_psi) { // breaking
        setTorqueLimsAllProtected(0.0f, 0.0f);
        setVelocityInt16All(0);
        return; // skip the rest of YRC
    }

    // get requested torque from throttle
    const float throttle_pos_torque_Nm = maxFastTorque_Nm * (float)(throttlePos_u8) / (float)(UINT8_MAX);

    // calculate left-right torque bias, positive values increase torque output on the right side and decreases that on the left side
    float left_right_torque_bias_Nm = getYawRateControlLeftRightBias(swAngle_deg) * 0.5f; // halved because the bias will be applied to two wheels per side
    left_right_torque_bias_Nm = fminf(left_right_torque_bias_Nm, throttle_pos_torque_Nm); // ensures left_right_torque_bias_Nm <= throttle_pos_torque_Nm
    left_right_torque_bias_Nm = fmaxf(left_right_torque_bias_Nm, -throttle_pos_torque_Nm); // ensures left_right_torque_bias_Nm >= -throttle_pos_torque_Nm

    const cmr_torqueDistributionNm_t pos_torques_Nm = {
        // if bias is negative, more torque will be applied to left wheels, turning right
        .fl = throttle_pos_torque_Nm - left_right_torque_bias_Nm,
        .rl = throttle_pos_torque_Nm - left_right_torque_bias_Nm,
        // if bias is positive, more torque will be applied to right wheels, turning left
        .fr = throttle_pos_torque_Nm + left_right_torque_bias_Nm,
        .rr = throttle_pos_torque_Nm + left_right_torque_bias_Nm
    };

    setTorqueLimsProtected(&pos_torques_Nm, NULL); // set torque limits according to the biased distribution
    // setVelocityInt16All(maxFastSpeed_rpm); // set wheel speed setpoints to maximum


    if (!clampbyside){ // clamp only front wheels to rear wheel average

        float rearWhlVelocity_RL_mps = motorSpeedToWheelLinearSpeed_mps(getMotorSpeed_radps(MOTOR_RL));
        float rearWhlVelocity_RR_mps = motorSpeedToWheelLinearSpeed_mps(getMotorSpeed_radps(MOTOR_RR));

        float wheelVelocityRPM = maxFastSpeed_rpm;//GEAR_RATIO * 60.0f * scheduledBodyVel_mps / (2 * M_PI * EFFECTIVE_WHEEL_RAD_M);
        float rearWhlAvgVelocity_mps = (rearWhlVelocity_RL_mps + rearWhlVelocity_RR_mps) / 2.0f;
        float frontWhlTargetVelocity_mps = rearWhlAvgVelocity_mps + 0.1f;
        float frontWheelVelocityRPM = GEAR_RATIO * 60.0f * frontWhlTargetVelocity_mps / (2 * M_PI * EFFECTIVE_WHEEL_RAD_M);

        setVelocityFloat(MOTOR_FL, wheelVelocityRPM); // Converts rad/s to rpm
        setVelocityFloat(MOTOR_FR, wheelVelocityRPM);
        setVelocityFloat(MOTOR_RL, wheelVelocityRPM);
        setVelocityFloat(MOTOR_RR, wheelVelocityRPM);

    } else { // clamp front left to rear left and front right to rear right

        float rearWhlVelocity_RL_RPM = getMotorSpeed_radps(MOTOR_RL) / (2 * M_PI) * 60;
        float rearWhlVelocity_RR_RPM = getMotorSpeed_radps(MOTOR_RR) / (2 * M_PI) * 60;

        float wheelVelocityRPM = maxFastSpeed_rpm;//GEAR_RATIO * 60.0f * scheduledBodyVel_mps / (2 * M_PI * EFFECTIVE_WHEEL_RAD_M);
        // float rearWhlAvgVelocity_mps = (rearWhlVelocity_RL_mps + rearWhlVelocity_RR_mps) / 2.0f;
        // float frontWhlTargetVelocity_mps = rearWhlAvgVelocity_mps + 0.1f;
        // float frontWheelVelocityRPM = GEAR_RATIO * 60.0f * frontWhlTargetVelocity_mps / (2 * M_PI * EFFECTIVE_WHEEL_RAD_M);

        float clamped_FL_RPM = fminf(rearWhlVelocity_RL_RPM*1.12f     , (rearWhlVelocity_RR_RPM)*1.12f+3000) + 0.1f;
        float clamped_FR_RPM = fminf((rearWhlVelocity_RL_RPM*1.12f)+3000, rearWhlVelocity_RR_RPM*1.12f) + 0.1f;//why? slip ratio vibes

        setVelocityFloat(MOTOR_FL, clamped_FL_RPM); // Converts rad/s to rpm
        setVelocityFloat(MOTOR_FR, clamped_FR_RPM);
        setVelocityFloat(MOTOR_RL, wheelVelocityRPM);
        setVelocityFloat(MOTOR_RR, wheelVelocityRPM);
    }
}

/**
 * @brief Runs traction control algorithm on all 4 wheels.
 *
 * @param throttlePos_u8 Throttle position, 0-255.
 * @param brakePressurePsi_u8 Brake Pressure PSI
 * @param swAngle_deg Steering wheel angle, IGNORED FOR TC if assumeNoTurn is true
 * @param assumeNoTurn Forces the behavior of TC to be left-right symmetric, ONLY APPLIES TO TC
 * @param ignoreYawRate Ignores yaw rate when calculating slip ratios, ONLY APPLIES TO TC
 * @param allowRegen Allow regenerative breaking
 * @param critical_speed_mps A NONNEGATIVE value that is added to the speed of each wheel when calculating wheelspeed setpoint
 *                           This ensures that wheelspeed setpoint is strictly positive when the vehicle is stationary.
 *                           If this value is too low, the vehicle might not respont to throttle when stationary.
 *                           If this value is too high, there might be excessive wheelspin.
 */
void setYawRateAndTractionControl (
    uint8_t throttlePos_u8,
    uint8_t brakePressurePsi_u8,
    int16_t swAngle_deg, /* IGNORED FOR TC if assumeNoTurn is true */
    bool assumeNoTurn,
    bool ignoreYawRate,
    bool allowRegen,
    float critical_speed_mps
) {
    // calculate left-right torque bias, positive values increase torque output on the right side and decreases that on the left side
    const float left_right_torque_bias_Nm = getYawRateControlLeftRightBias(swAngle_deg) * 0.5f; // halved because the bias will be applied to two wheels per side

    // set torques though the traction controller
    setTractionControl(throttlePos_u8, brakePressurePsi_u8, swAngle_deg, left_right_torque_bias_Nm, assumeNoTurn, ignoreYawRate, allowRegen, critical_speed_mps);
}

/**
 * @brief Sets cruise control for aero tests
 *
 * @note Need to hold Action Button 1 and throttle to maintain speed
 *
 * @param throttlePos_u8 Throttle position, 0-255.
 */
void setCruiseControlTorque (
    uint8_t throttlePos_u8,
    uint8_t brakePressurePsi_u8,
    int32_t avgMotorSpeed_RPM
) {
    static bool cruiseControl = false;
    static uint16_t cruiseVelocity = 0;

    bool action1 = (((volatile cmr_canDIMActions_t *) canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON))->buttons) & BUTTON_ACT;

    if (throttlePos_u8 == 0 || brakePressurePsi_u8 >= 40) {
        cruiseControl = false;
        cruiseVelocity = 0;
        setTorqueLimsAllProtected(0.0f, 0.0f);
        setVelocityInt16All(0);
    } else if (cruiseControl && action1) {
        setTorqueLimsAllProtected(maxFastTorque_Nm, 0.0f);
        setVelocityInt16All(cruiseVelocity);
    } else {
        float reqTorque = maxFastTorque_Nm * (float)(throttlePos_u8) / (float)(UINT8_MAX);
        setTorqueLimsAllProtected(reqTorque, 0.0f);
        setVelocityInt16All(maxMediumSpeed_rpm);
        cruiseControl = false;
        cruiseVelocity = 0;
    }

    if (!cruiseControl && action1) {
        cruiseControl = true;
        cruiseVelocity = avgMotorSpeed_RPM;
    }
}

/**
 * @brief Calculates and sets motor torques and velocities for endurance.
 *
 * @param throttlePos_u8 Throttle position, 0-255.
 * @param brakePos_u8
 * @param swAngle_deg Steering wheel angle in degrees. Zero-centered, right turn positive.
 */
void setEnduranceTorque (
    int32_t avgMotorSpeed_RPM,
    uint8_t throttlePos_u8,
    uint8_t brakePos_u8,
    int16_t swAngle_deg,
    int32_t battVoltage_V_hvc,
    int32_t battCurrent_A_hvc,
    uint8_t brakePressurePsi_u8
) {
    // if braking
    if (setRegen(&throttlePos_u8, brakePressurePsi_u8, avgMotorSpeed_RPM)){
        return;
    }

    // Determine aggrigate torque request be combining acceleration pedal position
    // with brake pedal position.
    const bool regen_button_pressed = (((volatile cmr_canDIMActions_t *) canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON))->buttons) & BUTTON_SCRN ;

    uint8_t pedal_regen_strength = 0;
    getProcessedValue(&pedal_regen_strength, PEDAL_REGEN_STRENGTH_INDEX, unsigned_integer);
    const float regentPcnt_f = ((float)pedal_regen_strength) * 1e-2; // convert a coefficient between 0 and 1

    int32_t pedal_request = (int32_t)throttlePos_u8;
    if (regen_button_pressed) {
        // Pressing button equal to a bit over 30% on brake pedal
        pedal_request -= (int32_t)(regentPcnt_f * ((float)UINT8_MAX));
    }

    // Requested torque that may be positive or negative
    float reqTorque = maxFastTorque_Nm * ((float)(pedal_request)) / ((float)(UINT8_MAX));
    const float recuperative_limit = getMotorRegenerativeCapacity(avgMotorSpeed_RPM);

    // Accelerative torque requested
    if (reqTorque >= 0) {
        // apply power limit. Simply scale power down linearly in the last 5kW of power
        volatile uint8_t power_limit_kW = 70; // uint8, must be between 0 and 255, inclusive
        const bool ret_val = getProcessedValue((void*) &power_limit_kW, POWER_LIM_INDEX, unsigned_integer);
        (void)ret_val; // placate compiler
        if (power_limit_kW < 10) {
            power_limit_kW = 10; // lower-bound the power limit by 5kW to avoid divide-by-zero
        }

        const float power_limit_W = ((float)power_limit_kW) * 1e3f;
        float power_limit_start_derate_W = power_limit_W - 10000.0f;
        power_limit_start_derate_W = fmaxf(power_limit_start_derate_W, 0.0f); // clamp to zero in case of negative value due to lower than 10kw limit

        volatile cmr_canHVIHeartbeat_t *HVISense = canTractiveGetPayload(CANRX_TRAC_HVI_SENSE);
        const float hv_voltage_V = ((float)(HVISense->packVoltage_cV)) * 1e-2f; // convert to volts

        volatile cmr_canVSMSensors_t *vsmSensor = canVehicleGetPayload(CANRX_VEH_VSM_SENSORS);
        const float currentA = ((float)(vsmSensor->hallEffect_cA)) * 1e-2f; // convert to amps
        // apply power limit.
        const float power_consumed_W = hv_voltage_V * currentA;

        if (power_consumed_W > power_limit_start_derate_W) {
            float power_derate_multiplier = (power_consumed_W - power_limit_start_derate_W) / (power_limit_W - power_limit_start_derate_W);
            // backup in case of hysterisis in the AC or latency in the system
            power_derate_multiplier = fminf(power_derate_multiplier, 1.0f);
            power_derate_multiplier = fmaxf(power_derate_multiplier, 0.0f);
            reqTorque *= 1.0f - power_derate_multiplier;
        }

        reqTorque = fminf(reqTorque, maxFastTorque_Nm);
        reqTorque = fmaxf(reqTorque, 0.0f);
        setTorqueLimsAllProtected(reqTorque, 0.0f);
        setVelocityInt16All(maxFastSpeed_rpm);
    }
    // Regen button not pressed, set zero torque
    else if (!regen_button_pressed) {
        setTorqueLimsAllProtected(0.0f, 0.0f);
        setVelocityInt16All(0);
    }
    // Requested recuperation that is less than the maximum-power regen point possible
    else if (reqTorque > recuperative_limit) {
        setTorqueLimsAllProtected(0.0f, reqTorque);
        setVelocityInt16All(0);
    }
    // Requested recuperation is even more negative than the limit
    else {
        setTorqueLimsAllProtected(0.0f, recuperative_limit);
        setVelocityInt16All(0);
    }
}

void setEnduranceTestTorque(
    int32_t avgMotorSpeed_RPM,
    uint8_t throttlePos_u8,
    uint8_t brakePos_u8,
    int16_t swAngle_deg,
    int32_t battVoltage_mV,
    int32_t battCurrent_mA,
    uint8_t brakePressurePsi_u8,
    bool clampbyside
) {
     // if braking
    if (setRegen(&throttlePos_u8, brakePressurePsi_u8, avgMotorSpeed_RPM)){
        return;
    }

    // Requested torque that may be positive or negative
    float reqTorque = maxFastTorque_Nm * ((float)throttlePos_u8) / ((float)UINT8_MAX);
    const float recuperative_limit = getMotorRegenerativeCapacity(avgMotorSpeed_RPM);

    // Accelerative torque requested
    if (reqTorque >= 0) {
        // apply power limit. Simply scale power down linearly in the last 5kW of power
        uint8_t power_limit_kW = 150; // uint8, must be between 0 and 255, inclusive
        const bool ret_val = getProcessedValue(&power_limit_kW, POWER_LIM_INDEX, unsigned_integer);
        (void)ret_val; // placate compiler
        if (power_limit_kW == 0) {
            power_limit_kW = 1; // lower-bound the power limit by 1kW to avoid divide-by-zero
        }

        const float power_limit_W = ((float)power_limit_kW) * 1e3f;
        float power_limit_start_derate_W = power_limit_W - 5000.0f;
        power_limit_start_derate_W = fmaxf(power_limit_start_derate_W, 0.0f); // clamp to zero in case of negative value due to lower than 10kw limit

        volatile cmr_canHVIHeartbeat_t *HVISense = canTractiveGetPayload(CANRX_TRAC_HVI_SENSE);
        const float hv_voltage_V = ((float)(HVISense->packVoltage_cV)) * 1e-2f; // convert to volts

        volatile cmr_canVSMSensors_t *vsmSensor = canVehicleGetPayload(CANRX_VEH_VSM_SENSORS);
        const float currentA = ((float)(vsmSensor->hallEffect_cA)) * 1e-2f; // convert to amps
        // apply power limit.
        const float power_consumed_W = hv_voltage_V * currentA;

        if (power_consumed_W > power_limit_start_derate_W) {
            float power_derate_multiplier = (power_consumed_W - power_limit_start_derate_W) / (power_limit_W - power_limit_start_derate_W);
            // backup in case of hysterisis in the AC or latency in the system
            power_derate_multiplier = fminf(power_derate_multiplier, 1.0f);
            power_derate_multiplier = fmaxf(power_derate_multiplier, 0.0f);
            reqTorque *= 1.0f - power_derate_multiplier;
        }

        reqTorque = fminf(reqTorque, maxFastTorque_Nm);
        reqTorque = fmaxf(reqTorque, 0.0f);

        // Adjust throttle so that traction control commands reqTorque (after power limit calc) to the motors
        uint8_t adjustedThrottlePos_u8 = (uint8_t)(fminf(fmaxf((reqTorque * ((float)UINT8_MAX) / maxFastTorque_Nm), 0.0f), (float)UINT8_MAX));
        const bool assumeNoTurn = true; // TC is not allowed to behave left-right asymmetrically due to the lack of testing
        const bool ignoreYawRate = false; // TC takes yaw rate into account to prevent the vehicle from stopping unintendedly when turning at low speeds
        const bool allowRegen = true; // regen-braking is allowed to protect the AC by keeping charge level high
        const float critical_speed_mps = 5.0f; // using a high value to prevent the vehicle from stopping unintendedly when turning at low speeds
        if (brakePressurePsi_u8 < braking_threshold_psi)
        	// setFastTorque(adjustedThrottlePos_u8);
            setYawRateControl(throttlePos_u8, brakePressurePsi_u8, swAngle_deg, clampbyside);
        //setYawRateAndTractionControl(adjustedThrottlePos_u8, brakePressurePsi_u8, swAngle_deg, assumeNoTurn, ignoreYawRate, allowRegen, critical_speed_mps);
    }
    // Requested recuperation that is less than the maximum-power regen point possible
    else if (reqTorque > recuperative_limit) {
        setTorqueLimsAllProtected(0.0f, reqTorque);
        setVelocityInt16All(0);
    }
    // Requested recuperation is even more negative than the limit
    else {
        setTorqueLimsAllProtected(0.0f, recuperative_limit);
        setVelocityInt16All(0);
    }
}
