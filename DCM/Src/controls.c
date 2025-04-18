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
#include "constants.h"
#include "controls.h"
#include "motors.h"
#include "lut_3d.h"
#include "safety_filter.h"
#include "CMR/can_types.h"
#include "../optimizer/optimizer.h"
#include "movella.h"
#include "lut.h"
#include "constants.h"

#define PI 3.1415926535897932384626f

#define X1000_INT16(x) ((int16_t)((float)x * 1000.0f))


// ------------------------------------------------------------------------------------------------
// Globals

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

/** @brief FF Launch Control Start Tick and Button Released  **/
static TickType_t startTickCount;
static bool launchControlButtonPressed = false;
static bool launchControlActive = false;

/** @brief CAN data for CVXGEN*/
volatile cmr_can_solver_inputs_t solver_inputs;
volatile cmr_can_solver_aux_t solver_aux;
volatile cmr_can_solver_settings_t solver_settings;
volatile cmr_canCDCWheelTorque_t solver_torques;

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

float getYawRateControlLeftRightBias(int32_t swAngle_millideg);

/** @brief Coulomb counting info **/
static TickType_t previousTickCount;

// ------------------------------------------------------------------------------------------------
// Function implementations
void setLaunchControl(
	uint8_t throttlePos_u8,
	uint16_t brakePressurePsi_u8,
	int32_t swAngle_millideg, /** IGNORED if assumeNoTurn is true */
	float leftRightBias_Nm, /** IGNORED UNLESS traction_control_mode (defined in the function) is TC_MODE_TORQUE */
	bool assumeNoTurn,
	bool ignoreYawRate,
	bool allowRegen,
	float critical_speed_mps
);
/** @brief initialize yaw rate control */
static void initYawRateControl() {
    // read yrc_kp from DIM
    yrc_kp = 1.0f;
    // getProcessedValue(&yrc_kp, YRC_KP_INDEX, float_1_decimal);

    yrc_kp = yrc_kp*100.0f;
    // yrc_kp = 200;
    //yrcDebug = getPidDebug();
    yrcDebug.controls_pid = yrc_kp;
    // set yrc_ki to 0 because we don't aim to eliminate steady-state error
    //REMOVE const float yrc_ki = 0.0f;

    // disable derivate separation because no significant derivative kick was observed
    // steering angle and the car's yaw seem to have similar timescales
    //REMOVE const bool enable_derivative_separation = false;
}

static void load_solver_settings() {
	float k_lin = 0, k_yaw = 0, k_tie = 0;

	// Hot fix: interpret raw k_lin and k_yaw values as integers.
	if(getProcessedValue(&k_lin, K_LIN_INDEX, float_1_decimal)) {
		solver_set_k_lin(k_lin); // [0, 25.5].
	}

	if(getProcessedValue(&k_yaw, K_YAW_INDEX, float_1_decimal)) {
		solver_set_k_yaw(k_yaw); // [0, 25.5].
	}

    if(getProcessedValue(&k_tie, K_TIE_INDEX, float_1_decimal)) {
        solver_set_k_tie(k_tie); // [0, 25.5].
    }
}

/** @brief initialize controls */
void initControls() {
    initYawRateControl();
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

// Placeholder for now.
static float get_motor_regen_capacity(float wheelspeed_radps) {
    return -maxTorque_continuous_stall_Nm;
}

// For sensor validation.
static void set_slow_motor_speed(float speed_mps, bool rear_only) {
    speed_mps = fmaxf(speed_mps, 0.0f);
    speed_mps = fminf(speed_mps, 7.5f);
    float target_rpm = speed_mps / (PI * effective_wheel_dia_m) * gear_ratio * 60.0f;
    cmr_torqueDistributionNm_t torquesPos_Nm;
    if(rear_only) {
        setVelocityInt16(MOTOR_FL, 0);
        setVelocityInt16(MOTOR_FR, 0);
        setVelocityInt16(MOTOR_RL, (int16_t) target_rpm);
        setVelocityInt16(MOTOR_RR, (int16_t) target_rpm);
        torquesPos_Nm.fl = 0.0f;
        torquesPos_Nm.fr = 0.0f;
        torquesPos_Nm.rl = maxSlowTorque_Nm;
        torquesPos_Nm.rr = maxSlowTorque_Nm;
    } else {
        setVelocityInt16(MOTOR_FL, (int16_t) target_rpm);
        setVelocityInt16(MOTOR_FR, (int16_t) target_rpm);
        setVelocityInt16(MOTOR_RL, (int16_t) target_rpm);
        setVelocityInt16(MOTOR_RR, (int16_t) target_rpm);
        torquesPos_Nm.fl = maxSlowTorque_Nm;
        torquesPos_Nm.fr = maxSlowTorque_Nm;
        torquesPos_Nm.rl = maxSlowTorque_Nm;
        torquesPos_Nm.rr = maxSlowTorque_Nm;
    }
	cmr_torqueDistributionNm_t torquesNeg_Nm = {
        .fl = 0.0f,
        .fr = 0.0f,
        .rl = 0.0f,
        .rr = 0.0f,
    };
    setTorqueLimsProtected(&torquesPos_Nm, &torquesNeg_Nm);
}

static inline void set_motor_speed_and_torque(
    motorLocation_t motor,
    float val,
    cmr_torqueDistributionNm_t *torquesPos_Nm,
    cmr_torqueDistributionNm_t *torquesNeg_Nm
) {
    if(val > 0.0) {
        switch (motor)
        {
            case MOTOR_FL:
                torquesPos_Nm->fl = val;
                torquesNeg_Nm->fl = 0.0f;
                break;
            case MOTOR_FR:
                torquesPos_Nm->fr = val;
                torquesNeg_Nm->fr = 0.0f;
                break;
            case MOTOR_RL:
                torquesPos_Nm->rl = val;
                torquesNeg_Nm->rl = 0.0f;
                break;
            case MOTOR_RR:
                torquesPos_Nm->rr = val;
                torquesNeg_Nm->rr = 0.0f;
                break;
            default:
                assert(false);
        }
        setVelocityInt16(motor, maxFastSpeed_rpm);

    } else {
        switch (motor)
        {
            case MOTOR_FL:
                torquesPos_Nm->fl = 0.0f;
                torquesNeg_Nm->fl = val;
                break;
            case MOTOR_FR:
                torquesPos_Nm->fr = 0.0f;
                torquesNeg_Nm->fr = val;
                break;
            case MOTOR_RL:
                torquesPos_Nm->rl = 0.0f;
                torquesNeg_Nm->rl = val;
                break;
            case MOTOR_RR:
                torquesPos_Nm->rr = 0.0f;
                torquesNeg_Nm->rr = val;
                break;
            default:
                assert(false);
        }
        setVelocityInt16(motor, 0);
    }
}

static float get_load_cell_angle_rad(canDaqRX_t loadIndex) {
    switch (loadIndex)
    {
    case CANRX_DAQ_LOAD_FL:
    case CANRX_DAQ_LOAD_FR:
        return 35.0f / 180.0f * PI;
    case CANRX_DAQ_LOAD_RL:
    case CANRX_DAQ_LOAD_RR:
        return 30.0f / 180.0f * PI;
    default:
        return 0.0f;
    }
}

/**
 * @brief Return downforce given motor location
 */
static float get_downforce(canDaqRX_t loadIndex, bool use_true_downforce) {
    float downforce_N;
    bool not_timeout = cmr_canRXMetaTimeoutWarn(&canDaqRXMeta[loadIndex],  xTaskGetTickCount()) == 0;
    if (use_true_downforce && not_timeout) {
        volatile cmr_canIZZELoadCell_t *downforcePayload = (volatile cmr_canIZZELoadCell_t*) canDAQGetPayload(loadIndex);
        float angle = get_load_cell_angle_rad(loadIndex);
        volatile int16_t raw = parse_int16(&downforcePayload->force_output_N);
        downforce_N = (float) raw * 0.1f * sinf(angle);
    } else {
        downforce_N = (float) car_mass_kg * 9.81f * 0.25f;
    }
    return downforce_N;
}

/**
 * @param normalized_throttle A value in [-1, 1].
 * In [0, 1] if without regen.
 */
static void set_optimal_control(
	float normalized_throttle,
	int32_t swAngle_millideg_FL,
    int32_t swAngle_millideg_FR,
    bool allow_regen
) {

    int32_t swAngle_millideg = (swAngle_millideg_FL + swAngle_millideg_FR) / 2;

    if (true == allow_regen) {
        assert(-1.0f <= normalized_throttle && normalized_throttle <= 1.0f);
    } else {
        assert(0.0f <= normalized_throttle && normalized_throttle <= 1.0f);
    }

    load_solver_settings();

	float wheel_fl_speed_radps = getMotorSpeed_radps(MOTOR_FL);
	float wheel_fr_speed_radps = getMotorSpeed_radps(MOTOR_FR);
	float wheel_rl_speed_radps = getMotorSpeed_radps(MOTOR_RL);
	float wheel_rr_speed_radps = getMotorSpeed_radps(MOTOR_RR);
    
	// float tractive_cap_fl = getKappaFxGlobalMax(MOTOR_FL, UINT8_MAX, true).Fx;
	// float tractive_cap_fr = getKappaFxGlobalMax(MOTOR_FR, UINT8_MAX, true).Fx;
	// float tractive_cap_rl = getKappaFxGlobalMax(MOTOR_RL, UINT8_MAX, true).Fx;
	// float tractive_cap_rr = getKappaFxGlobalMax(MOTOR_RR, UINT8_MAX, true).Fx;

    bool use_true_downforce = true;
    float tractive_cap_fl = lut_get_max_Fx_kappa(0.0, get_downforce(CANRX_DAQ_LOAD_FL, use_true_downforce)).Fx;
    float tractive_cap_fr = lut_get_max_Fx_kappa(0.0, get_downforce(CANRX_DAQ_LOAD_FR, use_true_downforce)).Fx;
    float tractive_cap_rl = lut_get_max_Fx_kappa(0.0, get_downforce(CANRX_DAQ_LOAD_RL, use_true_downforce)).Fx;
    float tractive_cap_rr = lut_get_max_Fx_kappa(0.0, get_downforce(CANRX_DAQ_LOAD_RR, use_true_downforce)).Fx;

	// The most naive approach is to convert force to torque linearly, ignoring rolling resistance and any inefficiency.
	float torque_limit_fl = tractive_cap_fl * effective_wheel_rad_m / gear_ratio;
	float torque_limit_fr = tractive_cap_fr * effective_wheel_rad_m / gear_ratio;
	float torque_limit_rl = tractive_cap_rl * effective_wheel_rad_m / gear_ratio;
	float torque_limit_rr = tractive_cap_rr * effective_wheel_rad_m / gear_ratio;

	torque_limit_fl = fminf(tractive_cap_fl, maxTorque_continuous_stall_Nm);
	torque_limit_fr = fminf(tractive_cap_fr, maxTorque_continuous_stall_Nm);
	torque_limit_rl = fminf(tractive_cap_rl, maxTorque_continuous_stall_Nm);
	torque_limit_rr = fminf(tractive_cap_rr, maxTorque_continuous_stall_Nm);

	static optimizer_state_t optimizer_state;

	optimizer_state.power_limit = getPowerLimit_W();
	optimizer_state.omegas[0] = wheel_fl_speed_radps;
	optimizer_state.omegas[1] = wheel_fr_speed_radps;
	optimizer_state.omegas[2] = wheel_rl_speed_radps;
	optimizer_state.omegas[3] = wheel_rr_speed_radps;

    if(true == allow_regen) {
        optimizer_state.variable_profile[0].lower = get_motor_regen_capacity(wheel_fl_speed_radps);
        optimizer_state.variable_profile[1].lower = get_motor_regen_capacity(wheel_fr_speed_radps);
        optimizer_state.variable_profile[2].lower = get_motor_regen_capacity(wheel_rl_speed_radps);
        optimizer_state.variable_profile[3].lower = get_motor_regen_capacity(wheel_rr_speed_radps);
    } else {
        optimizer_state.variable_profile[0].lower = 0.0;
        optimizer_state.variable_profile[1].lower = 0.0;
        optimizer_state.variable_profile[2].lower = 0.0;
        optimizer_state.variable_profile[3].lower = 0.0;
    }

	optimizer_state.variable_profile[0].upper = torque_limit_fl;
	optimizer_state.variable_profile[1].upper = torque_limit_fr;
	optimizer_state.variable_profile[2].upper = torque_limit_rl;
	optimizer_state.variable_profile[3].upper = torque_limit_rr;

	const float thoeretical_mass_accel = maxTorque_continuous_stall_Nm * MOTOR_LEN * gear_ratio / effective_wheel_rad_m / car_mass_kg;
	// areq can be either expressed in torque or actual accel. Both ways are equivalent. Here uses actual accel.
	optimizer_state.areq = normalized_throttle * thoeretical_mass_accel;
    // Solver treats Mreq as around -z axis.
	optimizer_state.mreq = -getYawRateControlLeftRightBias(swAngle_millideg);
    
	optimizer_state.theta_left = -swAngleMillidegToSteeringAngleRad(swAngle_millideg_FL);
    optimizer_state.theta_right = -swAngleMillidegToSteeringAngleRad(swAngle_millideg_FR);

	solve(&optimizer_state);

	// Logging solver outputs, x1000 to make it more intuitive.
	solver_torques.frontLeft_Nm = X1000_INT16(optimizer_state.optimal_assignment[0].val);
	solver_torques.frontRight_Nm = X1000_INT16(optimizer_state.optimal_assignment[1].val);
	solver_torques.rearLeft_Nm = X1000_INT16(optimizer_state.optimal_assignment[2].val);
	solver_torques.rearRight_Nm = X1000_INT16(optimizer_state.optimal_assignment[3].val);

    // Logging solver inputs.
	solver_inputs.lin_accel_Nm = optimizer_state.areq;
	solver_inputs.moment_req_Nm = optimizer_state.mreq;

    // Logging solver aux.
	solver_aux.combined_normalized_throttle = X1000_INT16(normalized_throttle);
	solver_aux.allow_regen = allow_regen;

    // Logging solver settings.
	solver_settings.k_lin = X1000_INT16(solver_get_k_lin());
	solver_settings.k_yaw = X1000_INT16(solver_get_k_yaw());
	solver_settings.k_tie = X1000_INT16(solver_get_k_tie());

	static cmr_torqueDistributionNm_t torquesPos_Nm;
	static cmr_torqueDistributionNm_t torquesNeg_Nm;

    if(true == allow_regen) {

        set_motor_speed_and_torque(MOTOR_FL, optimizer_state.optimal_assignment[0].val, &torquesPos_Nm, &torquesNeg_Nm);
        set_motor_speed_and_torque(MOTOR_FR, optimizer_state.optimal_assignment[1].val, &torquesPos_Nm, &torquesNeg_Nm);
        set_motor_speed_and_torque(MOTOR_RL, optimizer_state.optimal_assignment[2].val, &torquesPos_Nm, &torquesNeg_Nm);
        set_motor_speed_and_torque(MOTOR_RR, optimizer_state.optimal_assignment[3].val, &torquesPos_Nm, &torquesNeg_Nm);
        setTorqueLimsProtected(&torquesPos_Nm, &torquesNeg_Nm);
        // The API for setting speeds and torques is not optimal.
        // It should allow setting velocities the same way as setting torques, by passing a struct.

    } else {

        torquesPos_Nm.fl = optimizer_state.optimal_assignment[0].val;
        torquesPos_Nm.fr = optimizer_state.optimal_assignment[1].val;
        torquesPos_Nm.rl = optimizer_state.optimal_assignment[2].val;
        torquesPos_Nm.rr = optimizer_state.optimal_assignment[3].val;

        torquesNeg_Nm.fl = 0.0f;
        torquesNeg_Nm.fr = 0.0f;
        torquesNeg_Nm.rl = 0.0f;
        torquesNeg_Nm.rr = 0.0f;

        setVelocityInt16All(maxFastSpeed_rpm);
	    setTorqueLimsProtected(&torquesPos_Nm, &torquesNeg_Nm);

    }
}

void set_optimal_control_with_regen(
	int throttlePos_u8,
	int32_t swAngle_millideg_FL,
	int32_t swAngle_millideg_FR
) {
    uint8_t paddle_pressure = ((volatile cmr_canDIMActions_t *) canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON))->paddle;

    uint8_t paddle_regen_strength_raw = 100;
    // getProcessedValue(&paddle_regen_strength_raw, PADDLE_MAX_REGEN_INDEX, unsigned_integer);
    float paddle_regen_strength = paddle_regen_strength_raw * 0.01;

    float paddle_request = 0.0f;
    if (paddle_pressure > paddle_pressure_start) {
        paddle_request = ((float)(paddle_pressure - paddle_pressure_start)) / (UINT8_MAX - paddle_pressure_start);
        paddle_request *= paddle_regen_strength; // [0, 1].
    }

    float throttle = (float)throttlePos_u8 / UINT8_MAX;
    float combined_request = throttle - paddle_request; // [0, 1].
    set_optimal_control(combined_request, swAngle_millideg_FL, swAngle_millideg_FR, true);
}

static void set_regen(uint8_t throttlePos_u8) {
    uint8_t paddle_pressure = ((volatile cmr_canDIMActions_t *) canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON))->paddle;

    uint8_t paddle_regen_strength_raw = 100;
    // getProcessedValue(&paddle_regen_strength_raw, PADDLE_MAX_REGEN_INDEX, unsigned_integer);
    float paddle_regen_strength = paddle_regen_strength_raw * 0.01;

    float paddle_request = 0.0f;
    if (paddle_pressure > paddle_pressure_start) {
        paddle_request = ((float)(paddle_pressure - paddle_pressure_start)) / (UINT8_MAX - paddle_pressure_start);
        paddle_request *= paddle_regen_strength; // [0, 1].
    }

    float throttle = (float)throttlePos_u8 / UINT8_MAX;
    float combined_request = throttle - paddle_request; // [0, 1].

    static cmr_torqueDistributionNm_t torquesPos_Nm;
	static cmr_torqueDistributionNm_t torquesNeg_Nm;

    float torque_request_Nm = combined_request * maxFastTorque_Nm;
    float torque_request_fl_Nm;
    float torque_request_fr_Nm;
    float torque_request_rl_Nm;
    float torque_request_rr_Nm;
    if(torque_request_Nm < 0) {
        torque_request_fl_Nm = fmaxf(getMotorRegenerativeCapacity(getMotorSpeed_rpm(MOTOR_FL)), torque_request_Nm);
        torque_request_fr_Nm = fmaxf(getMotorRegenerativeCapacity(getMotorSpeed_rpm(MOTOR_FR)), torque_request_Nm);
        torque_request_rl_Nm = fmaxf(getMotorRegenerativeCapacity(getMotorSpeed_rpm(MOTOR_RL)), torque_request_Nm);
        torque_request_rr_Nm = fmaxf(getMotorRegenerativeCapacity(getMotorSpeed_rpm(MOTOR_RR)), torque_request_Nm);
    } else {
        torque_request_fl_Nm = torque_request_Nm;
        torque_request_fr_Nm = torque_request_Nm;
        torque_request_rl_Nm = torque_request_Nm;
        torque_request_rr_Nm = torque_request_Nm;
    }

    set_motor_speed_and_torque(MOTOR_FL, torque_request_fl_Nm, &torquesPos_Nm, &torquesNeg_Nm);
    set_motor_speed_and_torque(MOTOR_FR, torque_request_fr_Nm, &torquesPos_Nm, &torquesNeg_Nm);
    set_motor_speed_and_torque(MOTOR_RL, torque_request_rl_Nm, &torquesPos_Nm, &torquesNeg_Nm);
    set_motor_speed_and_torque(MOTOR_RR, torque_request_rr_Nm, &torquesPos_Nm, &torquesNeg_Nm);
    setTorqueLimsProtected(&torquesPos_Nm, &torquesNeg_Nm);
}

/**
 * @brief Runs control loops and sets motor torque limits and velocity targets accordingly.
 *
 * @param gear Which gear the vehicle is in.
 * @param throttlePos_u8 Throttle position, 0-255.
 * @param brakePos_u8 Brake position, 0-255.
 * @param swAngle_millideg Steering wheel angle in degrees. Zero-centered, right turn positive.
 * @param battVoltage_mV Accumulator voltage in millivolts.
 * @param battCurrent_mA Accumulator current in milliamps.
 * @param blank_command Additional signal that forces the motor commands to zero vel. and zero torque
 */
void runControls (
    cmr_canGear_t gear,
    uint8_t throttlePos_u8,
    uint8_t brakePos_u8,
    uint16_t brakePressurePsi_u8,
    int32_t swAngle_millideg_FL,
    int32_t swAngle_millideg_FR,
    int32_t battVoltage_mV,
    int32_t battCurrent_mA,
    bool blank_command )
{

    int32_t swAngle_millideg = (swAngle_millideg_FL + swAngle_millideg_FR) / 2;
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
            setSlowTorque(throttlePos_u8, swAngle_millideg);
            break;
        }
        case CMR_CAN_GEAR_FAST: {
            setFastTorque(throttlePos_u8);
            break;
        }
        case CMR_CAN_GEAR_ENDURANCE: {
            // set_optimal_control_with_regen(throttlePos_u8, swAngle_millideg_FL, swAngle_millideg_FR);
            // setFastTorqueWithParallelRegen(brakePressurePsi_u8, throttlePos_u8);
            set_regen(throttlePos_u8);
            // float avgMotorSpeed_RPM = getTotalMotorSpeed_rpm()* 0.25f;
            // uint8_t paddle_pressure = ((volatile cmr_canDIMActions_t *) canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON))->paddle;
            // uint8_t paddle_regen_strength_raw = 100;
            // float paddle_regen_strength = paddle_regen_strength_raw * 0.01;
            // uint8_t throttlePos_u8_temp = throttlePos_u8;
            // setPaddleRegen(&throttlePos_u8_temp, brakePressurePsi_u8, avgMotorSpeed_RPM, paddle_pressure, paddle_regen_strength);
            break;
        }
        case CMR_CAN_GEAR_AUTOX: {
            // const bool assumeNoTurn = true; // TC is not allowed to behave left-right asymmetrically due to the lack of testing
            // const bool ignoreYawRate = false; // TC takes yaw rate into account to prevent the vehicle from stopping unintendedly when turning at low speeds
            // const bool allowRegen = true; // regen-braking is allowed to protect the AC by keeping charge level high
            // const float critical_speed_mps = 5.0f; // using a high value to prevent the vehicle from stopping unintendedly when turning at low speeds
            const bool clampbyside = true;
            setYawRateControl(throttlePos_u8, brakePressurePsi_u8, swAngle_millideg, clampbyside);
            //setYawRateAndTractionControl(throttlePos_u8, brakePressurePsi_u8, swAngle_millideg, assumeNoTurn, ignoreYawRate, allowRegen, critical_speed_mps);
            break;
        }
        case CMR_CAN_GEAR_SKIDPAD: {
        	set_optimal_control((float)throttlePos_u8 / UINT8_MAX, swAngle_millideg_FL, swAngle_millideg_FR, false);
            break;
        }
        case CMR_CAN_GEAR_ACCEL: {
            const bool assumeNoTurn = true; // TC is not allowed to behave left-right asymmetrically because it's meaningless in accel
            const bool ignoreYawRate = true;  // TC ignores yaw rate because it's meaningless in accel
            const bool allowRegen = false; // regen-braking is not allowed because it's meaningless in accel
            const float critical_speed_mps = 0.0f; // using a low value to prevent excessive wheel spin at low speeds
            const float leftRightBias_Nm = 0.0f; // YRC is not enabled for accel, so there should be no left-right torque bias
            setLaunchControl(throttlePos_u8, brakePressurePsi_u8, swAngle_millideg, leftRightBias_Nm, assumeNoTurn, ignoreYawRate, allowRegen, critical_speed_mps);
            // Dry testing only.
            // setLaunchControl(255, 0, 0, leftRightBias_Nm, assumeNoTurn, ignoreYawRate, allowRegen, critical_speed_mps);
            break;
        }
        case CMR_CAN_GEAR_TEST: {
            set_optimal_control_with_regen(throttlePos_u8, swAngle_millideg_FL, swAngle_millideg_FR);
            // float target_speed_mps = 5.0f;
            // getProcessedValue(&target_speed_mps, FFLAUNCH_FEEDBACK_INDEX, float_1_decimal);
            // set_slow_motor_speed(target_speed_mps, false);
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

void test_solver(uint8_t throttlePos_u8, uint8_t swAngle_millideg, bool clampbyside) {

	const float throttle_pos_torque_Nm = maxFastTorque_Nm * (float)(throttlePos_u8) / (float)(UINT8_MAX);

	// calculate left-right torque bias, positive values increase torque output on the right side and decreases that on the left side
	float left_right_torque_bias_Nm = getYawRateControlLeftRightBias(swAngle_millideg) * 0.5f; // halved because the bias will be applied to two wheels per side
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

		float wheelVelocityRPM = maxFastSpeed_rpm;//gear_ratio * 60.0f * scheduledBodyVel_mps / (2 * M_PI * effective_wheel_rad_m);
		float rearWhlAvgVelocity_mps = (rearWhlVelocity_RL_mps + rearWhlVelocity_RR_mps) / 2.0f;
		float frontWhlTargetVelocity_mps = rearWhlAvgVelocity_mps + 0.1f;
		float frontWheelVelocityRPM = gear_ratio * 60.0f * frontWhlTargetVelocity_mps / (2 * M_PI * effective_wheel_rad_m);

		setVelocityFloat(MOTOR_FL, wheelVelocityRPM); // Converts rad/s to rpm
		setVelocityFloat(MOTOR_FR, wheelVelocityRPM);
		setVelocityFloat(MOTOR_RL, wheelVelocityRPM);
		setVelocityFloat(MOTOR_RR, wheelVelocityRPM);

	} else { // clamp front left to rear left and front right to rear right

		float rearWhlVelocity_RL_RPM = getMotorSpeed_radps(MOTOR_RL) / (2 * M_PI) * 60;
		float rearWhlVelocity_RR_RPM = getMotorSpeed_radps(MOTOR_RR) / (2 * M_PI) * 60;

		float wheelVelocityRPM = maxFastSpeed_rpm;//gear_ratio * 60.0f * scheduledBodyVel_mps / (2 * M_PI * effective_wheel_rad_m);
		// float rearWhlAvgVelocity_mps = (rearWhlVelocity_RL_mps + rearWhlVelocity_RR_mps) / 2.0f;
		// float frontWhlTargetVelocity_mps = rearWhlAvgVelocity_mps + 0.1f;
		// float frontWheelVelocityRPM = gear_ratio * 60.0f * frontWhlTargetVelocity_mps / (2 * M_PI * effective_wheel_rad_m);

		float clamped_FL_RPM = fminf(rearWhlVelocity_RL_RPM*1.12f     , (rearWhlVelocity_RR_RPM)*1.12f+3000) + 0.1f;
		float clamped_FR_RPM = fminf((rearWhlVelocity_RL_RPM*1.12f)+3000, rearWhlVelocity_RR_RPM*1.12f) + 0.1f;//why? slip ratio vibes

		setVelocityFloat(MOTOR_FL, clamped_FL_RPM); // Converts rad/s to rpm
		setVelocityFloat(MOTOR_FR, clamped_FR_RPM);
		setVelocityFloat(MOTOR_RL, wheelVelocityRPM);
		setVelocityFloat(MOTOR_RR, wheelVelocityRPM);
	}


}

/**
 * @brief Sets motor torques and velocities according to speed limit for slow gear.
 *
 * @param throttlePos_u8 Throttle position, 0-255.
 */
void setSlowTorque (
    uint8_t throttlePos_u8,
    int32_t swAngle_millideg
) {
    const float reqTorque = maxSlowTorque_Nm * (float)(throttlePos_u8) / (float)(UINT8_MAX);
    // setTorqueLimsAllProtected(reqTorque, 0.0f);

    setTorqueLimsUnprotected(MOTOR_FL, reqTorque, 0.0f);
    setTorqueLimsUnprotected(MOTOR_FR, reqTorque, 0.0f);
    setTorqueLimsUnprotected(MOTOR_RR, reqTorque, 0.0f);
    setTorqueLimsUnprotected(MOTOR_RL, reqTorque, 0.0f);

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
//    setTorqueLimsAllProtected(reqTorque, 0.0f);
   setTorqueLimsUnprotected(MOTOR_FL, reqTorque, 0.0f);
   setTorqueLimsUnprotected(MOTOR_FR, reqTorque, 0.0f);
   setTorqueLimsUnprotected(MOTOR_RR, reqTorque, 0.0f);
   setTorqueLimsUnprotected(MOTOR_RL, reqTorque, 0.0f);
    setVelocityInt16All(maxFastSpeed_rpm);
}

void setFastTorqueWithParallelRegen(uint16_t brakePressurePsi_u8, uint8_t throttlePos_u8)
{
    if (brakePressurePsi_u8 >= braking_threshold_psi) {
        setParallelRegen(throttlePos_u8, brakePressurePsi_u8, 0);
    }
    else {
        const float reqTorque = maxFastTorque_Nm * (float)(throttlePos_u8) / (float)(UINT8_MAX);
        setTorqueLimsAllProtected(reqTorque, 0.0f);
        setVelocityInt16All(maxFastSpeed_rpm);
    }
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
    frontWhlSetpoints.omega_FL = ((V_fl + critical_speed_mps) * slip_ratio_fl + V_fl) / effective_wheel_rad_m;
    frontWhlSetpoints.omega_FR = ((V_fr + critical_speed_mps) * slip_ratio_fr + V_fr) / effective_wheel_rad_m;
    rearWhlSetpoints.omega_RL = ((V_rl + critical_speed_mps) * slip_ratio_rl + V_rl) / effective_wheel_rad_m;
    rearWhlSetpoints.omega_RR = ((V_rr + critical_speed_mps) * slip_ratio_rr + V_rr) / effective_wheel_rad_m;
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

    float scheduleVelocity_mps2 = 11.29;
    // getProcessedValue(&scheduleVelocity_mps2, K_EFF_INDEX, float_1_decimal);

    if (t_sec < 0.0f) {
        scheduleVelocity_mps = 0.0f;
    } else if(t_sec < tMax) {
        float startingVel_mps = 0.0;
        scheduleVelocity_mps = (scheduleVelocity_mps2 * t_sec) + startingVel_mps;
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
	uint16_t brakePressurePsi_u8,
	int32_t swAngle_millideg, /** IGNORED if assumeNoTurn is true */
	float leftRightBias_Nm, /** IGNORED UNLESS traction_control_mode (defined in the function) is TC_MODE_TORQUE */
	bool assumeNoTurn,
	bool ignoreYawRate,
	bool allowRegen,
	float critical_speed_mps
){
	static const float launch_control_speed_threshold_mps = 0.05f;
	static const float launch_control_max_duration_s = 5.0;
    static const bool use_solver = false;

	bool action_button_pressed = false; 
	const float nonnegative_odometer_velocity_mps = motorSpeedToWheelLinearSpeed_mps(getTotalMotorSpeed_radps() * 0.25f);
	if (nonnegative_odometer_velocity_mps < launch_control_speed_threshold_mps) { // odometer velocity is below the launch control threshold
		action_button_pressed = (((volatile cmr_canDIMActions_t *)(canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON)))->buttons) & BUTTON_ACT;

		if (action_button_pressed) {
			launchControlButtonPressed = true;
		}

		if(launchControlButtonPressed && !action_button_pressed) {

			if(!launchControlActive)
			{
				startTickCount = xTaskGetTickCount();
				launchControlActive = true;
			}
		}
	}

    // Not braking, throttle engaged, no button pressed, launch control is active.
    // bool ready_to_accel = brakePressurePsi_u8 < braking_threshold_psi && throttlePos_u8 > 0 && !action_button_pressed && launchControlActive;
    bool ready_to_accel = throttlePos_u8 > 0 && !action_button_pressed && launchControlActive;
    if(false == ready_to_accel) {
        //setTorqueLimsAllProtected(0.0f, 0.0f);
        setTorqueLimsUnprotected(MOTOR_FL, 0.0, 0.0f);
        setTorqueLimsUnprotected(MOTOR_FR, 0.0, 0.0f);
        setTorqueLimsUnprotected(MOTOR_RR, 0.0, 0.0f);
        setTorqueLimsUnprotected(MOTOR_RL, 0.0, 0.0f);
		setVelocityInt16All(0);
        return;
    }

    float time_s = (float)(xTaskGetTickCount() - startTickCount) * (0.001f);
	if(time_s >= launch_control_max_duration_s) {
        setTorqueLimsUnprotected(MOTOR_FL, 0.0, 0.0f);
        setTorqueLimsUnprotected(MOTOR_FR, 0.0, 0.0f);
        setTorqueLimsUnprotected(MOTOR_RR, 0.0, 0.0f);
        setTorqueLimsUnprotected(MOTOR_RL, 0.0, 0.0f);
		setVelocityInt16All(0);
		return;
	}

    if (use_solver) {
        // swAngle_millideg = 0 means assume no turn.
        set_optimal_control((float) throttlePos_u8 / UINT8_MAX, 0, 0, false);
    } else {
        TickType_t tick = xTaskGetTickCount();
        float seconds = (float)(tick - startTickCount) * (0.001f); //convert Tick Count to Seconds

        float scheduled_wheel_vel_mps = getFFScheduleVelocity(seconds);
        float motor_rpm = gear_ratio * 60.0f * scheduled_wheel_vel_mps / (2 * M_PI * effective_wheel_rad_m);
        setVelocityFloat(MOTOR_FL, motor_rpm);
        setVelocityFloat(MOTOR_FR, motor_rpm);
        setVelocityFloat(MOTOR_RL, motor_rpm);
        setVelocityFloat(MOTOR_RR, motor_rpm);

        const float reqTorque = maxFastTorque_Nm * (float)(throttlePos_u8) / (float)(UINT8_MAX);
        cmr_torqueDistributionNm_t pos_torques_Nm = {.fl = reqTorque, .fr = reqTorque, .rl = reqTorque, .rr = reqTorque};
        cmr_torqueDistributionNm_t neg_torques_Nm = {.fl = 0.0f, .fr = 0.0f, .rl = 0.0f, .rr = 0.0f};
        setTorqueLimsUnprotected(MOTOR_FL, pos_torques_Nm.fl, neg_torques_Nm.fl);
        setTorqueLimsUnprotected(MOTOR_FR, pos_torques_Nm.fr, neg_torques_Nm.fr);
        setTorqueLimsUnprotected(MOTOR_RR, pos_torques_Nm.rr, neg_torques_Nm.rr);
        setTorqueLimsUnprotected(MOTOR_RL, pos_torques_Nm.rl, neg_torques_Nm.rl);
        return;
    }
}

/**
 * @brief Set wheelspeed setpoints and torque limits based on the traction control algorithm
 *
 * @param throttlePos_u8 Throttle position, 0-255
 * @param brakePressurePsi_u8 Brake Pressure PSI
 * @param swAngle_millideg Steering wheel angle, IGNORED if assumeNoTurn is true
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
    uint16_t brakePressurePsi_u8,
    int32_t swAngle_millideg, /** IGNORED if assumeNoTurn is true */
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
        swAngle_millideg = 0;
    }
    const float steering_angle_rad = swAngleMillidegToSteeringAngleRad(swAngle_millideg);

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
        // WHY???
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
    setVelocityFloat(MOTOR_FL, frontWhlSetpoints.omega_FL * gear_ratio * 60.0f / (2.0f * M_PI)); // Converts rad/s to rpm
    setVelocityFloat(MOTOR_FR, frontWhlSetpoints.omega_FR * gear_ratio * 60.0f / (2.0f * M_PI));
    setVelocityFloat(MOTOR_RL, rearWhlSetpoints.omega_RL * gear_ratio * 60.0f / (2.0f * M_PI));
    setVelocityFloat(MOTOR_RR, rearWhlSetpoints.omega_RR * gear_ratio * 60.0f / (2.0f * M_PI));

    // set torques
    setTorqueLimsProtected(&pos_torques_Nm, &neg_torques_Nm);
}

float get_optimal_yaw_rate(float swangle_rad, float velocity_x_mps) {
    
    static const float natural_understeer_gradient = 0.011465f; //rad/g

    const float distance_between_axles_m = chassis_a + chassis_b;
    const float yaw_rate_setpoint_radps = swangle_rad * velocity_x_mps /
        (distance_between_axles_m + velocity_x_mps * velocity_x_mps * natural_understeer_gradient);
    
    return yaw_rate_setpoint_radps;
}

/**
 * @brief Calculate the control action (left-right torque bias) of the yaw rate controller
 * @param swAngle_millideg Steering wheel angle
 */
float getYawRateControlLeftRightBias(int32_t swAngle_millideg) {

    float velocity_x_mps;
    if(movella_state.status.gnss_fix) {
        velocity_x_mps = movella_state.velocity.x;
        yrcDebug.controls_bias = 1;
    } else {
        velocity_x_mps = getTotalMotorSpeed_radps() * 0.25f * effective_wheel_rad_m;
        yrcDebug.controls_bias = -1;
    }
    
    const float swangle_rad = swAngleMillidegToSteeringAngleRad(swAngle_millideg);
    const float actual_yaw_rate_radps_sae = movella_state.gyro.z;
    const float optimal_yaw_rate_radps = get_optimal_yaw_rate(swangle_rad, velocity_x_mps);

    yrcDebug.controls_current_yaw_rate = (int16_t)(1000.0f * actual_yaw_rate_radps_sae);
    yrcDebug.controls_target_yaw_rate = (int16_t)(1000.0f * optimal_yaw_rate_radps);
    yrcDebug.controls_pid = yrc_kp;
    
    const float left_right_bias = yrc_kp * (actual_yaw_rate_radps_sae - optimal_yaw_rate_radps);
    return left_right_bias;
}


/**
 * @brief Set wheelspeed setpoints and torque limits based on the yaw rate control algorithm
 *
 * @param throttlePos_u8 Throttle position, 0-255
 * @param brakePressurePsi_u8 Brake Pressure PSI
 * @param swAngle_millideg Steering wheel angle, IGNORED if assumeNoTurn is true
 */
void setYawRateControl (
    uint8_t throttlePos_u8,
    uint16_t brakePressurePsi_u8,
    int32_t swAngle_millideg,
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
    float left_right_torque_bias_Nm = getYawRateControlLeftRightBias(swAngle_millideg) * 0.5f; // halved because the bias will be applied to two wheels per side

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

        float wheelVelocityRPM = maxFastSpeed_rpm;//gear_ratio * 60.0f * scheduledBodyVel_mps / (2 * M_PI * effective_wheel_rad_m);
        float rearWhlAvgVelocity_mps = (rearWhlVelocity_RL_mps + rearWhlVelocity_RR_mps) / 2.0f;
        float frontWhlTargetVelocity_mps = rearWhlAvgVelocity_mps + 0.1f;
        float frontWheelVelocityRPM = gear_ratio * 60.0f * frontWhlTargetVelocity_mps / (2 * M_PI * effective_wheel_rad_m);

        setVelocityFloat(MOTOR_FL, wheelVelocityRPM); // Converts rad/s to rpm
        setVelocityFloat(MOTOR_FR, wheelVelocityRPM);
        setVelocityFloat(MOTOR_RL, wheelVelocityRPM);
        setVelocityFloat(MOTOR_RR, wheelVelocityRPM);

    } else { // clamp front left to rear left and front right to rear right

        float rearWhlVelocity_RL_RPM = getMotorSpeed_radps(MOTOR_RL) / (2 * M_PI) * 60;
        float rearWhlVelocity_RR_RPM = getMotorSpeed_radps(MOTOR_RR) / (2 * M_PI) * 60;

        float wheelVelocityRPM = maxFastSpeed_rpm;//gear_ratio * 60.0f * scheduledBodyVel_mps / (2 * M_PI * effective_wheel_rad_m);
        // float rearWhlAvgVelocity_mps = (rearWhlVelocity_RL_mps + rearWhlVelocity_RR_mps) / 2.0f;
        // float frontWhlTargetVelocity_mps = rearWhlAvgVelocity_mps + 0.1f;
        // float frontWheelVelocityRPM = gear_ratio * 60.0f * frontWhlTargetVelocity_mps / (2 * M_PI * effective_wheel_rad_m);

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
 * @param swAngle_millideg Steering wheel angle, IGNORED FOR TC if assumeNoTurn is true
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
    uint16_t brakePressurePsi_u8,
    int32_t swAngle_millideg, /* IGNORED FOR TC if assumeNoTurn is true */
    bool assumeNoTurn,
    bool ignoreYawRate,
    bool allowRegen,
    float critical_speed_mps
) {
    // calculate left-right torque bias, positive values increase torque output on the right side and decreases that on the left side
    const float left_right_torque_bias_Nm = getYawRateControlLeftRightBias(swAngle_millideg) * 0.5f; // halved because the bias will be applied to two wheels per side

    // set torques though the traction controller
    setTractionControl(throttlePos_u8, brakePressurePsi_u8, swAngle_millideg, left_right_torque_bias_Nm, assumeNoTurn, ignoreYawRate, allowRegen, critical_speed_mps);
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
    uint16_t brakePressurePsi_u8,
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
 * @param swAngle_millideg Steering wheel angle in degrees. Zero-centered, right turn positive.
 */
void setEnduranceTorque (
    int32_t avgMotorSpeed_RPM,
    uint8_t throttlePos_u8,
    uint8_t brakePos_u8,
    int32_t swAngle_millideg,
    int32_t battVoltage_V_hvc,
    int32_t battCurrent_A_hvc,
    uint16_t brakePressurePsi_u8
) {
    // if braking
    if (setRegen(&throttlePos_u8, brakePressurePsi_u8, avgMotorSpeed_RPM)){
        return;
    }

    // Determine aggrigate torque request be combining acceleration pedal position
    // with brake pedal position.
    const bool regen_button_pressed = (((volatile cmr_canDIMActions_t *) canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON))->buttons) & BUTTON_SCRN ;

    uint8_t pedal_regen_strength = 0;
    const float regentPcnt_f = ((float)pedal_regen_strength) * 1e-2; // convert a coefficient between 0 and 1
    getProcessedValue(&pedal_regen_strength, PEDAL_REGEN_STRENGTH_INDEX, unsigned_integer);

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
    int32_t swAngle_millideg,
    int32_t battVoltage_mV,
    int32_t battCurrent_mA,
    uint16_t brakePressurePsi_u8,
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
            setYawRateControl(throttlePos_u8, brakePressurePsi_u8, swAngle_millideg, clampbyside);
        //setYawRateAndTractionControl(adjustedThrottlePos_u8, brakePressurePsi_u8, swAngle_millideg, assumeNoTurn, ignoreYawRate, allowRegen, critical_speed_mps);
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
