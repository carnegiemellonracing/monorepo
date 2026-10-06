/**
 * @file controls.c
 * @brief Vehicle control loops.
 *
 * @author Carnegie Mellon Racing
 */

// ------------------------------------------------------------------------------------------------
// Includes
#include "CMR/can_types.h"
#include "CMR/utils.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include "constants.h"
#include "controls.h"
#include "controls_helper.h"
#include "motors.h"
#include "motors_helper.h"
#include "safety_filter.h"
#include "../optimizer/optimizer.h"
#include "26x_sensors.h"
#include "sensors.h"
#include "lut.h"

#define PI 3.1415926535897932384626f

#define X1000_INT16(x) ((int16_t)((float)x * 1000.0f))
#define INSPECTION_MISSION_TIME_MS 27000


// ------------------------------------------------------------------------------------------------
// Globals

/** @brief Yaw rate control kp */
volatile cmr_can_controls_pid_debug_t yrcDebug;
float yrc_pers = 120.0f;
float bias_margin = 12.0f; 
static float yrc_kp;
/// The maximum scaling factor applied to the phantom differential when turning.
static float maxPhantomDiffScalingFactor; 

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
static float manual_cruise_control_speed;

float getYawRateControlLeftRightBias(int32_t swAngle_millideg);
void set_fast_torque_with_slew(uint8_t throttlePos_u8, int16_t slew);
void setRegenTorques (float regen_pct);

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
    getProcessedValue(&yrc_kp, YRC_KP_INDEX, float_1_decimal);

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

void initPhantomDiff(){
    maxPhantomDiffScalingFactor = 0.25f;
    getProcessedValue(&maxPhantomDiffScalingFactor, PHANTOM_DIFF_CONSTANT_INDEX, float_2_decimal);
    //for now, for testing purposes 
    // int send = (int)(maxPhantomDiffScalingFactor * 100.0f); 
    // canTX(CMR_CAN_BUS_VEH, 0x526, &send, sizeof(int), 200); 
}

static void load_solver_settings() {
	float k_lin = 0, k_yaw = 0, k_tie = 0;

	// Hot fix: interpret raw k_lin and k_yaw values as integers.
	if(getProcessedValue(&k_lin, K_LIN_INDEX, float_1_decimal)) {
		solver_set_k_lin(k_lin * 10.0f); // [0, 255.0].
	}

	if(getProcessedValue(&k_yaw, K_YAW_INDEX, float_1_decimal)) {
		solver_set_k_yaw(k_yaw * 0.1f); // [0, 2.55].
	}

    if(getProcessedValue(&k_tie, K_TIE_INDEX, float_1_decimal)) {
        solver_set_k_tie(k_tie * 0.01f); // [0, 0.255].
    }
}

/** @brief initialize controls */
void initControls() {
    initYawRateControl();
    initPhantomDiff(); 
    startTickCount = xTaskGetTickCount();
	launchControlButtonPressed = false;
	launchControlActive = false;
	coulombCounting.KCoulombs = 0.0f;
    manual_cruise_control_speed = 1.0;
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


// For sensor validation.
static void set_motor_speed(uint8_t throttlePos_u8, float speed_mps, bool rear_only) {
    float throttle = (float)throttlePos_u8 / UINT8_MAX;
    float req_torque_Nm = throttle * maxFastTorque_Nm;
    const float min_speed_mps = 0.0f;
    const float max_speed_mps = 20.0f;
    speed_mps = fmaxf(speed_mps, min_speed_mps);
    speed_mps = fminf(speed_mps, max_speed_mps);
    float target_rpm = speed_mps / (PI * effective_wheel_dia_m) * gear_ratio * 60.0f;
    cmr_torqueDistributionNm_t torquesPos_Nm;
    if(rear_only) {
        setVelocityInt16(MOTOR_FL, 0);
        setVelocityInt16(MOTOR_FR, 0);
        setVelocityInt16(MOTOR_RL, (int16_t) target_rpm);
        setVelocityInt16(MOTOR_RR, (int16_t) target_rpm);
        torquesPos_Nm.fl = 0.0f;
        torquesPos_Nm.fr = 0.0f;
        torquesPos_Nm.rl = req_torque_Nm;
        torquesPos_Nm.rr = req_torque_Nm;
    } else {
        setVelocityInt16(MOTOR_FL, (int16_t) target_rpm);
        setVelocityInt16(MOTOR_FR, (int16_t) target_rpm);
        setVelocityInt16(MOTOR_RL, (int16_t) target_rpm);
        setVelocityInt16(MOTOR_RR, (int16_t) target_rpm);
        torquesPos_Nm.fl = req_torque_Nm;
        torquesPos_Nm.fr = req_torque_Nm;
        torquesPos_Nm.rl = req_torque_Nm;
        torquesPos_Nm.rr = req_torque_Nm;
    }
	cmr_torqueDistributionNm_t torquesNeg_Nm = {
        .fl = 0.0f,
        .fr = 0.0f,
        .rl = 0.0f,
        .rr = 0.0f,
    };
    setTorqueLimsProtected(&torquesPos_Nm, &torquesNeg_Nm);
}

static void set_manual_cruise_control(uint8_t throttlePos_u8) {
    static bool prev_button = false;
    const float max_speed_mps = 20.0f;
    volatile cmr_canDIMActions_t *actions = (volatile cmr_canDIMActions_t *) canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON);
    bool button = (actions->buttonStates & BUTTON_ACT) != 0;
    if(prev_button == false && button == true) {
        manual_cruise_control_speed += 1.0f;
        manual_cruise_control_speed = fminf(manual_cruise_control_speed, max_speed_mps);
    }
    prev_button = button;
    set_motor_speed(throttlePos_u8, manual_cruise_control_speed, false);
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
        // TODO: fix later, temp change force_output units
        volatile int16_t raw = parse_int16(&downforcePayload->force_output_lb);
        downforce_N = (float) raw * 0.1f * sinf(angle);
    } else {
        downforce_N = (float) car_mass_kg * 9.81f * 0.25f;
    }
    return downforce_N;
}

/**
 * @brief Compute per-wheel Fz from longitudinal acceleration (ax) load transfer.
 *        Use instead of load cells when they are unavailable.
 *        Assumes straight-line driving (no lateral load transfer).
 *
 * @param motor   Which wheel (MOTOR_FL, MOTOR_FR, MOTOR_RL, MOTOR_RR)
 * @param ax_mps2 Longitudinal acceleration in m/s^2 (positive = forward accel)
 * @return Estimated vertical load in Newtons (clamped >= 0)
 */
static float get_accel_downforce(motorLocation_t motor, float ax_mps2) {
    static const float total_mass_kg = 185.0f + 75.0f; // car + driver
    static const float cg_height_m   = 0.2895f;

    // static corner load (assume 50/50 front-rear, equal left-right)
    const float static_fz = total_mass_kg * 9.81f * 0.25f;

    // longitudinal load transfer: positive ax (forward accel) shifts load rearward
    //   delta per axle = m * ax * h / wheelbase, split equally left-right
    float delta_fz_long = total_mass_kg * ax_mps2 * cg_height_m / (float)wheelbase_m * 0.5f;

    float fz;
    switch (motor) {
        case MOTOR_FL: // fall through
        case MOTOR_FR:
            fz = static_fz - delta_fz_long; // fronts lose load under accel
            break;
        case MOTOR_RL: // fall through
        case MOTOR_RR:
            fz = static_fz + delta_fz_long; // rears gain load under accel
            break;
        default:
            fz = static_fz;
            break;
    }
    return fmaxf(fz, 0.0f);
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

	float wheel_fl_speed_radps = getMotorSpeed_radps(MOTOR_FL);
	float wheel_fr_speed_radps = getMotorSpeed_radps(MOTOR_FR);
	float wheel_rl_speed_radps = getMotorSpeed_radps(MOTOR_RL);
	float wheel_rr_speed_radps = getMotorSpeed_radps(MOTOR_RR);

    const float corner_weight_Nm = 80.0f;
    bool use_true_downforce = false;
    float tractive_cap_fl = lut_get_max_Fx_kappa(0.0, get_downforce(CANRX_DAQ_LOAD_FL, use_true_downforce) + corner_weight_Nm).Fx;
    float tractive_cap_fr = lut_get_max_Fx_kappa(0.0, get_downforce(CANRX_DAQ_LOAD_FR, use_true_downforce) + corner_weight_Nm).Fx;
    float tractive_cap_rl = lut_get_max_Fx_kappa(0.0, get_downforce(CANRX_DAQ_LOAD_RL, use_true_downforce) + corner_weight_Nm).Fx;
    float tractive_cap_rr = lut_get_max_Fx_kappa(0.0, get_downforce(CANRX_DAQ_LOAD_RR, use_true_downforce) + corner_weight_Nm).Fx;

    static const float motor_resistance_Nm[MOTOR_LEN] = {
        [MOTOR_FL] = 0.5f,
        [MOTOR_FR] = 0.5f,
        [MOTOR_RL] = 0.5f,
        [MOTOR_RR] = 0.5f,
    };
	// The most naive approach is to convert force to torque linearly, ignoring rolling resistance and any inefficiency.
	float torque_limit_fl = tractive_cap_fl * effective_wheel_rad_m / gear_ratio + motor_resistance_Nm[MOTOR_FL];
	float torque_limit_fr = tractive_cap_fr * effective_wheel_rad_m / gear_ratio + motor_resistance_Nm[MOTOR_FR];
	float torque_limit_rl = tractive_cap_rl * effective_wheel_rad_m / gear_ratio + motor_resistance_Nm[MOTOR_RL];
	float torque_limit_rr = tractive_cap_rr * effective_wheel_rad_m / gear_ratio + motor_resistance_Nm[MOTOR_RR];

	torque_limit_fl = fminf(torque_limit_fl, maxTorque_continuous_stall_Nm);
	torque_limit_fr = fminf(torque_limit_fr, maxTorque_continuous_stall_Nm);
	torque_limit_rl = fminf(torque_limit_rl, maxTorque_continuous_stall_Nm);
	torque_limit_rr = fminf(torque_limit_rr, maxTorque_continuous_stall_Nm);

	static optimizer_state_t optimizer_state;

	optimizer_state.power_limit = getPowerLimit_W();
	optimizer_state.omegas[0] = wheel_fl_speed_radps;
	optimizer_state.omegas[1] = wheel_fr_speed_radps;
	optimizer_state.omegas[2] = wheel_rl_speed_radps;
	optimizer_state.omegas[3] = wheel_rr_speed_radps;

    if(true == allow_regen) {
        optimizer_state.variable_profile[0].lower = fmaxf(-torque_limit_fl + motor_resistance_Nm[MOTOR_FL], getMotorRegenerativeCapacity(getMotorSpeed_rpm(MOTOR_FL)));
        optimizer_state.variable_profile[1].lower = fmaxf(-torque_limit_fr + motor_resistance_Nm[MOTOR_FR], getMotorRegenerativeCapacity(getMotorSpeed_rpm(MOTOR_FR)));
        optimizer_state.variable_profile[2].lower = fmaxf(-torque_limit_rl + motor_resistance_Nm[MOTOR_RL], getMotorRegenerativeCapacity(getMotorSpeed_rpm(MOTOR_RL)));
        optimizer_state.variable_profile[3].lower = fmaxf(-torque_limit_rr + motor_resistance_Nm[MOTOR_RR], getMotorRegenerativeCapacity(getMotorSpeed_rpm(MOTOR_RR)));
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
	optimizer_state.mreq = getYawRateControlLeftRightBias(swAngle_millideg);
	optimizer_state.theta_left = swAngleMillidegToSteeringAngleRad(swAngle_millideg_FL);
    optimizer_state.theta_right = swAngleMillidegToSteeringAngleRad(swAngle_millideg_FR);

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
    uint8_t paddle_pressure = ((volatile cmr_canDIMActions_t *) canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON))->regenPercent;

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
    uint8_t paddle_pressure = ((volatile cmr_canDIMActions_t *) canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON))->regenPercent;

    uint8_t paddle_regen_strength_raw = 50;
    getProcessedValue(&paddle_regen_strength_raw, PADDLE_MAX_REGEN_INDEX, unsigned_integer);
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
 * @param swAngle_millideg Steering wheel angle in degrees. Zero-centered, right turn positive.
 * @param battVoltage_mV Accumulator voltage in millivolts.
 * @param battCurrent_mA Accumulator current in milliamps.
 * @param blank_command Additional signal that forces the motor commands to zero vel. and zero torque
 */
void runControls (
    cmr_canGear_t gear,
    uint8_t throttlePos_u8,
    uint16_t brakePressurePsi_u8,
    int32_t swAngle_millideg_FL,
    int32_t swAngle_millideg_FR,
    int32_t battVoltage_mV,
    int32_t battCurrent_mA,
    bool ctrlOff,
    bool blank_command )
{

    int32_t swAngle_millideg = (swAngle_millideg_FL + swAngle_millideg_FR) / 2;
    integrateCurrent();
    if (blank_command) {
        setTorqueLimsAllProtected(0.0f, 0.0f);
        setVelocityInt16All(0);
        return;
    }

    int32_t dtiERPM_FL = getDTIERPM(CANRX_TRAC_FL_ERPM);
    int32_t dtiERPM_FR = getDTIERPM(CANRX_TRAC_FR_ERPM);
    int32_t dtiERPM_RL = getDTIERPM(CANRX_TRAC_RL_ERPM);
    int32_t dtiERPM_RR = getDTIERPM(CANRX_TRAC_RR_ERPM);

    volatile cmr_canHeartbeat_t   *heartbeatVSM = canVehicleGetPayload(CANRX_VEH_HEARTBEAT_VSM);

    const int32_t avgMotorSpeed_RPM = (
        + (int32_t)(dtiERPM_FL / pole_pairs)
        + (int32_t)(dtiERPM_FR / pole_pairs)
        + (int32_t)(dtiERPM_RL / pole_pairs)
        + (int32_t)(dtiERPM_RR / pole_pairs)
    ) / MOTOR_LEN;

    // Update odometer
    /* Wheel Speed to Vehicle Speed Conversion
    *      (x rotations / 1min) * (16" * PI) *  (2.54*10^-5km/inch)
    *      (1min / 60sec) * (1sec/1000ms) * (5ms period) * (1/13.93 gear ratio)
    *      = x * 7.6378514861 × 10^-9 */
    odometer_km += ((float)avgMotorSpeed_RPM) * 7.6378514861e-9;
    /** @todo check floating point granularity for potential issues with adding small numbers repeatedly to large numbers */
    
    bool sensoric_timeout = cmr_canRXMetaTimeoutError(&canDaqRXMeta[CANRX_DAQ_SENSORIC_VEL_ANG], xTaskGetTickCount()) != 0;
    // We currently don't use sensoric in endurance mode, so its timeout should not cause the vehicle to switch to fast mode. This is to prevent unintended fast mode when sensoric data is lost during endurance.
    cmr_canGear_t real_gear = (ctrlOff || sensoric_timeout) &&
                                           (gear == CMR_CAN_GEAR_AUTOX ||
                                            gear == CMR_CAN_GEAR_SKIDPAD ||
                                            gear == CMR_CAN_GEAR_ACCEL ||
                                            gear == CMR_CAN_GEAR_TEST ||
                                            gear == CMR_CAN_GEAR_REVERSE)
                                ? CMR_CAN_GEAR_FAST : gear;
    
    if (ctrlOff && real_gear == CMR_CAN_GEAR_ENDURANCE){
        real_gear = CMR_CAN_GEAR_FAST;
    }
    
    switch (real_gear) {
        case CMR_CAN_GEAR_SLOW: {
            disableTorqueMode();
            setSlowTorque(throttlePos_u8, swAngle_millideg);
            break;
        }
        case CMR_CAN_GEAR_FAST: {
            disableTorqueMode();
            setFastTorqueWithBias(throttlePos_u8, front_bias);
            setPowerLimit(false, MOTOR_FL, 35.0f * front_bias);
            setPowerLimit(false, MOTOR_FR, 35.0f * front_bias);
            setPowerLimit(false, MOTOR_RL, 35.0f * (1 - front_bias));
            setPowerLimit(false, MOTOR_RR, 35.0f * (1 - front_bias));
            break;
        }
        case CMR_CAN_GEAR_ENDURANCE: {
            disableTorqueMode();

            uint8_t regen_paddle_percent = ((volatile cmr_canDIMActions_t *) canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON))->regenPercent;

            if(regen_paddle_percent > regenPaddlePercentThreshold)
            {
                const float regen_percent = 
                    CLAMP(
                        0.0f,
                        (float)(regen_paddle_percent - regenPaddlePercentThreshold) / (100 - regenPaddlePercentThreshold),
                        1.0f
                    );
                setRegenTorques(regen_percent);
            }
            else{
                // Don't set power limit as it is being sent from DAQ-Live
                setFastTorqueWithPhantomDiff(throttlePos_u8, swAngle_millideg, front_bias, maxPhantomDiffScalingFactor);
            }
            break;
        }
        case CMR_CAN_GEAR_AUTOX: {
            disableTorqueMode();
            //getYawRateControlLeftRightBias should be called in set_optimal_control_* functions 
            set_optimal_control_with_regen(throttlePos_u8, swAngle_millideg_FL, swAngle_millideg_FR);
            break;
        }
        case CMR_CAN_GEAR_SKIDPAD: {
            disableTorqueMode();
            //calculatePersistentYRCmreq should be called set_optimal_control_* functions 
        	set_optimal_control((float)throttlePos_u8 / UINT8_MAX, swAngle_millideg_FL, swAngle_millideg_FR, false);
            break;
        }
        case CMR_CAN_GEAR_ACCEL: {
            disableTorqueMode();
            //set_manual_cruise_control(throttlePos_u8);
            float vx, va; 
            volatile cmr_canSensoricVelAng_t *sensoricVelAng = (cmr_canSensoricVelAng_t*)canDAQGetPayload(CANRX_DAQ_SENSORIC_VEL_ANG);
            vx = SENSORIC_VEL_TO_MPS((float)(sensoricVelAng->vel_X)); // dont use this one
            va = SENSORIC_VEL_TO_MPS((float)(sensoricVelAng->vel_A));
            // sensors_get_vel_xy(&vx, &vy);

            // Toggle: set to true to estimate Fz from accelerometer, false to use load cells
            const bool use_accel_downforce = true;

            float fz_fl_N, fz_fr_N, fz_rl_N, fz_rr_N;

            if (use_accel_downforce) {
                // Estimate Fz from Sensoric longitudinal acceleration (no load cells needed)
                float ax, ay, az;
                sensors_get_accel_xyz(&ax, &ay, &az);
                fz_fl_N = get_accel_downforce(MOTOR_FL, ax);
                fz_fr_N = get_accel_downforce(MOTOR_FR, ax);
                fz_rl_N = get_accel_downforce(MOTOR_RL, ax);
                fz_rr_N = get_accel_downforce(MOTOR_RR, ax);
            } else {
                // Use load cells
                volatile cmr_canIZZELoadCell_t *fl_load = (cmr_canIZZELoadCell_t *)canDAQGetPayload(CANRX_DAQ_LOAD_FL);
                volatile cmr_canIZZELoadCell_t *fr_load = (cmr_canIZZELoadCell_t *)canDAQGetPayload(CANRX_DAQ_LOAD_FR);
                volatile cmr_canIZZELoadCell_t *rl_load = (cmr_canIZZELoadCell_t *)canDAQGetPayload(CANRX_DAQ_LOAD_RL);
                volatile cmr_canIZZELoadCell_t *rr_load = (cmr_canIZZELoadCell_t *)canDAQGetPayload(CANRX_DAQ_LOAD_RR);

                fz_fl_N = (float)(parse_int16(&(fl_load->force_output_lb))) * 4.448f * sinf(0.785);
                fz_fr_N = (float)(parse_int16(&(fr_load->force_output_lb))) * 4.448f * sinf(0.785);
                fz_rl_N = (float)(parse_int16(&(rl_load->force_output_lb))) * 4.448f * sinf(0.524);
                fz_rr_N = (float)(parse_int16(&(rr_load->force_output_lb))) * 4.448f * sinf(0.524);
            }

            setAccelLaunchControl(throttlePos_u8, brakePressurePsi_u8, va,
                fz_fl_N, fz_fr_N, fz_rl_N, fz_rr_N);
            break;
        }
        case CMR_CAN_GEAR_TEST: {
            disableTorqueMode();

            // Zero-initialised so an unavailable sensor source leaves the phantom diff inactive
            // rather than reading an uninitialised value.
            float ax_mps2 = 0.0f, ay_mps2 = 0.0f, az_mps2 = 0.0f;
            sensors_get_accel_xyz(&ax_mps2, &ay_mps2, &az_mps2);

            setFastTorqueWithPhantomDiff(throttlePos_u8, swAngle_millideg, ay_mps2, front_bias);
            int send = (int)(maxPhantomDiffScalingFactor * 100.0f);
            canTX(CMR_CAN_BUS_VEH, 0x526, &send, sizeof(int), 200);

            setPowerLimit(false, MOTOR_FL, maxPowerPerMotor_kW * front_bias);
            setPowerLimit(false, MOTOR_FR, maxPowerPerMotor_kW * front_bias);
            setPowerLimit(false, MOTOR_RL, maxPowerPerMotor_kW * (1 - front_bias));
            setPowerLimit(false, MOTOR_RR, maxPowerPerMotor_kW * (1 - front_bias));
            break;
        }

        case CMR_CAN_GEAR_REVERSE: {
            // for rule-compliance, the car shouldn't reverse
            disableTorqueMode();
            setTorqueLimsAllProtected(0.0f, 0.0f);
            setVelocityInt16All(0);
            break;
        }

        case CMR_CAN_GEAR_DV_MISSION_INSPECTION: {
            disableTorqueMode();
            // initiateTorqueMode();
            static bool inspectionStarted = false;
            static TickType_t inspectionStartTime = 0;
            TickType_t now = xTaskGetTickCount();
            if(!inspectionStarted && heartbeatVSM->state == CMR_CAN_AS_DRIVING) {
                inspectionStarted = true;
                inspectionStartTime = now;
            }
            if(inspectionStarted 
            && heartbeatVSM->state == CMR_CAN_AS_DRIVING
            && now - inspectionStartTime < INSPECTION_MISSION_TIME_MS){
                setVelocityInt16All(maxSlowSpeed_rpm);
                float torque = maxSlowTorque_Nm; 
                setTorqueLimsUnprotected(MOTOR_FL, torque, 0.0f);
                setTorqueLimsUnprotected(MOTOR_FR, torque, 0.0f);
                setTorqueLimsUnprotected(MOTOR_RR, torque, 0.0f);
                setTorqueLimsUnprotected(MOTOR_RL, torque, 0.0f);
            }
            else {
                setVelocityInt16All(0);
                float torque = 0.0f; 
                setTorqueLimsUnprotected(MOTOR_FL, torque, 0.0f);
                setTorqueLimsUnprotected(MOTOR_FR, torque, 0.0f);
                setTorqueLimsUnprotected(MOTOR_RR, torque, 0.0f);
                setTorqueLimsUnprotected(MOTOR_RL, torque, 0.0f);
                uint8_t missionFinished = 1;
                canTX(CMR_CAN_BUS_VEH, CMR_CANID_AS_MISSION_FINISHED, &missionFinished, sizeof(missionFinished), 100);
            }
            break;
        }

        case CMR_CAN_GEAR_DV_MISSION_ACCEL: 
        case CMR_CAN_GEAR_DV_MISSION_SKIDPAD:
        case CMR_CAN_GEAR_DV_MISSION_AUTOX:     
        case CMR_CAN_GEAR_DV_MISSION_TRACKD:     
        case CMR_CAN_GEAR_DV_MISSION_EBS: {
            disableTorqueMode();
            volatile cmr_canAutonomousControlAction_t*  autonomousAction = canDAQGetPayload(CANRX_DAQ_AUTONOMOUS_ACTION);
            
            float front_torque_Nm;
            float rear_torque_Nm; 
            float maxVelocity_rpm;

            if(cmr_canRXMetaTimeoutError(&canDaqRXMeta[CANRX_DAQ_AUTONOMOUS_ACTION], xTaskGetTickCount())) {
                front_torque_Nm = 0;
                rear_torque_Nm = 0;
                maxVelocity_rpm = 0;
            }
            else {
                front_torque_Nm = CLAMP(-maxDVTorque_Nm, ((float)(autonomousAction->frontTorque_mNm))/1000.0f, maxDVTorque_Nm); 
                rear_torque_Nm  = CLAMP(-maxDVTorque_Nm, ((float)(autonomousAction->rearTorque_mNm))/1000.0f, maxDVTorque_Nm); 
                maxVelocity_rpm = (float)(autonomousAction->maxVelocity_decimeters_s) * 60.0f / 10.0f / ( PI * effective_wheel_dia_m) * gear_ratio;
                maxVelocity_rpm = CLAMP(0, maxVelocity_rpm, maxDVSpeed_rpm); 
            }

            setVelocityInt16All(maxVelocity_rpm);
            
            if (front_torque_Nm > 0.0f) {
                setTorqueLimsUnprotected(MOTOR_FL, front_torque_Nm, 0.0f);
                setTorqueLimsUnprotected(MOTOR_FR, front_torque_Nm, 0.0f);
            } else {
                setTorqueLimsUnprotected(MOTOR_FL, 0.0f, front_torque_Nm);
                setTorqueLimsUnprotected(MOTOR_FR, 0.0f, front_torque_Nm);
            }

            if (rear_torque_Nm > 0.0f) {
                setTorqueLimsUnprotected(MOTOR_RR, rear_torque_Nm, 0.0f);
                setTorqueLimsUnprotected(MOTOR_RL, rear_torque_Nm, 0.0f);
            } else {
                setTorqueLimsUnprotected(MOTOR_RR, 0.0f, rear_torque_Nm);
                setTorqueLimsUnprotected(MOTOR_RL, 0.0f, rear_torque_Nm);
            }
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
        const float packCurrent_mA = getCurrent_mA();
        const TickType_t currentTick = xTaskGetTickCount();
        coulombCounting.KCoulombs += ((currentTick-previousTickCount)*0.001f)*packCurrent_mA / 1000000.0f;
        previousTickCount = currentTick;
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

    setTorqueLimsUnprotected(MOTOR_FL, reqTorque, 0.0f);
    setTorqueLimsUnprotected(MOTOR_FR, reqTorque, 0.0f);
    setTorqueLimsUnprotected(MOTOR_RR, reqTorque, 0.0f);
    setTorqueLimsUnprotected(MOTOR_RL, reqTorque, 0.0f);

    setVelocityInt16All(maxSlowSpeed_rpm);
}

/**
 * @brief Calculates and sets motor torques and velocities for fast gear.
 *
 * @param throttlePos_u8 Throttle position, 0-255.
 */
void setFastTorque (uint8_t throttlePos_u8) {
    const float reqTorque = maxFastTorque_Nm * (float)(throttlePos_u8) / (float)(UINT8_MAX);
   //setTorqueLimsAllProtected(reqTorque, 0.0f);
   
   setTorqueLimsUnprotected(MOTOR_FL, reqTorque, 0.0f);
   setTorqueLimsUnprotected(MOTOR_FR, reqTorque, 0.0f);
   setTorqueLimsUnprotected(MOTOR_RR, reqTorque, 0.0f);
   setTorqueLimsUnprotected(MOTOR_RL, reqTorque, 0.0f);
   setVelocityInt16All(maxFastSpeed_rpm);
}

void setFastTorqueWithBias (uint8_t throttlePos_u8, float front_bias) {
    const float reqTorque = maxFastTorque_Nm * (float)(throttlePos_u8) / (float)(UINT8_MAX);
   //setTorqueLimsAllProtected(reqTorque, 0.0f);
   float reqTorque_front = reqTorque * front_bias / (1-front_bias);
   float reqTorque_rear = reqTorque;
   
   setTorqueLimsUnprotected(MOTOR_FL, reqTorque_front, 0.0f);
   setTorqueLimsUnprotected(MOTOR_FR, reqTorque_front, 0.0f);
   setTorqueLimsUnprotected(MOTOR_RR, reqTorque_rear, 0.0f);
   setTorqueLimsUnprotected(MOTOR_RL, reqTorque_rear, 0.0f);
   setVelocityInt16All(maxFastSpeed_rpm);
}

void setFastTorqueWithPhantomDiff(
    uint8_t throttlePos_u8,
    int32_t swAngle_millideg,
    float lateral_accel_mps2,
    float front_bias
)
{
    const float reqTorque =
        maxFastTorque_Nm * (float)throttlePos_u8 / (float)UINT8_MAX;

    // Compute a base set of torques with persistent front-rear bias.
    const float reqTorque_front =
        reqTorque * front_bias / (1.0f - front_bias);
    const float reqTorque_rear = reqTorque;

    const int32_t clamped_swAngle_millideg =
        CLAMP(
            -swAngleMax_millideg,
            swAngle_millideg,
            swAngleMax_millideg
        );

    /// Phantom diff scales with lateral load transfer, the transfer of vertical tire load from
    /// the inside wheels to the outside wheels during cornering. For a given axle:
    ///     dFz/Fz_static = 4 * K_axle * cg_ht * a_y / (t * g)
    /// where t is track width, g is gravity, and K_axle is that axle's share of the total
    /// transfer. Vehicle mass cancels, since both the transfer and the static corner load
    /// scale with it.
    ///
    /// phantomDiffGain absorbs the constant vehicle geometry, so each axle's scaling factor is
    /// directly proportional to measured lateral acceleration.
    ///
    /// The transfer splits between axles by roll stiffness distribution, not evenly, which is
    /// why the front and rear get separate scaling factors.
    ///
    /// The magnitude comes from the IMU; the steering angle is used only for its sign, to decide
    /// which side is outer.
    const float lateral_load_ratio = phantomDiffGain * fabsf(lateral_accel_mps2);

    const float phantom_diff_front =
        CLAMP(
            0.0f,
            lateral_load_ratio * lateralLoadTransferDistFront,
            maxPhantomDiffScalingFactor
        );

    const float phantom_diff_rear =
        CLAMP(
            0.0f,
            lateral_load_ratio * (1.0f - lateralLoadTransferDistFront),
            maxPhantomDiffScalingFactor
        );

    const float inner_front_torque_fraction = 1.0f - phantom_diff_front;
    const float inner_rear_torque_fraction  = 1.0f - phantom_diff_rear;

    // If we are turning right, left wheels are treated as outer and right wheels as inner.
    const float reqTorque_rear_outer =
        CLAMP(
            reqTorque_rear,
            reqTorque_rear * (1.0f + phantom_diff_rear),
            maxFastTorque_Nm
        );

    const float reqTorque_front_outer =
        CLAMP(
            reqTorque_front,
            reqTorque_front * (1.0f + phantom_diff_front),
            maxFastTorque_Nm
        );

    if (clamped_swAngle_millideg >= 0) {
        setTorqueLimsUnprotected(MOTOR_FL, reqTorque_front_outer, 0.0f);
        setTorqueLimsUnprotected(MOTOR_RL, reqTorque_rear_outer, 0.0f);
        setTorqueLimsUnprotected(MOTOR_FR, reqTorque_front * inner_front_torque_fraction, 0.0f);
        setTorqueLimsUnprotected(MOTOR_RR, reqTorque_rear * inner_rear_torque_fraction, 0.0f);
    }
    // If we are turning left, right wheels are treated as outer and left wheels as inner.
    else {
        setTorqueLimsUnprotected(MOTOR_FL, reqTorque_front * inner_front_torque_fraction, 0.0f);
        setTorqueLimsUnprotected(MOTOR_RL, reqTorque_rear * inner_rear_torque_fraction, 0.0f);
        setTorqueLimsUnprotected(MOTOR_FR, reqTorque_front_outer, 0.0f);
        setTorqueLimsUnprotected(MOTOR_RR, reqTorque_rear_outer, 0.0f);
    }

    setVelocityInt16All(maxFastSpeed_rpm);
}

void setRegenTorques (float regen_pct) {
    const float reqTorque = max_regen_torque_Nm * regen_pct;
   
    setTorqueLimsUnprotected(MOTOR_FL, 0.0f, reqTorque);
    setTorqueLimsUnprotected(MOTOR_FR, 0.0f, reqTorque);
    setTorqueLimsUnprotected(MOTOR_RR, 0.0f, reqTorque * (1 - frontRegenBiasRatio) / frontRegenBiasRatio);
    setTorqueLimsUnprotected(MOTOR_RL, 0.0f, reqTorque * (1 - frontRegenBiasRatio) / frontRegenBiasRatio);
    setVelocityInt16All(0);
}

//real or cake (== one pedal regen?)
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


float get_optimal_yaw_rate(float swangle_rad, float velocity_x_mps) {

    static const float natural_understeer_gradient = 0.011465f; //rad/g

    const float distance_between_axles_m = chassis_a + chassis_b;
    // const float yaw_rate_setpoint_radps = swangle_rad * velocity_x_mps /
    //     (distance_between_axles_m + velocity_x_mps * velocity_x_mps * natural_understeer_gradient);
    const float yaw_rate_setpoint_radps = swangle_rad * velocity_x_mps / distance_between_axles_m;
    return yaw_rate_setpoint_radps;
}

void setAccelLaunchControl(
    uint8_t throttlePos_u8,
    uint16_t brakePressurePsi_u8,
    float car_velocity_mps,
    float fz_fl, float fz_fr,
    float fz_rl, float fz_rr
) {

    // persistent state
    static bool button_was_held = false;
    static bool launch_armed = false; 
    static bool launch_active = false;
    static TickType_t launch_tick = 0;

    static const float LAUNCH_SPEED_THRESH_MPS = 0.05f; // below this = "still stationary"
    static const float LAUNCH_TIMEOUT_S = 8.0f;  // kill switch after N seconds
    static bool was_active = false;

    // read button
    bool button_held = (((volatile cmr_canDIMActions_t *)canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON))->buttonStates) & BUTTON_ACT;

    float odometer_vel_mps = motorSpeedToWheelLinearSpeed_mps(
        getTotalMotorSpeed_radps() * 0.25f);

    // state transitions
    // armed state
    if (odometer_vel_mps < LAUNCH_SPEED_THRESH_MPS && button_held) {
        launch_armed = true;
    }

    // active state
    if (launch_armed && button_was_held && !button_held) {
        launch_active = true;
        launch_tick   = xTaskGetTickCount();
        launch_armed  = false;
    }

    // kill switch
    if (brakePressurePsi_u8 > braking_threshold_psi || (throttlePos_u8 == 0)) {
        launch_active = false;
        launch_armed = false;
    }

    button_was_held = button_held;

    // track launch transition for controller reset
    bool just_launched = launch_active && !was_active;
    was_active = launch_active;

    // if not active, zero torque and return
    if (!launch_active) {
        setTorqueLimsAllProtected(0.0f, 0.0f);
        setVelocityInt16All(0);
        return;
    }

    // check timeout, handoff to regular fast torque if timed out
    float elapsed_s = (float)(xTaskGetTickCount() - launch_tick) * 0.001f;
    if(elapsed_s >= LAUNCH_TIMEOUT_S) {
        setFastTorque(throttlePos_u8);
        return;
    }

    // --- Clamp vertical loads ---
    fz_fl = fmaxf(accel_min_fz_N, fminf(fz_fl, accel_max_fz_N));
    fz_fr = fmaxf(accel_min_fz_N, fminf(fz_fr, accel_max_fz_N));
    fz_rl = fmaxf(accel_min_fz_N, fminf(fz_rl, accel_max_fz_N));
    fz_rr = fmaxf(accel_min_fz_N, fminf(fz_rr, accel_max_fz_N));

    // --- Velocity targets: car velocity * (1 + target slip), converted to motor RPM ---
    // Minimum velocity so we can actually get moving from standstill
    static const float min_launch_vel_mps = 1.0f;
    float effective_vel_mps = fmaxf(car_velocity_mps, min_launch_vel_mps);

    float target_whl_vel_front_mps = effective_vel_mps * (1.0f + slip_ratio_front);
    float target_whl_vel_rear_mps  = effective_vel_mps * (1.0f + slip_ratio_rear);

    // wheel m/s -> motor RPM: motor_rpm = wheel_mps * gear_ratio * 60 / (2*pi*wheel_rad)
    float vel_to_rpm = gear_ratio * 60.0f / (2.0f * PI * effective_wheel_rad_m);
    float rpm_front = fmaxf(0.0f, fminf(target_whl_vel_front_mps * vel_to_rpm, (float)maxSpeed_rpm));
    float rpm_rear  = fmaxf(0.0f, fminf(target_whl_vel_rear_mps  * vel_to_rpm, (float)maxSpeed_rpm));
    setVelocityFloat(MOTOR_FL, rpm_front);
    setVelocityFloat(MOTOR_FR, rpm_front);
    setVelocityFloat(MOTOR_RL, rpm_rear);
    setVelocityFloat(MOTOR_RR, rpm_rear);

    // --- Torque distribution: proportional to vertical load (Fz) on each tire ---
    // Full torque above 80% throttle, scale down below that for safety.
    float total_fz = fz_fl + fz_fr + fz_rl + fz_rr;
    if (total_fz < 1.0f) total_fz = 1.0f;

    static const float throttle_threshold = 0.80f * (float)UINT8_MAX; // 80%
    float torque_scale = (throttlePos_u8 >= (uint8_t)throttle_threshold) ? 1.0f
                       : (float)throttlePos_u8 / throttle_threshold;

    float reqTorque = maxFastTorque_Nm * torque_scale;

    setTorqueLimsUnprotected(MOTOR_FL, reqTorque, 0.0f);
    setTorqueLimsUnprotected(MOTOR_FR, reqTorque, 0.0f);
    setTorqueLimsUnprotected(MOTOR_RL, reqTorque, 0.0f);
    setTorqueLimsUnprotected(MOTOR_RR, reqTorque, 0.0f);

    // --- Per-wheel power split: proportional to (clamped) vertical load ---
    // sorry written at comp should be in constants
    float max_power_to_inverter_kw = 35.0f; 
    float max_total_power_kw = 79.0f;
    float power_fl_kw = CLAMP (0, (fz_fl / total_fz) * max_total_power_kw, max_power_to_inverter_kw);
    float power_fr_kw = CLAMP (0, (fz_fr / total_fz) * max_total_power_kw, max_power_to_inverter_kw);
    float power_rl_kw = CLAMP (0, (fz_rl / total_fz) * max_total_power_kw, max_power_to_inverter_kw);
    float power_rr_kw = CLAMP (0, (fz_rr / total_fz) * max_total_power_kw, max_power_to_inverter_kw);

    setPowerLimit(false, MOTOR_FL, power_fl_kw);
    setPowerLimit(false, MOTOR_FR, power_fr_kw);
    setPowerLimit(false, MOTOR_RL, power_rl_kw);
    setPowerLimit(false, MOTOR_RR, power_rr_kw);
}

/**
 * @brief Calculate the control action (left-right torque bias) of the yaw rate controller
 * @param swAngle_millideg Steering wheel angle
 */
float getYawRateControlLeftRightBias(int32_t swAngle_millideg) {

    // using new abstraction
    float gx, gy, gz;
    // sensors_get_gyro_xyz(&gx, &gy, &gz);
    const float actual_yaw_rate_radps_sae = gz;

    float velocity_x_mps;
    const volatile car_state_t *cs;
    //  = sensors_get_car_state();
    float calculated_velocity_x_mps_fallback = getTotalMotorSpeed_radps() * 0.25f / gear_ratio * effective_wheel_rad_m;

    // add yrc debug here
    if (cs && movella_state.status.gnss_fix) {
    velocity_x_mps = cs->velocity.x;
    yrcDebug.controls_bias = 1;

    } else {
    velocity_x_mps = calculated_velocity_x_mps_fallback;
    yrcDebug.controls_bias = -1;
    }

    // float velocity_x_mps;
    // if(movella_state.status.gnss_fix) {
    //     velocity_x_mps = movella_state.velocity.x;
    //     yrcDebug.controls_bias = 1;
    // } else {
    //     velocity_x_mps = getTotalMotorSpeed_radps() * 0.25f / gear_ratio * effective_wheel_rad_m;
    //     yrcDebug.controls_bias = -1;
    // }

    const float swangle_rad = swAngleMillidegToSteeringAngleRad(swAngle_millideg);
    // const float actual_yaw_rate_radps_sae = movella_state.gyro.z; using old movella
    const float optimal_yaw_rate_radps = get_optimal_yaw_rate(swangle_rad, velocity_x_mps);

    yrcDebug.controls_current_yaw_rate = (int16_t)(1000.0f * actual_yaw_rate_radps_sae);
    yrcDebug.controls_target_yaw_rate = (int16_t)(1000.0f * optimal_yaw_rate_radps);
    yrcDebug.controls_pid = yrc_kp;
    const float left_right_bias = yrc_kp * (optimal_yaw_rate_radps - actual_yaw_rate_radps_sae);
    return left_right_bias;
}


/**
 * @brief Calculate left-right torque bias of the yaw rate controller with persistent bias in long turns
 * @param swAngle_millideg Steering wheel angle
 * @param bias_margin Maximum range of (desired - actual yaw rate) values to consider
 * @param yrc_pers Strength of persistent bias factor
 * @return Requested moment (mreq)
 */
float calculatePersistentYRCmreq(int32_t swAngle_millideg, float bias_margin, float yrc_pers) {
    // yrc_kp calcs copied from current mreq function getYawRateControlLeftRightBias()
    float gx, gy, gz;
    sensors_get_gyro_xyz(&gx, &gy, &gz);
    const float actual_yaw_rate_radps_sae = gz;

    float velocity_x_mps;
    const volatile car_state_t *cs = sensors_get_car_state();
    float calculated_velocity_x_mps_fallback = getTotalMotorSpeed_radps() * 0.25f / gear_ratio * effective_wheel_rad_m;

    // check swangle to make sure that we dont kick in pers if we're driving straight


    // add yrc debug here
    if (cs && movella_state.status.gnss_fix) {
    velocity_x_mps = cs->velocity.x;
    yrcDebug.controls_bias = 1;
    } else {
    velocity_x_mps = calculated_velocity_x_mps_fallback;
    yrcDebug.controls_bias = -1;
    }

    const float swangle_rad = swAngleMillidegToSteeringAngleRad(swAngle_millideg);
    const float desired_yaw_rate_radps = get_optimal_yaw_rate(swangle_rad, velocity_x_mps);

    yrcDebug.controls_current_yaw_rate = (int16_t)(1000.0f * actual_yaw_rate_radps_sae);
    yrcDebug.controls_target_yaw_rate = (int16_t)(1000.0f * desired_yaw_rate_radps);
    yrcDebug.controls_pid = yrc_kp;
    const float mreq_kp = yrc_kp * (desired_yaw_rate_radps - actual_yaw_rate_radps_sae);

    // pers calculation
    const float yaw_rate_diff_radps = desired_yaw_rate_radps - actual_yaw_rate_radps_sae;

    float pers_bias;
    const bool pers_off = fabsf(desired_yaw_rate_radps) < fabsf(actual_yaw_rate_radps_sae) // coming out of a turn
                            || (desired_yaw_rate_radps * actual_yaw_rate_radps_sae) < 0 // different directions
                            || fabsf(yaw_rate_diff_radps) >= (bias_margin) // strong yrc_kp
                            || fabsf(swangle_rad) < YRC_PERS_SWANGLE_DEADZONE_RAD; // small steering angle
                            // checks
    if (pers_off) {
        pers_bias = 0;
    } else { // pers_bias (0% to 100%) scales quadratically as yaw_rate_diff_radps approaches 0
        const float squared_ratio = (yaw_rate_diff_radps * yaw_rate_diff_radps) / (bias_margin * bias_margin);
        // const float squared_ratio = (yaw_rate_diff_radps * yaw_rate_diff_radps) / bias_margin;
        pers_bias = 1.0f - squared_ratio;
    }
    // pers_bias is the percentage of desired_yaw_rate based off of diff, yrc_pers scales this further
    const float mreq_pers = desired_yaw_rate_radps * pers_bias * yrc_pers;

    return mreq_kp + mreq_pers;
}

