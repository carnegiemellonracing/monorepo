/**
 * @file controls.c
 * @brief Vehicle control loops.
 *
 * @author Carnegie Mellon Racing
 */

// ------------------------------------------------------------------------------------------------
// Includes

#include "controls.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../optimizer/optimizer.h"
#include "CMR/can_types.h"
#include "constants.h"
#include "lut.h"
#include "lut_3d.h"
#include "motors.h"
#include "movella.h"
#include "safety_filter.h"

#define PI 3.1415926535897932384626f
#define G 9.81f
#define DAQ_PUSHROD_ANGLE_FR 35.0f
#define DAQ_PUSHROD_ANGLE_RR 30.0f

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
    .tcOn = (uint8_t)false, .yrcOn = (uint8_t)false};

volatile cmr_canCDCKiloCoulombs_t coulombCounting;
static float manual_cruise_control_speed;

float getYawRateControlLeftRightBias(int32_t swAngle_millideg);
void set_fast_torque_with_slew(uint8_t throttlePos_u8, int16_t slew);

/** @brief Coulomb counting info **/
static TickType_t previousTickCount;

// ------------------------------------------------------------------------------------------------
// Function implementations
/** @brief initialize yaw rate control */
static void initYawRateControl() {
    // read yrc_kp from DIM
    yrc_kp = 1.0f;
    getProcessedValue(&yrc_kp, YRC_KP_INDEX, float_1_decimal);

    yrc_kp = yrc_kp * 100.0f;
    // yrc_kp = 200;
    // yrcDebug = getPidDebug();
    yrcDebug.controls_pid = yrc_kp;
    // set yrc_ki to 0 because we don't aim to eliminate steady-state error
    // REMOVE const float yrc_ki = 0.0f;

    // disable derivate separation because no significant derivative kick was
    // observed steering angle and the car's yaw seem to have similar timescales
    // REMOVE const bool enable_derivative_separation = false;
}

static void load_solver_settings() {
    float k_lin = 0, k_yaw = 0, k_tie = 0;

    // Hot fix: interpret raw k_lin and k_yaw values as integers.
    if (getProcessedValue(&k_lin, K_LIN_INDEX, float_1_decimal)) {
        solver_set_k_lin(k_lin * 10.0f);  // [0, 255.0].
    }

    if (getProcessedValue(&k_yaw, K_YAW_INDEX, float_1_decimal)) {
        solver_set_k_yaw(k_yaw * 0.1f);  // [0, 2.55].
    }

    if (getProcessedValue(&k_tie, K_TIE_INDEX, float_1_decimal)) {
        solver_set_k_tie(k_tie * 0.01f);  // [0, 0.255].
    }
}

/** @brief initialize controls */
void initControls() {
    initYawRateControl();
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
    return (const cmr_canCDCControlsStatus_t *)&controlsStatus;
}

static inline void set_motor_speed_and_torque(
    motorLocation_t motor, float val, cmr_torqueDistributionNm_t *torquesPos_Nm,
    cmr_torqueDistributionNm_t *torquesNeg_Nm) {
    if (val > 0.0) {
        switch (motor) {
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
        switch (motor) {
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

/**
 * @param normalized_throttle A value in [-1, 1].
 * In [0, 1] if without regen.
 */
void set_optimal_control(float normalized_throttle, int32_t swAngle_millideg_FL, int32_t swAngle_millideg_FR, bool allow_regen) {
    int32_t swAngle_millideg = (swAngle_millideg_FL + swAngle_millideg_FR) / 2;

    // verify limits of throttle
    if (allow_regen) {
        assert(-1.0f <= normalized_throttle && normalized_throttle <= 1.0f);
    } else {
        assert(0.0f <= normalized_throttle && normalized_throttle <= 1.0f);
    }

    // get tractive capacities based on downforce
    float cap_fl, cap_fr, cap_rl, cap_rr;
    bool use_true_downforce = true;
    get_tractive_capabilities(&cap_fl, &cap_fr, &cap_rl, &cap_rr, use_true_downforce);

    // compute torque lims for each motor
    float lim_fl, lim_fr, lim_rl, lim_rr;
    compute_torque_limits(cap_fl, cap_fr, cap_rl, cap_rr,
    &lim_fl, &lim_fr, &lim_rl, &lim_rr);

    // prepare optimizer
    static optimizer_state_t optimizer_state;
    load_optimizer_state(&optimizer_state, normalized_throttle, swAngle_millideg_FL,
    swAngle_millideg_FR, lim_fl, lim_fr, lim_rl, lim_rr, allow_regen);

    // run solver
    solve(&optimizer_state);

    //log outputs as normal (unchanged) Logging solver outputs, x1000 to make it more intuitive.
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

    if (allow_regen) {
        set_motor_speed_and_torque(MOTOR_FL, optimizer_state.optimal_assignment[0].val, &torquesPos_Nm, &torquesNeg_Nm);
        set_motor_speed_and_torque(MOTOR_FR, optimizer_state.optimal_assignment[1].val, &torquesPos_Nm, &torquesNeg_Nm);
        set_motor_speed_and_torque(MOTOR_RL, optimizer_state.optimal_assignment[2].val, &torquesPos_Nm, &torquesNeg_Nm);
        set_motor_speed_and_torque(MOTOR_RR, optimizer_state.optimal_assignment[3].val, &torquesPos_Nm, &torquesNeg_Nm);
        setTorqueLimsProtected(&torquesPos_Nm, &torquesNeg_Nm);
        // The API for setting speeds and torques is not optimal.
        // It should allow setting velocities the same way as setting torques,
        // by passing a struct.

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


float get_combined_request(int throttlePos_u8) {
    uint8_t paddle_pressure =
        ((volatile cmr_canDIMActions_t *)canVehicleGetPayload(
             CANRX_VEH_DIM_ACTION_BUTTON))
            ->regenPercent;

    uint8_t paddle_regen_strength_raw = 100;
    // getProcessedValue(&paddle_regen_strength_raw, PADDLE_MAX_REGEN_INDEX,
    // unsigned_integer);
    float paddle_regen_strength = paddle_regen_strength_raw * 0.01;

    float paddle_request = 0.0f;
    if (paddle_pressure > paddle_pressure_start) {
        paddle_request = ((float)(paddle_pressure - paddle_pressure_start)) /
                         (UINT8_MAX - paddle_pressure_start);
        paddle_request *= paddle_regen_strength;  // [0, 1].
    }

    float throttle = (float)throttlePos_u8 / UINT8_MAX;
    float combined_request = throttle - paddle_request;  // [-1, 1].

    return combined_request;
}

static void set_regen(uint8_t throttlePos_u8) {
    float combined_request = get_combined_request(throttlePos_u8);

    static cmr_torqueDistributionNm_t torquesPos_Nm;
    static cmr_torqueDistributionNm_t torquesNeg_Nm;

    float torque_request_Nm = combined_request * maxFastTorque_Nm;
    float torque_request_fl_Nm;
    float torque_request_fr_Nm;
    float torque_request_rl_Nm;
    float torque_request_rr_Nm;
    if (torque_request_Nm < 0) {
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

    set_motor_speed_and_torque(MOTOR_FL, torque_request_fl_Nm, &torquesPos_Nm,
                               &torquesNeg_Nm);
    set_motor_speed_and_torque(MOTOR_FR, torque_request_fr_Nm, &torquesPos_Nm,
                               &torquesNeg_Nm);
    set_motor_speed_and_torque(MOTOR_RL, torque_request_rl_Nm, &torquesPos_Nm,
                               &torquesNeg_Nm);
    set_motor_speed_and_torque(MOTOR_RR, torque_request_rr_Nm, &torquesPos_Nm,
                               &torquesNeg_Nm);
    setTorqueLimsProtected(&torquesPos_Nm, &torquesNeg_Nm);
}

/**
 * @brief Runs control loops and sets motor torque limits and velocity targets
 * accordingly.
 *
 * @param gear Which gear the vehicle is in.
 * @param throttlePos_u8 Throttle position, 0-255.
 * @param brakePos_u8 Brake position, 0-255.
 * @param swAngle_millideg Steering wheel angle in degrees. Zero-centered, right
 * turn positive.
 * @param battVoltage_mV Accumulator voltage in millivolts.
 * @param battCurrent_mA Accumulator current in milliamps.
 * @param blank_command Additional signal that forces the motor commands to zero
 * vel. and zero torque
 */
void runControls(cmr_canGear_t gear, uint8_t throttlePos_u8,
                 uint8_t brakePos_u8, uint16_t brakePressurePsi_u8,
                 int32_t swAngle_millideg_FL, int32_t swAngle_millideg_FR,
                 int32_t battVoltage_mV, int32_t battCurrent_mA,
                 bool blank_command) {
    int32_t swAngle_millideg = (swAngle_millideg_FL + swAngle_millideg_FR) / 2;
    integrateCurrent();
    if (blank_command) {
        setTorqueLimsAllProtected(0.0f, 0.0f);
        setVelocityInt16All(0);
        return;
    }

    volatile cmr_canAMKActualValues1_t *amkAct1FL =
        canTractiveGetPayload(CANRX_TRAC_INV_FL_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1FR =
        canTractiveGetPayload(CANRX_TRAC_INV_FR_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1RL =
        canTractiveGetPayload(CANRX_TRAC_INV_RL_ACT1);
    volatile cmr_canAMKActualValues1_t *amkAct1RR =
        canTractiveGetPayload(CANRX_TRAC_INV_RR_ACT1);

    const int32_t avgMotorSpeed_RPM = (+(int32_t)(amkAct1FL->velocity_rpm) +
                                       (int32_t)(amkAct1FR->velocity_rpm) +
                                       (int32_t)(amkAct1RL->velocity_rpm) +
                                       (int32_t)(amkAct1RR->velocity_rpm)) /
                                      MOTOR_LEN;

    // Update odometer
    /* Wheel Speed to Vehicle Speed Conversion
     *      (x rotations / 1min) * (16" * PI) *  (2.54*10^-5km/inch)
     *      (1min / 60sec) * (1sec/1000ms) * (5ms period) * (1/13.93 gear ratio)
     *      = x * 7.6378514861 × 10^-9 */
    odometer_km += ((float)avgMotorSpeed_RPM) * 7.6378514861e-9;
    /** @todo check floating point granularity for potential issues with adding
     * small numbers repeatedly to large numbers */

    switch (gear) {
        case CMR_CAN_GEAR_SLOW: {
            setSlowTorque(throttlePos_u8, swAngle_millideg);
            break;
        }
        case CMR_CAN_GEAR_FAST: {
            setFastTorque(throttlePos_u8);
            // set_fast_torque_with_slew(throttlePos_u8, 29.0f);
            break;
        }
        case CMR_CAN_GEAR_ENDURANCE: {
            // setFastTorqueWithParallelRegen(brakePressurePsi_u8,
            // throttlePos_u8);
            set_regen(throttlePos_u8);
            // set_regen_with_slew(throttlePos_u8, 29.0f);
            break;
        }
        case CMR_CAN_GEAR_AUTOX: {
            // const bool assumeNoTurn = true; // TC is not allowed to behave
            // left-right asymmetrically due to the lack of testing const bool
            // ignoreYawRate = false; // TC takes yaw rate into account to
            // prevent the vehicle from stopping unintendedly when turning at
            // low speeds const bool allowRegen = true; // regen-braking is
            // allowed to protect the AC by keeping charge level high const
            // float critical_speed_mps = 5.0f; // using a high value to prevent
            // the vehicle from stopping unintendedly when turning at low speeds
            // const bool clampbyside = true;
            // setYawRateControl(throttlePos_u8, brakePressurePsi_u8,
            // swAngle_millideg, clampbyside);
            // setYawRateAndTractionControl(throttlePos_u8, brakePressurePsi_u8,
            // swAngle_millideg, assumeNoTurn, ignoreYawRate, allowRegen,
            // critical_speed_mps);
            set_optimal_control(get_combined_request(throttlePos_u8),
                                swAngle_millideg_FL, swAngle_millideg_FR, true);
            break;
        }
        case CMR_CAN_GEAR_SKIDPAD: {
            set_optimal_control((float)throttlePos_u8 / UINT8_MAX,
                                swAngle_millideg_FL, swAngle_millideg_FR,
                                false);
            break;
        }
        case CMR_CAN_GEAR_ACCEL: {
            const bool assumeNoTurn =
                true;  // TC is not allowed to behave left-right asymmetrically
                       // because it's meaningless in accel
            const bool ignoreYawRate =
                true;  // TC ignores yaw rate because it's meaningless in accel
            const bool allowRegen = false;  // regen-braking is not allowed
                                            // because it's meaningless in accel
            const float critical_speed_mps =
                0.0f;  // using a low value to prevent excessive wheel spin at
                       // low speeds
            const float leftRightBias_Nm =
                0.0f;  // YRC is not enabled for accel, so there should be no
                       // left-right torque bias
            setLaunchControl(throttlePos_u8, brakePressurePsi_u8,
                             swAngle_millideg, leftRightBias_Nm, assumeNoTurn,
                             ignoreYawRate, allowRegen, critical_speed_mps);
            // Dry testing only.
            // setLaunchControl(255, 0, 0, leftRightBias_Nm, assumeNoTurn,
            // ignoreYawRate, allowRegen, critical_speed_mps);
            break;
        }
        case CMR_CAN_GEAR_TEST: {
            // float target_speed_mps = 5.0f;
            // getProcessedValue(&target_speed_mps, SLOW_SPEED_INDEX, float_1_decimal);
            set_fast_torque_with_slew(throttlePos_u8, 360);
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
    if (cmr_canRXMetaTimeoutError(timeoutMsg, xTaskGetTickCount()) == (-1))
        return;

    if (coulombCounting.KCoulombs == 0.0f) {
        previousTickCount = xTaskGetTickCount();
        coulombCounting.KCoulombs = 0.001f;
    } else {
        const float packCurrent_kA = getPackCurrent() * 0.001f;
        const TickType_t currentTick = xTaskGetTickCount();
        coulombCounting.KCoulombs +=
            ((currentTick - previousTickCount) * 0.001f) * packCurrent_kA;
        previousTickCount = currentTick;
    }
}

/**
 * @brief Sets motor torques and velocities according to speed limit for slow
 * gear.
 *
 * @param throttlePos_u8 Throttle position, 0-255.
 */
void setSlowTorque(uint8_t throttlePos_u8, int32_t swAngle_millideg) {
    const float reqTorque = maxSlowTorque_Nm * (float)(throttlePos_u8) / (float)(UINT8_MAX);

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
void setFastTorque(uint8_t throttlePos_u8) {
    const float reqTorque =
        maxFastTorque_Nm * (float)(throttlePos_u8) / (float)(UINT8_MAX);
    //    setTorqueLimsAllProtected(reqTorque, 0.0f);
    setTorqueLimsUnprotected(MOTOR_FL, reqTorque, 0.0f);
    setTorqueLimsUnprotected(MOTOR_FR, reqTorque, 0.0f);
    setTorqueLimsUnprotected(MOTOR_RR, reqTorque, 0.0f);
    setTorqueLimsUnprotected(MOTOR_RL, reqTorque, 0.0f);
    setVelocityInt16All(maxFastSpeed_rpm);
}

void set_fast_torque_with_slew(uint8_t throttlePos_u8, int16_t slew) {
    const float reqTorque = maxFastTorque_Nm * (float)(throttlePos_u8) / (float)(UINT8_MAX);

    if(reqTorque > 0.0f) {
        int16_t fl_rpm = getMotorSpeed_rpm(MOTOR_FL);
        int16_t fr_rpm = getMotorSpeed_rpm(MOTOR_FR);
        int16_t rl_rpm = getMotorSpeed_rpm(MOTOR_RL);
        int16_t rr_rpm = getMotorSpeed_rpm(MOTOR_RR);
        setVelocityFloat(MOTOR_FL, fl_rpm + slew);
        setVelocityFloat(MOTOR_FR, fr_rpm + slew);
        setVelocityFloat(MOTOR_RL, rl_rpm + slew);
        setVelocityFloat(MOTOR_RR, rr_rpm + slew);
    } else {
        setVelocityInt16All(maxFastSpeed_rpm);
    }
    setTorqueLimsUnprotected(MOTOR_FL, reqTorque, 0.0f);
    setTorqueLimsUnprotected(MOTOR_FR, reqTorque, 0.0f);
    setTorqueLimsUnprotected(MOTOR_RL, reqTorque, 0.0f);
    setTorqueLimsUnprotected(MOTOR_RR, reqTorque, 0.0f);
}

void setFastTorqueWithParallelRegen(uint16_t brakePressurePsi_u8, uint8_t throttlePos_u8)
{
    if (brakePressurePsi_u8 >= braking_threshold_psi) {
        setParallelRegen(throttlePos_u8, brakePressurePsi_u8, 0);
    } else {
        const float reqTorque =
            maxFastTorque_Nm * (float)(throttlePos_u8) / (float)(UINT8_MAX);
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
 * @param yaw_rate_radps_sae The yaw rate of the vehicle in SAE coordinates (a
 * right turn is positive)
 */
static void update_whl_vels_and_angles(  // HELPFUL IF WE WANT TO MODEL LATERAL FORCES
    float steering_ang_fl, float steering_ang_fr,
    float longitudinal_velocity_mps, float lateral_velocity_mps,
    float yaw_rate_radps_sae) {
    // project vehicle velocities onto each wheel
    const float V_whl_flx =
        +cosf(steering_ang_fl) *
            (longitudinal_velocity_mps + yaw_rate_radps_sae * chassis_w_f) +
        sinf(steering_ang_fl) *
            (lateral_velocity_mps + yaw_rate_radps_sae * chassis_a);
    const float V_whl_fly =
        -sinf(steering_ang_fl) *
            (longitudinal_velocity_mps + yaw_rate_radps_sae * chassis_w_f) +
        cosf(steering_ang_fl) *
            (lateral_velocity_mps + yaw_rate_radps_sae * chassis_a);
    const float V_whl_frx =
        +cosf(steering_ang_fr) *
            (longitudinal_velocity_mps - yaw_rate_radps_sae * chassis_w_f) +
        sinf(steering_ang_fr) *
            (lateral_velocity_mps + yaw_rate_radps_sae * chassis_a);
    const float V_whl_fry =
        -sinf(steering_ang_fr) *
            (longitudinal_velocity_mps - yaw_rate_radps_sae * chassis_w_f) +
        cosf(steering_ang_fr) *
            (lateral_velocity_mps + yaw_rate_radps_sae * chassis_a);
    const float V_whl_rlx =
        longitudinal_velocity_mps + yaw_rate_radps_sae * chassis_w_r;
    const float V_whl_rly =
        lateral_velocity_mps - yaw_rate_radps_sae * chassis_b;
    const float V_whl_rrx =
        longitudinal_velocity_mps - yaw_rate_radps_sae * chassis_w_r;
    const float V_whl_rry =
        lateral_velocity_mps - yaw_rate_radps_sae * chassis_b;

    // take the magnitude of wheel velocities
    frontWhlVelocities.v_whl_fl = hypotf(V_whl_flx, V_whl_fly);
    frontWhlVelocities.v_whl_fr = hypotf(V_whl_frx, V_whl_fry);
    rearWhlVelocities.v_whl_rl = hypotf(V_whl_rlx, V_whl_rly);
    rearWhlVelocities.v_whl_rr = hypotf(V_whl_rrx, V_whl_rry);
}

/**
 * @brief velocity schedule -- target vehicle velocity given elapsed accel time
 *
 * @param t_sec elapsed accel time (sec)
 *
 * @return the target vehicle velocity (m/s)
 */
static float getFFScheduleVelocity(float t_sec) {
    float tMax = 6.4f;  // limit time for safety reasons
    float scheduleVelocity_mps = 1.0f;

    float scheduleVelocity_mps2 = 12.1;
    // getProcessedValue(&scheduleVelocity_mps2, LAUNCH_SLOPE_INDEX,
    // float_1_decimal);

    if (t_sec < 0.0f) {
        scheduleVelocity_mps = 0.0f;
    } else if (t_sec < tMax) {
        float startingVel_mps = 5.0f;
        scheduleVelocity_mps =
            (scheduleVelocity_mps2 * t_sec) + startingVel_mps;
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
    uint8_t throttlePos_u8, uint16_t brakePressurePsi_u8,
    int32_t swAngle_millideg, /** IGNORED if assumeNoTurn is true */
    float leftRightBias_Nm, /** IGNORED UNLESS traction_control_mode (defined in
                               the function) is TC_MODE_TORQUE */
    bool assumeNoTurn, bool ignoreYawRate, bool allowRegen,
    float critical_speed_mps) {
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

        if (launchControlButtonPressed && !action_button_pressed) {
            if (!launchControlActive) {
                startTickCount = xTaskGetTickCount();
                launchControlActive = true;
            }
        }
    }

    // Not braking, throttle engaged, no button pressed, launch control is
    // active. bool ready_to_accel = brakePressurePsi_u8 < braking_threshold_psi
    // && throttlePos_u8 > 0 && !action_button_pressed && launchControlActive;
    bool ready_to_accel =
        throttlePos_u8 > 0 && !action_button_pressed && launchControlActive;
    if (false == ready_to_accel) {
        // setTorqueLimsAllProtected(0.0f, 0.0f);
        setTorqueLimsUnprotected(MOTOR_FL, 0.0, 0.0f);
        setTorqueLimsUnprotected(MOTOR_FR, 0.0, 0.0f);
        setTorqueLimsUnprotected(MOTOR_RR, 0.0, 0.0f);
        setTorqueLimsUnprotected(MOTOR_RL, 0.0, 0.0f);
        setVelocityInt16All(0);
        return;
    }

    float time_s = (float)(xTaskGetTickCount() - startTickCount) * (0.001f);
    if (time_s >= launch_control_max_duration_s) {
        setFastTorque(throttlePos_u8);
        return;
    }

    if (use_solver) {
        // swAngle_millideg = 0 means assume no turn.
        set_optimal_control((float)throttlePos_u8 / UINT8_MAX, 0,
                                          0, false);
    } else {
        TickType_t tick = xTaskGetTickCount();
        float seconds = (float)(tick - startTickCount) *
                        (0.001f);  // convert Tick Count to Seconds

        float scheduled_wheel_vel_mps = getFFScheduleVelocity(seconds);
        float motor_rpm = gear_ratio * 60.0f * scheduled_wheel_vel_mps /
                          (2 * M_PI * effective_wheel_rad_m);

        // Feedforward only.
        // setVelocityFloat(MOTOR_RL, motor_rpm);
        // setVelocityFloat(MOTOR_RR, motor_rpm);
        // setVelocityFloat(MOTOR_FL, motor_rpm);
        // setVelocityFloat(MOTOR_FR, motor_rpm);

        // Feedforward with front clamping.
        // setVelocityFloat(MOTOR_RL, motor_rpm);
        // setVelocityFloat(MOTOR_RR, motor_rpm);
        // int16_t clamp_rpm = (getMotorSpeed_rpm(MOTOR_RL) +
        // getMotorSpeed_rpm(MOTOR_RR)) / 2; setVelocityInt16(MOTOR_FL,
        // clamp_rpm); setVelocityInt16(MOTOR_FR, clamp_rpm);

        // Feedforward with front clamping with multiplier.
        setVelocityFloat(MOTOR_RL, maxFastSpeed_rpm);
        setVelocityFloat(MOTOR_RR, maxFastSpeed_rpm);
        float clamp_rpm =
            (float)(getMotorSpeed_rpm(MOTOR_RL) + getMotorSpeed_rpm(MOTOR_RR)) *
            0.5f;
        // 12Nm torque * 67.5N traction per Nm / 1600N downforce * 0.11 max slip
        // ratio = 0.0556875 1.11 / 1.0556875 = 1.051447516
        setVelocityFloat(MOTOR_FL, clamp_rpm * 1.07f);
        setVelocityFloat(MOTOR_FR, clamp_rpm * 1.07f);

        // Go crazy.
        // motor_rpm = 20000.0f;
        // setVelocityFloat(MOTOR_RL, motor_rpm);
        // setVelocityFloat(MOTOR_RR, motor_rpm);
        // setVelocityFloat(MOTOR_FL, motor_rpm);
        // setVelocityFloat(MOTOR_FR, motor_rpm);
        // const float reqTorque = maxFastTorque_Nm;

        const float reqTorque =
            maxFastTorque_Nm * (float)(throttlePos_u8) / (float)(UINT8_MAX);

        cmr_torqueDistributionNm_t pos_torques_Nm = {
            .fl = reqTorque, .fr = reqTorque, .rl = reqTorque, .rr = reqTorque};
        cmr_torqueDistributionNm_t neg_torques_Nm = {
            .fl = 0.0f, .fr = 0.0f, .rl = 0.0f, .rr = 0.0f};
        // setTorqueLimsUnprotected(MOTOR_FL, pos_torques_Nm.fl,
        // neg_torques_Nm.fl); setTorqueLimsUnprotected(MOTOR_FR,
        // pos_torques_Nm.fr, neg_torques_Nm.fr);
        // setTorqueLimsUnprotected(MOTOR_RR, pos_torques_Nm.rr,
        // neg_torques_Nm.rr); setTorqueLimsUnprotected(MOTOR_RL,
        // pos_torques_Nm.rl, neg_torques_Nm.rl);
        setTorqueLimsProtected(&pos_torques_Nm, &neg_torques_Nm);
        return;
    }
}

float get_optimal_yaw_rate(float swangle_rad, float velocity_x_mps) {

    static const float natural_understeer_gradient = 0.011465f; //rad/g

    const float distance_between_axles_m = chassis_a + chassis_b;
    // const float yaw_rate_setpoint_radps = swangle_rad * velocity_x_mps /
    //     (distance_between_axles_m + velocity_x_mps * velocity_x_mps *
    //     natural_understeer_gradient);
    const float yaw_rate_setpoint_radps =
        swangle_rad * velocity_x_mps / distance_between_axles_m;
    return yaw_rate_setpoint_radps;
}

/**
 * @brief Calculate the control action (left-right torque bias) of the yaw rate
 * controller
 * @param swAngle_millideg Steering wheel angle
 */
float getYawRateControlLeftRightBias(int32_t swAngle_millideg) {
    float velocity_x_mps;
    if (movella_state.status.gnss_fix) {
        velocity_x_mps = movella_state.velocity.x;
        yrcDebug.controls_bias = 1;
    } else {
        velocity_x_mps = getTotalMotorSpeed_radps() * 0.25f / gear_ratio *
                         effective_wheel_rad_m;
        yrcDebug.controls_bias = -1;
    }

    const float swangle_rad = swAngleMillidegToSteeringAngleRad(swAngle_millideg);
    const float actual_yaw_rate_radps_sae = movella_state.gyro.z;
    const float optimal_yaw_rate_radps = get_optimal_yaw_rate(swangle_rad, velocity_x_mps);

    yrcDebug.controls_current_yaw_rate =
        (int16_t)(1000.0f * actual_yaw_rate_radps_sae);
    yrcDebug.controls_target_yaw_rate =
        (int16_t)(1000.0f * optimal_yaw_rate_radps);
    yrcDebug.controls_pid = yrc_kp;

    const float left_right_bias = yrc_kp * (actual_yaw_rate_radps_sae - optimal_yaw_rate_radps);
    return left_right_bias;
}