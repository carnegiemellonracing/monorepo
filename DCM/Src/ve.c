#include "ve.h"
#include "lut_3d.h"
#include "daq.h"
#include <math.h>
#include <controls_23e.h>

#define PI 3.1415926535897932384626 // Can't include <arm_math.h>

/**
 * @brief Clamps val to [lo, hi]
 */
#define fclamp(val, lo, hi) ( fmin(fmax(val, lo), hi) )

static float rpm_to_mps(float rpm) {
    return rpm / 60 * 2 * PI * 0.239;
}

/**
 * @brief Estimate body velocity from wheelspeed and kappa
 * @param wheelspeed: wheelspeed in m / s
 * @param kappa: slip ratio (starting from 0)
 * @returns estimated body velocity
 */
static float wheelspeed_to_body_velocity(float wheelspeed, float kappa) {
    return wheelspeed / (1 + kappa);
}

static cmr_task_t ve_task;

/**
 * @brief Velocity belief in previous timestep 
 * Unit: m / s
 * Used for guessing the next velocity
 */
static float prev_forward_velocity = 0.0f;

/**
 * @brief Current velocity belief 
 * Unit: m / s
 */
static float cur_forward_velocity = 0.0f;

/**
 * @brief Current measured acceleration by SBG
 * Unit: m / s^2
 */
static float cur_forward_accel = 0.0f;

/**
 * @brief Bag of states used for velocity estimation
 * This is sent to CAN
 */
volatile cmr_canCDCVelocityEstimation1_t velocity_estimator1;
volatile cmr_canCDCVelocityEstimation2_t velocity_estimator2;

/**
 * @brief Finds weight transfer from front to back
 * Unit: N
 */
float get_load_transfer(float acceleration, float aero_lift, float aero_drag) {
    // This is currently crude
    return (acceleration * car_mass + aero_drag) * cg_height / wheelbase;
}

void make_at_least_zero(float *val) {
    *val = fmaxf(*val, 0.0f);
}

float merge_predicted_body_velocities(float FL, float FR, float RL, float RR) {
    return fminf(fmaxf(FL, FR), fmaxf(RL, RR));
}

/**
 * @brief Currently unused
 */
void guess_current_traction(float guessed_vel, 
    float wheelspeed_FL, float wheelspeed_FR, float wheelspeed_RL, float wheelspeed_RR,
    float downforce_FL, float downforce_FR, float downforce_RL, float downforce_RR
) {
    float guessed_slip_FL = (wheelspeed_FL + 1e-8) / (guessed_vel + 1e-8) - 1;
    float guessed_slip_FR = (wheelspeed_FR + 1e-8) / (guessed_vel + 1e-8) - 1;
    float guessed_slip_RL = (wheelspeed_RL + 1e-8) / (guessed_vel + 1e-8) - 1;
    float guessed_slip_RR = (wheelspeed_RR + 1e-8) / (guessed_vel + 1e-8) - 1;
    make_at_least_zero(&guessed_slip_FL);
    make_at_least_zero(&guessed_slip_FR);
    make_at_least_zero(&guessed_slip_RL);
    make_at_least_zero(&guessed_slip_RR);
    float guessed_traction_FL = getFxByKappaDownforceSlipangle(downforce_FL, 0.0f, 0);
    float guessed_traction_FR = getFxByKappaDownforceSlipangle(downforce_FR, 0.0f, 0);
    float guessed_traction_RL = getFxByKappaDownforceSlipangle(downforce_RL, 0.0f, 0);
    float guessed_traction_RR = getFxByKappaDownforceSlipangle(downforce_RR, 0.0f, 0);
}

void ve_routine(void *pvParameters) {
    
    static TickType_t last_it = 0;
    
    while(true) {
        
        const volatile cmr_canDIMActions_t *actions = canVehicleGetPayload(CANRX_VEH_DIM_ACTION_BUTTON);
        const volatile bool reset = (actions->buttons) & BUTTON_SCRN;
        if(reset) veReset();

        const volatile cmr_canSBGIMUAcceleration_t *cur_accel = canDAQGetPayload(CANRX_DAQ_SBG_IMU_ACCEL);
        cur_forward_accel = cur_accel->accel_x_mps2;
        float inertial_force = cur_forward_accel * car_mass;

        float guessed_vel = prev_forward_velocity;
        float aero_lift = aero_lift_coe * guessed_vel * guessed_vel;
        float aero_drag = aero_drag_coe * guessed_vel * guessed_vel;

        /**
         * @brief Free body diagram for car      
         *                            
         *                       /------\
         *             _________/        \ ----> Fdrag
         *          <      Fnet <-- O (cg)
         *            \__O_  Fx <-------  O   --> Frolling
         * 
         * ma = Fnet = Fx - Fdrag - Frolling
         * 
         * Currenly ignoring Frolling
         */
        float tractive_force = inertial_force + aero_drag;

        float load_transfer = get_load_transfer(cur_forward_accel, aero_lift, aero_drag);
        float downforce = car_weight + aero_lift;

        float downforce_FL = (downforce * 0.5 - load_transfer) * 0.5;
        float downforce_FR = downforce_FL;
        float downforce_RL = (downforce * 0.5 + load_transfer) * 0.5;
        float downforce_RR = downforce_RL;

        cmr_canCDCWheelVelocity_t speedFeedback;
        daqWheelSpeedFeedback(&speedFeedback);

        float wheelspeed_FL = rpm_to_mps(speedFeedback.frontLeft_rpm);
        float wheelspeed_FR = rpm_to_mps(speedFeedback.frontRight_rpm);
        float wheelspeed_RL = rpm_to_mps(speedFeedback.rearLeft_rpm);
        float wheelspeed_RR = rpm_to_mps(speedFeedback.rearRight_rpm);
        make_at_least_zero(&wheelspeed_FL);
        make_at_least_zero(&wheelspeed_FR);
        make_at_least_zero(&wheelspeed_RL);
        make_at_least_zero(&wheelspeed_RR);

        float front_percentage = (downforce_FL + downforce_FR) / downforce;
        float tractive_force_FL = tractive_force * front_percentage;
        float tractive_force_FR = tractive_force_FL;
        float tractive_force_RL = tractive_force * (1 - front_percentage);
        float tractive_force_RR = tractive_force_RL;

        float kappa_FL = getKappaByFxDownforceSlipangle(downforce_FL, 0.0f, 0, tractive_force_FL, ASSUME_NO_TURN);
        float kappa_FR = getKappaByFxDownforceSlipangle(downforce_FR, 0.0f, 0, tractive_force_FR, ASSUME_NO_TURN);
        float kappa_RL = getKappaByFxDownforceSlipangle(downforce_RL, 0.0f, 0, tractive_force_RL, ASSUME_NO_TURN);
        float kappa_RR = getKappaByFxDownforceSlipangle(downforce_RR, 0.0f, 0, tractive_force_RR, ASSUME_NO_TURN);
        make_at_least_zero(&kappa_FL);
        make_at_least_zero(&kappa_FR);
        make_at_least_zero(&kappa_RL);
        make_at_least_zero(&kappa_RR);

        float estimated_velocity = merge_predicted_body_velocities(
            wheelspeed_to_body_velocity(wheelspeed_FL, kappa_FL),
            wheelspeed_to_body_velocity(wheelspeed_FR, kappa_FR),
            wheelspeed_to_body_velocity(wheelspeed_RL, kappa_RL),
            wheelspeed_to_body_velocity(wheelspeed_RR, kappa_RR)
        );
        make_at_least_zero(&estimated_velocity);

        cmr_canCDCVelocityEstimation1_t temp1 = {
            .guessed_vel = guessed_vel,
            .estimated_velocity = estimated_velocity
        };
        cmr_canCDCVelocityEstimation2_t temp2 = {
            .aero_drag = aero_drag,
            .aero_lift = aero_lift
        };

        velocity_estimator1 = temp1;
        velocity_estimator2 = temp2;

        prev_forward_velocity = estimated_velocity;
        cur_forward_velocity = estimated_velocity;

        vTaskDelayUntil(&last_it, ve_period_ms);
    }
}

void veInit() {
    veReset();
    cmr_taskInit(
        &ve_task,
        "motorsCommand",
        ve_priority,
        ve_routine,
        NULL
    );
}

void veReset() {
    velocity_estimator1.guessed_vel = 0;
    velocity_estimator1.estimated_velocity = 0;
    velocity_estimator2.aero_drag = 0;
    velocity_estimator2.aero_lift = 0;
}
