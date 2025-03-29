#include "power_regulator.h"
#include "FreeRTOS.h"
#include <stdbool.h>

float power_regulator_init(float odometer_m, float ac_soc_J, endurance_part_t part) {
    // Drive until 15% SoC.
    pr_state.ac_target_soc_J = power_regulator_ac_margin_J;
    pr_state.odometer_target_m = endurance_half_length_m;
    switch (part)
    {
    case ENDURANCE_FIRST_HALF:
        // Consume half of AC budget in first half.
        pr_state.ac_budget_J = 0.5f * (ac_soc_J - pr_state.ac_target_soc_J);
        break;
    case ENDURANCE_SECOND_HALF:
        pr_state.ac_budget_J = ac_soc_J - pr_state.ac_target_soc_J;
        break;
    default:
        configASSERT(false);
    }
    pr_state.initial_odometer_m = odometer_m;
    pr_state.odometer_m = 0.0f;
    pr_state.ac_completion = 0.0f;
    pr_state.endurance_completion = 0.0f;
    pr_state.PI.error = 0.0f;
    pr_state.PI.error_int = 0.0f;
    pr_state.power_setpoint_W = power_feedforward_W;
}

float power_regulator_step_200Hz(float odometer_m, float ac_soc_J) {
    pr_state.odometer_m = odometer_m - pr_state.initial_odometer_m;
    pr_state.ac_budget_used_J = ac_soc_J - pr_state.ac_target_soc_J;
    pr_state.ac_completion = pr_state.ac_budget_used_J / pr_state.ac_budget_J;
    pr_state.endurance_completion = pr_state.odometer_m / pr_state.odometer_target_m;

    float error = pr_state.ac_completion - pr_state.endurance_completion;
    pr_state.PI.error = error;
    float error_int = pr_state.PI.error_int + error;
    error_int = CLAMP(error_int + error, -100.0, 100.0);
    pr_state.PI.error_int = error_int;

    float power_setpoint_W = power_feedforward_W - error * Kp - error_int * Ki;
    pr_state.power_setpoint_W = power_setpoint_W;
}
