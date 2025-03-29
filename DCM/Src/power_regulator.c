#include "power_regulator.h"
#include "FreeRTOS.h"
#include <stdbool.h>

void power_regulator_init(float odometer_m, float ac_soc_J, endurance_part_t part) {
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

void power_regulator_step_1Hz(float odometer_m, float ac_soc_J) {
    pr_state.odometer_m = (odometer_m - pr_state.initial_odometer_m) * odometer_correction_factor;
    pr_state.ac_budget_used_J = ac_soc_J - pr_state.ac_target_soc_J;
    pr_state.ac_completion = pr_state.ac_budget_used_J / pr_state.ac_budget_J;
    pr_state.endurance_completion = pr_state.odometer_m / pr_state.odometer_target_m;

    float error = pr_state.ac_completion - pr_state.endurance_completion;
    pr_state.PI.error = error;
    float error_int = pr_state.PI.error_int + error * power_regulator_step_s;
    error_int = CLAMP(error_int + error, -100.0, 100.0);
    pr_state.PI.error_int = error_int;

    float power_setpoint_W = power_feedforward_W - error * Kp - error_int * Ki;
    pr_state.power_setpoint_W = power_setpoint_W;
}

float power_regulator_get_power_limit_W() {
    return pr_state.power_setpoint_W;
}

void power_regulator_test() {
    float odometer_initial_m = 1.0f;
    float ac_initial_soc_J = 21.6e6f;
    float simulation_length_s = 10.0f;
    float odometer_speed_mps = 12.22f;

    float average_power_W = (ac_initial_soc_J - power_regulator_ac_margin_J) / 1800.0f;
    power_regulator_init(odometer_initial_m, ac_initial_soc_J, ENDURANCE_FIRST_HALF);
    for(int i = 0; i < (int)(simulation_length_s * power_regulator_frequency_Hz); i++) {
        float odometer_m = odometer_initial_m + odometer_speed_mps * simulation_length_s;
        float ac_soc_J = ac_initial_soc_J - (i + 1) * average_power_W;
        power_regulator_step_1Hz(odometer_m, ac_soc_J);
        volatile float power_W = power_regulator_get_power_limit_W();
    }
}
