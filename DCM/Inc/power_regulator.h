#include "constants.h"

#define CLAMP(x, lo, hi) (fmaxf(fminf(x, hi), lo))

typedef enum {
    ENDURANCE_FIRST_HALF = 0,
    ENDURANCE_SECOND_HALF,
} endurance_part_t;

static const power_feedforward_W = 25000.0f;
static const float Kp = 1000.0f;
static const float Ki = 100.0f;

typedef struct {
    float ac_target_soc_J;
    float ac_budget_J;
    float initial_odometer_m;
    float odometer_target_m;

    float odometer_m;
    float endurance_completion; // Percentage.

    float ac_budget_used_J;
    float ac_completion; // Percentage.

    struct {
        float error;
        float error_int;
    } PI;

    float power_setpoint_W;
} power_regulator_state_t;

volatile power_regulator_state_t pr_state; 

float power_regulator_init(float odometer_m, float ac_soc_J, endurance_part_t part);
float power_regulator_step_200Hz(float odometer_m, float ac_soc_J);
float power_regulator_get_power_limit_W();
