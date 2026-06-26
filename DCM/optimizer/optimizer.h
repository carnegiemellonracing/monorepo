//
// Created by 26958 on 6/30/2024.
//

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "qform.h"

typedef struct {
    box_variable_role_e role;
    float val;
} box_variable_assignment_t;

typedef struct {
    float weights[NUM_VARS];
    float limit;
} lin_constraint_t;

typedef struct {
    float content[NUM_VARS];
    void* link[NUM_VARS];
} optimum_t;

typedef struct {
    float areq;
    float mreq;
    float theta_left;
    float theta_right;
    float accel_weights[NUM_VARS];
    float moment_weights[NUM_VARS];
    float diagonal_weights[NUM_VARS];
    float omegas[NUM_VARS];
    float power_limit;
    int dim;
    lin_constraint_t new_constraint;
    qform_t qform;
    qform_t qform_full;
    optimum_t optimum;
    box_variable_assignment_t optimal_assignment[NUM_VARS];
    float optimal_cost;
    box_variable_t variable_profile[NUM_VARS];
    float Qinv[NUM_VARS * NUM_VARS];
} optimizer_state_t;

/**
 * Try all 3^4 = 81 cases and finds the best.
 */
void solve(optimizer_state_t *state);
void solve_one_case(optimizer_state_t *state);
void solver_set_k_lin(float d);
void solver_set_k_yaw(float d);
void solver_set_k_tie(float d);
float solver_get_k_lin();
float solver_get_k_yaw();
float solver_get_k_tie();

#endif //OPTIMIZER_H
