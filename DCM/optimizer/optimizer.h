//
// Created by 26958 on 6/30/2024.
//

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "qform.h"

typedef struct {
    box_variable_role_e role;
    double val;
} box_variable_assignment_t;

typedef struct {
    double weights[NUM_VARS];
    double limit;
} lin_constraint_t;

typedef struct {
    double content[NUM_VARS];
    void* link[NUM_VARS];
} optimum_t;

typedef struct {
    double areq;
    double mreq;
    double theta;
    double accel_weights[NUM_VARS];
    double moment_weights[NUM_VARS];
    double diagonal_weights[NUM_VARS];
    double omegas[NUM_VARS];
    double power_limit;
    int dim;
    lin_constraint_t new_constraint;
    qform_t qform;
    optimum_t optimum;
    box_variable_assignment_t optimal_assignment[NUM_VARS];
    double optimal_cost;
    box_variable_t variable_profile[NUM_VARS];
    double Qinv[NUM_VARS * NUM_VARS];
} optimizer_state_t;

/**
 * Try all 3^4 = 81 cases and finds the best.
 */
void solve(optimizer_state_t *state);
void solver_set_k_lin(double d);
void solver_set_k_yaw(double d);
void solver_set_k_tie(double d);
double solver_get_k_lin();
double solver_get_k_yaw();
double solver_get_k_tie();

#endif //OPTIMIZER_H
