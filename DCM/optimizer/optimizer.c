//
// Created by 26958 on 6/30/2024.
//

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <float.h>
#include <math.h>

#include "constants.h"
#include "optimizer.h"
#include "qform.h"

#define MINIMUM_K_VALUE (0.005f) // Prevent numerical instability.

static double k_lin = 80.0;
static double k_yaw = 0.30;
static double k_tie = 0.008;

static double k_power = 0.000; // tunable, new 

/**
 * Computes 1/2 x^T Q x + q x + c.
 */
void compute_accel_weights(optimizer_state_t *state) {
    double temp = gear_ratio / (effective_wheel_rad_m * car_mass_kg);
    state->accel_weights[0] = temp;
    state->accel_weights[1] = temp;
    state->accel_weights[2] = temp;
    state->accel_weights[3] = temp;
}

/**
 * Generates weights for computing yaw moment.
 * @note Only considers longitudinal force.
 */
void compute_moment_weights(optimizer_state_t *state) {
    double temp = gear_ratio / effective_wheel_rad_m;
    double theta_left = state->theta_left;
    double theta_right = state->theta_right;
    state->moment_weights[0] = (half_wheelbase_m * sin(theta_left) - half_trackwidth_m * cos(theta_left)) * temp;
    state->moment_weights[1] = (half_wheelbase_m * sin(theta_right) + half_trackwidth_m * cos(theta_right)) * temp;
    state->moment_weights[2] = -half_trackwidth_m * temp;
    state->moment_weights[3] = half_trackwidth_m * temp;
}

/**
 * Generates weights for computing longitudinal acceleration.
 * @note Only considers longitudinal force.
 */
void load_diagonal_weights(optimizer_state_t *state) {
    state->diagonal_weights[0] = k_tie;
    state->diagonal_weights[1] = k_tie;
    state->diagonal_weights[2] = k_tie;
    state->diagonal_weights[3] = k_tie;
}


// *****NEW*****
/// loads power weights
void load_power_weights(const optimizer_state_t *state, double power_diag[4], double k_power) {
    for (int i = 0; i < 4; i++) {
        double w = state->omegas[i];
        power_diag[i] = k_power * (w * w);
    }
}

static inline bool box_variable_is_valid(box_variable_t *v, double value) {
    assert(v->role == UNCONSTRAINED);
    return v->lower <= value && value <= v->upper;
}

static bool check_box_constraints(optimizer_state_t *state) {
    bool all_variables_legal = true;
    for(int i = 0; i < state->dim; i++) {
        bool valid = box_variable_is_valid((box_variable_t*)state->optimum.link[i], state->optimum.content[i]);
        if(false == valid) {
            all_variables_legal = false;
            break;
        }
    }
    return all_variables_legal;
}

static void evaluate_candidate(optimizer_state_t *state) {
    int dim = state->dim;
    box_variable_t *vp = state->variable_profile;
    const double cost = evaluate_cost(&state->qform, dim, state->optimum.content);
    if(cost < state->optimal_cost) {
        state->optimal_cost = cost;
        int index = 0;
        for(int i = 0; i < NUM_VARS; i++) {
            const box_variable_role_e role = vp[i].role;
            state->optimal_assignment[i].role = role;
            if(role == LOWER)
                state->optimal_assignment[i].val = vp[i].lower;
            else if(role == UPPER)
                state->optimal_assignment[i].val = vp[i].upper;
            else {
                state->optimal_assignment[i].val = state->optimum.content[index];
                index += 1;
            }
        }
    }
}

static int load_free_variable_refs(optimizer_state_t *state) {
    uint32_t dim = 0;
    double power_leftover = state->power_limit;
    box_variable_t *vp = state->variable_profile;
    for(int i = 0; i < NUM_VARS; i++) {
        switch (vp[i].role) {
            case UNCONSTRAINED:
                state->optimum.link[dim] = &vp[i];
                state->new_constraint.weights[dim] = state->omegas[i];
                dim += 1;
                break;
            case LOWER:
                power_leftover -= vp[i].lower * state->omegas[i];
                break;
            case UPPER:
                power_leftover -= vp[i].upper * state->omegas[i];
                break;
        }
    }
    state->new_constraint.limit = power_leftover;
    return dim;
}

static void solve_with_equality_linear_constraint(optimizer_state_t *state) {
    static double temp[NUM_VARS];
    static double bar[NUM_VARS];

    // Perform linear equality constrained solve
    qform_t *qf = &state->qform;
    int dim = state->dim;
    mat_vec_mul(state->Qinv, state->new_constraint.weights, temp, dim);
    double denom = dot_product(state->new_constraint.weights, temp, dim);
    if(denom == 0.0)
        return;
    double dual = (-state->new_constraint.limit - dot_product(temp, qf->q, dim)) / denom;
    for(int i = 0; i < dim; i++)
        bar[i] = -qf->q[i] - dual * state->new_constraint.weights[i];
    mat_vec_mul(state->Qinv, bar, state->optimum.content, dim);
}

/**
 * Solves for a given configuration of variable roles.
 */
void solve_one_case(optimizer_state_t *state) {
    box_variable_t *vp = state->variable_profile;

    uint32_t dim = load_free_variable_refs(state);
    state->dim = dim;

    zero_qform(&state->qform, dim);

    compose_error_qform_addto(vp, state->accel_weights, state->areq, k_lin, &state->qform, NUM_VARS);
    compose_error_qform_addto(vp, state->moment_weights, state->mreq, k_yaw, &state->qform, NUM_VARS);
    compose_diagonal_qform_addto(vp, state->diagonal_weights, &state->qform, dim, NUM_VARS);

    find_optimum(&state->qform, dim, state->optimum.content, state->Qinv);

    bool all_variables_legal = check_box_constraints(state);
    double power_acc = 0.0;
    for(int i = 0; i < dim; i++) {
        power_acc += state->optimum.content[i] * state->new_constraint.weights[i];
    }

    if(all_variables_legal && power_acc <= state->new_constraint.limit) {
        evaluate_candidate(state);
        return;
    }

    if(dim == 0)
        return;

    if(dim == 1)
        state->optimum.content[0] = state->new_constraint.limit / state->new_constraint.weights[0];
    else
        solve_with_equality_linear_constraint(state);

    all_variables_legal = check_box_constraints(state);

    if(all_variables_legal) {
        evaluate_candidate(state);
    }
}

void solve(optimizer_state_t *state) {

    compute_accel_weights(state);
    compute_moment_weights(state);
    load_diagonal_weights(state);

    state->optimal_cost = FLT_MAX;

    for(box_variable_role_e t1 = LOWER; t1 <= UNCONSTRAINED; t1++)
        for(box_variable_role_e t2 = LOWER; t2 <= UNCONSTRAINED; t2++)
            for(box_variable_role_e t3 = LOWER; t3 <= UNCONSTRAINED; t3++)
                for(box_variable_role_e t4 = LOWER; t4 <= UNCONSTRAINED; t4++) {

                    state->variable_profile[0].role = t1;
                    state->variable_profile[1].role = t2;
                    state->variable_profile[2].role = t3;
                    state->variable_profile[3].role = t4;

                    solve_one_case(state);
                }
}

void solver_set_k_lin(double d) {
	d = fmax(d, MINIMUM_K_VALUE);
	k_lin = d;
}

void solver_set_k_yaw(double d) {
	d = fmax(d, MINIMUM_K_VALUE);
	k_yaw = d;
}

void solver_set_k_tie(double d) {
	d = fmax(d, MINIMUM_K_VALUE);
	k_tie = d;
}

double solver_get_k_lin() {
    return k_lin;
}

double solver_get_k_yaw() {
    return k_yaw;
}

double solver_get_k_tie() {
    return k_tie;
}
