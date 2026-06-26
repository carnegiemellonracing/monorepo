//
// Created by 26958 on 6/30/2024.
//

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <float.h>
#include <math.h>
#include <assert.h>

#include "optimizer_const.h"
#include "optimizer.h"
#include "qform.h"

#define MINIMUM_K_VALUE (0.005f) // Prevent numerical instability.

static float k_lin = 80.0;
static float k_yaw = 0.30;
static float k_tie = 0.008;

/**
 * Computes 1/2 x^T Q x + q x + c.
 */
void compute_accel_weights(optimizer_state_t *state) {
    float temp = gear_ratio / (effective_wheel_rad_m * car_mass_kg);
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
    float temp = gear_ratio / effective_wheel_rad_m;
    float theta_left = state->theta_left;
    float theta_right = state->theta_right;
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

static inline bool box_variable_is_valid(box_variable_t *v, float value) {
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
    const float cost = evaluate_cost(&state->qform, dim, state->optimum.content);
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

/**
 * Builds the full unconstrained quadratic form once per solve() — Q, q, c over all NUM_VARS.
 * Each of the 81 cases then derives its reduced problem from this via a Schur-style submatrix
 * extraction in build_reduced_problem(), avoiding repeated outer products.
 */
static void compute_full_qform(optimizer_state_t *state) {
    qform_t *qf = &state->qform_full;
    zero_matrix(qf->Q, NUM_VARS);
    zero_array(qf->q, NUM_VARS);
    qf->c = 0.0;

    outer_product_addto(state->accel_weights, k_lin, qf->Q, NUM_VARS);
    arrmul_addto(state->accel_weights, -state->areq * k_lin, qf->q, NUM_VARS);
    qf->c += 0.5 * state->areq * state->areq * k_lin;

    outer_product_addto(state->moment_weights, k_yaw, qf->Q, NUM_VARS);
    arrmul_addto(state->moment_weights, -state->mreq * k_yaw, qf->q, NUM_VARS);
    qf->c += 0.5 * state->mreq * state->mreq * k_yaw;

    for(int i = 0; i < NUM_VARS; i++)
        qf->Q[i * NUM_VARS + i] += state->diagonal_weights[i];
}

/**
 * Walks variable_profile once: counts free vars, fills the power-constraint helper, and
 * derives the reduced QP for the free indices F from the precomputed full qform.
 *   Q_new = Q_full[F,F]
 *   q_new = q_full[F]  +  Q_full[F,X] · x_X
 *   c_new = c_full     +  q_full[X]·x_X  +  ½ x_X^T Q_full[X,X] x_X
 */
static int build_reduced_problem(optimizer_state_t *state) {
    const qform_t *full = &state->qform_full;
    qform_t *dest = &state->qform;
    box_variable_t *vp = state->variable_profile;

    int free_idx[NUM_VARS];
    bool is_fixed[NUM_VARS];
    float x_fixed[NUM_VARS];
    int dim = 0;
    float power_leftover = state->power_limit;

    for(int i = 0; i < NUM_VARS; i++) {
        switch (vp[i].role) {
            case UNCONSTRAINED:
                is_fixed[i] = false;
                state->optimum.link[dim] = &vp[i];
                state->new_constraint.weights[dim] = state->omegas[i];
                free_idx[dim] = i;
                dim++;
                break;
            case LOWER:
                is_fixed[i] = true;
                x_fixed[i] = vp[i].lower;
                power_leftover -= vp[i].lower * state->omegas[i];
                break;
            case UPPER:
                is_fixed[i] = true;
                x_fixed[i] = vp[i].upper;
                power_leftover -= vp[i].upper * state->omegas[i];
                break;
        }
    }
    state->new_constraint.limit = power_leftover;

    for(int a = 0; a < dim; a++) {
        int i = free_idx[a];
        for(int b = 0; b < dim; b++) {
            int j = free_idx[b];
            dest->Q[a * dim + b] = full->Q[i * NUM_VARS + j];
        }
    }

    for(int a = 0; a < dim; a++) {
        int i = free_idx[a];
        float v = full->q[i];
        for(int j = 0; j < NUM_VARS; j++)
            if(is_fixed[j])
                v += full->Q[i * NUM_VARS + j] * x_fixed[j];
        dest->q[a] = v;
    }

    float c_new = full->c;
    for(int i = 0; i < NUM_VARS; i++) {
        if(!is_fixed[i]) continue;
        c_new += full->q[i] * x_fixed[i];
        for(int j = 0; j < NUM_VARS; j++)
            if(is_fixed[j])
                c_new += 0.5 * full->Q[i * NUM_VARS + j] * x_fixed[i] * x_fixed[j];
    }
    dest->c = c_new;

    return dim;
}

static void solve_with_equality_linear_constraint(optimizer_state_t *state) {
    static float temp[NUM_VARS];
    static float bar[NUM_VARS];

    // Perform linear equality constrained solve
    qform_t *qf = &state->qform;
    int dim = state->dim;
    mat_vec_mul(state->Qinv, state->new_constraint.weights, temp, dim);
    float denom = dot_product(state->new_constraint.weights, temp, dim);
    if(denom == 0.0)
        return;
    float dual = (-state->new_constraint.limit - dot_product(temp, qf->q, dim)) / denom;
    for(int i = 0; i < dim; i++)
        bar[i] = -qf->q[i] - dual * state->new_constraint.weights[i];
    mat_vec_mul(state->Qinv, bar, state->optimum.content, dim);
}

/**
 * Solves for a given configuration of variable roles.
 */
void solve_one_case(optimizer_state_t *state) {
    uint32_t dim = build_reduced_problem(state);
    state->dim = dim;

    find_optimum(&state->qform, dim, state->optimum.content, state->Qinv);

    bool all_variables_legal = check_box_constraints(state);
    float power_acc = 0.0;
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
    compute_full_qform(state);

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

void solver_set_k_lin(float d) {
	d = fmax(d, MINIMUM_K_VALUE);
	k_lin = d;
}

void solver_set_k_yaw(float d) {
	d = fmax(d, MINIMUM_K_VALUE);
	k_yaw = d;
}

void solver_set_k_tie(float d) {
	d = fmax(d, MINIMUM_K_VALUE);
	k_tie = d;
}

float solver_get_k_lin() {
    return k_lin;
}

float solver_get_k_yaw() {
    return k_yaw;
}

float solver_get_k_tie() {
    return k_tie;
}
