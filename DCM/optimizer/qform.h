//
// Created by 26958 on 7/11/2024.
//

#ifndef QFORM_H
#define QFORM_H

#include "common.h"

typedef struct {
    float Q[NUM_VARS * NUM_VARS];
    float q[NUM_VARS];
    float c;
} qform_t;

void zero_matrix(float *matrix, int n);

void mat_vec_mul(float* A, float* x, float* y, int n);

/**
 * Computes outer(arr, arr) * scalar and adds the result to @param dest.
 * @param n size of arr.
 */
void outer_product_addto(const float *arr, const float scalar, float *dest, const int n);

/**
 * Computes arr * scalar and adds the result to @param dest.
 * @param n size of arr.
 */
void arrmul_addto(const float *arr, const float scalar, float *dest, const int n);

/**
 * Computes weight @ (x ⊙ x) and adds the result to @param dest.
 * @note It does not need a scalar as input as that can be multiplied to weight beforehand.
 */
void compose_diagonal_qform_addto(const box_variable_t *profile, const float *weight_arr, qform_t* const dest, int dim, const int n);

/**
 * Computes (weight @ x - target)^2 * scalar and adds the result to @param dest.
 * Constant variables only affect q and c, while free variables only affect Q.
 * @param n size of arr.
 * @note Conforms to the format 1/2 x^T Q x + q x + c, so the raw q and c are halved.
 */
void compose_error_qform_addto(const box_variable_t *profile, const float *weight_arr, const float target, const float scalar, qform_t* const dest, const int n);

void print_vector(float* vec, int n);

float quadratic(const float *Q, const int dim, const float *x);


void find_optimum(qform_t *qf, int dim, float *optimum, float *inverse_dest);

/**
 * Must be called before weight accumulation.
 */
void zero_qform(qform_t *qf, int dim);

float dot_product(float *a, float *b, int n);

/**
 * Prints out a quadratic form
 */
void print_qform(qform_t *qf, int dim);

float evaluate_cost(qform_t *qf, int dim, float *x);

#endif //QFORM_H
