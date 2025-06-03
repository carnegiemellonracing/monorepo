//
// Created by 26958 on 7/11/2024.
//

#include <assert.h>
#include <stdio.h>

#include "optimizer.h"

/**
 * Zeros @param matrix.
 * @param n size of matrix.
 */
void zero_matrix(double *matrix, int n) {
    for (int i = 0; i < n * n; i++) matrix[i] = 0.0;
}

/**
 * Zeros @param arr.
 * @param n size of arr.
 */
void zero_array(double *arr, int n) {
    for (int i = 0; i < n; i++) arr[i] = 0.0;
}

/**
 * Computes outer(arr, arr) * scalar and adds the result to @param dest.
 * @param n size of arr.
 */
void outer_product_addto(const double *arr, const double scalar, double *dest,
                         const int n) {
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) dest[n * i + j] += arr[i] * arr[j] * scalar;
}

/**
 * Computes arr * scalar and adds the result to @param dest.
 * @param n size of arr.
 */
void arrmul_addto(const double *arr, const double scalar, double *dest,
                  const int n) {
    for (int i = 0; i < n; i++) dest[i] += arr[i] * scalar;
}

void zero_qform(qform_t *qf, int dim) {
    zero_matrix(qf->Q, dim);
    zero_array(qf->q, dim);
    qf->c = 0;
}

void compose_error_qform_addto(const box_variable_t *profile,
                               const double *weight_arr, const double target,
                               const double scalar, qform_t *const dest,
                               const int n) {
    double new_weight[n];
    double new_target = -target;
    int unconstrained_count = 0;
    for (int i = 0; i < n; i++) {
        const box_variable_t *var = &profile[i];
        if (var->role == UNCONSTRAINED)
            new_weight[unconstrained_count++] = weight_arr[i];
        else {
            if (var->role == LOWER)
                new_target += var->lower * weight_arr[i];
            else
                new_target += var->upper * weight_arr[i];
        }
    }
    outer_product_addto(new_weight, scalar, dest->Q, unconstrained_count);
    arrmul_addto(new_weight, new_target * scalar, dest->q, unconstrained_count);
    dest->c += 0.5 * new_target * new_target * scalar;
}

void compose_diagonal_qform_addto(const box_variable_t *profile,
                                  const double *weight_arr, qform_t *const dest,
                                  int dim, const int n) {
    int index = 0;
    double sum_of_squares = 0.0;
    for (int i = 0; i < n; i++) {
        const box_variable_t *var = &profile[i];
        if (var->role == UNCONSTRAINED) {
            dest->Q[index * dim + index] += weight_arr[i];
            index++;
        } else {
            if (var->role == LOWER)
                sum_of_squares += var->lower * var->lower * weight_arr[i];
            else
                sum_of_squares += var->upper * var->upper * weight_arr[i];
        }
    }
    dest->c += 0.5 * sum_of_squares;
}

void mat_vec_mul(double *A, double *x, double *y, int n) {
    for (int i = 0; i < n; i++) {
        y[i] = 0;
        for (int j = 0; j < n; j++) {
            y[i] += A[i * n + j] * x[j];
        }
    }
}

void invert1x1(double *matrix, double *inverse) {
    assert(matrix[0] != 0.0);
    inverse[0] = 1.0 / matrix[0];
}

void invert2x2(double *matrix, double *inverse) {
    double det = matrix[0] * matrix[3] - matrix[1] * matrix[2];
    assert(det != 0.0);
    inverse[0] = matrix[3] / det;
    inverse[1] = -matrix[1] / det;
    inverse[2] = -matrix[2] / det;
    inverse[3] = matrix[0] / det;
}

void invert3x3(double *matrix, double *inverse) {
    double det = matrix[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7]) -
                 matrix[1] * (matrix[3] * matrix[8] - matrix[5] * matrix[6]) +
                 matrix[2] * (matrix[3] * matrix[7] - matrix[4] * matrix[6]);
    assert(det != 0.0);

    inverse[0] = (matrix[4] * matrix[8] - matrix[5] * matrix[7]) / det;
    inverse[1] = -(matrix[1] * matrix[8] - matrix[2] * matrix[7]) / det;
    inverse[2] = (matrix[1] * matrix[5] - matrix[2] * matrix[4]) / det;
    inverse[3] = -(matrix[3] * matrix[8] - matrix[5] * matrix[6]) / det;
    inverse[4] = (matrix[0] * matrix[8] - matrix[2] * matrix[6]) / det;
    inverse[5] = -(matrix[0] * matrix[5] - matrix[2] * matrix[3]) / det;
    inverse[6] = (matrix[3] * matrix[7] - matrix[4] * matrix[6]) / det;
    inverse[7] = -(matrix[0] * matrix[7] - matrix[1] * matrix[6]) / det;
    inverse[8] = (matrix[0] * matrix[4] - matrix[1] * matrix[3]) / det;
}

void invert4x4(double *matrix, double *inverse) {
    double L[16] = {0.0};

    // Compute L
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j <= i; j++) {
            double sum = matrix[i * 4 + j];

            for (int k = 0; k < j; k++) {
                sum -= L[i * 4 + k] * L[j * 4 + k];
            }
            if (i == j) {
                if (sum <= 0.0) {
                    return -1;  // Not pd
                }
                L[i * 4 + i] = sqrt(sum);
            } else {
                L[i * 4 + j] = sum / L[j * 4 + j];
            }
        }
    }

    // Invert L
    double Linv[16] = {0.0};
    for (int col = 0; col < 4; col++) {
        for (int i = 0; i < 4; i++) {
            double sum = (i == col) ? 1.0 : 0.0;

            for (int k = 0; k < i; ++k) {
                sum -= L[i * 4 + k] * Linv[k * 4 + col];
            }

            Linv[i * 4 + col] = sum / L[i * 4 + i];
        }
    }

    // Calculate L^-1 * L^-T
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            double sum = 0.0;
            for (int k = 0; k < 4; k++) {
                sum += Linv[k * 4 + i] * Linv[k * 4 + j];
            }
            inverse[i * 4 + j] = sum;
        }
    }

    return 0;
}

void find_optimum(qform_t *qf, int dim, double *optimum, double *inverse_dest) {
    switch (dim) {
        case 1:
            invert1x1(qf->Q, inverse_dest);
            break;
        case 2:
            invert2x2(qf->Q, inverse_dest);
            break;
        case 3:
            invert3x3(qf->Q, inverse_dest);
            break;
        case 4:
            invert4x4(qf->Q, inverse_dest);
            break;
        default:
            break;
    }
    mat_vec_mul(inverse_dest, qf->q, optimum, dim);
    for (int i = 0; i < dim; i++) optimum[i] *= -1.0;
}

double dot_product(double *a, double *b, int n) {
    double acc = 0;
    for (int i = 0; i < n; i++) {
        acc += a[i] * b[i];
    }
    return acc;
}

double quadratic(const double *Q, const int dim, const double *x) {
    double diag_acc = 0.0, cross_acc = 0.0;
    for (int i = 0; i < dim; i++) {
        diag_acc += Q[i * dim + i] * x[i] * x[i];
        for (int j = i + 1; j < dim; j++) {
            cross_acc += Q[i * dim + j] * x[i] * x[j];
        }
    }
    return diag_acc + cross_acc * 2;
}

double evaluate_cost(qform_t *qf, int dim, double *x) {
    return 0.5 * quadratic(qf->Q, dim, x) + dot_product(qf->q, x, dim) + qf->c;
}
