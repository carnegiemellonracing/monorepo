//
// Created by 26958 on 1/16/2025.
//

#ifndef LUT3D_H
#define LUT3D_H

#define MAX(a,b) \
    ({ __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b; })

#define MIN(a,b) \
    ({ __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b; })

#define G 9.81f
#define CAR_MASS 310.0f
#define CAR_WEIGHT_N (G * CAR_MASS)

#define ALPHA_DIM 21U
#define ALPHA_MIN_DEG 0.0f
#define ALPHA_SPACING_DEG 1.0f

#define FZ_DIM 41U
#define FZ_MIN_N (CAR_WEIGHT_N / 4.0f * 0.20f)
#define FZ_SPACING_N 30.0f

#define FX_MIN_N 0.0f
#define FX_SPACING_N 19.0f

#define KAPPA_SPACING 0.00001461697747f

typedef struct {
    float Fx;
    float kappa;
} Fx_kappa_t;

float lut_get_kappa(const float alpha_degree, const float Fz_N, const float Fx_N);
Fx_kappa_t lut_get_max_Fx_kappa(const float alpha_degree, const float Fz_N);

#endif //LUT3D_H
