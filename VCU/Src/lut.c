//
// Created by 26958 on 1/16/2025.
//

#include <stdint.h>
#include "lut.h"
#include <math.h>
#include <assert.h>

typedef struct
{
    uint32_t alpha_index;
    uint32_t Fz_index;
    uint32_t segment_start;
    uint32_t segment_end;

    uint32_t Fx_index;
    uint16_t kappa_index;
} lookup_t;

static const uint16_t lut_raw[] =
{
    #include "lut_raw.rawh"
};

static const uint32_t lut_indices[ALPHA_DIM][FZ_DIM] =
{
    #include "lut_indices.rawh"
};

typedef struct
{
    uint32_t min_index;
    uint32_t max_index;
} lut_segment_t;

lut_segment_t lut_find_segment(uint32_t alpha_index, uint32_t Fz_index)
{
    assert(alpha_index < ALPHA_DIM && Fz_index < FZ_DIM);

    const uint32_t max_Fx_index_in_cur_segment = lut_indices[alpha_index][Fz_index];
    uint32_t min_Fx_index_in_cur_segment;
    if (Fz_index > 0)
    {
        min_Fx_index_in_cur_segment = lut_indices[alpha_index][Fz_index - 1] + 1;
    }
    else if (alpha_index > 0)
    {
        min_Fx_index_in_cur_segment = lut_indices[alpha_index - 1][FZ_DIM - 1] + 1;
    }
    else
    {
        min_Fx_index_in_cur_segment = 0;
    }

    return (lut_segment_t) {
        .min_index = min_Fx_index_in_cur_segment,
        .max_index = max_Fx_index_in_cur_segment,
    };
}

static float lut_lookup_kappa_raw(uint32_t alpha_index, uint32_t Fz_index, uint32_t Fx_index)
{
    alpha_index = MIN(alpha_index, ALPHA_DIM - 1);
    Fz_index = MIN(Fz_index, FZ_DIM - 1);

    lut_segment_t segment = lut_find_segment(alpha_index, Fz_index);

    uint32_t Fx_index_in_lut_raw = segment.min_index + Fx_index;
    Fx_index_in_lut_raw = MIN(Fx_index_in_lut_raw, segment.max_index);
    return lut_raw[Fx_index_in_lut_raw];
}

static Fx_kappa_t lut_lookup_max_Fx_and_kappa_raw(uint32_t alpha_index, uint32_t Fz_index)
{
    alpha_index = MIN(alpha_index, ALPHA_DIM - 1);
    Fz_index = MIN(Fz_index, FZ_DIM - 1);

    const lut_segment_t segment = lut_find_segment(alpha_index, Fz_index);
    const uint32_t max_Fx_index_in_cur_segment = lut_indices[alpha_index][Fz_index];

    return (Fx_kappa_t) {
        .Fx = segment.max_index - segment.min_index,
        .kappa = lut_raw[max_Fx_index_in_cur_segment],
    };
}

/**
 * floor: The higest lattice point lower than x.
 * offset: [0, 1), the fraction of unit above floor.
 */
typedef struct
{
    uint32_t lattice_floor;
    float offset;
} lattice_projection_t;

static lattice_projection_t lut_project_to_lattice(float x, float x0, float unit)
{
    assert(x >= x0);
    const float num_units = (x - x0) / unit;
    const uint32_t lattice_floor = (uint32_t) floor(num_units);
    const float offset = num_units - (float) lattice_floor;
    return (lattice_projection_t) {
        .lattice_floor = lattice_floor,
        .offset = offset,
    };
}

static float lut_get_kappa_raw(float alpha_degree, float Fz_N, float Fx_N)
{
    alpha_degree = MAX(alpha_degree, ALPHA_MIN_DEG);
    Fz_N = MAX(Fz_N, FZ_MIN_N);
    Fx_N = MAX(Fx_N, FX_MIN_N);

    const lattice_projection_t alpha_proj = lut_project_to_lattice(alpha_degree, ALPHA_MIN_DEG, ALPHA_SPACING_DEG);
    const lattice_projection_t Fz_proj = lut_project_to_lattice(Fz_N, FZ_MIN_N, FZ_SPACING_N);
    const lattice_projection_t Fx_proj = lut_project_to_lattice(Fx_N, FX_MIN_N, FX_SPACING_N);

    const float lookups_alpha_Fz_Fx[8] = {
        lut_lookup_kappa_raw(alpha_proj.lattice_floor, Fz_proj.lattice_floor, Fx_proj.lattice_floor),
        lut_lookup_kappa_raw(alpha_proj.lattice_floor, Fz_proj.lattice_floor, Fx_proj.lattice_floor + 1),
        lut_lookup_kappa_raw(alpha_proj.lattice_floor, Fz_proj.lattice_floor + 1, Fx_proj.lattice_floor),
        lut_lookup_kappa_raw(alpha_proj.lattice_floor, Fz_proj.lattice_floor + 1, Fx_proj.lattice_floor + 1),
        lut_lookup_kappa_raw(alpha_proj.lattice_floor + 1, Fz_proj.lattice_floor, Fx_proj.lattice_floor),
        lut_lookup_kappa_raw(alpha_proj.lattice_floor + 1, Fz_proj.lattice_floor, Fx_proj.lattice_floor + 1),
        lut_lookup_kappa_raw(alpha_proj.lattice_floor + 1, Fz_proj.lattice_floor + 1, Fx_proj.lattice_floor),
        lut_lookup_kappa_raw(alpha_proj.lattice_floor + 1, Fz_proj.lattice_floor + 1, Fx_proj.lattice_floor + 1),
    };

    const float lookups_alpha_Fz[4] = {
        lookups_alpha_Fz_Fx[0] * (1.0f - Fx_proj.offset) + lookups_alpha_Fz_Fx[1] * Fx_proj.offset,
        lookups_alpha_Fz_Fx[2] * (1.0f - Fx_proj.offset) + lookups_alpha_Fz_Fx[3] * Fx_proj.offset,
        lookups_alpha_Fz_Fx[6] * (1.0f - Fx_proj.offset) + lookups_alpha_Fz_Fx[7] * Fx_proj.offset,
        lookups_alpha_Fz_Fx[4] * (1.0f - Fx_proj.offset) + lookups_alpha_Fz_Fx[5] * Fx_proj.offset,
    };

    const float lookups_alpha[2] = {
        lookups_alpha_Fz[0] * (1.0f - Fz_proj.offset) + lookups_alpha_Fz[1] * Fz_proj.offset,
        lookups_alpha_Fz[2] * (1.0f - Fz_proj.offset) + lookups_alpha_Fz[3] * Fz_proj.offset,
    };

    const float lookup = lookups_alpha[0] * (1.0f - alpha_proj.offset) + lookups_alpha[1] * alpha_proj.offset;
    return lookup;
}

static Fx_kappa_t lut_get_max_Fx_kappa_raw(float alpha_degree, float Fz_N)
{
    alpha_degree = MAX(alpha_degree, ALPHA_MIN_DEG);
    Fz_N = MAX(Fz_N, FZ_MIN_N);

    const lattice_projection_t alpha_proj = lut_project_to_lattice(alpha_degree, ALPHA_MIN_DEG, ALPHA_SPACING_DEG);
    const lattice_projection_t Fz_proj = lut_project_to_lattice(Fz_N, FZ_MIN_N, FZ_SPACING_N);

    const Fx_kappa_t lookups_alpha_Fz[4] = {
        lut_lookup_max_Fx_and_kappa_raw(alpha_proj.lattice_floor, Fz_proj.lattice_floor),
        lut_lookup_max_Fx_and_kappa_raw(alpha_proj.lattice_floor, Fz_proj.lattice_floor + 1),
        lut_lookup_max_Fx_and_kappa_raw(alpha_proj.lattice_floor + 1, Fz_proj.lattice_floor),
        lut_lookup_max_Fx_and_kappa_raw(alpha_proj.lattice_floor + 1, Fz_proj.lattice_floor + 1),
    };

    const float max_Fx_lookups_alpha[2] = {
        lookups_alpha_Fz[0].Fx * (1.0f - Fz_proj.offset) + lookups_alpha_Fz[1].Fx * Fz_proj.offset,
        lookups_alpha_Fz[2].Fx * (1.0f - Fz_proj.offset) + lookups_alpha_Fz[3].Fx * Fz_proj.offset,
    };
    const float max_Fx_lookup = max_Fx_lookups_alpha[0] * (1.0f - alpha_proj.offset) + max_Fx_lookups_alpha[1] * alpha_proj.offset;

    const float max_kappa_lookups_alpha[2] = {
        lookups_alpha_Fz[0].kappa * (1.0f - Fz_proj.offset) + lookups_alpha_Fz[1].kappa * Fz_proj.offset,
        lookups_alpha_Fz[2].kappa * (1.0f - Fz_proj.offset) + lookups_alpha_Fz[3].kappa * Fz_proj.offset,
    };
    const float max_kappa_lookup = max_kappa_lookups_alpha[0] * (1.0f - alpha_proj.offset) + max_kappa_lookups_alpha[1] * alpha_proj.offset;

    return (Fx_kappa_t) {
        .Fx = max_Fx_lookup,
        .kappa = max_kappa_lookup,
    };
}

static float lut_get_Fz_scaling_factor() {
    return 0.5f;
}

static float lut_get_kappa_scaling_factor() {
    return 0.85;
}

float lut_get_kappa(const float alpha_degree, const float Fz_N, const float Fx_N)
{
    float adjusted_Fz_N = Fz_N * lut_get_Fz_scaling_factor();
    float kappa = lut_get_kappa_raw(alpha_degree, adjusted_Fz_N, Fx_N) * KAPPA_SPACING;
    float adjusted_kappa = kappa * lut_get_kappa_scaling_factor();
    return adjusted_kappa;
}

Fx_kappa_t lut_get_max_Fx_kappa(const float alpha_degree, const float Fz_N)
{
    float adjusted_Fz_N = Fz_N * lut_get_Fz_scaling_factor();
    // Ensures nonnegativity.
    adjusted_Fz_N = MAX(adjusted_Fz_N, 0.0f);

    // Since downforce is shrunken by this operation,
    // the minimum downforce supported by LUT needs to be at most 
    // our minimum downforce * scaling factor.
    // That is, if our minimum downforce is 400N and scaling factor is 0.4,
    // then the lUT needs to start at 160N downforce or below.

    // If queried downforce is below the minimum downforce supported,
    // try our best to interpolate.

    if(adjusted_Fz_N < FZ_MIN_N) {
        Fx_kappa_t result = lut_get_max_Fx_kappa_raw(alpha_degree, FZ_MIN_N);

        float Fx = result.Fx * FX_SPACING_N;
        float kappa = result.kappa * KAPPA_SPACING;
        float adjusted_kappa = kappa * lut_get_kappa_scaling_factor();

        float shrink_factor = adjusted_Fz_N / FZ_MIN_N;
        return (Fx_kappa_t) {
            .Fx = Fx * shrink_factor,
            .kappa = adjusted_kappa * shrink_factor,
        };
    } 

    const Fx_kappa_t result = lut_get_max_Fx_kappa_raw(alpha_degree, adjusted_Fz_N);
    float Fx = result.Fx * FX_SPACING_N;
    float kappa = result.kappa * KAPPA_SPACING;
    float adjusted_kappa = kappa * lut_get_kappa_scaling_factor();
    
    return (Fx_kappa_t) {
        .Fx = Fx,
        .kappa = adjusted_kappa,
    };
}
