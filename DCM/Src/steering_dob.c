/**
 * @file steering_dob.c
 * @brief Disturbance Observer (DOB) for the CubeMars steering controller.
 *
 * For a double-integrator plant G(s) = K/s², the Q-filter DOB reduces to:
 *
 *   d_hat(s) = A(s)*u_pd(s) - B(s)*pos(s)
 *
 * where:
 *   A(s) = ω_q² / (s*(s + 2ω_q))     — integrating lowpass on control input
 *   B(s) = ω_q²*s / (K*(s + 2ω_q))   — bandpass on position
 *
 * Both filters are discretized via Tustin (bilinear) at the 10 ms task period.
 * Coefficients are computed once in dobInit() from ω_q and K_plant.
 *
 * Discrete difference equations (derived by substituting s = 2/T*(z-1)/(z+1)):
 *
 *   A filter (2nd order):
 *     w[n] = a_p1*w[n-1] - a_p2*w[n-2] + a_n*(u[n] + 2*u[n-1] + u[n-2])
 *     where a_n  = ω_q²*T² / (4*(1+c))
 *           a_p1 = 2 / (1+c)
 *           a_p2 = (1-c) / (1+c)
 *           c    = ω_q*T
 *
 *   B filter (1st order):
 *     v[n] = b_p*v[n-1] + b_n*(pos[n] - pos[n-1])
 *     where b_n = ω_q² / (K*(1+c))
 *           b_p = (1-c) / (1+c)   (same pole as a_p2)
 *
 *   d_hat[n] = w[n] - v[n]
 */

#include "steering_dob.h"

#include <math.h>

#define DOB_PERIOD_S     0.010f     // must match steeringPeriod_ms in steering.c
#define DOB_OUTPUT_LIMIT 15000.0f   // mA — clamp d_hat to prevent runaway

static struct {
    bool  active;

    // A filter coefficients
    float a_n;      // numerator gain (applied to u, 2u_prev1, u_prev2)
    float a_p1;     // pole coefficient on w[n-1]
    float a_p2;     // pole coefficient on w[n-2]

    // B filter coefficients
    float b_n;      // numerator gain (applied to pos[n] - pos[n-1])
    float b_p;      // pole coefficient on v[n-1]

    // A filter state
    float w1, w2;   // w[n-1], w[n-2]
    float u1, u2;   // u_pd[n-1], u_pd[n-2]

    // B filter state
    float v1;       // v[n-1]
    float y1;       // pos[n-1]
} dob = { .active = false };

void dobInit(float omega_q, float K_plant) {
    float c = omega_q * DOB_PERIOD_S;   // dimensionless: ω_q * T

    dob.a_n  = (omega_q * omega_q * DOB_PERIOD_S * DOB_PERIOD_S) / (4.0f * (1.0f + c));
    dob.a_p1 = 2.0f / (1.0f + c);
    dob.a_p2 = (1.0f - c) / (1.0f + c);

    dob.b_n  = (omega_q * omega_q) / (K_plant * (1.0f + c));
    dob.b_p  = (1.0f - c) / (1.0f + c);

    dob.active = true;
    dobReset();
}

float dobTick(float u_pd, float pos) {
    if (!dob.active) return 0.0f;

    // A filter: w[n] = a_p1*w[n-1] - a_p2*w[n-2] + a_n*(u[n] + 2*u[n-1] + u[n-2])
    float w = dob.a_p1 * dob.w1
            - dob.a_p2 * dob.w2
            + dob.a_n  * (u_pd + 2.0f * dob.u1 + dob.u2);

    // B filter: v[n] = b_p*v[n-1] + b_n*(pos[n] - pos[n-1])
    float v = dob.b_p * dob.v1
            + dob.b_n * (pos - dob.y1);

    // Update state
    dob.w2 = dob.w1;  dob.w1 = w;
    dob.u2 = dob.u1;  dob.u1 = u_pd;
    dob.v1 = v;
    dob.y1 = pos;

    float d_hat = w - v;

    // Clamp to prevent runaway if K_plant is poorly identified
    if (d_hat >  DOB_OUTPUT_LIMIT) return  DOB_OUTPUT_LIMIT;
    if (d_hat < -DOB_OUTPUT_LIMIT) return -DOB_OUTPUT_LIMIT;
    return d_hat;
}

void dobReset(void) {
    dob.w1 = dob.w2 = 0.0f;
    dob.u1 = dob.u2 = 0.0f;
    dob.v1 = 0.0f;
    dob.y1 = 0.0f;
}

bool dobIsActive(void) {
    return dob.active;
}
