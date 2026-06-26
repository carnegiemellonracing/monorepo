#ifndef CONSTANTS_H
#define CONSTANTS_H

// Plausible Formula-Student-ish vehicle parameters. Stubbed for profiling;
// replace with real values before drawing engineering conclusions.

static const float gear_ratio            = 3.9;
static const float effective_wheel_rad_m = 0.2032;   // ~8 in
static const float car_mass_kg           = 280.0;    // car + driver
static const float half_wheelbase_m      = 0.7875;   // 1.575 m wheelbase
static const float half_trackwidth_m     = 0.6100;   // 1.220 m track

// Steering ratio from steering-wheel angle to road-wheel angle.
static const float steering_ratio        = 5.0;

#endif // CONSTANTS_H
