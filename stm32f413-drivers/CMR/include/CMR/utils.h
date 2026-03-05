/**
 * @file utils.g
 * @brief Some basic utility functions/macros
 *
 *
 * @author Ayush Garg
 */

// Linear interpolation between a and b using parameter t
// t = 0 returns a
// t = 1 returns b
// values in between blend linearly
#define LERP(a, b, t) ((a) + (t) * ((b) - (a)))


// Maps value t from range [a, b] to range [0, scale] using integer math.
// Avoids floating point and preserves precision by multiplying before dividing.
#define INVLERP_SCALED(a, b, t, scale) (((scale) * ((t) - (a))) / ((b) - (a)))

// Returns the smaller of a or b
#define MIN(a,b) ((a) < (b) ? (a) : (b))

// Returns the larger of a or b
#define MAX(a,b) ((a) > (b) ? (a) : (b))

// Restricts x to the range [min, max]
// If x < min → returns min
// If x > max → returns max
// Otherwise returns x
#define CLAMP(min, x, max) (MAX((min), MIN((x), (max))))