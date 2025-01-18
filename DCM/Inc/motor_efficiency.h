#ifndef MOTOR_EFFICIENCY_H
#define MOTOR_EFFICIENCY_H

#include <stddef.h>

float getEstimatedMaxTorque(
    float max_power,
    float base_efficiency,
    float total_motor_speed,
    size_t num_iterations
);

#endif /* MOTOR_EFFICIENCY_H */
