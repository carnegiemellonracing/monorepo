#include "motor_efficiency.h"

#include <math.h>
#include <stdint.h>
#include <stddef.h>

/** @brief Dimensions of the efficiency LUT */
#define EFFICIENCY_LUT_NUM_ROWS (11)
#define EFFICIENCY_LUT_NUM_COLS (10)

/**
 * @brief The motor efficiency LUT
 * @note Rows are indexed by motor torque (Nm), columns are indexed by motor speed (rad/s)
 * @note Contents are taken from the motor efficiency data sheet
 */
static const uint16_t efficiency_lut[EFFICIENCY_LUT_NUM_ROWS][EFFICIENCY_LUT_NUM_COLS] = {
    #include "motor_efficiency_lut.rawh"
};

/** @brief The offset and scale of the efficiency LUT */
static const float efficiency_lut_offset = 0.1117f;
static const float efficiency_lut_scale = 0.7969f;

/** @brief The row (torque) anchors of the efficiency LUT in Nm */
static const float efficiency_lut_row_torque_anchors[EFFICIENCY_LUT_NUM_ROWS] = {
    #include "motor_efficiency_lut_row_torque_anchors.rawh"
};

/** @brief The column (speed) anchors of the efficiency LUT in rad/s */
static const float efficiency_lut_col_speed_anchors[EFFICIENCY_LUT_NUM_COLS] = {
    #include "motor_efficiency_lut_col_speed_anchors.rawh"
};

/**
 * @brief Look up the motor efficiency LUT by index
 * @note Worst case run time: O(1)
 *
 * @param row_torque_idx The row (torque) index,
 *                       MUST BE STRICTLY LESS THAN EFFICIENCY_LUT_NUM_ROWS
 * @param col_speed_idx The column (speed) index,
 *                      MUST BE STRICTLY LESS THAN EFFICIENCY_LUT_NUM_COLS
 *
 * @return The looked-up efficiency, between 0 and 1 (inclusive)
 */
static float lookUpEfficiencyByIndex(
    size_t row_torque_idx,
    size_t col_speed_idx
) {
    static const float one_over_int16_max = 1.0f / (float)(UINT16_MAX);
    const float raw_efficiency = (float)(efficiency_lut[row_torque_idx][col_speed_idx]);
    return raw_efficiency * one_over_int16_max * efficiency_lut_scale + efficiency_lut_offset;
}

/**
 * @brief Compute the average value of an array
 * @note Worst case run time: O(len)
 *
 * @param array The array to compute the average value, MUST HAVE LENGTH AT LEAST len
 * @param len The length of array
 *
 * @return The average value of array
 */
static float getAverage(
    const float *array,
    size_t len
) {
    float sum = 0.0f;
    for (size_t i = 0; i < len; i++) {
        sum += array[i];
    }
    return sum / ((float)len);
}

/**
 * @brief Find the index to the tightest upper bound of a value in a sorted array
 * @note Worst case run time: O(log(len))
 *
 * @param array The array to look up the upper bound of value in,
 *              MUST BE SORTED IN NON-DECREASING ORDER AND HAVE LENGTH AT LEAST len
 * @param len The length of the array
 * @param value The value to find the index to the upper bound of
 *
 * @return If array contains an upper bound of value, its index is returned;
 *         otherwise, len is returned
 */
static size_t getUpperBoundIndex(
    const float *array,
    size_t len,
    float value
) {
    size_t index_low = 0; // the first index that could contain the tightest upper bound
    size_t index_high = len; // the first index that couldn't contain the tightest upper bound
    while (index_low + 1 < index_high) {
        const size_t center_idx = (index_low + index_high) / 2;
        if (value <= array[center_idx - 1]) { // center_idx - 1 indexes an upper bound
            // look for the tightest upper bound in the lower-half
            index_high = center_idx;
        } else { // center_idx - 1 doesn't index an upper bound
            // look for the tightest upper bound in the upper-half
            index_low = center_idx;
        }
    }
    if (value <= array[index_low]) { // index_low indexes an upper bound
        return index_low;
    } else { // array doesn't contain an upper bound
        return len;
    }
}

/**
 * @brief Given an interval, compute a number between 0 and 1 (inclusive)
 *        which indicates how close x is to the lower bound
 * @note Worst case run time: O(1)
 *
 * @param x The independent variable
 * @param lower_bound The lower bound of the interval, MUST BE STRICTLY LESS THAN upper_bound
 * @param upper_bound The upper bound of the interval, MUST BE STRICTLY GREATER THAN lower_bound
 *
 * @return theta, where:
 *         theta * lower_bound + (1 - theta) * upper_bound == x
 *         but theta is clamped between 0 and 1;
 *         NAN when parameters are invalid
 */
static float getInterpolationFactor(
    float x,
    float lower_bound,
    float upper_bound
) {
    // check parameters
    if (lower_bound >= upper_bound) { // parameters are invalid
        return NAN;
    }

    float theta = (x - lower_bound) / (upper_bound - lower_bound);
    theta = fmaxf(theta, 0.0f); // ensure theta >= 0
    theta = fminf(theta, 1.0f); // ensure theta <= 1
    return theta;
}

/**
 * @brief Estimate max absolute value of torque based on motor efficiency
 * @note Worst case run time: O(num_iterations)
 *
 * @param max_power The upper limit of power draw from the accumulator, MUST BE NON-NEGATIVE
 * @param base_efficiency Efficiency of the circuit between the accumulator and the motors,
 *                        MUST BE NON-NEGATIVE
 * @param total_motor_speed Total motor speed in rad/s, negative values are clamped to 0
 * @param num_iterations Number of Newton steps to run when estimating motor efficiency;
 *                       if num_iterations is 0, a motor efficiency of 1 would be used
 *
 * @return The estimated upper limit of motor torque, NAN if parameters are invalid
 */
float getEstimatedMaxTorque(
    float max_power,
    float base_efficiency,
    float total_motor_speed,
    size_t num_iterations
) {
    // check parameters
    if (max_power < 0.0f || base_efficiency < 0.0f) { // parameters are invalid
        return NAN;
    }
    total_motor_speed = fmaxf(total_motor_speed, 0.0f); // ensure total_motor_speed >= 0.0f

    // pre-compute the ratio between max torque and motor efficiency
    const float max_torque_over_motor_efficiency = max_power * base_efficiency / total_motor_speed;

    // use 1 as motor efficiency and skip Newton's method if num_iterations is 0
    if (num_iterations == 0) {
        return max_torque_over_motor_efficiency; // implicitly multiplied by a motor efficiency of 1
    }

    // look up a column by motor speed
    float torque_to_efficiency_lut[EFFICIENCY_LUT_NUM_ROWS];
    const float average_motor_speed = total_motor_speed * 0.25f;
    const size_t upper_bound_col_speed_idx = getUpperBoundIndex(
        efficiency_lut_col_speed_anchors,
        EFFICIENCY_LUT_NUM_COLS,
        average_motor_speed
    );
    if (upper_bound_col_speed_idx == 0) {
        // average_motor_speed is less than the left-most speed anchor, use the left-most column
        for (size_t i = 0; i < EFFICIENCY_LUT_NUM_ROWS; i++) {
            torque_to_efficiency_lut[i] = lookUpEfficiencyByIndex(i, 0);
        }
    } else if (EFFICIENCY_LUT_NUM_COLS <= upper_bound_col_speed_idx) {
        // average_motor_speed is greater than the right-most speed anchor, use the right-most column
        for (size_t i = 0; i < EFFICIENCY_LUT_NUM_ROWS; i++) {
            torque_to_efficiency_lut[i] = lookUpEfficiencyByIndex(i, EFFICIENCY_LUT_NUM_COLS - 1);
        }
    } else {
        // average_motor_speed is between the highest and lowest speed anchors,
        // interpolate between the two closest columns in efficiency_lut
        const float theta = getInterpolationFactor(
            average_motor_speed,
            efficiency_lut_col_speed_anchors[upper_bound_col_speed_idx - 1],
            efficiency_lut_col_speed_anchors[upper_bound_col_speed_idx]
        );
        for (size_t i = 0; i < EFFICIENCY_LUT_NUM_ROWS; i++) {
            torque_to_efficiency_lut[i] =
                  theta          * lookUpEfficiencyByIndex(i, upper_bound_col_speed_idx - 1)
                + (1.0f - theta) * lookUpEfficiencyByIndex(i, upper_bound_col_speed_idx);
        }
    }

    // obtain an initial guess of motor efficiency by averaging torque_to_efficiency_lut
    float estimated_motor_efficiency = getAverage(torque_to_efficiency_lut, EFFICIENCY_LUT_NUM_ROWS);

    // estimate max torque using Newton's method
    float estimated_max_torque = max_torque_over_motor_efficiency * estimated_motor_efficiency;
    for (size_t i = 0; i < num_iterations; i++) {
        // look up estimated_motor_efficiency using estimated_max_torque
        const size_t upper_bound_idx = getUpperBoundIndex(
            efficiency_lut_row_torque_anchors,
            EFFICIENCY_LUT_NUM_ROWS,
            estimated_max_torque
        );
        if (upper_bound_idx == 0) {
            // estimated_max_torque is less than the top torque anchor, use the top entry
            estimated_motor_efficiency = torque_to_efficiency_lut[0];
        } else if (EFFICIENCY_LUT_NUM_ROWS <= upper_bound_idx) {
            // estimated_max_torque is greater than the bottom torque anchor, use the bottom entry
            estimated_motor_efficiency = torque_to_efficiency_lut[EFFICIENCY_LUT_NUM_ROWS - 1];
        } else {
            // estimated_max_torque is between the highest and lowest torque anchors,
            // interpolate between the two closest entries in torque_to_efficiency_lut
            const float theta = getInterpolationFactor(
                estimated_max_torque,
                efficiency_lut_row_torque_anchors[upper_bound_idx - 1],
                efficiency_lut_row_torque_anchors[upper_bound_idx]
            );
            estimated_motor_efficiency =
                  theta          * torque_to_efficiency_lut[upper_bound_idx - 1]
                + (1.0f - theta) * torque_to_efficiency_lut[upper_bound_idx];
        }

        // recompute estimated_max_torque using estimated_motor_efficiency
        estimated_max_torque = max_torque_over_motor_efficiency * estimated_motor_efficiency;
    }

    return estimated_max_torque;
}
