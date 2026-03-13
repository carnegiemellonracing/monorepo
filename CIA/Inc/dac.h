#pragma once
/**
 * @file dac.h
 * @brief DAC interface
 *
 * @author Ayush Garg
 */

void dacInit();

/**
 * @brief Represents a DAC pin.
 * @note DAC_B_REF and DAC_C_REF should be disabled for comp
 */
typedef enum {
    DAC_C_REF = 0,      /**< @brief Hall Effect reference */
    DAC_LEN         /**< @brief Total DAC pins. */
} DAC_t;