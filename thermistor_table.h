#include <stdint.h>
#include <stddef.h>

/*
 * thermistor_table.h
 *
 *  Created on: Sep 21, 2019
 */

#ifndef THERMISTOR_TABLE_H_
#define THERMISTOR_TABLE_H_

/** @brief Represents a thermistor temperature conversion. */
typedef struct {
    uint32_t resistance_Ohm;    /**< @brief Resistance (Ohms). */
    int32_t temp_dC;             /**< @brief Temperature (10th degrees Celsius). */
} thermistorTempConversion_t;

extern const thermistorTempConversion_t switchThermTempConvs[];
extern const thermistorTempConversion_t radThermTempConvs[];

extern size_t radThermTempConvs_len;
extern size_t switchThermTempConvs_len;

#endif /* THERMISTOR_TABLE_H_ */
