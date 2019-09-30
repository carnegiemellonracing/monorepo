/**
 * @file thermistor_table.h
 * @brief header for thermistor resistance to temperature tables.
 *
 * @author carnegie mellon racing
 */

#include <stdint.h>
#include <stddef.h>

#ifndef THERMISTOR_TABLE_H_
#define THERMISTOR_TABLE_H_

/** @brief Represents a thermistor temperature conversion. */
typedef struct {
    uint32_t resistance_Ohm;    /**< @brief Resistance (Ohms). */
    int32_t temp_dC;             /**< @brief Temperature (10th degrees Celsius). */
} thermistorTempConversion_t;

extern const thermistorTempConversion_t switchThermTempConvs[];
extern const thermistorTempConversion_t thermTempConvsRadiator[];

const extern size_t radThermTempConvs_len;
const extern size_t switchThermTempConvs_len;

#endif /* THERMISTOR_TABLE_H_ */
