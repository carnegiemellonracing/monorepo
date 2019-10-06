/**
 * @file thermistor_table.h
 * @brief header for thermistor resistance to temperature tables.
 *
 * @author Carnegie Mellon Racing
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

extern const thermistorTempConversion_t thermTempConvsSwitch[];
extern const thermistorTempConversion_t thermTempConvsRadiator[];

const extern size_t thermTempConvsRadiator_len;
const extern size_t thermTempConvsSwitch_len;

#endif /* THERMISTOR_TABLE_H_ */
