/**
 * @file data.h
 * @brief Declares LV-BMS CAN transmission functions
 *
 * @author Carnegie Mellon Racing
 */

#ifndef DATA_H
#define DATA_H

#include <stdint.h>

// Converts float to uint16 (scale by 100)
#define float_to_uint16(value) ((uint16_t)((value) * 100.0f))
#define VREF_NOM 1.5 
#define ADC_COUNT 4096L //assuming 12 bit adc
#define GVCOUT 0.3 //(1.5V-->0.3)(3V-->0.6)
#define GVSENSE 8 //assuming gain of 8
#define VTHERM_NUM 8
#define VREF_THERM 3.3 //lowkey have no clue
#define RESISTOR 10000 //10k ohm resistor from temps. again idk!!!

void getVoltages(void);
void sendVoltages();
void sendOvervoltageFlags(uint16_t voltages[6]);
void sendBusVoltage(uint16_t voltages[6]);

void AFE_SETUP(void);

void getTemps(void);
uint16_t tempConvert(uint16_t adc_value);
void sendTemps(uint16_t temps[8]);
void sendOvertempFlags(uint16_t temps[8]);

void sendCurrent(void);

#endif /* DATA_H */