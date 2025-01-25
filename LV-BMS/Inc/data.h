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

//#define VC_TRANS(x)     ((unsigned int) (x * GVCOUT * FS_CNT * 1e6 / 65536))
//#define VI_TRANS(x)     ((unsigned int) (x * GVSENSE * FS_CNT * 1e3 / 256))

void getVoltages(void);
void sendVoltages(uint16_t voltages[6]);
void sendOvervoltageFlags(uint16_t voltages[6]);
void sendBusVoltage(uint16_t voltages[6]);

void AFE_SETUP(void);

void getTemps(void);
void sendTemps(uint16_t temps[8]);
void sendOvertempFlags(uint16_t temps[8]);

void sendCurrent(void);

#endif /* DATA_H */