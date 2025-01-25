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

void getVoltages(void);
void sendVoltages(uint16_t voltages[6]);
void sendOvervoltageFlags(uint16_t voltages[6]);
void sendBusVoltage(uint16_t voltages[6]);

void getTemps(void);
void sendTemps(uint16_t temps[8]);
void sendOvertempFlags(uint16_t temps[8]);

void sendCurrent(void);

#endif /* DATA_H */