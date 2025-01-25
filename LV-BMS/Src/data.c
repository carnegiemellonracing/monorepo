/**
 * @file data.h
 * @brief Implements LV-BMS CAN transmission functions
 *
 * @author Carnegie Mellon Racing
 */

#include <stdint.h>
#include <can.c>

void getVoltages(void) {
    // Implement
}

// Sends cell voltages (1-6) split into two CAN messages
void sendVoltages(uint16_t voltages[6]) {
    uint16_t data1[3], data2[3];

    // Split voltages into two groups
    memcpy(data1, voltages, 3 * sizeof(uint16_t));     // Voltages 1-3
    memcpy(data2, &voltages[3], 3 * sizeof(uint16_t)); // Voltages 4-6

    canTX(CAN_ID_LV_BMS_CELL_VOLTAGE_1_3, data1, sizeof(data1), canTX10Hz_period_ms);
    canTX(CAN_ID_LV_BMS_CELL_VOLTAGE_4_6, data2, sizeof(data2), canTX10Hz_period_ms);
}

// Sends overvoltage flags
void sendOvervoltageFlags(uint16_t voltages[6]) {
    uint8_t flag = 0;
    uint8_t overVolt = 0; // TBD

    for (int i = 0; i < 6; i++) {
        if (voltages[i] > overVolt) flag |= (1 << i);
    }

    canTX(CAN_ID_LV_BMS_CELL_OVERVOLTAGE, &flag, sizeof(flag), canTX10Hz_period_ms);
}

// Sends bus voltage derived from cell voltages
void sendBusVoltage(uint16_t voltages[6]) {
    uint16_t totalVoltage = 0;

    for (int i = 0; i < 6; i++) {
        totalVoltage += voltages[i];
    }

    canTX(CAN_ID_LV_BMS_BUS_VOLTAGE, &totalVoltage, sizeof(totalVoltage), canTX10Hz_period_ms);
}

void getTemps(void) {
    // Implement
}

// Sends cell temperatures (1-8) split into two CAN messages
void sendTemps(uint16_t temps[8]) {
    uint16_t data1[4], data2[4];

    // Split temperatures into two groups
    memcpy(data1, temps, 4 * sizeof(uint16_t));     // Temps 1-4
    memcpy(data2, &temps[4], 4 * sizeof(uint16_t)); // Temps 5-8

    canTX(CAN_ID_LV_BMS_CELL_TEMP_1_4, data1, sizeof(data1), canTX10Hz_period_ms);
    canTX(CAN_ID_LV_BMS_CELL_TEMP_5_8, data2, sizeof(data2), canTX10Hz_period_ms);
}

// Sends overtemperature flags (derived from temperature data)
void sendOvertempFlags(uint16_t temps[8]) {
    uint8_t flag = 0;
    
    for (int i = 0; i < 8; i++) {
        if (temps[i] > 6000) flag |= (1 << i);
    }

    // Send CAN messages
    canTX(CAN_ID_LV_BMS_CELL_OVERTEMP, &flag, sizeof(flag), canTX10Hz_period_ms);
}

// Sends the bus current
void sendCurrent(void) {
    // Implement

    canTX(CAN_ID_LV_BMS_CURRENT, &current, sizeof(current), canTX10Hz_period_ms);
}