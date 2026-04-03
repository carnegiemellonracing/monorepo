/**
 * @file analysis.h
 * @brief Helpers for analyzing data
 *
 * @author Ayush Garg
 */

#pragma once 

typedef struct {
    uint16_t max_cell_voltage_mV;
    uint16_t min_cell_voltage_mV;
    uint8_t  max_cell_voltage_idx;
    uint8_t  min_cell_voltage_idx;
    uint16_t max_cell_temp_centi_deg;
    uint16_t min_cell_temp_centi_deg;
    uint8_t  max_cell_temp_idx;
    uint8_t  min_cell_temp_idx;
    uint16_t pack_voltage_mV;
} BMS_stats_t;

BMS_stats_t BMS_stats;

void analysisInit(void);


