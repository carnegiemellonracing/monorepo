/**
 * @file uart.h
 * @brief Board-specific UART interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef UART_H
#define UART_H

#include <CMR/tasks.h>      // CMR task interface
#include <CMR/uart.h>
#include <stdbool.h>

#include "charger.h"

#define NUM_OF_SEGMENTS 3

// TODO: check which are floats/ints
typedef struct {
    int batt_voltage;
    int hv_voltage;
    int evse_max_current;
    struct {
        uint8_t state;
        uint8_t mode;
        uint16_t error_vector; // note yet determined
        bool alive;
    } hvc;
    struct {
        uint8_t id;
        uint8_t state;
        uint16_t voltage;
        uint16_t current;
    } chargers[CHARGER_LEN];
    int max_cell_temp;
    int min_cell_volt;
    int max_cell_volt;
    bool ptc_alive;
} cmr_ccmData_t;

void uartInit(void);
void uartTX(cmr_uartMsg_t *msg, void *data, size_t len);

#endif /* UART_H */
