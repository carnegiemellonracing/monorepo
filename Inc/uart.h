/**
 * @file uart.h
 * @brief Board-specific UART interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef UART_H
#define UART_H

#include <CMR/tasks.h> // CMR task interface
#include <CMR/uart.h>
#include <stdbool.h>

void uartInit(void);

extern const TickType_t boron_tx_period_ms;

#endif /* UART_H */
