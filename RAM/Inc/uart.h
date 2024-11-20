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
void uartTX(cmr_uartMsg_t *msg, void *data, size_t len);

extern const TickType_t boron_tx_period_ms;

#endif /* UART_H */