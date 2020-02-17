/**
 * @file uart.h
 * @brief Board-specific UART interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef UART_H
#define UART_H

#include <CMR/uart.h>
#include <stdbool.h>

void uartInit(void);
void uartTX(cmr_uartMsg_t *msg, void *data, size_t len);

#endif /* UART_H */

