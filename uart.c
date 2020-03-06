/**
 * @file uart.c
 * @brief Board-specific UART implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <stdio.h>      // snprintf()
#include <string.h>     // memcpy()
#include <stdbool.h>    // bool

#include <CMR/uart.h>       // CMR UART interface
#include <CMR/tasks.h>      // CMR task interface
#include <CMR/config.h>     // CMR configuration itnerface
#include <CMR/panic.h>      // bad things

#include "uart.h"       // Interface to implement
#include "sample.h"     // Sample formatting
#include "can.h"        // Can interface

/** @brief Represents a UART interface. */
typedef struct uart uart_t;

struct uart {
    cmr_uart_t port;    /**< @brief The underlying UART port. */

    cmr_task_t rxTask;  /**< @brief Receive task. */
    cmr_task_t txTask;  /**< @brief Receive task. */
};

/** @brief Primary UART interface. */
static uart_t uart;

/** @brief Buffer to send via UART DMA */
static uint8_t send_buf[MAX_MESSAGE_LEN];

/** @brief Boron TX period (milliseconds). (Shared with sample.c) */
const TickType_t boron_tx_period_ms = 1000;

/**
 * @brief UART TX
 *
 * @param pvParameters ignored.
 */
static void uartTX_Task(void *pvParameters) {
    (void) pvParameters;
    TickType_t last_wake = xTaskGetTickCount();
    while(1) {
        vPortEnterCritical();
            /* Formatting must be atomic w.r.t. CAN stream
             * TODO modify to drop messages during this instead */
           ssize_t msg_len = sampleFmtMsg();

           if (msg_len <= 0) {
               cmr_panic("The CBOR parser exploded");
           }

            cmr_uartMsg_t txMsg;
            cmr_uartMsgInit(&txMsg);
            memcpy(send_buf, raw_msg, msg_len);
            sampleClearMsg();
        vPortExitCritical();
        cmr_uartTX(&uart.port, &txMsg, send_buf, msg_len);
        cmr_uartMsgWait(&txMsg);

        vTaskDelayUntil(&last_wake, boron_tx_period_ms);
    }
}

/**
 * @brief Initializes a commandline interface.
 *
 */
void uartInit(void) {
    // UART interface initialization.
    const UART_InitTypeDef uartInit = {
        .BaudRate = 9600,
		.WordLength = UART_WORDLENGTH_8B,
		.StopBits = UART_STOPBITS_1,
		.Parity = UART_PARITY_NONE,
		.HwFlowCtl = UART_HWCONTROL_NONE,
		.Mode = UART_MODE_TX,
		.OverSampling = UART_OVERSAMPLING_16
    };

    cmr_uartInit(
        &uart.port, USART1, &uartInit,
        GPIOA, GPIO_PIN_10,     /* rx */
        GPIOA, GPIO_PIN_9,      /* tx */
        DMA2_Stream2, DMA_CHANNEL_4,    /* See reference manual */
        DMA2_Stream7, DMA_CHANNEL_4
    );

    cmr_taskInit(&uart.txTask, "UART RX", 8, uartTX_Task, NULL);
}
