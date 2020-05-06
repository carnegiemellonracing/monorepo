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
#include "config.h"     // Config interface
#include <cn-cbor/cn-cbor.h>                /* CBOR decoding */

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

/** @brief Boron RX period (milliseconds). (Shared with sample.c) */
const TickType_t boron_rx_period_ms = 1000;

/** @brief Width of N-way UART receive buffering (e.g. 2 for double buffereing) */
#define NUM_RX_BUFFERS 2

/**
 * @brief This is required by cbor, but we don't care about it
 * (outside of debugging)
 */
static cn_cbor_errback err;

static void handle_command(cn_cbor *command);

/**
 * @brief UART TX
 *
 * @param pvParameters ignored.
 */
static void uartTX_Task(void *pvParameters) {
    (void) pvParameters;
    TickType_t last_wake = xTaskGetTickCount();
    while(1) {
        taskENTER_CRITICAL();
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
        taskEXIT_CRITICAL();
        cmr_uartTX(&uart.port, &txMsg, send_buf, msg_len);
        cmr_uartMsgWait(&txMsg);

        vTaskDelayUntil(&last_wake, boron_tx_period_ms);
    }
}


/**
 * @brief UART RX
 *
 * @param pvParameters ignored.
 */
static void uartRX_Task(void *pvParameters) {
    (void) pvParameters;
    TickType_t last_wake = xTaskGetTickCount();
    struct {
        cmr_uartMsg_t msg;
        uint8_t buf[100];
        size_t bytes_present;
    } rx[NUM_RX_BUFFERS] = {
        [0 ... NUM_RX_BUFFERS - 1] = {
            .bytes_present = 0,
        },
    };

    while(1) {
        for (size_t i = 0; i < sizeof(rx) / sizeof(rx[0]); i++) {
            size_t space_left = sizeof(rx[i].buf) - rx[i].bytes_present;
            cmr_uartMsgInit(&rx[i].msg);
            cmr_uartRX(
                &uart.port, &rx[i].msg,
                rx[i].buf + rx[i].bytes_present, space_left,
                CMR_UART_RXOPTS_IDLEABORT
            );
            size_t rxLen = cmr_uartMsgWait(&rx[i].msg);
            rx[i].bytes_present += rxLen;

            /* Try to parse out a cbor structure from the received data.
             * We may have been sent back-to-back, so assume the worst. */
            cn_cbor *command = NULL;
            size_t bytes_parsed;
            for (size_t buflen = 1; buflen <= rx[i].bytes_present; buflen++) {
                command = cn_cbor_decode(rx[i].buf, buflen, &err);
                if (command != NULL) {
                    bytes_parsed = buflen;
                    break;
                }
            }

            if (command == NULL && rx[i].bytes_present == sizeof(rx[i].buf)) {
                /* If we didn't parse a command out, and our buffer is full,
                 * drop the buffer and continue on with our lives */
                rx[i].bytes_present = 0;
                continue;
            } else if (command == NULL) {
                /* We have space left, and we haven't gotten a full command yet,
                 * so continue on until we do have one */
                continue;
            }

            /* We have a valid command after parsing some number of bytes.
             * Consume that number of bytes, and make use of the command. */
            memmove(
                rx[i].buf,
                rx[i].buf + bytes_parsed,
                rx[i].bytes_present - bytes_parsed
            );
            rx[i].bytes_present -= bytes_parsed;

            handle_command(command);
        }

        vTaskDelayUntil(&last_wake, boron_rx_period_ms);
    }
}

/**
 *  @brief Handle some CBOR command from the outside world
 *
 *  @param command The command to handle
 */
static void handle_command(cn_cbor *command) {
    cn_cbor *msg, *params, *id, *data;
    msg = cn_cbor_mapget_string(command, "msg");
    params = cn_cbor_mapget_string(command, "params");

    if (msg != NULL) {
        /* Have a message to transmit */
        id   = cn_cbor_mapget_string(msg, "id");
        data = cn_cbor_mapget_string(msg, "data");
        if (
            (id != NULL &&
                (id->type == CN_CBOR_INT || id->type == CN_CBOR_UINT)) &&
            (data != NULL && data->type == CN_CBOR_BYTES)
        ) {
            canTX((uint16_t) id->v.uint, data->v.bytes, data->length, portMAX_DELAY);
        }
    }

    struct param_pair {
        uint8_t kind;
        uint8_t cutoff_enum;
    };

    if (
        params != NULL && params->type == CN_CBOR_BYTES &&
        params->length >= sizeof(struct param_pair)
    ) {
        /* Have some parameters to update */
        /* Expects an byte-string of 2-byte values, each byte pairs with form
         * kind:cutoff */
        int len = params->length;
        for (size_t i = 0; i < len; i += sizeof(struct param_pair)) {
            struct param_pair *pair = (struct param_pair *) (params->v.bytes + i);

            if (
                pair->kind < MAX_SIGNALS &&
                pair->cutoff_enum < SAMPLE_NUM_FREQS
            ) {
                /* Questionable update parameters, just move on */
                continue;
            }

            struct signal_cfg *cfg = &current_settings.signal_cfg[pair->kind];
            cfg->sample_cutoff_freq = pair->cutoff_enum;
        }
    }
}

/**
 * @brief Initializes a commandline interface.
 *
 */
void uartInit(void) {
    // UART interface initialization.
    const UART_InitTypeDef uartInit = {
        .BaudRate = 19200,
		.WordLength = UART_WORDLENGTH_8B,
		.StopBits = UART_STOPBITS_1,
		.Parity = UART_PARITY_NONE,
		.HwFlowCtl = UART_HWCONTROL_NONE,
		.Mode = UART_MODE_TX_RX,
		.OverSampling = UART_OVERSAMPLING_16
    };

    cmr_uartInit(
        &uart.port, USART1, &uartInit,
        GPIOA, GPIO_PIN_10,     /* rx */
        GPIOA, GPIO_PIN_9,      /* tx */
        DMA2_Stream2, DMA_CHANNEL_4,    /* See reference manual */
        DMA2_Stream7, DMA_CHANNEL_4
    );

    cmr_taskInit(&uart.txTask, "UART TX", 8, uartTX_Task, NULL);
    cmr_taskInit(&uart.rxTask, "UART RX", 8, uartRX_Task, NULL);
}
