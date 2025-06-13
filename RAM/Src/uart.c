/**
 * @file uart.c
 * @brief Board-specific UART implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "uart.h"  // Interface to implement

#include <CMR/config.h>  // CMR configuration itnerface
#include <CMR/config_screen_helper.h>
#include <cn-cbor/cn-cbor.h> /* CBOR decoding */
#include <stdio.h>           // snprintf()
#include <string.h>          // memcpy()

#include "can.h"     // Can interface
#include "config.h"  // Config interface
#include "sample.h"  // Sample formatting

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

// Constants to replace magic numbers
#define RX_BUFFER_SIZE 200
#define DIM_TEXT_BYTES_PER_MESSAGE 4
#define CAN_TX_TIMEOUT_MS 200
#define SKETCHY_DIM_ACK_SIGNAL_INDEX 10

/** @brief Represents a UART interface. */
typedef struct uart uart_t;

extern sample_data_t raw_sample_data[MAX_SIGNALS];

#define SIGNAL_MAP_OFFSET 24
extern struct signal signal_map[MAX_SIGNALS];

struct uart {
    cmr_uart_t port; /**< @brief The underlying UART port. */

    cmr_task_t rxTask; /**< @brief Receive task. */
    cmr_task_t txTask; /**< @brief Receive task. */
};

/** @brief Primary UART interface. */
static uart_t uart;

/** @brief Buffer to send via UART DMA */
static uint8_t send_buf[MAX_MESSAGE_LEN];

/** @brief Boron TX period (milliseconds). (Shared with sample.c) */
const TickType_t boron_tx_period_ms = 1000;

/** @brief Boron RX period (milliseconds). (Shared with sample.c) */
const TickType_t boron_rx_period_ms = 1000;

/** @brief Delay between each can Message with text for dim */
const TickType_t dim_message_delay_ms = 5;

/** @brief Width of N-way UART receive buffering (e.g. 2 for double buffereing)
 */
#define NUM_RX_BUFFERS 2

/**
 * @brief This is required by cbor, but we don't care about it
 * (outside of debugging)
 */
static cn_cbor_errback err;

/**
 * @brief Parameter pair structure for configuration updates
 */
struct param_pair {
    uint8_t kind;
    uint8_t cutoff_enum;
};

// Function declarations for extracted command handlers
static void handle_message_command(cn_cbor *msg);
static void handle_params_command(cn_cbor *params,
                                  struct param_pair *response_data,
                                  int *response_num_pairs);
static void handle_pull_command(cn_cbor *pull, struct param_pair *response_data,
                                int *response_num_pairs);
static void handle_signal_enable_command(cn_cbor *signal_en);
static void send_response_packet(cn_cbor *response_packet,
                                 struct param_pair *response_data,
                                 int response_num_pairs);
static void process_can_message(cn_cbor *id, cn_cbor *bus, cn_cbor *data);
static void process_dim_text_write(cn_cbor *bus, cn_cbor *data);
static void process_cdc_power_update(cn_cbor *bus, cn_cbor *data);
static size_t parse_cbor_from_buffer(uint8_t *buffer, size_t bytes_present,
                                     cn_cbor **command);
static void process_rx_buffer(uint8_t *buffer, size_t *bytes_present,
                              size_t buffer_size);

static void handle_command(cn_cbor *command);

__STATIC_INLINE void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // allow to use counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // start counter
}

/**
 * @brief UART TX
 *
 * @param pvParameters ignored.
 */
static void uartTX_Task(void *pvParameters) {
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();
    DWT_Init();
    while (1) {
        taskENTER_CRITICAL();
        /* Formatting must be atomic w.r.t. CAN stream
         * TODO modify to drop messages during this instead */
        ssize_t msg_len = sampleFmtMsg();

        if (msg_len <= 0) {
            taskEXIT_CRITICAL();
            vTaskDelayUntil(&last_wake, boron_tx_period_ms);
            continue;
        }
        cmr_uartMsg_t txMsg;
        cmr_uartMsgInit(&txMsg);
        memcpy(send_buf, raw_msg, msg_len);
        sampleClearMsg();

        // Clear DIM acknowledgment signal data (competition workaround)
        raw_sample_data[SKETCHY_DIM_ACK_SIGNAL_INDEX].count = 0;
        raw_sample_data[SKETCHY_DIM_ACK_SIGNAL_INDEX].len = 0;
        memset(raw_sample_data[SKETCHY_DIM_ACK_SIGNAL_INDEX].values, 0,
               MAX_SAMPLEVEC_LEN);

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
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();
    typedef struct {
        cmr_uartMsg_t msg;
        uint8_t buf[RX_BUFFER_SIZE];
        size_t bytes_present;
    } RxBuffer;

    RxBuffer rx[NUM_RX_BUFFERS] = {
        [0 ... NUM_RX_BUFFERS - 1] =
            {
                .bytes_present = 0,
            },
    };
    while (1) {
        for (size_t i = 0; i < NUM_RX_BUFFERS; i++) {
            RxBuffer *curr = &rx[i];
            size_t space_left = sizeof(curr->buf) - curr->bytes_present;
            cmr_uartMsgInit(&curr->msg);
            cmr_uartRX(&uart.port, &curr->msg, curr->buf + curr->bytes_present,
                       space_left, CMR_UART_RXOPTS_IDLEABORT);
            size_t rxLen = cmr_uartMsgWait(&curr->msg);
            curr->bytes_present += rxLen;

            process_rx_buffer(curr->buf, &curr->bytes_present,
                              sizeof(curr->buf));
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
    cn_cbor *msg = cn_cbor_mapget_string(command, "msg");
    cn_cbor *params = cn_cbor_mapget_string(command, "params");
    cn_cbor *pull = cn_cbor_mapget_string(command, "pull");
    cn_cbor *signal_en = cn_cbor_mapget_string(command, "signal_en");

    if (msg != NULL) {
        handle_message_command(msg);
    }

    // Handle parameter updates and pulls with response
    struct param_pair response_data[MAX_SIGNALS];
    int response_num_pairs = 0;
    cn_cbor *response_packet = cn_cbor_map_create(&err);
    if (response_packet == NULL) {
        return;
    }

    if (params != NULL) {
        handle_params_command(params, response_data, &response_num_pairs);
    } else if (pull != NULL) {
        handle_pull_command(pull, response_data, &response_num_pairs);
    }

    if (signal_en != NULL) {
        handle_signal_enable_command(signal_en);
    }

    send_response_packet(response_packet, response_data, response_num_pairs);
    cn_cbor_free(response_packet);
}

/**
 * @brief Initializes a commandline interface.
 *
 */
void uartInit(void) {
    // UART interface initialization.
    const UART_InitTypeDef uartInit = {.BaudRate = 19200,
                                       .WordLength = UART_WORDLENGTH_8B,
                                       .StopBits = UART_STOPBITS_1,
                                       .Parity = UART_PARITY_NONE,
                                       .HwFlowCtl = UART_HWCONTROL_NONE,
                                       .Mode = UART_MODE_TX_RX,
                                       .OverSampling = UART_OVERSAMPLING_16};

    cmr_uartInit(
        &uart.port, USART1, &uartInit, GPIOA, GPIO_PIN_10, /* rx */
        GPIOA, GPIO_PIN_9,                                 /* tx */
        DMA2_Stream5,
        DMA_CHANNEL_4, /* See reference manual pp. 218/1324
                          https://www.st.com/resource/en/reference_manual/rm0430-stm32f413423-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
                        */
        DMA2_Stream7, DMA_CHANNEL_4);

    cmr_taskInit(&uart.txTask, "UART TX", 2, uartTX_Task, NULL);
    cmr_taskInit(&uart.rxTask, "UART RX", 2, uartRX_Task, NULL);
}

/**
 * @brief Parse CBOR data from buffer
 *
 * @param buffer Buffer containing potential CBOR data
 * @param bytes_present Number of bytes in buffer
 * @param command Output parameter for parsed command
 * @return Number of bytes parsed, 0 if no valid CBOR found
 */
static size_t parse_cbor_from_buffer(uint8_t *buffer, size_t bytes_present,
                                     cn_cbor **command) {
    *command = NULL;

    /* Try to parse out a cbor structure from the received data.
     * We may have been sent back-to-back, so assume the worst. */
    for (size_t buflen = 1; buflen <= bytes_present; buflen++) {
        *command = cn_cbor_decode(buffer, buflen, &err);
        if (*command != NULL) {
            return buflen;
        }
    }

    return 0;
}

/**
 * @brief Process received data buffer for CBOR commands
 *
 * @param buffer Buffer containing received data
 * @param bytes_present Pointer to number of bytes present (modified)
 * @param buffer_size Size of the buffer
 */
static void process_rx_buffer(uint8_t *buffer, size_t *bytes_present,
                              size_t buffer_size) {
    cn_cbor *command = NULL;
    size_t bytes_parsed =
        parse_cbor_from_buffer(buffer, *bytes_present, &command);

    if (command == NULL && *bytes_present == buffer_size) {
        /* If we didn't parse a command out, and our buffer is full,
         * drop the buffer and continue on with our lives */
        *bytes_present = 0;
        return;
    } else if (command == NULL) {
        /* We have space left, and we haven't gotten a full command yet,
         * so continue on until we do have one */
        return;
    }

    /* We have a valid command after parsing some number of bytes.
     * Consume that number of bytes, and make use of the command. */
    memmove(buffer, buffer + bytes_parsed, *bytes_present - bytes_parsed);
    *bytes_present -= bytes_parsed;

    handle_command(command);

    /* Don't forget to free this */
    cn_cbor_free(command);
}

/**
 * @brief Handle DIM text write messages
 *
 * @param bus CAN bus identifier
 * @param data Message data
 */
static void process_dim_text_write(cn_cbor *bus, cn_cbor *data) {
    for (size_t i = 0; i < (size_t)data->length;
         i += DIM_TEXT_BYTES_PER_MESSAGE) {
        cmr_canDIMTextWrite_t canData = {
            .address = i / DIM_TEXT_BYTES_PER_MESSAGE, .data = {0}};

        for (size_t j = 0; j < DIM_TEXT_BYTES_PER_MESSAGE; j++) {
            if (i + j < (size_t)data->length) {
                canData.data[j] = data->v.bytes[i + j];
            }
        }

        canTX((cmr_canBusID_t)bus->v.uint, CMR_CANID_DIM_TEXT_WRITE,
              (void *)&canData, sizeof(cmr_canDIMTextWrite_t), portMAX_DELAY);

        TickType_t last_wake = xTaskGetTickCount();
        vTaskDelayUntil(&last_wake, dim_message_delay_ms);
    }
}

/**
 * @brief Handle CDC power update messages
 *
 * @param bus CAN bus identifier
 * @param data Message data
 */
static void process_cdc_power_update(cn_cbor *bus, cn_cbor *data) {
    if (data->length == sizeof(cmr_canCDCPowerLimit_t)) {
        cmr_canCDCPowerLimit_t *powerLimit =
            (cmr_canCDCPowerLimit_t *)data->v.bytes;
        canTX((cmr_canBusID_t)bus->v.uint, CMR_CANID_CDC_POWER_UPDATE,
              powerLimit, sizeof(cmr_canCDCPowerLimit_t), CAN_TX_TIMEOUT_MS);
    }
}

/**
 * @brief Process general CAN messages
 *
 * @param id Message ID
 * @param bus CAN bus identifier
 * @param data Message data
 */
static void process_can_message(cn_cbor *id, cn_cbor *bus, cn_cbor *data) {
    uint16_t message_id = (uint16_t)id->v.uint;

    if (message_id == CMR_CANID_DIM_TEXT_WRITE) {
        process_dim_text_write(bus, data);
    } else if (message_id == CMR_CANID_CDC_POWER_UPDATE) {
        process_cdc_power_update(bus, data);
    } else {
        // Do not allow transmission on tractive CAN for security
        if (bus->v.uint == CMR_CAN_BUS_VEH || bus->v.uint == CMR_CAN_BUS_DAQ) {
            canTX((cmr_canBusID_t)bus->v.uint, message_id, data->v.bytes,
                  data->length, portMAX_DELAY);
        }
    }
}

/**
 * @brief Handle message transmission commands
 *
 * @param msg Message CBOR object
 */
static void handle_message_command(cn_cbor *msg) {
    cn_cbor *id = cn_cbor_mapget_string(msg, "id");
    cn_cbor *data = cn_cbor_mapget_string(msg, "data");
    cn_cbor *bus = cn_cbor_mapget_string(msg, "bus");

    // Validate all required fields are present and have correct types
    bool id_valid =
        (id != NULL && (id->type == CN_CBOR_INT || id->type == CN_CBOR_UINT));
    bool bus_valid = (bus != NULL &&
                      (bus->type == CN_CBOR_INT || bus->type == CN_CBOR_UINT) &&
                      (bus->v.uint < CMR_CAN_BUS_NUM));
    bool data_valid = (data != NULL && (data->type == CN_CBOR_BYTES ||
                                        data->type == CN_CBOR_MAP));

    if (id_valid && bus_valid && data_valid) {
        process_can_message(id, bus, data);
    }
}

/**
 * @brief Handle parameter configuration commands
 *
 * @param params Parameters CBOR object
 * @param response_packet Response packet to populate
 * @param response_data Array to store response data
 * @param response_num_pairs Pointer to response count
 */
static void handle_params_command(cn_cbor *params,
                                  struct param_pair *response_data,
                                  int *response_num_pairs) {
    if ((params->type != CN_CBOR_BYTES && params->type != CN_CBOR_TEXT) ||
        (size_t)params->length < sizeof(struct param_pair)) {
        return;
    }

    /* Expects a byte-string of 2-byte values, each byte pair with form
     * kind:cutoff */
    uint32_t len = params->length;
    for (size_t i = 0; i < len; i += sizeof(struct param_pair)) {
        struct param_pair *pair = (struct param_pair *)(params->v.bytes + i);

        if (pair->kind >= MAX_SIGNALS ||
            pair->cutoff_enum >= SAMPLE_NUM_FREQS) {
            /* Invalid parameters, skip */
            continue;
        }

        response_data[(*response_num_pairs)++] = (struct param_pair){
            .kind = pair->kind,
            .cutoff_enum = pair->cutoff_enum,
        };

        /* Update the configuration */
        struct signal_cfg *cfg = &current_settings.signal_cfg[pair->kind];
        cfg->sample_cutoff_freq = pair->cutoff_enum;
    }

    /* Commit the modified settings */
    commit_settings();
}

/**
 * @brief Handle configuration pull requests
 *
 * @param pull Pull request CBOR object
 * @param response_data Array to store response data
 * @param response_num_pairs Pointer to response count
 */
static void handle_pull_command(cn_cbor *pull, struct param_pair *response_data,
                                int *response_num_pairs) {
    /* Return current settings for all parsed signals */
    for (uint32_t i = 0; i < signals_parsed; i++) {
        response_data[(*response_num_pairs)++] = (struct param_pair){
            .kind = i,
            .cutoff_enum = current_settings.signal_cfg[i].sample_cutoff_freq,
        };
    }
}

/**
 * @brief Handle signal enable/disable commands
 *
 * @param signal_en Signal enable CBOR object
 */
static void handle_signal_enable_command(cn_cbor *signal_en) {
    // Early validation
    if (!signal_en ||
        (signal_en->type != CN_CBOR_BYTES && signal_en->type != CN_CBOR_TEXT)) {
        return;
    }

    const uint8_t *data = (signal_en->type == CN_CBOR_BYTES)
                              ? signal_en->v.bytes
                              : (const uint8_t *)signal_en->v.str;

    uint32_t signal_count = MIN((uint32_t)signal_en->length, MAX_SIGNALS);

    for (uint32_t i = 0; i < signal_count; i++) {
        if (data[i]) {  // Non-zero values enable the signal
            setSignalEnable(i, true);
        }
    }
}

/**
 * @brief Send response packet back to requester
 *
 * @param response_packet Response packet CBOR object
 * @param response_data Response data array
 * @param response_num_pairs Number of response pairs
 */
static void send_response_packet(cn_cbor *response_packet,
                                 const struct param_pair *response_data,
                                 int response_num_pairs) {
    static uint8_t response_buffer[MAX_MESSAGE_LEN];
    cn_cbor_errback err = CN_CBOR_NO_ERROR;

    if (!response_packet || response_num_pairs < 0) {
        return;
    }

    if (response_num_pairs > 0 && response_data) {
        size_t data_size = response_num_pairs * sizeof(struct param_pair);
        cn_cbor *formatted_response_data = cn_cbor_data_create(
            (const uint8_t *)response_data, data_size, &err);

        if (formatted_response_data && err == CN_CBOR_NO_ERROR) {
            cn_cbor_mapput_string(response_packet, "params",
                                  formatted_response_data, &err);
        }
    }

    ssize_t encoded_size = cn_cbor_encoder_write(
        response_buffer, 0, sizeof(response_buffer), response_packet);

    if (encoded_size > 0) {
        cmr_uartMsg_t tx_msg;
        cmr_uartMsgInit(&tx_msg);
        cmr_uartTX(&uart.port, &tx_msg, response_buffer, encoded_size);
        cmr_uartMsgWait(&tx_msg);
    }
}