/**
 * @file uart.c
 * @brief Board-specific UART implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <stdio.h>   // snprintf()
#include <string.h>  // memcpy()

#include <CMR/config.h> // CMR configuration itnerface
#include <CMR/config_screen_helper.h>

#include "uart.h"            // Interface to implement
#include "sample.h"          // Sample formatting
#include "can.h"             // Can interface
#include "config.h"          // Config interface
#include <cn-cbor/cn-cbor.h> /* CBOR decoding */
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

/** @brief Represents a UART interface. */
typedef struct uart uart_t;

extern sample_data_t raw_sample_data[MAX_SIGNALS];

#define SIGNAL_MAP_OFFSET 24
extern struct signal signal_map[MAX_SIGNALS];

struct uart
{
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

/** @brief Width of N-way UART receive buffering (e.g. 2 for double buffereing) */
#define NUM_RX_BUFFERS 2

/**
 * @brief This is required by cbor, but we don't care about it
 * (outside of debugging)
 */
static cn_cbor_errback err;

static void handle_command(cn_cbor *command);

__STATIC_INLINE void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // allow to use counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // start counter
}

/**
 * @brief UART TX
 *
 * @param pvParameters ignored.
 */
static void uartTX_Task(void *pvParameters)
{
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();
    DWT_Init();
    while (1)
    {
        taskENTER_CRITICAL();
        /* Formatting must be atomic w.r.t. CAN stream
         * TODO modify to drop messages during this instead */
        ssize_t msg_len = sampleFmtMsg();

        if (msg_len <= 0)
        {
            taskEXIT_CRITICAL();
            vTaskDelayUntil(&last_wake, boron_tx_period_ms);
            continue;
        }

        cmr_uartMsg_t txMsg;
        cmr_uartMsgInit(&txMsg);
        memcpy(send_buf, raw_msg, msg_len);
        sampleClearMsg();
        // Sketchy things done at comp for DIM Ack
        raw_sample_data[10].count = 0;
        raw_sample_data[10].len = 0;
        memset(raw_sample_data[10].values, 0, MAX_SAMPLEVEC_LEN);

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
static void uartRX_Task(void *pvParameters)
{
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();
    struct
    {
        cmr_uartMsg_t msg;
        uint8_t buf[200];
        size_t bytes_present;
    } rx[NUM_RX_BUFFERS] = {
        [0 ... NUM_RX_BUFFERS - 1] = {
            .bytes_present = 0,
        },
    };

    while (1)
    {
        // QUESTION why cant you do NUM_RX_BUFFERS
        for (size_t i = 0; i < NUM_RX_BUFFERS; i++)
        {
            size_t space_left = sizeof(rx[i].buf) - rx[i].bytes_present;
            cmr_uartMsgInit(&rx[i].msg);
            cmr_uartRX(
                &uart.port, &rx[i].msg,
                rx[i].buf + rx[i].bytes_present, space_left,
                CMR_UART_RXOPTS_IDLEABORT);
            size_t rxLen = cmr_uartMsgWait(&rx[i].msg);
            rx[i].bytes_present += rxLen;

            /* Try to parse out a cbor structure from the received data.
             * We may have been sent back-to-back, so assume the worst. */
            cn_cbor *command = NULL;
            size_t bytes_parsed;
            for (size_t buflen = 1; buflen <= rx[i].bytes_present; buflen++)
            {
                command = cn_cbor_decode(rx[i].buf, buflen, &err);
                if (command != NULL)
                {
                    bytes_parsed = buflen;
                    break;
                }
            }

            if (command == NULL && rx[i].bytes_present == sizeof(rx[i].buf))
            {
                /* If we didn't parse a command out, and our buffer is full,
                 * drop the buffer and continue on with our lives */
                rx[i].bytes_present = 0;
                continue;
            }
            else if (command == NULL)
            {
                /* We have space left, and we haven't gotten a full command yet,
                 * so continue on until we do have one */
                continue;
            }

            /* We have a valid command after parsing some number of bytes.
             * Consume that number of bytes, and make use of the command. */
            memmove(
                rx[i].buf,
                rx[i].buf + bytes_parsed,
                rx[i].bytes_present - bytes_parsed);
            rx[i].bytes_present -= bytes_parsed;

            handle_command(command);

            /* Don't forget to free this */
            cn_cbor_free(command);
        }

        vTaskDelayUntil(&last_wake, boron_rx_period_ms);
    }
}

/**
 *  @brief Handle some CBOR command from the outside world
 *
 *  @param command The command to handle
 */
static void handle_command(cn_cbor *command)
{
    cn_cbor *msg, *params, *pull, *id, *data, *bus, *signal_en, *num_ids;
    msg = cn_cbor_mapget_string(command, "msg");
    params = cn_cbor_mapget_string(command, "params");
    pull = cn_cbor_mapget_string(command, "pull");
    signal_en = cn_cbor_mapget_string(command, "signal_en");


    if (msg != NULL)
    {
        /* Have a message to transmit */
        id = cn_cbor_mapget_string(msg, "id");
        data = cn_cbor_mapget_string(msg, "data");
        bus = cn_cbor_mapget_string(msg, "bus");
        num_ids = cn_cbor_mapget_string(msg, "num_ids");
        cn_cbor *arr = cn_cbor_mapget_string(msg, "config_ids");

        if (
            (id != NULL &&
             (id->type == CN_CBOR_INT || id->type == CN_CBOR_UINT)) &&
            (bus != NULL &&
             (bus->type == CN_CBOR_INT || bus->type == CN_CBOR_UINT) &&
             (bus->v.uint < CMR_CAN_BUS_NUM)) &&
            (data != NULL && (data->type == CN_CBOR_BYTES || data->type == CN_CBOR_MAP)))
        {
            if ((uint16_t)id->v.uint == CMR_CANID_DIM_TEXT_WRITE)
            {
                const size_t sizePerMessage = 4;
                for (size_t i = 0; i < data->length; i += sizePerMessage)
                {
                    cmr_canDIMTextWrite_t canData = (cmr_canDIMTextWrite_t){
                        .address = i / sizePerMessage,
                        .data = {0}};
                    for (size_t j = 0; j < sizePerMessage; j++)
                    {
                        if (i + j < data->length)
                        {
                            canData.data[j] = data->v.bytes[i + j];
                        }
                    }
                    canTX(
                        (cmr_canBusID_t)bus->v.uint, (uint16_t)id->v.uint,
                        (void *)&canData, sizeof(cmr_canDIMTextWrite_t),
                        portMAX_DELAY);
                    TickType_t last_wake = xTaskGetTickCount();
                    vTaskDelayUntil(&last_wake, dim_message_delay_ms);
                }
            }
            else if ((uint16_t)id->v.uint == CMR_CANID_CDC_POWER_UPDATE)
            {
                // Set the power limit in config params from DAQ Live
                if (data->length == sizeof(cmr_canCDCPowerLimit_t))
                {
                    cmr_canCDCPowerLimit_t *powerLimit = (cmr_canCDCPowerLimit_t *)data->v.bytes;

                    canTX((cmr_canBusID_t)bus->v.uint, (uint16_t)id->v.uint,
                    	  powerLimit, sizeof(cmr_canCDCPowerLimit_t),
					      200);
                }
            } else {
                // Do not allow transmission on tractive CAN.
                if (bus->v.uint == CMR_CAN_BUS_VEH || bus->v.uint == CMR_CAN_BUS_DAQ)
                {
                    canTX(
                        (cmr_canBusID_t)bus->v.uint, (uint16_t)id->v.uint,
                        data->v.bytes, data->length,
                        portMAX_DELAY);
                }
            }
        }
    }

    struct param_pair
    {
        uint8_t kind;
        uint8_t cutoff_enum;
    };

    /* For updating params and pulling we'll need a response packet */
    /* We'll also keep the state to buffer the encoded packet here. */
    static uint8_t response_buffer[MAX_MESSAGE_LEN];

    cn_cbor *response_packet = cn_cbor_map_create(&err);
    int response_num_pairs = 0;
    struct param_pair response_data[MAX_SIGNALS];
    if (response_packet == NULL)
    {
        return;
    }
    /* Apparently the CBOR javascript lib is going to send as text. */
    if (
        params != NULL &&
        (params->type == CN_CBOR_BYTES || params->type == CN_CBOR_TEXT) &&
        params->length >= sizeof(struct param_pair)) {
        /* Have some parameters to update */
        /* Expects an byte-string of 2-byte values, each byte pairs with form
         * kind:cutoff */
        uint32_t len = params->length;
        for (size_t i = 0; i < len; i += sizeof(struct param_pair))
        {
            struct param_pair *pair = (struct param_pair *)(params->v.bytes + i);

            if (
                pair->kind >= MAX_SIGNALS ||
                pair->cutoff_enum >= SAMPLE_NUM_FREQS)
            {
                /* Questionable update parameters, just move on */
                continue;
            }

            response_data[response_num_pairs++] = (struct param_pair){
                .kind = pair->kind,
                .cutoff_enum = pair->cutoff_enum,
            };

            /* If we run out of memory later, we'll not be able to notify
             * the server, but that's low risk. */
            struct signal_cfg *cfg = &current_settings.signal_cfg[pair->kind];
            cfg->sample_cutoff_freq = pair->cutoff_enum;
        }

        /* Commit any modified settings. We could put a flag in for the
         * no-update-detected case, but it's fiiine. */
        commit_settings();
    } else if (pull != NULL) {
        /* We were requested a dump of the current settings.
         * Note that this must be exclusive with the parameter update path.
         * (which has priority). */
        /* We could encode this as a diff to save on bandwidth,
         * but realistically this will basically never happen.*/

        /* Gather the settings up. We don't need to transmit -all- of the
         * settings, as most will probably be unused; The parser
         * can inform us how many are in use. */
        for (uint32_t i = 0; i < signals_parsed; i++)
        {
            response_data[response_num_pairs++] = (struct param_pair){
                .kind = i,
                .cutoff_enum = current_settings.signal_cfg[i].sample_cutoff_freq,
            };
        }
    }
    if (signal_en != NULL &&
        (signal_en->type == CN_CBOR_BYTES || signal_en->type == CN_CBOR_TEXT))
    {
        for (uint32_t i = 0; i < MIN((uint32_t)signal_en->length, MAX_SIGNALS); i++)
        {
            if (signal_en->type == CN_CBOR_BYTES)
            {
                if (signal_en->v.bytes[i])
                    setSignalEnable(i, (bool)signal_en->v.bytes[i]);
            }
            else
            {
                if (signal_en->v.str[i])
                    setSignalEnable(i, (bool)signal_en->v.str[i]);
            }
        }
    }

    if (response_num_pairs)
    {
        /* We have a response to send out */
        cn_cbor *formatted_response_data = cn_cbor_data_create(
            (uint8_t *)response_data,
            response_num_pairs * sizeof(struct param_pair),
            &err);

        if (formatted_response_data)
        {
            /* If this fails, tough, you're getting an empty map */
            (void)cn_cbor_mapput_string(
                response_packet,
                "params",
                formatted_response_data, &err);
        }

        ssize_t ret = cn_cbor_encoder_write(
            response_buffer, 0, sizeof(response_buffer),
            response_packet);

        if (ret > 0)
        {
            /* Finally ready to send the dang thing */
            cmr_uartMsg_t txMsg;
            cmr_uartMsgInit(&txMsg);
            cmr_uartTX(&uart.port, &txMsg, response_buffer, ret);
            cmr_uartMsgWait(&txMsg);
        }
    } else { // this is the case where we're not updating signal configs.
        ssize_t ret = cn_cbor_encoder_write(
            response_buffer, 0, sizeof(response_buffer),
            response_packet);

        if (ret > 0)
        {
            /* Finally ready to send the dang thing */
            cmr_uartMsg_t txMsg;
            cmr_uartMsgInit(&txMsg);
            cmr_uartTX(&uart.port, &txMsg, response_buffer, ret);
            cmr_uartMsgWait(&txMsg);
        }
    }

    /* Free the response packet */
    cn_cbor_free(response_packet);
}

/**
 * @brief Initializes a commandline interface.
 *
 */
void uartInit(void)
{
    // UART interface initialization.
    const UART_InitTypeDef uartInit = {
        .BaudRate = 19200,
        .WordLength = UART_WORDLENGTH_8B,
        .StopBits = UART_STOPBITS_1,
        .Parity = UART_PARITY_NONE,
        .HwFlowCtl = UART_HWCONTROL_NONE,
        .Mode = UART_MODE_TX_RX,
        .OverSampling = UART_OVERSAMPLING_16};

    cmr_uartInit(
        &uart.port, USART1, &uartInit,
        GPIOA, GPIO_PIN_10,          /* rx */
        GPIOA, GPIO_PIN_9,           /* tx */
        DMA2_Stream5, DMA_CHANNEL_4, /* See reference manual pp. 218/1324 https://www.st.com/resource/en/reference_manual/rm0430-stm32f413423-advanced-armbased-32bit-mcus-stmicroelectronics.pdf*/
        DMA2_Stream7, DMA_CHANNEL_4);

    cmr_taskInit(&uart.txTask, "UART TX", 2, uartTX_Task, NULL);
    cmr_taskInit(&uart.rxTask, "UART RX", 2, uartRX_Task, NULL);
}