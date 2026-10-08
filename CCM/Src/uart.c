/**
 * @file uart.c
 * @brief Board-specific UART implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <stdio.h>      // snprintf()
#include <string.h>     // memcpy()
#include <stdbool.h>    // bool
#include <assert.h>

#include <CMR/uart.h>       // CMR UART interface
#include <CMR/tasks.h>      // CMR task interface
#include <CMR/config.h>     // CMR configuration itnerface
#include <CMR/panic.h>      // bad things
#include <cn-cbor/cn-cbor.h>

#include "uart.h"       // Interface to implement
#include "can.h"        // Can interface
#include "state.h"      // State machine interface
#include "sensors.h"    // Sensors interface
#include "evse.h"    // EVSE interface

/** @brief Represents a UART interface. */
typedef struct uart uart_t;

struct uart {
    cmr_uart_t port;    /**< @brief The underlying UART port. */

    cmr_task_t rxTask;  /**< @brief Receive task. */
    cmr_task_t txTask;  /**< @brief Receive task. */
};

/** @brief UART TX period */
static const TickType_t uart_tx_period_ms = 200;

/** @brief UART RX period */
static const TickType_t uart_rx_period_ms = 10;

/** @brief Width of N-way UART receive buffering (e.g. 2 for double buffereing) */
#define NUM_RX_BUFFERS 2

/**
 * @brief This is required by cbor, but we don't care about it
 * (outside of debugging)
 */
static cn_cbor_errback err;

/** @brief Primary UART interface. */
static uart_t uart;

#define MAX_OUTPUT_LEN 500
static uint8_t output_buf[MAX_OUTPUT_LEN];


/**
 * @brief Convert a CCMData struct to a CBOR string
 *
 * @param buf Output string
 * @param buflen Length of the buffer
 * @param data The struct to encode
 */
int ccmdata_convert_string(unsigned char* buf, int buflen, cmr_ccmData_t* data) {
    cn_cbor* root_map = cn_cbor_map_create(&err);

    ////// Battery voltage
    cn_cbor* batt_voltage_val = cn_cbor_int_create(data->batt_voltage, &err);
    cn_cbor_mapput_string(root_map, "batt_voltage", batt_voltage_val, &err);

    cn_cbor* hv_voltage_val = cn_cbor_int_create(data->hv_voltage, &err);
    cn_cbor_mapput_string(root_map, "hv_voltage", hv_voltage_val, &err);
    ///////////


    ////// Evse max current val
    cn_cbor* evse_max_current_val = cn_cbor_int_create(data->evse_max_current, &err);
    cn_cbor_mapput_string(root_map, "evse_max_current", evse_max_current_val, &err);
    ///////////

    ////// BMS state
    cn_cbor* hvc_state = cn_cbor_map_create(&err);

    cn_cbor* state_val = cn_cbor_int_create(data->hvc.state, &err);
    cn_cbor_mapput_string(hvc_state, "state", state_val, &err);

    cn_cbor* mode_val = cn_cbor_int_create(data->hvc.mode, &err);
    cn_cbor_mapput_string(hvc_state, "mode", mode_val, &err);

    cn_cbor* error_vector_val = cn_cbor_int_create(data->hvc.error_vector, &err);
    cn_cbor_mapput_string(hvc_state, "error_vector", error_vector_val, &err);

    cn_cbor* alive_val = cn_cbor_int_create(data->hvc.alive, &err);
    cn_cbor_mapput_string(hvc_state, "alive", alive_val, &err);

    cn_cbor_mapput_string(root_map, "hvc", hvc_state, &err);
    ///////////

    ////// Chargers
    cn_cbor* chargers_array = cn_cbor_array_create(&err);
    for (int i = 0; i < CHARGER_LEN; i++) {
        cn_cbor* charger_map = cn_cbor_map_create(&err);

        cn_cbor* id_val = cn_cbor_int_create(data->chargers[i].id, &err);
        cn_cbor_mapput_string(charger_map, "id", id_val, &err);

        cn_cbor* state_val = cn_cbor_int_create(data->chargers[i].state, &err);
        cn_cbor_mapput_string(charger_map, "state", state_val, &err);

        cn_cbor* voltage_val = cn_cbor_int_create(data->chargers[i].voltage, &err);
        cn_cbor_mapput_string(charger_map, "voltage", voltage_val, &err);

        cn_cbor* current_val = cn_cbor_int_create(data->chargers[i].current, &err);
        cn_cbor_mapput_string(charger_map, "current", current_val, &err);

        cn_cbor_array_append(chargers_array, charger_map, &err);
    }
    cn_cbor_mapput_string(root_map, "chargers", chargers_array, &err);
    ///////////

    /////////// HVC cell temperature and voltages
    cn_cbor* max_cell_temp_val = cn_cbor_int_create(data->max_cell_temp, &err);
    cn_cbor_mapput_string(root_map, "max_cell_temp", max_cell_temp_val, &err);

    cn_cbor* max_cell_volt_val = cn_cbor_int_create(data->max_cell_volt, &err);
    cn_cbor_mapput_string(root_map, "max_cell_volt", max_cell_volt_val, &err);
    
    cn_cbor* min_cell_volt_val = cn_cbor_int_create(data->min_cell_volt, &err);
    cn_cbor_mapput_string(root_map, "min_cell_volt", min_cell_volt_val, &err);

    ///////////

    int written = cn_cbor_encoder_write(buf, 0, buflen, root_map);
    cn_cbor_free(root_map); // recursively does all the others
    return written;
}

/**
 * @brief UART TX
 *
 * @param pvParameters ignored.
 */
static void uartTX_Task(void *pvParameters) {
    (void) pvParameters;
    TickType_t last_wake = xTaskGetTickCount();

    cmr_ccmData_t data;

    while(1) {
        // TODO: Figure out how to read sensors
        // TODO: Add correct num of chargers and segments
        taskENTER_CRITICAL();
            cmr_canRXMeta_t *metaHVCPackVoltage = canVehicleRXMeta + CANRX_HVC_PACK_VOLTAGE;
            volatile cmr_canHVCPackVoltage_t *canHVCPackVoltage = (void *) metaHVCPackVoltage->payload;
            data.hv_voltage = canHVCPackVoltage->hvVoltage_mV;
            data.batt_voltage = canHVCPackVoltage->battVoltage_mV;

            cmr_canRXMeta_t *metaHVCHeartbeat = canVehicleRXMeta + CANRX_HVC_HEARTBEAT;
            volatile cmr_canHVCHeartbeat_t *canHVCHeartbeat = (void *) metaHVCHeartbeat->payload;
            uint8_t hvcMode = canHVCHeartbeat->hvcMode;
            uint8_t hvcState = canHVCHeartbeat->hvcState;
            uint16_t hvcErrorStatus = canHVCHeartbeat->errorStatus;
            data.hvc.state = hvcState;
            data.hvc.mode = hvcMode;
            data.hvc.error_vector = hvcErrorStatus;
            data.hvc.alive = (cmr_canRXMetaTimeoutError((canVehicleRXMeta + CANRX_HVC_HEARTBEAT), last_wake) >= 0);

            int32_t pilotDuty = cmr_sensorListGetValue(&sensorList, SENSOR_CH_PILOT_DUTY);
            uint32_t evse_max_current = getEvseCurrentLimit(pilotDuty);
            data.evse_max_current = evse_max_current;

            cmr_canRXMeta_t *metaHVCCellTemps = canVehicleRXMeta + CANRX_HVC_CELL_TEMPS;
            volatile cmr_canHVCPackMinMaxCellTemps_t *canHVCCellTemps = (void *)metaHVCCellTemps->payload;

            cmr_canRXMeta_t *metaHVCCellVoltages = canVehicleRXMeta + CANRX_HVC_CELL_VOLTAGE;
            volatile cmr_canHVCPackMinMaxCellVolages_t *canHVCCellVoltages = (void *)metaHVCCellVoltages->payload;

            for (int i = 0; i < CHARGER_LEN; i++) {
                cmr_canRXMeta_t *metaChargerState = NULL;
                switch (i) {
                    case 0:
                        metaChargerState = canChargerOneRXMeta + CANRX_CHARGER_ONE_STATE;
                        break;
                    case 1:
                        metaChargerState = canChargerTwoRXMeta + CANRX_CHARGER_TWO_STATE;
                        break;
                    default:
                        cmr_panic("Invalid charger (charger length is too high)");
                }
                volatile cmr_canDilongState_t *canChargerState = (void *) metaChargerState->payload;

                data.chargers[i].id = i;
                data.chargers[i].state = canChargerState->status_bv;
                data.chargers[i].voltage = (canChargerState->outputVoltageHigh << 8) + canChargerState->outputVoltageLow;
                data.chargers[i].current = (canChargerState->outputCurrentHigh << 8) + canChargerState->outputCurrentLow;
            }

            data.max_cell_temp = canHVCCellTemps->maxCellTemp_dC;
            data.max_cell_volt = canHVCCellVoltages->maxCellVoltage_mV;
            data.min_cell_volt = canHVCCellVoltages->minCellVoltage_mV;

            int message_len = ccmdata_convert_string(output_buf, MAX_OUTPUT_LEN - 1, &data);

            if (message_len < 0) {
                cmr_panic("Unable to generate CBOR data");
            }

            // Stick a newline at the end to make it easier for the RPi
            if (message_len >= MAX_OUTPUT_LEN) {
                cmr_panic("Can't send this much data");
            }

            output_buf[message_len] = '\n';
            message_len += 1;

            if (message_len > MAX_OUTPUT_LEN) {
                cmr_panic("Can't send this much data");
            }

            cmr_uartMsg_t msg;
            cmr_uartMsgInit(&msg);
        taskEXIT_CRITICAL();

        cmr_uartTX(&uart.port, &msg, output_buf, message_len);
        cmr_uartMsgWait(&msg);

        vTaskDelayUntil(&last_wake, uart_tx_period_ms);
    }
}

/**
 *  @brief Handle some CBOR command from the outside world
 *
 *  @param command The command to handle
 */
static void handle_command(cn_cbor *command) {
    cn_cbor *requestedCommand;
    requestedCommand = cn_cbor_mapget_string(command, "command");

    if (requestedCommand != NULL &&
        (requestedCommand->type == CN_CBOR_INT || requestedCommand->type == CN_CBOR_UINT)) {
        setCommand(requestedCommand->v.uint);
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
        unsigned char buf[100];
        size_t bytes_present;
    } rx[NUM_RX_BUFFERS];

    memset(rx, 0, sizeof(rx));

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

        vTaskDelayUntil(&last_wake, uart_rx_period_ms);
    }
}

/**
 * @brief Initializes a command line interface.
 *
 */
void uartInit(void) {
    // UART interface initialization.
    const UART_InitTypeDef uartInit = {
        .BaudRate = 115200,
        .WordLength = UART_WORDLENGTH_8B,
        .StopBits = UART_STOPBITS_1,
        .Parity = UART_PARITY_NONE,
        .HwFlowCtl = UART_HWCONTROL_NONE,
        .Mode = UART_MODE_TX_RX,
        .OverSampling = UART_OVERSAMPLING_16
    };

    cmr_uartInit(
        &uart.port, USART2, &uartInit,
        GPIOA, GPIO_PIN_3,              /* rx */
        GPIOA, GPIO_PIN_2,              /* tx */
        DMA1_Stream5, DMA_CHANNEL_4,    /* See reference manual */
        DMA1_Stream6, DMA_CHANNEL_4
    );

    cmr_taskInit(&uart.txTask, "UART TX", 8, uartTX_Task, NULL);
    cmr_taskInit(&uart.rxTask, "UART RX", 8, uartRX_Task, NULL);
}
