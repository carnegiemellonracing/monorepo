/**
 * @file tft.c
 * @brief TFT display implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/tasks.h>  // Task interface

#include <stdbool.h>     // bool

#include "tft.h"            // Interface to implement
#include "tftPrivate.h"     // Private interface
#include "tftContent.h"     // Content interface
#include "tftDL.h"          // Display list interface
#include "gpio.h"           // Board-specific GPIO interface
#include "can.h"            // Board-specific CAN interface
#include "state.h"          // State interface

/** @brief Expected chip ID. */
#define TFT_CHIP_ID 0x00011208

/** @brief Display reset time, in milliseconds. */
#define TFT_RESET_MS 50

/** @brief Display initialization QuadSPI prescaler. */
#define TFT_INIT_QSPI_PRESCALER 32

/** @brief Display initialization time, in milliseconds. */
#define TFT_INIT_MS 400

/** @brief Display QuadSPI prescaler */
#define TFT_QSPI_PRESCALER 2

/** @brief Display startup time, in milliseconds. */
#define TFT_STARTUP_MS 3000

/** @brief Flag for indicating a write to the display. */
#define TFT_WRITE_FLAG (1 << 23)

/** @brief Dummy cycles for reading data from the display. */
#define TFT_READ_DUMMY_CYCLES 8

/** @brief The display. */
static tft_t tft;

static void drawErrorScreen(void);
static void drawRTDScreen(void);

/**
 * @brief Sends a command to the display.
 *
 * @param tft The display.
 * @param cmd The command.
 * @param param The command's parameter.
 */
void tftCmd(tft_t *tft, tftCmd_t cmd, uint8_t param) {
    uint32_t addr = (cmd << 16) | (param << 8);

    const QSPI_CommandTypeDef qspiCmd = {
        .Instruction = 0,
        .Address = addr,
        .AlternateBytes = 0,
        .AddressSize = QSPI_ADDRESS_24_BITS,
        .AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS,
        .DummyCycles = 0,
        .InstructionMode = QSPI_INSTRUCTION_NONE,
        .AddressMode = QSPI_ADDRESS_1_LINE,
        .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
        .DataMode = QSPI_DATA_NONE,
        .NbData = 0,
        .DdrMode = QSPI_DDR_MODE_DISABLE,
        .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,
        .SIOOMode = QSPI_SIOO_INST_EVERY_CMD
    };

    cmr_qspiCmd(&tft->qspi, &qspiCmd);
}

/**
 * @brief Writes data to the display.
 *
 * @param tft The display.
 * @param addr The address to write to.
 * @param len The length of the data.
 * @param data The data to write.
 */
void tftWrite(tft_t *tft, tftAddr_t addr, size_t len, const void *data) {
    const QSPI_CommandTypeDef cmd = {
        .Instruction = 0,
        .Address = addr | TFT_WRITE_FLAG,
        .AlternateBytes = 0,
        .AddressSize = QSPI_ADDRESS_24_BITS,
        .AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS,
        .DummyCycles = 0,
        .InstructionMode = QSPI_INSTRUCTION_NONE,
        .AddressMode = QSPI_ADDRESS_1_LINE,
        .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
        .DataMode = QSPI_DATA_1_LINE,
        .NbData = len,
        .DdrMode = QSPI_DDR_MODE_DISABLE,
        .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,
        .SIOOMode = QSPI_SIOO_INST_EVERY_CMD
    };

    cmr_qspiTX(&tft->qspi, &cmd, data);
}

/**
 * @brief Reads data from the display.
 *
 * @param tft The display.
 * @param addr The address to read from.
 * @param len The length of the data.
 * @param data The buffer for received data.
 */
void tftRead(tft_t *tft, tftAddr_t addr, size_t len, void *data) {
    const QSPI_CommandTypeDef cmd = {
        .Instruction = 0,
        .Address = addr,
        .AlternateBytes = 0,
        .AddressSize = QSPI_ADDRESS_24_BITS,
        .AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS,
        .DummyCycles = TFT_READ_DUMMY_CYCLES,
        .InstructionMode = QSPI_INSTRUCTION_NONE,
        .AddressMode = QSPI_ADDRESS_1_LINE,
        .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
        .DataMode = QSPI_DATA_1_LINE,
        .NbData = len,
        .DdrMode = QSPI_DDR_MODE_DISABLE,
        .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,
        .SIOOMode = QSPI_SIOO_INST_EVERY_CMD
    };

    cmr_qspiRX(&tft->qspi, &cmd, data);
}

/**
 * @brief Gets the number of available bytes in the coprocessor command buffer.
 *
 * @note Also updates the coprocessor read address.
 *
 * @param tft The display.
 *
 * @return The number of available bytes.
 */
static size_t tftCoCmdRemLen(tft_t *tft) {
    tftRead(tft, TFT_ADDR_CMD_READ, sizeof(tft->coCmdRd), &tft->coCmdRd);

    size_t used = (tft->coCmdWr - tft->coCmdRd);
    if (used >= TFT_RAM_CMD_SIZE) {
        used -= TFT_RAM_CMD_SIZE;
    }

    // Maintain 1-word separation.
    return (TFT_RAM_CMD_SIZE - sizeof(uint32_t)) - used;
}

/**
 * @brief Writes coprocessor commands to the display.
 *
 * @warning When `wait` is `false`, `len` MUST be less than `TFT_RAM_CMD_SIZE`!
 *
 * @param tft The display.
 * @param len The length of the command.
 * @param data The command's data.
 * @param wait `true` to wait for partial writes to finish; `false` to wait for
 * enough space at the beginning.
 */
void tftCoCmd(tft_t *tft, size_t len, const void *data, bool wait) {
    configASSERT(wait || len < TFT_RAM_CMD_SIZE);

    const uint8_t *dataBuf = data;
    size_t written = 0;

    while (written < len) {
        // Calculate length to write.
        size_t wrLen = len - written;

        if (!wait) {
            // Wait for free space to write the entire command.
            size_t remLen;
            do {
                remLen = tftCoCmdRemLen(tft);
            } while (remLen < wrLen);
        } else {
            // Write as much as possible.
            size_t remLen = tftCoCmdRemLen(tft);
            if (remLen == 0) {
                continue;   // No space yet.
            }
            if (wrLen > remLen) {
                wrLen = remLen;
            }
        }

        tftWrite(
            tft, TFT_ADDR_RAM_CMD + tft->coCmdWr,
            wrLen, dataBuf + written
        );
        if (wrLen % sizeof(uint32_t) != 0) {
            // Round-up to word-aligned length.
            wrLen /= sizeof(uint32_t);
            wrLen++;
            wrLen *= sizeof(uint32_t);
        }

        written += wrLen;

        // Update the command write address.
        uint16_t coCmdWr = tft->coCmdWr + wrLen;
        if (coCmdWr >= TFT_RAM_CMD_SIZE) {
            coCmdWr -= TFT_RAM_CMD_SIZE;
        }
        tftWrite(tft, TFT_ADDR_CMD_WRITE, sizeof(coCmdWr), &coCmdWr);
        tft->coCmdWr = coCmdWr;

        if (!wait) {
            // No waiting; we must have written the whole buffer.
            configASSERT(written == wrLen);
            break;
        }

        // Wait for the command to finish.
        while (
            tftRead(
                tft, TFT_ADDR_CMD_READ,
                sizeof(tft->coCmdRd), &tft->coCmdRd
            ),
            tft->coCmdRd != coCmdWr
        ) {
            continue;
        }
    }
}

/** @brief Display update priority. */
static const uint32_t tftUpdate_priority = 1;

/** @brief Display update period. */
static const TickType_t tftUpdate_period_ms = 20;

/** @brief Display update task. */
static cmr_task_t tftUpdate_task;

/**
 * @brief Task for updating the display.
 *
 * @param pvParameters (tft_t *) The display.
 *
 * @return Does not return.
 */
static void tftUpdate(void *pvParameters) {
    /** @brief Represents a display initialization value. */
    typedef struct {
        tftAddr_t addr;     /**< @brief Address to initialize. */
        uint16_t val;       /**< @brief Value to use. */
    } tftInit_t;

    /** @brief Display register initialization values. */
    static const tftInit_t tftInits[] = {
        { .addr = TFT_ADDR_HCYCLE, .val = 408 },
        { .addr = TFT_ADDR_HOFFSET, .val = 70 },
        { .addr = TFT_ADDR_HSYNC0, .val = 0 },
        { .addr = TFT_ADDR_HSYNC1, .val = 10 },
        { .addr = TFT_ADDR_VCYCLE, .val = 263 },
        { .addr = TFT_ADDR_VOFFSET, .val = 13 },
        { .addr = TFT_ADDR_VSYNC0, .val = 0 },
        { .addr = TFT_ADDR_VSYNC1, .val = 2 },
        { .addr = TFT_ADDR_SWIZZLE, .val = 2 },
        { .addr = TFT_ADDR_PCLK_POL, .val = 1 },
        { .addr = TFT_ADDR_CSPREAD, .val = 0 },
        { .addr = TFT_ADDR_HSIZE, .val = 320 },
        { .addr = TFT_ADDR_VSIZE, .val = 240 },
        { .addr = TFT_ADDR_GPIOX_DIR, .val = (1 << 15) },
        { .addr = TFT_ADDR_GPIOX, .val = (1 << 15) },
        { .addr = TFT_ADDR_PCLK, .val = 6 }
    };

    tft_t *tft = pvParameters;

    /* Restarting the Display. */
    TickType_t lastWakeTime = xTaskGetTickCount();
    cmr_gpioWrite(GPIO_PD_N, 0);
    vTaskDelayUntil(&lastWakeTime, TFT_RESET_MS);
    cmr_gpioWrite(GPIO_PD_N, 1);
    vTaskDelayUntil(&lastWakeTime, TFT_RESET_MS);

    /* Initialize the display. */
    tftCmd(tft, TFT_CMD_CLKEXT, 0x00);
    tftCmd(tft, TFT_CMD_ACTIVE, 0x00);
    tftCmd(tft, TFT_CMD_ACTIVE, 0x00);

    /* Wait for display to initialize. */
    vTaskDelayUntil(&lastWakeTime, TFT_INIT_MS);

    /* Ensure that Chip ID is read correctly */
    uint32_t chipID;
    tftRead(tft, TFT_ADDR_CHIP_ID, sizeof(chipID), &chipID);
    configASSERT(chipID == TFT_CHIP_ID);

    /* Initialize Video Registers. */
    for (size_t i = 0; i < sizeof(tftInits) / sizeof(tftInits[0]); i++) {
        const tftInit_t *init = tftInits + i;
        tftWrite(tft, init->addr, sizeof(init->val), &init->val);
    }

    tft->inited = true;

    /* Enable Faster Clock Rate now that initialization is complete */
    cmr_qspiSetPrescaler(&tft->qspi, TFT_QSPI_PRESCALER);

    /* Display Startup Screen for fixed time */
    tftDLContentLoad(tft, &tftDL_startup);
    tftDLWrite(tft, &tftDL_startup);
    vTaskDelayUntil(&lastWakeTime, TFT_STARTUP_MS);

    /* Update Screen Info from CAN Indefinitely */
    while (
        vTaskDelayUntil(&lastWakeTime, tftUpdate_period_ms), 1
    ) {
        if(stateGetVSM() == CMR_CAN_ERROR){
            drawErrorScreen();
        } else {
            drawRTDScreen();
        }
    }
}

/**
 * @brief Draws the Display Updated List to the Screen
 *
 */
static void drawErrorScreen(void) {
    cmr_canRXMeta_t *metaVSMStatus = canRXMeta + CANRX_VSM_STATUS;
    volatile cmr_canVSMStatus_t *canVSMStatus =
        (void *) metaVSMStatus->payload;

    cmr_canRXMeta_t *metaHVCHeartbeat = canRXMeta + CANRX_HVC_HEARTBEAT;
    volatile cmr_canHVCHeartbeat_t *canHVCHeartbeat =
        (void *) metaHVCHeartbeat->payload;

    cmr_canRXMeta_t *metaCDCMotorFaults = canRXMeta + CANRX_CDC_MOTOR_FAULTS;
    volatile cmr_canCDCMotorFaults_t *canCDCMotorFaults =
        (void *) metaCDCMotorFaults->payload;

    tftDLContentLoad(&tft, &tftDL_error);

    tft_errors_t err;

    /* Timeouts */
    err.fsmTimeout = (canVSMStatus->moduleTimeoutMatrix & CMR_CAN_VSM_ERROR_SOURCE_FSM);
    err.cdcTimeout = (canVSMStatus->moduleTimeoutMatrix & CMR_CAN_VSM_ERROR_SOURCE_CDC);
    err.ptcTimeout = (canVSMStatus->moduleTimeoutMatrix & CMR_CAN_VSM_ERROR_SOURCE_PTC);
    err.vsmTimeout = 0;
    err.afc1Timeout = (canVSMStatus->moduleTimeoutMatrix & CMR_CAN_VSM_ERROR_SOURCE_AFC0);
    err.afc2Timeout = (canVSMStatus->moduleTimeoutMatrix & CMR_CAN_VSM_ERROR_SOURCE_AFC1);

    /* Latched Errors */
    err.imdError = (canVSMStatus->latchMatrix & CMR_CAN_VSM_LATCH_IMD);
    err.amsError = (canVSMStatus->latchMatrix & CMR_CAN_VSM_LATCH_AMS);
    err.bspdError = (canVSMStatus->latchMatrix & CMR_CAN_VSM_LATCH_BSPD);

    /* HVC Errors */
    err.overVolt = (canHVCHeartbeat->errorStatus & CMR_CAN_HVC_ERROR_PACK_OVERVOLT);
    err.underVolt = (canHVCHeartbeat->errorStatus & CMR_CAN_HVC_ERROR_PACK_UNDERVOLT);
    err.hvcoverTemp = (canHVCHeartbeat->errorStatus & CMR_CAN_HVC_ERROR_CELL_OVERTEMP);
    err.hvc_Error = (canHVCHeartbeat->errorStatus & CMR_CAN_HVC_ERROR_BMB_FAULT);
    err.hvcErrorNum = (canHVCHeartbeat->errorStatus);

    /* CDC Motor Faults */
    err.overSpeed = (canCDCMotorFaults->run & 1);
    err.mcoverTemp = (canCDCMotorFaults->run & (0x7f << 17));
    err.overCurrent = (canCDCMotorFaults->run & 2);
    err.mcError = (canCDCMotorFaults->run);
    err.mcErrorNum = (canCDCMotorFaults->run);

    /* Update Display List*/
    tftDL_errorUpdate(&err);

    /* Write Display List to Screen */
    tftDLWrite(&tft, &tftDL_error);
}

/**
 * @brief Draws the Display Updated List to the Screen
 *
 */
static void drawRTDScreen(void) {
    /* Setup the Required CAN info for Display */
    cmr_canRXMeta_t *metaCDLBroadcast = canRXMeta + CANRX_CDL_BROADCAST;

    cmr_canRXMeta_t *metaHVCPackVoltage = canRXMeta + CANRX_HVC_PACK_VOLTAGE;
    volatile cmr_canHVCPackVoltage_t *canHVCPackVoltage =
        (void *) metaHVCPackVoltage->payload;

    cmr_canRXMeta_t *metaCDCWheelSpeeds = canRXMeta + CANRX_CDC_WHEEL_SPEEDS;
    volatile cmr_canCDCWheelSpeeds_t *canCDCWheelSpeeds =
        (void *) metaCDCWheelSpeeds->payload;

    cmr_canRXMeta_t *metaCDCMotorData = canRXMeta + CANRX_CDC_MOTOR_DATA;
    volatile cmr_canCDCMotorData_t *canCDCMotorData =
        (void *) metaCDCMotorData->payload;

    cmr_canRXMeta_t *metaHVCPackTemps = canRXMeta + CANRX_HVC_PACK_TEMPS;
    volatile cmr_canHVCPackMinMaxCellTemps_t *canHVCPackTemps =
        (void *) metaHVCPackTemps->payload;

    cmr_canRXMeta_t *metaCDCMotorTemps = canRXMeta + CANRX_CDC_MOTOR_TEMPS;
    volatile cmr_canCDCMotorTemps_t *canCDCMotorTemps =
        (void *) metaCDCMotorTemps->payload;

    cmr_canRXMeta_t *metaPTCCoolingStatus = canRXMeta + CANRX_PTC_COOLING_STATUS;
    volatile cmr_canPTCCoolingStatus_t *canPTCCoolingStatus =
        (void *) metaPTCCoolingStatus->payload;

    tftDLContentLoad(&tft, &tftDL_RTD);

    /* Memorator present? */
    bool memorator_present = (cmr_canRXMetaTimeoutWarn(metaCDLBroadcast, xTaskGetTickCount()) < 0) ? false : true;

    /* Pack Voltage */
    int32_t hvVoltage_mV = canHVCPackVoltage->hvVoltage;

    /* Motor Power Draw*/
    int32_t power_kW =
        (canCDCMotorData->current_dA * canCDCMotorData->voltage_dV) /
        100000;

    /* Wheel Speed */
        /* Wheel Speed to Vehicle Speed Conversion
         *      Avg Front Wheel Speed * (1 rpm / 10 drpm) *
         *      (18" * PI) * (1' / 12") * (60min / 1hr) * (1 mi / 5280')
         *      = AvgWheelSpeed * 0.00535                                   */
        uint32_t wheelSpeed_drpm = (
            ((uint32_t) canCDCWheelSpeeds->frontLeft) +
            ((uint32_t) canCDCWheelSpeeds->frontRight)
        ) / 2;
        uint32_t speed_mph = (wheelSpeed_drpm * 535) / 100000;

    /* Accumulator Temperature */
    int32_t acTemp_C = (canHVCPackTemps->maxCellTemp_dC)/10;

    /* DCDC Temperature */
    int32_t num = 0;

    /* Motor Controller Temperature */
    int32_t mcTemp_C = (canPTCCoolingStatus->preRadiatorTemp_dC);
    /* Motor Temperature */
    int32_t motorTemp_C = (canCDCMotorTemps->motorTemp_dC) / 10;

    /* Update Display List*/
    tftDL_RTDUpdate(memorator_present, speed_mph, hvVoltage_mV, power_kW, num, motorTemp_C, acTemp_C, mcTemp_C);

    /* Write Display List to Screen */
    tftDLWrite(&tft, &tftDL_RTD);
}

/**
 * @brief Initializes the display.
 */
void tftInit(void) {
    const QSPI_InitTypeDef qspiInit = {
        .ClockPrescaler = TFT_INIT_QSPI_PRESCALER,
        .FifoThreshold = 4,
        .SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE,
        .FlashSize = 23,
        .ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE,
        .ClockMode = QSPI_CLOCK_MODE_0,
        .FlashID = QSPI_FLASH_ID_2,
        .DualFlash = QSPI_DUALFLASH_DISABLE
    };

    const cmr_qspiPinConfig_t pins = {
        .io = {
            { .port = GPIOA, .pin = GPIO_PIN_6 },
            { .port = GPIOA, .pin = GPIO_PIN_7 },
            { .port = GPIOC, .pin = GPIO_PIN_5 },
            { .port = GPIOC, .pin = GPIO_PIN_5 }
        },
        .sck = { .port = GPIOB, .pin = GPIO_PIN_1 },
        .nss = { .port = GPIOC, .pin = GPIO_PIN_11 }
    };

    cmr_qspiInit(
        &tft.qspi, QUADSPI, &qspiInit, &pins,
        DMA2_Stream7, DMA_CHANNEL_3
    );

    tft.inited = false;

    cmr_taskInit(
        &tftUpdate_task, "tftUpdate", tftUpdate_priority,
        tftUpdate, &tft
    );
}

