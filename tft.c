/**
 * @file tft.c
 * @brief TFT display implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/tasks.h>      // Task interface

#include <stdbool.h>        // bool
#include <math.h>           // sqrt

#include "tft.h"            // Interface to implement
#include "tftPrivate.h"     // Private interface
#include "tftContent.h"     // Content interface
#include "tftDL.h"          // Display list interface
#include "gpio.h"           // Board-specific GPIO interface
#include "can.h"            // Board-specific CAN interface
#include "state.h"          // State interface
#include "adc.h"

/** @brief Expected chip ID. */
#define TFT_CHIP_ID 0x00011308

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
static void drawConfigScreen(void);

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
        { .addr = TFT_ADDR_HCYCLE, .val = 928 },
        { .addr = TFT_ADDR_HOFFSET, .val = 88 },
        { .addr = TFT_ADDR_HSYNC0, .val = 0 },
        { .addr = TFT_ADDR_HSYNC1, .val = 48 },
        { .addr = TFT_ADDR_VCYCLE, .val = 525 },
        { .addr = TFT_ADDR_VOFFSET, .val = 32 },
        { .addr = TFT_ADDR_VSYNC0, .val = 0 },
        { .addr = TFT_ADDR_VSYNC1, .val = 3 },
        { .addr = TFT_ADDR_SWIZZLE, .val = 0 },
        { .addr = TFT_ADDR_DITHER, .val = 1 },
        { .addr = TFT_ADDR_PCLK_POL, .val = 0 },
        { .addr = TFT_ADDR_CSPREAD, .val = 1 },
        { .addr = TFT_ADDR_HSIZE, .val = 800 },
        { .addr = TFT_ADDR_VSIZE, .val = 480 },
        { .addr = TFT_ADDR_GPIOX_DIR, .val = (1 << 15) },
        { .addr = TFT_ADDR_GPIOX, .val = (1 << 15) },
        { .addr = TFT_ADDR_PCLK, .val = 2 }
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
//    vTaskDelayUntil(&lastWakeTime, TFT_STARTUP_MS); //TODO: Uncomment

    /* Update Screen Info from CAN Indefinitely */
    while (
        vTaskDelayUntil(&lastWakeTime, TFT_UPDATE_PERIOD_MS), 1
    ) {
        if (inConfigScreen()){
            drawConfigScreen();
        }
        else if (stateGetVSM() == CMR_CAN_ERROR){
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
static void drawConfigScreen(void) {
    tftDLContentLoad(&tft, &tftDL_config);
    tftDL_configUpdate();
    tftDLWrite(&tft, &tftDL_config);
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

    /* Low Voltage */
    unsigned int voltage_mV = adcRead(ADC_VSENSE) * 8 * 11 / 10;
    err.glvVoltage_V =  voltage_mV / 1000;
    err.glvLowVolt = voltage_mV < 20*1000;

    /* Timeouts */
    err.fsmTimeout = (canVSMStatus->moduleTimeoutMatrix & CMR_CAN_VSM_ERROR_SOURCE_FSM);
    err.cdcTimeout = (canVSMStatus->moduleTimeoutMatrix & CMR_CAN_VSM_ERROR_SOURCE_CDC);
    err.ptcTimeout = (canVSMStatus->moduleTimeoutMatrix & CMR_CAN_VSM_ERROR_SOURCE_PTC);
    err.apcTimeout = (canVSMStatus->moduleTimeoutMatrix & CMR_CAN_VSM_ERROR_SOURCE_APC);
    err.hvcTimeout = (canVSMStatus->moduleTimeoutMatrix & CMR_CAN_VSM_ERROR_SOURCE_HVC);
    err.vsmTimeout = 0;

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
 * @brief computes max of 4 numbers
 */
int16_t findMax(int16_t a, int16_t b, int16_t c, int16_t d) {
    int16_t maximum = a;
    if (b > maximum) {
        maximum = b;
    }
    if (c > maximum) {
        maximum = c;
    }
    if (d > maximum) {
        maximum = d;
    }
    return maximum;
}

/**
 * @brief Computes the total current of a motor
 * https://drive.google.com/file/d/1dyoIuW85M110q4x2OXapvWxm-WnFBys2/view pg76
 */
uint32_t computeCurrent_A(volatile cmr_canAMKActualValues1_t *canAMK_Act1) {
    int32_t Iq_A = (int32_t) canAMK_Act1->torqueCurrent_raw;
    int32_t Id_A = (int32_t) canAMK_Act1->magCurrent_raw;
    uint32_t Is_A = sqrt(Iq_A*Iq_A + Id_A*Id_A);
    return Is_A;
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

    cmr_canRXMeta_t *metaHVCPackTemps = canRXMeta + CANRX_HVC_PACK_TEMPS;
    volatile cmr_canHVCPackMinMaxCellTemps_t *canHVCPackTemps =
        (void *) metaHVCPackTemps->payload;

    cmr_canRXMeta_t *metaEMDvalues = canRXMeta + CANRX_EMD_VALUES;
    volatile cmr_canEMDMeasurements_t *canEMDvalues =
    	(void *) metaEMDvalues->payload;

    // PTC Temps
    /* cmr_canRXMeta_t *metaPTCfLoopA = canRXMeta + CANRX_PTCf_LOOP_A_TEMPS;
    volatile cmr_canPTCfLoopTemp_A_t *canPTCfLoopTemp_A = (void *) metaPTCfLoopA->payload;
    
    cmr_canRXMeta_t *metaPTCfLoopB = canRXMeta + CANRX_PTCf_LOOP_B_TEMPS;
    volatile cmr_canPTCfLoopTemp_B_t *canPTCfLoopTemp_B = (void *) metaPTCfLoopB->payload;
    
    cmr_canRXMeta_t *metaPTCpLoopA = canRXMeta + CANRX_PTCp_LOOP_A_TEMPS;
    volatile cmr_canPTCpLoopTemp_A_t *canPTCpLoopTemp_A = (void *) metaPTCpLoopA->payload;
    
    cmr_canRXMeta_t *metaPTCpLoopB = canRXMeta + CANRX_PTCp_LOOP_B_TEMPS;
    volatile cmr_canPTCpLoopTemp_B_t *canPTCpLoopTemp_B = (void *) metaPTCpLoopB->payload;*/

    // AMK Inverter
    // Front Left
    cmr_canRXMeta_t *metaAMK_FL_Act1 = canRXMeta + CANRX_AMK_FL_ACT_1;
    volatile cmr_canAMKActualValues1_t *canAMK_FL_Act1 =
        (void *) metaAMK_FL_Act1->payload;
    cmr_canRXMeta_t *metaAMK_FL_Act2 = canRXMeta + CANRX_AMK_FL_ACT_2;
    volatile cmr_canAMKActualValues2_t *canAMK_FL_Act2 =
        (void *) metaAMK_FL_Act2->payload;
    // Front Right
    cmr_canRXMeta_t *metaAMK_FR_Act1 = canRXMeta + CANRX_AMK_FR_ACT_1;
    volatile cmr_canAMKActualValues1_t *canAMK_FR_Act1 =
        (void *) metaAMK_FR_Act1->payload;
    cmr_canRXMeta_t *metaAMK_FR_Act2 = canRXMeta + CANRX_AMK_FR_ACT_2;
    volatile cmr_canAMKActualValues2_t *canAMK_FR_Act2 =
        (void *) metaAMK_FR_Act2->payload;
    // Rear Left
    cmr_canRXMeta_t *metaAMK_RL_Act1 = canRXMeta + CANRX_AMK_RL_ACT_1;
    volatile cmr_canAMKActualValues1_t *canAMK_RL_Act1 =
        (void *) metaAMK_RL_Act1->payload;
    cmr_canRXMeta_t *metaAMK_RL_Act2 = canRXMeta + CANRX_AMK_RL_ACT_2;
        volatile cmr_canAMKActualValues2_t *canAMK_RL_Act2 =
            (void *) metaAMK_RL_Act2->payload;
    // Rear Right
    cmr_canRXMeta_t *metaAMK_RR_Act1 = canRXMeta + CANRX_AMK_RR_ACT_1;
        volatile cmr_canAMKActualValues1_t *canAMK_RR_Act1 =
            (void *) metaAMK_RR_Act1->payload;
    cmr_canRXMeta_t *metaAMK_RR_Act2 = canRXMeta + CANRX_AMK_RR_ACT_2;
        volatile cmr_canAMKActualValues2_t *canAMK_RR_Act2 =
            (void *) metaAMK_RR_Act2->payload;

    tftDLContentLoad(&tft, &tftDL_RTD);

    /* Memorator present? */
    // Wait to update if hasn't seen in 2 sec (2000 ms)
    bool memoratorPresent = cmr_canRXMetaTimeoutWarn(metaCDLBroadcast, xTaskGetTickCount()) == 0;

    /* GPS present? */
    // Checks broadcast from CDC to see status of SBG
    cmr_canRXMeta_t *metaSBGStatus = canRXMeta + CANRX_SBG_STATUS_3;
    // Check timeout
    bool sbgConnected = cmr_canRXMetaTimeoutWarn(metaSBGStatus, xTaskGetTickCount()) == 0;
    volatile cmr_canSBGStatus3_t *sbgPayload = (void *) metaSBGStatus->payload;
    SBG_status_t sbgStatus = SBG_STATUS_NOT_CONNECTED;
    if (sbgConnected) {
        sbgStatus = SBG_STATUS_WORKING_NO_POS_FOUND;
        uint32_t solutionStatus = sbgPayload->solution_status;
        // solution mode is first 4 bits of solution status
        uint32_t solutionStatusMode = solutionStatus & 0xF;
        // Get bits 4 through 7
        solutionStatus = solutionStatus & 0xF0;
        uint32_t solutionMask = CMR_CAN_SBG_SOL_ATTITUDE_VALID | CMR_CAN_SBG_SOL_HEADING_VALID | CMR_CAN_SBG_SOL_VELOCITY_VALID | CMR_CAN_SBG_SOL_POSITION_VALID;
        if (solutionStatusMode == CMR_CAN_SBG_SOL_MODE_NAV_POSITION && solutionStatus == solutionMask) {
            // Got fix on position
            sbgStatus = SBG_STATUS_WORKING_POS_FOUND;
        }
    }



    /* Pack Voltage */
  int32_t hvVoltage_mV = canHVCPackVoltage->battVoltage_mV;
    //  float hvVoltage_mV_f = canEmdHvVoltage(*canEMDvalues) * 1000.0f;
    //  int32_t hvVoltage_mV = (int32_t) hvVoltage_mV_f;

    // value * 0.8 (mV per bit) * 11 (1:11 voltage divider)
    int32_t glvVoltage = adcRead(ADC_VSENSE) * 8 * 11 / 10 / 1000;

    /* Motor Power Draw*/
//   int33_t current_A = computeCurrent_A(canAMK_FL_Act1) +
//                       computeCurrent_A(canAMK_FR_Act2) +
//                       computeCurrent_A(canAMK_RL_Act2) +
//                       computeCurrent_A(canAMK_RR_Act2);
    int32_t current_A = 0;

    int32_t power_kW = (current_A * (hvVoltage_mV / 1000)) / 1000;

    uint8_t speed_kmh = (uint8_t)getSpeedKmh();

    float odometer_km = getOdometer();

    volatile cmr_canBMSLowVoltage_t *bmsLV = (volatile cmr_canBMSLowVoltage_t*)&(canRXMeta[CANRX_HVC_LOW_VOLTAGE]);
    bool ssOk = (bmsLV->safety_mV > 18);

    volatile cmr_canCDCDRSStates_t *drsState = (volatile cmr_canCDCDRSStates_t*)&(canRXMeta[CANRX_DRS_STATE]);
    bool drsClosed = (bool)drsState->state;

    /* Accumulator Temperature */
    int32_t acTemp_C = (canHVCPackTemps->maxCellTemp_dC)/10;

    /* Motor Controller Temperature */
    int32_t mcTemp_C = findMax(canAMK_FL_Act2->coldPlateTemp_dC,
                               canAMK_FR_Act2->coldPlateTemp_dC,
                               canAMK_RL_Act2->coldPlateTemp_dC,
                               canAMK_RR_Act2->coldPlateTemp_dC) / 10;

    /* Motor Temperature */
    int32_t motorTemp_C = findMax(canAMK_FL_Act2->motorTemp_dC,
                                  canAMK_FR_Act2->motorTemp_dC,
                                  canAMK_RL_Act2->motorTemp_dC,
                                  canAMK_RR_Act2->motorTemp_dC) / 10;

    /* Temperature warnings */
    bool motorTemp_yellow = motorTemp_C >= MOTOR_YELLOW_THRESHOLD;
    bool motorTemp_red = motorTemp_C >= MOTOR_RED_THRESHOLD;
    bool acTemp_yellow = acTemp_C >= AC_YELLOW_THRESHOLD;
    bool acTemp_red = acTemp_C >= AC_RED_THRESHOLD;
    bool mcTemp_yellow = mcTemp_C >= MC_YELLOW_THRESHOLD;
    bool mcTemp_red = mcTemp_C >= MC_RED_THRESHOLD;

    //TODO: get real vals
    uint8_t glvSoC = 80;
    uint8_t hvSoC = 35;
    bool yrcOn = false;
    bool tcOn = false;

    /* Update Display List*/
    tftDL_RTDUpdate(memoratorPresent, 
                    sbgStatus, 
                    hvVoltage_mV, 
                    power_kW,
                    speed_kmh,
                    motorTemp_yellow, 
                    motorTemp_red, 
                    acTemp_yellow, 
                    acTemp_red, 
                    mcTemp_yellow, 
                    mcTemp_red, 
                    motorTemp_C, 
                    acTemp_C, 
                    mcTemp_C, 
                    glvVoltage,
                    glvSoC,
                    hvSoC,
                    yrcOn,
                    tcOn,
                    ssOk,
					odometer_km,
                    drsClosed);

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
            { .port = GPIOC, .pin = GPIO_PIN_4 },
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

