/**
 * @file tft.c
 * @brief TFT display implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "tft.h"  // Interface to implement

#include <CMR/tasks.h>  // Task interface
#include <math.h>       // sqrt
#include <stdbool.h>    // bool

#include "adc.h"
#include "can.h"         // Board-specific CAN interface
#include "tftContent.h"  // Content interface
#include "tftDL.h"       // Display list interface
#include "CMR/can_types.h" //can_types
#include "state.h"    //New State Machine

/** @brief Expected chip ID. */
#define TFT_CHIP_ID 0x00011308

/** @brief Display initialization QuadSPI prescaler. */
#define TFT_INIT_QSPI_PRESCALER 32

/** @brief Display initialization time, in milliseconds. */
#define TFT_INIT_MS 400

/** @brief Display QuadSPI prescaler */
#define TFT_QSPI_PRESCALER 2

/** @brief Flag for indicating a write to the display. */
#define TFT_WRITE_FLAG (1 << 23)

/** @brief Dummy cycles for reading data from the display. */
#define TFT_READ_DUMMY_CYCLES 8

/** @brief The display. */
tft_t tft;

void drawErrorScreen(void);
void drawRTDScreen(void);
void drawConfigScreen(void);
void drawSafetyScreen(void);

/*Prev HVC errors to latch on display*/
static bool prevOverVolt = false;
static bool prevUnderVolt = false;
static bool prevOverTemp = false;

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
        .Instruction = 0, // Not used
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
    uint32_t space;
    tftRead(tft, TFT_ADDR_CMDB_SPACE, sizeof(space), &space);
    if (len < space) {
        tftWrite(tft, TFT_ADDR_CMDB_WRITE, len, dataBuf);
        return;
    }

    while (written < len) {
        // Calculate length to write.
        size_t wrLen = len - written;
        size_t remLen = tftCoCmdRemLen(tft);

        if (wait) {
            // If no space, wait until enough space is available
            while (remLen == 0) {
                remLen = tftCoCmdRemLen(tft);
            }
        } else {
            // Ensure there's enough space for the full command
            while (remLen < wrLen) {
                remLen = tftCoCmdRemLen(tft);
            }
        }

        tftWrite(
            tft, TFT_ADDR_RAM_CMD + tft->coCmdWr,
            wrLen, dataBuf + written);

        // Round-up to word-aligned length.
        wrLen = (wrLen + sizeof(uint32_t)-1) & ~(sizeof(uint32_t)-1);
        written += wrLen;

        // Update the command write address.
        tft->coCmdWr += wrLen;
        if (tft->coCmdWr >= TFT_RAM_CMD_SIZE) {
            tft->coCmdWr -= TFT_RAM_CMD_SIZE;
        }
        tftWrite(tft, TFT_ADDR_CMD_WRITE, sizeof(tft->coCmdWr), &tft->coCmdWr);

        if (!wait) {
            // No waiting; we must have written the whole buffer.
            configASSERT(written == wrLen);
            break;
        }

        // Wait for the command to finish.
        do {
            tftRead(tft, TFT_ADDR_CMD_READ, sizeof(tft->coCmdRd), &tft->coCmdRd);
        } while (tft->coCmdRd != tft->coCmdWr);
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
void tftUpdate(void *pvParameters) {
    (void) pvParameters;
    /** @brief Represents a display initialization value. */
    typedef struct {
        tftAddr_t addr; /**< @brief Address to initialize. */
        uint16_t val;   /**< @brief Value to use. */
    } tftInit_t;

    /** @brief Display register initialization values. */
	const tftInit_t tftInits[] = {
        { .addr = TFT_ADDR_HCYCLE, .val = 480 },
        { .addr = TFT_ADDR_HOFFSET, .val = 43 },
        { .addr = TFT_ADDR_HSYNC0, .val = 0 },
        { .addr = TFT_ADDR_HSYNC1, .val = 41 },
        { .addr = TFT_ADDR_VCYCLE, .val = 292 },
        { .addr = TFT_ADDR_VOFFSET, .val = 12 },
        { .addr = TFT_ADDR_VSYNC0, .val = 0 },
        { .addr = TFT_ADDR_VSYNC1, .val = 10 },
        { .addr = TFT_ADDR_SWIZZLE, .val = 0 },
        { .addr = TFT_ADDR_DITHER, .val = 1 },
        { .addr = TFT_ADDR_PCLK_POL, .val = 1 },
        { .addr = TFT_ADDR_CSPREAD, .val = 1 },
        { .addr = TFT_ADDR_HSIZE, .val = 480 },
        { .addr = TFT_ADDR_VSIZE, .val = 548 },
        { .addr = TFT_ADDR_GPIOX_DIR, .val = (1 << 15) },
        { .addr = TFT_ADDR_GPIOX, .val = (1 << 15) },
        { .addr = TFT_ADDR_PCLK, .val = 5 }
    };


	TickType_t lastWakeTime = xTaskGetTickCount();
    /* Wait for display to initialize. */
    vTaskDelayUntil(&lastWakeTime, TFT_INIT_MS);

    /* Ensure that Chip ID is read correctly */
    uint32_t chipID;
    tftRead(&tft, TFT_ADDR_CHIP_ID, sizeof(chipID), &chipID);
    configASSERT(chipID == TFT_CHIP_ID);

    /* Init Sequence*/
    tftInitSequence();

    /* Initialize Video Registers. */
    size_t tftInitsLen = sizeof(tftInits) / sizeof(tftInits[0]);
    for (size_t i = 0; i < tftInitsLen; i++) {
        const tftInit_t *init = &tftInits[i];
        tftWrite(&tft, init->addr, sizeof(init->val), &init->val);
    }


    tft.inited = true;

    /* Enable Faster Clock Rate now that initialization is complete */
    cmr_qspiSetPrescaler(&tft.qspi, TFT_QSPI_PRESCALER);

    /* Display Startup Screen for fixed time */
    tftDLContentLoad(&tft, &tftDL_startup);
    tftDLWrite(&tft, &tftDL_startup);
    //    vTaskDelayUntil(&lastWakeTime, TFT_STARTUP_MS); //TODO: Uncomment

    /* Update Screen Info from CAN Indefinitely
    while (1) {
        vTaskDelayUntil(&lastWakeTime, TFT_UPDATE_PERIOD_MS);
        if (inConfigScreen()) {
            drawConfigScreen();
        } else if ((stateGetVSMReq() == CMR_CAN_HV_EN) && (stateGetVSM() == CMR_CAN_ERROR)) {
            drawSafetyScreen();
        } else if (stateGetVSM() == CMR_CAN_ERROR) {
            drawErrorScreen();
        } else {
        	//reset latching errors for ams as shown on screen
        	prevOverVolt = false;
        	prevUnderVolt = false;
        	prevOverTemp = false;
            // within drawRTDScreen, we decide if to draw testing or racing screen
            drawRTDScreen();
        }
    }
    */
}

void tftInitSequence() {
    tftCmd(&tft, TFT_CMD_CLKEXT, 0x00);
    tftCmd(&tft, TFT_CMD_ACTIVE, 0x00);
    vTaskDelay(300);
}

/**
 * @brief Draws the Display Updated List to the Screen
 *
 */
void drawConfigScreen(void) {
    tftDLContentLoad(&tft, &tftDL_config);
    tftDL_configUpdate();
    tftDLWrite(&tft, &tftDL_config);
}

/**
 * @brief Draws the Safety Circuit to the Screen
 *
 */
void drawSafetyScreen(void) {
    tftDLContentLoad(&tft, &tftDL_safety_screen);
    tftDLWrite(&tft, &tftDL_safety_screen);
}

 /** @brief Draws the motor, ac, cooling, etc. temps to the Screen
 *
 */
void drawRacingScreen(void){
	//TODO: Figure out what the heck this is
	uint32_t hvSoC = 0;
    tftDL_racingScreenUpdate(
        getMaxMotorTemp(),
        getACTemp(),
        getMCTemp(),
        hvSoC,
        DRSOpen());
    // /* Write Display List to Screen */
    tftDLWrite(&tft, &tftDL_racing_screen);
}

/**
 * @brief Draws the Display Updated List to the Screen
 *
 */
void drawErrorScreen(void) {
    volatile cmr_canVSMStatus_t *canVSMStatus = getPayload(CANRX_VSM_STATUS);

    volatile cmr_canHVCHeartbeat_t *canHVCHeartbeat = getPayload(CANRX_HVC_HEARTBEAT);

    volatile cmr_canAMKActualValues2_t *amkFLActualValues2 = getPayload(CANRX_AMK_FL_ACT_2);
    volatile cmr_canAMKActualValues2_t *amkFRActualValues2 = getPayload(CANRX_AMK_FR_ACT_2);
    volatile cmr_canAMKActualValues2_t *amkBLActualValues2 = getPayload(CANRX_AMK_RL_ACT_2);
    volatile cmr_canAMKActualValues2_t *amkBRActualValues2 = getPayload(CANRX_AMK_RR_ACT_2);
    volatile cmr_canBMSLowVoltage_t *canBMSLowVoltageStatus = getPayload(CANRX_HVC_LOW_VOLTAGE);

    tftDLContentLoad(&tft, &tftDL_error);

    tft_errors_t err;


    unsigned int voltage_mV = ((unsigned int) canBMSLowVoltageStatus->vbatt_mV) * 133u;
    //unsigned int voltage_mV = cmr_sensorListGetValue(&sensorList, SENSOR_CH_VOLTAGE_MV);
    err.glvVoltage_V = voltage_mV / 1000;
    err.glvLowVolt = voltage_mV < 20 * 1000;

    /* Timeouts */
    err.cdcTimeout = (canVSMStatus->moduleTimeoutMatrix & CMR_CAN_VSM_ERROR_SOURCE_CDC);
    err.ptcTimeout = (canVSMStatus->moduleTimeoutMatrix & CMR_CAN_VSM_ERROR_SOURCE_PTC);
    err.hvcTimeout = (canVSMStatus->moduleTimeoutMatrix & CMR_CAN_VSM_ERROR_SOURCE_HVC);
    err.vsmTimeout = 0;

    /* Latched Errors */
    err.imdError = (canVSMStatus->latchMatrix & CMR_CAN_VSM_LATCH_IMD);
    err.amsError = (canVSMStatus->latchMatrix & CMR_CAN_VSM_LATCH_AMS);
    err.bspdError = (canVSMStatus->latchMatrix & CMR_CAN_VSM_LATCH_BSPD);

    /* HVC Errors */
    /* Latch errors so we know what the issue is after AMS fault*/
    err.overVolt = (canHVCHeartbeat->errorStatus & (CMR_CAN_HVC_ERROR_CELL_OVERVOLT)) | prevOverVolt;
    if(err.overVolt) {
    	prevOverVolt = true;
    }
    err.underVolt = (canHVCHeartbeat->errorStatus & (CMR_CAN_HVC_ERROR_CELL_UNDERVOLT)) | prevUnderVolt;
    if(err.underVolt) {
		prevUnderVolt = true;
	}
    err.hvcoverTemp = (canHVCHeartbeat->errorStatus & (CMR_CAN_HVC_ERROR_CELL_OVERTEMP))  | prevOverTemp;
    if(err.hvcoverTemp) {
		prevOverTemp = true;
	}
    err.hvcBMBTimeout = (canHVCHeartbeat->errorStatus & CMR_CAN_HVC_ERROR_BMB_TIMEOUT);
    err.hvcBMBFault = (canHVCHeartbeat->errorStatus & CMR_CAN_HVC_ERROR_BMB_FAULT);
    err.hvcErrorNum = (canHVCHeartbeat->errorStatus);

    /* CDC Motor Faults */
    err.amkFLErrorCode = amkFLActualValues2->errorCode;
    err.amkFRErrorCode = amkFRActualValues2->errorCode;
    err.amkBLErrorCode = amkBLActualValues2->errorCode;
    err.amkBRErrorCode = amkBRActualValues2->errorCode;
    volatile cmr_canHVCBMBErrors_t *BMBerr = (volatile cmr_canHVCBMBErrors_t *)getPayload(CANRX_HVC_BMB_STATUS);

    /* Update Display List*/
    tftDL_errorUpdate(&err, BMBerr);

    /* Write Display List to Screen */
    tftDLWrite(&tft, &tftDL_error);
}





/**
 * @brief Draws the Display Updated List to the Screen
 *
 */
/**
 * @brief Draws the Display Updated List to the Screen
 *
 */
void drawRTDScreen(void){
    /* Setup the Required CAN info for Display */
    cmr_canRXMeta_t *metaMemoratorBroadcast = canRXMeta + CANRX_MEMORATOR_BROADCAST;


    volatile cmr_canHVCPackVoltage_t *canHVCPackVoltage = getPayload(CANRX_HVC_PACK_VOLTAGE);

    volatile cmr_canHVCPackMinMaxCellTemps_t *canHVCPackTemps = getPayload(CANRX_HVC_PACK_TEMPS);

    volatile cmr_canEMDMeasurements_t *canEMDvalues = getPayload(CANRX_EMD_VALUES);

    volatile cmr_canBMSLowVoltage_t *canBMSLowVoltageStatus = getPayload(CANRX_HVC_LOW_VOLTAGE);

    tftDLContentLoad(&tft, &tftDL_RTD);

    /* Memorator present? */
    // Wait to update if hasn't seen in 2 sec (2000 ms)
    memorator_status_t memoratorStatus = MEMORATOR_NOT_CONNECTED;
    volatile cmr_canHeartbeat_t *cdcHeartbeat = getPayload(CANRX_CDC_HEARTBEAT);
    if ((*(uint16_t *)(cdcHeartbeat->warning) & CMR_CAN_WARN_CDC_MEMORATOR_DAQ_TIMEOUT) != 0) {
        memoratorStatus = MEMORATOR_NOT_CONNECTED;
    }
    if (cmr_canRXMetaTimeoutWarn(metaMemoratorBroadcast, xTaskGetTickCount()) == 0) {
        memoratorStatus = MEMORATOR_CONNECTED_BAD_STATE;
        volatile cmr_canMemoratorHeartbeat_t *memoratorHeartbeat = (void *)metaMemoratorBroadcast->payload;
        if (memoratorHeartbeat->state == 0xA3) {
            memoratorStatus = MEMORATOR_CONNECTED_STATE_OK;
        }
    }

    /* GPS present? */
    // Checks broadcast from CDC to see status of SBG
    cmr_canRXMeta_t *metaSBGStatus = canRXMeta + CANRX_SBG_STATUS_3;
    // Check timeout
    bool sbgConnected = cmr_canRXMetaTimeoutWarn(metaSBGStatus, xTaskGetTickCount()) == 0;
    volatile cmr_canSBGStatus3_t *sbgPayload = (void *)metaSBGStatus->payload;
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


    float glvVoltage = ((float) canBMSLowVoltageStatus->vbatt_mV) * 2 / 15;
    //unsigned int voltage_mV = cmr_sensorListGetValue(&sensorList, SENSOR_CH_VOLTAGE_MV);
//    float glvVoltage = ((float)cmr_sensorListGetValue(&sensorList, SENSOR_CH_VOLTAGE_MV)) / 1000.0;

    volatile cmr_canVSMSensors_t *vsmSensors = (volatile cmr_canVSMSensors_t *)getPayload(CANRX_VSM_SENSORS);

    int32_t current_A = (int32_t)(vsmSensors->hallEffect_cA) / 100;
    int32_t hvVoltage_V = hvVoltage_mV / 1000;
    int32_t power_kW = (current_A * hvVoltage_V) / 1000;

    uint8_t speed_kmh = (uint8_t)getSpeedKmh();

    float odometer_km = getOdometer();

    volatile cmr_canBMSLowVoltage_t *bmsLV = (volatile cmr_canBMSLowVoltage_t *)getPayload(CANRX_HVC_LOW_VOLTAGE);

    // this is actually volts not mV but cant be bothered changing it :(
    // 18000mV * 15 / 2000 as sent by HVC = 135
    bool ssOk = (bmsLV->safety_mV > 135);

    /* Accumulator Temperature */
    int32_t acTemp_C = (canHVCPackTemps->maxCellTemp_dC) / 10;

    int32_t mcTemp_C = 0;
    int32_t motorTemp_C = 0;
    cornerId_t hottest_motor;

    /* Temperature warnings */
    bool motorTemp_yellow = motorTemp_C >= MOTOR_YELLOW_THRESHOLD;
    bool motorTemp_red = motorTemp_C >= MOTOR_RED_THRESHOLD;
    bool acTemp_yellow = acTemp_C >= AC_YELLOW_THRESHOLD;
    bool acTemp_red = acTemp_C >= AC_RED_THRESHOLD;
    bool mcTemp_yellow = mcTemp_C >= MC_YELLOW_THRESHOLD;
    bool mcTemp_red = mcTemp_C >= MC_RED_THRESHOLD;

    uint8_t glvSoC = getLVSoC(glvVoltage, LV_LIFEPO);

    uint8_t hvSoC = 0;

    volatile cmr_canCDCControlsStatus_t *controlsStatus = (volatile cmr_canCDCControlsStatus_t *)getPayload(CANRX_CDC_CONTROLS_STATUS);

    bool yrcOn = ((bool)controlsStatus->yrcOn) && (!(0 & 0x02));
    bool tcOn = ((bool)controlsStatus->tcOn) && (!(0 & 0x04));

    /* Write Display List to Screen */
    tftDLWrite(&tft, &tftDL_RTD);
}


/**
 * @brief Initializes the display.
 */
void tftInit(void){
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
            { .port = GPIOC, .pin = GPIO_PIN_4 },
            { .port = GPIOC, .pin = GPIO_PIN_5 },
            { .port = GPIOB, .pin = GPIO_PIN_0 },
            { .port = GPIOB, .pin = GPIO_PIN_10 } },
        .sck = { .port = GPIOB, .pin = GPIO_PIN_1 },
        .nss = { .port = GPIOC, .pin = GPIO_PIN_11 }
    };

    cmr_qspiInit(
        &tft.qspi, QUADSPI, &qspiInit, &pins,
        DMA2_Stream7, DMA_CHANNEL_3);

    tft.inited = false;

    cmr_taskInit(
        &tftUpdate_task, "tftUpdate", tftUpdate_priority,
        tftUpdate, &tft);
}
