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
#include "gpio.h"        // Board-specific GPIO interface
#include "state.h"       // State interface
#include "tftContent.h"  // Content interface
#include "tftDL.h"       // Display list interface

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

/*Prev HVC errors to latch on display*/
static bool prevOverVolt = false;
static bool prevUnderVolt = false;
static bool prevOverTemp = false;

//forward declarations
static void drawConfigScreen(void);
static void drawSafetyScreen(void);
static void drawErrorScreen(void);
static void drawRTDScreen(void);
static void drawRacingScreen(void);



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

void tftCoCmd(tft_t *tft, size_t len, const void *data) {
    // Bulk Write
    uint32_t space = -1;
    do {
        tftRead(tft, TFT_ADDR_CMDB_SPACE, sizeof(space), &space);
    } while (space < len);

    tftWrite(tft, TFT_ADDR_CMDB_WRITE, len, data);

    return;
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
        tftAddr_t addr; /**< @brief Address to initialize. */
        uint16_t val;   /**< @brief Value to use. */
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
        { .addr = TFT_ADDR_PCLK_POL, .val = 1 },
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
    cmr_gpioWrite(GPIO_PD_N, 0);  // TODO figure out pin
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
    // tftDLContentLoad(tft, &tftDL_startup);
    // tftDLWrite(tft, &tftDL_startup);
    //    vTaskDelayUntil(&lastWakeTime, TFT_STARTUP_MS); //TODO: Uncomment

    /* Update Screen Info from CAN Indefinitely */
    while (1) {
        vTaskDelayUntil(&lastWakeTime, TFT_UPDATE_PERIOD_MS);
        cmr_state state = getCurrState();
        switch(state){
            case NORMAL:
                drawRTDScreen();
                break;
            case CONFIG:
                drawConfigScreen();
                break;
            case RACING:
                drawSafetyScreen();
                break;
            case dimStateERROR:
                drawErrorScreen();
                break;
            default:
                prevOverVolt = false;
             	prevUnderVolt = false;
             	prevOverTemp = false;
                drawRTDScreen();
        }

        // if (true) {
        //     drawRTDScreen();
        // } else if ((stateGetVSMReq() == CMR_CAN_HV_EN) && (stateGetVSM() == CMR_CAN_ERROR)) {
        //     drawSafetyScreen();
        // } else if (stateGetVSM() == CMR_CAN_ERROR) {
        //     drawErrorScreen();
        // } else {
        // 	//reset latching errors for ams as shown on screen
        // 	prevOverVolt = false;
        // 	prevUnderVolt = false;
        // 	prevOverTemp = false;
        //     // within drawRTDScreen, we decide if to draw testing or racing screen
        //     drawRTDScreen();
        // }
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
 * @brief Draws the Safety Circuit to the Screen
 *
 */
static void drawSafetyScreen(void) {
    tftDLContentLoad(&tft, &tftDL_safety_screen);
    tftDLWrite(&tft, &tftDL_safety_screen);
}

/**
 * @brief Draws the Display Updated List to the Screen
 *
 */
static void drawErrorScreen(void) {
    cmr_canRXMeta_t *metaVSMStatus = canRXMeta + CANRX_VSM_STATUS;
    volatile cmr_canVSMStatus_t *canVSMStatus =
        (void *)metaVSMStatus->payload;

    cmr_canRXMeta_t *metaHVCHeartbeat = canRXMeta + CANRX_HVC_HEARTBEAT;
    volatile cmr_canHVCHeartbeat_t *canHVCHeartbeat =
        (void *)metaHVCHeartbeat->payload;

    cmr_canRXMeta_t *metaDTIFLTempFault = canRXMeta + CANRX_DTI_FL_TEMPFAULT;
    volatile cmr_canDTI_TX_TempFault_t *dtiFLTempFault =
        (void *)metaDTIFLTempFault->payload;

    cmr_canRXMeta_t *metaDTIFRTempFault = canRXMeta + CANRX_DTI_FR_TEMPFAULT;
    volatile cmr_canDTI_TX_TempFault_t *dtiFRTempFault =
        (void *)metaDTIFRTempFault->payload;

    cmr_canRXMeta_t *metaDTIRLTempFault = canRXMeta + CANRX_DTI_RL_TEMPFAULT;
    volatile cmr_canDTI_TX_TempFault_t *dtiRLTempFault =
        (void *)metaDTIRLTempFault->payload;

    cmr_canRXMeta_t *metaDTIRRTempFault = canRXMeta + CANRX_DTI_RR_TEMPFAULT;
    volatile cmr_canDTI_TX_TempFault_t *dtiRRTempFault =
        (void *)metaDTIRRTempFault->payload;

    cmr_canRXMeta_t *metaBMSLowVoltage = canRXMeta + CANRX_HVC_LOW_VOLTAGE;
    volatile cmr_canBMSLowVoltage_t *canBMSLowVoltageStatus =
        (void *)metaBMSLowVoltage->payload;

    cmr_canVSMPowerDiagnostics_t *vsmPowerDiagnostics = 
        (cmr_canVSMPowerDiagnostics_t *)getPayload(CANRX_VSM_POWER_DIAGNOSTICS); 

    tftDLContentLoad(&tft, &tftDL_error);

    tft_errors_t err;


    /* LVB */
    unsigned int voltage_mV = ((unsigned int) vsmPowerDiagnostics->busVoltage_mV);
    err.glvVoltage_V = voltage_mV / 1000;
    err.glvLowVolt = voltage_mV < (20 * 1000);

    /* Timeouts */
    err.cdcTimeout = (canVSMStatus->moduleTimeoutMatrix & CMR_CAN_VSM_BADSTATE_SOURCE_CDC);
    // err.ptcTimeout = (canVSMStatus->moduleTimeoutMatrix & CMR_CAN_VSM_BADSTATE_SOURCE_PTC);
    err.hvcTimeout = (canVSMStatus->moduleTimeoutMatrix & CMR_CAN_VSM_BADSTATE_SOURCE_HVC);
    err.vsmTimeout = 0;

    /* Latched Errors */
    err.imdError = (canVSMStatus->latchMatrix & CMR_CAN_VSM_LATCH_IMD);
    err.amsError = (canVSMStatus->latchMatrix & CMR_CAN_VSM_LATCH_AMS);
    err.bspdError = (canVSMStatus->latchMatrix & CMR_CAN_VSM_LATCH_BSPD);

    /* HVC Errors */
    /* Latch errors so we know what the issue is after AMS fault*/
    err.overVolt = (canHVCHeartbeat->errorStatus & (CMR_CAN_HVBMS_ERROR_CELL_OVERVOLT)) | prevOverVolt;
    if(err.overVolt) {
    	prevOverVolt = true;
    }
    err.underVolt = (canHVCHeartbeat->errorStatus & (CMR_CAN_HVBMS_ERROR_CELL_UNDERVOLT)) | prevUnderVolt;
    if(err.underVolt) {
		prevUnderVolt = true;
	}
    err.hvcoverTemp = (canHVCHeartbeat->errorStatus & (CMR_CAN_HVBMS_ERROR_CELL_OVERTEMP))  | prevOverTemp;
    if(err.hvcoverTemp) {
		prevOverTemp = true;
	}
    err.hvcBMBTimeout = (canHVCHeartbeat->errorStatus & CMR_CAN_HVBMS_ERROR_BMB_TIMEOUT);
    err.hvcBMBFault = (canHVCHeartbeat->errorStatus & CMR_CAN_HVC_ERROR_BMB_FAULT);
    err.hvcErrorNum = (canHVCHeartbeat->errorStatus);

    /* CDC Motor Faults */
    err.dtiFLErrorCode = dtiFLTempFault->fault_code;
    err.dtiFRErrorCode = dtiFRTempFault->fault_code;
    err.dtiRLErrorCode = dtiRLTempFault->fault_code;
    err.dtiRRErrorCode = dtiRRTempFault->fault_code;
    volatile cmr_canHVCBMBErrors_t *BMBerr = (volatile cmr_canHVCBMBErrors_t *)getPayload(CANRX_HVC_BMB_STATUS);

    /* Update Display List*/
    tftDL_errorUpdate(&err, BMBerr);

    /* Write Display List to Screen */
    tftDLWrite(&tft, &tftDL_error);
}

/**
 * @brief computes max of 4 numbers
 */
int16_t findMax(int16_t a, int16_t b, int16_t c, int16_t d, uint8_t *index) {
    int16_t maximum = a;
    *index = 0;
    if (b > maximum) {
        maximum = b;
        *index = 1;
    }
    if (c > maximum) {
        maximum = c;
        *index = 2;
    }
    if (d > maximum) {
        maximum = d;
        *index = 3;
    }
    return maximum;
}

/**
 * @brief Computes the total current of a motor
 * https://drive.google.com/file/d/1dyoIuW85M110q4x2OXapvWxm-WnFBys2/view pg76
 */
uint32_t computeCurrent_A(volatile cmr_canDTI_TX_IdIq_t *canDTI_IdIq) {
    int32_t Iq_A = (int32_t)(canDTI_IdIq->iq / 100);
    int32_t Id_A = (int32_t)(canDTI_IdIq->iq / 100);
    uint32_t Is_A = sqrt(Iq_A * Iq_A + Id_A * Id_A);
    return Is_A;
}


/**
 * @brief gets temps from motors and inverters
 * @returns mcTemp
 * @returns motorTemp
*/

static void getDTITemps(int32_t *mcTemp_C, int32_t *motorTemp_C, cornerId_t *hottest) {

    // If we're in GLV, we don't want temps to latch on their prev vals
	cmr_canState_t state = stateGetVSM();
    if (state == CMR_CAN_GLV_ON) {
        *mcTemp_C = 0;
        *motorTemp_C = 0;
        *hottest = NONE;
        return;
    }

    // Front Left
    cmr_canRXMeta_t *metaDTI_FL_TempFault = canRXMeta + CANRX_DTI_FL_TEMPFAULT;
    volatile cmr_canDTI_TX_TempFault_t *FL =
        (void *)metaDTI_FL_TempFault->payload;

    // Front Right
    cmr_canRXMeta_t *metaDTI_FR_TempFault = canRXMeta + CANRX_DTI_FR_TEMPFAULT;
    volatile cmr_canDTI_TX_TempFault_t *FR =
        (void *)metaDTI_FR_TempFault->payload;

    // Rear Left
    cmr_canRXMeta_t *metaDTI_RL_TempFault = canRXMeta + CANRX_DTI_RL_TEMPFAULT;
    volatile cmr_canDTI_TX_TempFault_t *RL =
        (void *)metaDTI_RL_TempFault->payload;

    // Rear Right
   cmr_canRXMeta_t *metaDTI_RR_TempFault = canRXMeta + CANRX_DTI_RR_TEMPFAULT;
    volatile cmr_canDTI_TX_TempFault_t *RR =
        (void *)metaDTI_RR_TempFault->payload;

    /* Motor Temperature */
    uint8_t hottest_motor_index = 0;
    *motorTemp_C = findMax(FL->motor_temp,
                                  FR->motor_temp,
                                  RL->motor_temp,
                                  RR->motor_temp,
                                  &hottest_motor_index) /
                          10;

    // provide hottest motor as corner type
    *hottest = (cornerId_t)(hottest_motor_index);

    uint8_t hottest_mc_index = 0;
    /* Motor Controller Temperature */
    *mcTemp_C = findMax(FL->ctlr_temp,
                               FR->ctlr_temp,
                               RL->ctlr_temp,
                               RR->ctlr_temp,
                               &hottest_mc_index) /
                       10;


}

/**
 * @brief Draws the Display Updated List to the Screen
 *
 */
static void drawRTDScreen(void) {
    /* Setup the Required CAN info for Display */
    cmr_canRXMeta_t *metaMemoratorBroadcast = canRXMeta + CANRX_MEMORATOR_BROADCAST;

    cmr_canRXMeta_t *metaCDCHeartbeat = canRXMeta + CANRX_CDC_HEARTBEAT;

    cmr_canRXMeta_t *metaHVCPackVoltage = canRXMeta + CANRX_HVC_PACK_VOLTAGE;
    volatile cmr_canHVCPackVoltage_t *canHVCPackVoltage =
        (void *)metaHVCPackVoltage->payload;

    cmr_canRXMeta_t *metaHVCPackTemps = canRXMeta + CANRX_HVC_PACK_TEMPS;
    volatile cmr_canBMSMinMaxCellTemperature_t *canHVCPackTemps =
        (void *)metaHVCPackTemps->payload;

    cmr_canRXMeta_t *metaEMDvalues = canRXMeta + CANRX_EMD_VALUES;
    volatile cmr_canEMDMeasurements_t *canEMDvalues =
        (void *)metaEMDvalues->payload;



    cmr_canRXMeta_t *metaBMSLowVoltage = canRXMeta + CANRX_HVC_LOW_VOLTAGE;
    volatile cmr_canBMSLowVoltage_t *canBMSLowVoltageStatus =
        (void *)metaBMSLowVoltage->payload;

    tftDLContentLoad(&tft, &tftDL_RTD);

    /* Memorator present? */
    // Wait to update if hasn't seen in 2 sec (2000 ms)
    memorator_status_t memoratorStatus = MEMORATOR_NOT_CONNECTED;
    volatile cmr_canHeartbeat_t *cdcHeartbeat = (cmr_canHeartbeat_t *)metaCDCHeartbeat->payload;
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
    cmr_canRXMeta_t *metaMovellaStatus = canRXMeta + CANRX_MOVELLA_STATUS;
    // Check timeout
    bool movellaConnected = cmr_canRXMetaTimeoutWarn(metaMovellaStatus, xTaskGetTickCount()) == 0;
    volatile cmr_canMovellaStatus_t *movellaPayload = (void *)metaMovellaStatus->payload;
    uint8_t movellaStatus = 0;
    if (movellaConnected) {
        movellaStatus = movellaPayload->gnss_fix;
    }
    else {
        movellaStatus = 0;
    }

    /* Pack Voltage */
    int32_t hvVoltage_mV = canHVCPackVoltage->battVoltage_mV;

    volatile cmr_canVSMPowerDiagnostics_t *vsmPowerDiagnostics = (volatile cmr_canVSMPowerDiagnostics_t *)getPayload(CANRX_VSM_POWER_DIAGNOSTICS); 
    float glvVoltage = ((float) vsmPowerDiagnostics->busVoltage_mV)/1000; 
    //unsigned int voltage_mV = cmr_sensorListGetValue(&sensorList, SENSOR_CH_VOLTAGE_MV);
//    float glvVoltage = ((float)cmr_sensorListGetValue(&sensorList, SENSOR_CH_VOLTAGE_MV)) / 1000.0;

    volatile cmr_canVSMSensors_t *vsmSensors = (volatile cmr_canVSMSensors_t *)getPayload(CANRX_VSM_SENSORS);

    int32_t current_A = (int32_t)(vsmSensors->hallEffect_cA) / 100;
    int32_t hvVoltage_V = hvVoltage_mV / 1000;
    int32_t power_kW = (current_A * hvVoltage_V) / 1000;

    uint8_t speed_kmh = getSpeedKmh();

    float odometer_km = getOdometer();

    volatile cmr_canBMSLowVoltage_t *bmsLV = (volatile cmr_canBMSLowVoltage_t *)getPayload(CANRX_HVC_LOW_VOLTAGE);

    // this is actually volts not mV but cant be bothered changing it :(
    // 18000mV / 250 as sent by HVC = 72
    bool ssOk = (bmsLV->safety_qV > 72);

    volatile cmr_canCDCDRSStates_t *drsState = (volatile cmr_canCDCDRSStates_t *)getPayload(CANRX_DRS_STATE);
    bool drsOpen = (drsState->state == CMR_CAN_DRS_STATE_OPEN);

    /* Accumulator Temperature */
    int32_t acTemp_C = (canHVCPackTemps->maxCellTemp_C) / 10;

    int32_t mcTemp_C, motorTemp_C = 0;
    cornerId_t hottest_motor;

    getDTITemps(&mcTemp_C, &motorTemp_C, &hottest_motor);

    /* Temperature warnings */
    bool motorTemp_yellow = motorTemp_C >= MOTOR_YELLOW_THRESHOLD;
    bool motorTemp_red = motorTemp_C >= MOTOR_RED_THRESHOLD;
    bool acTemp_yellow = acTemp_C >= AC_YELLOW_THRESHOLD;
    bool acTemp_red = acTemp_C >= AC_RED_THRESHOLD;
    bool mcTemp_yellow = mcTemp_C >= MC_YELLOW_THRESHOLD;
    bool mcTemp_red = mcTemp_C >= MC_RED_THRESHOLD;

    uint8_t glvSoC = getLVSoC(glvVoltage);

    uint8_t hvSoC = 0;

    volatile cmr_canCDCControlsStatus_t *controlsStatus = (volatile cmr_canCDCControlsStatus_t *)getPayload(CANRX_CDC_CONTROLS_STATUS);

    bool yrcOn = true;
    bool tcOn = true;
    //bool yrcOn = ((bool)controlsStatus->yrcOn) && (!(switchValues & 0x02));
    //bool tcOn = ((bool)controlsStatus->tcOn) && (!(switchValues & 0x04));

    if (getCurrState() == RACING) {
        tftDL_racingScreenUpdate(
            motorTemp_C,
            acTemp_C,
            mcTemp_C,
            hvSoC,
            drsOpen);
        // /* Write Display List to Screen */
        tftDLWrite(&tft, &tftDL_racing_screen);
    } else {
        /* Update Display List*/
        tftDL_RTDUpdate(memoratorStatus,
                        movellaStatus,
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
                        (int32_t)glvVoltage,
                        glvSoC,
                        hvSoC,
                        yrcOn,
                        tcOn,
                        ssOk,
                        odometer_km,
                        drsOpen,
                        hottest_motor);

        /* Write Display List to Screen */
        tftDLWrite(&tft, &tftDL_RTD);
    }
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
            { .port = GPIOC, .pin = GPIO_PIN_5 } },
        .sck = { .port = GPIOB, .pin = GPIO_PIN_1 },
        .nss = { .port = GPIOC, .pin = GPIO_PIN_11 }
    };

    cmr_qspiInit(
        &tft.qspi, QUADSPI, &qspiInit, &pins,
        DMA2_Stream7, DMA_CHANNEL_3);

    tft.inited = false;
    dim_first_time_config_screen = true;

    cmr_taskInit(
        &tftUpdate_task, "tftUpdate", tftUpdate_priority,
        tftUpdate, &tft);
}