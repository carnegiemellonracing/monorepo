/**
 * @file tft.c
 * @brief TFT display implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/qspi.h>   // QuadSPI interface
#include <CMR/tasks.h>  // Task interface

#include "tft.h"    // Interface to implement
#include "tftDL.h"  // Display lists
#include "gpio.h"   // Board-specific GPIO interface

/** @brief Expected chip ID. */
#define TFT_CHIP_ID 0x00011208

/** @brief Display reset time, in milliseconds. */
#define TFT_RESET_MS 50

/** @brief Display initialization time, in milliseconds. */
#define TFT_INIT_MS 400

/** @brief Display startup time, in milliseconds. */
#define TFT_STARTUP_MS 1000

/** @brief Flag for indicating a write to the display. */
#define TFT_WRITE_FLAG (1 << 23)

/** @brief Dummy cycles for reading data from the display. */
#define TFT_READ_DUMMY_CYCLES 8

/** @brief General purpose graphics RAM size, in bytes. */
#define TFT_RAM_G_SIZE (1024 * 1024)

/** @brief Display list RAM size, in bytes. */
#define TFT_RAM_DL_SIZE (8 * 1024)

/** @brief Coprocessor command buffer RAM size, in bytes. */
#define TFT_RAM_CMD_SIZE (4 * 1024)

/** @brief Represents a TFT display. */
typedef struct {
    cmr_qspi_t qspi;    /**< @brief The display's QuadSPI port. */
    uint16_t cmdWrite;  /**< @brief Command write address. */
} tft_t;

/** @brief Represents a display command. */
typedef enum {
    TFT_CMD_ACTIVE = 0x00,  /**< @brief Enter "ACTIVE" mode (send twice). */
    TFT_CMD_CLKEXT = 0x44   /**< @brief Use external clock. */
} tftCmd_t;

/** @brief Represents a display address. */
typedef enum {
    // Diagnostics.
    TFT_ADDR_CHIP_ID = 0x0C0000,    /**< @brief Chip identifier. */

    // Video parameters.
    TFT_ADDR_HCYCLE = 0x30202C,     /**< @brief Horizontal cycle time. */
    TFT_ADDR_HOFFSET = 0x302030,    /**< @brief Horizontal offset time. */
    TFT_ADDR_HSYNC0 = 0x302038,     /**< @brief Horizontal sync time 0. */
    TFT_ADDR_HSYNC1 = 0x30203C,     /**< @brief Horizontal sync time 1. */
    TFT_ADDR_VCYCLE = 0x302040,     /**< @brief Vertical cycle time. */
    TFT_ADDR_VOFFSET = 0x302044,    /**< @brief Vertical offset time. */
    TFT_ADDR_VSYNC0 = 0x30204C,     /**< @brief Vertical sync time 0. */
    TFT_ADDR_VSYNC1 = 0x302050,     /**< @brief Vertical sync time 1. */
    TFT_ADDR_SWIZZLE = 0x302064,    /**< @brief RGB signal swizzle. */
    TFT_ADDR_CSPREAD = 0x302068,    /**< @brief Clock spreading enable. */
    TFT_ADDR_HSIZE = 0x302034,      /**< @brief Horizontal pixel count. */
    TFT_ADDR_VSIZE = 0x302048,      /**< @brief Vertical pixel count. */

    // GPIO.
    TFT_ADDR_GPIOX_DIR = 0x302098,  /**< @brief GPIO directions. */
    TFT_ADDR_GPIOX = 0x30209C,      /**< @brief GPIO values. */

    // Clock configuration.
    TFT_ADDR_PCLK_POL = 0x30206C,   /**< @brief PCLK polarity. */
    TFT_ADDR_PCLK = 0x302070,       /**< @brief PCLK frequency divider. */

    // Coprocessor registers.
    TFT_ADDR_CMD_READ = 0x3020F8,   /**< @brief Coprocessor read pointer. */
    TFT_ADDR_CMD_WRITE = 0x3020FC,  /**< @brief Coprocessor write pointer. */
    TFT_ADDR_CMD_DL = 0x302100,     /**< @brief Coprocessor DL RAM offset. */

    // RAM areas.
    TFT_ADDR_RAM_G = 0x000000,      /**< @brief General purpose graphics RAM. */
    TFT_ADDR_RAM_DL = 0x300000,     /**< @brief Display list RAM. */
    TFT_ADDR_RAM_CMD = 0x308000     /**< @brief Coprocessor command buffer. */
} tftAddr_t;

/**
 * @brief Sends a command to the display.
 *
 * @param tft The display.
 * @param cmd The command.
 * @param param The command's parameter.
 */
static void tftCmd(tft_t *tft, tftCmd_t cmd, uint8_t param) {
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
static void tftWrite(tft_t *tft, tftAddr_t addr, size_t len, const void *data) {
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
static void tftRead(tft_t *tft, tftAddr_t addr, size_t len, void *data) {
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
 * @brief Displays a display list.
 *
 * @param tft The display.
 * @param tftDL The display list.
 */
static void tftDisplay(tft_t *tft, const tftDL_t *tftDL) {
    size_t len = tftDL->len;

    // Write the display list.
    tftWrite(tft, TFT_ADDR_RAM_CMD + tft->cmdWrite, len, tftDL->data);

    // Update the command write address.
    tft->cmdWrite += len;
    if (tft->cmdWrite > TFT_RAM_CMD_SIZE) {
        tft->cmdWrite -= TFT_RAM_CMD_SIZE;
    }
    tftWrite(tft, TFT_ADDR_CMD_WRITE, sizeof(tft->cmdWrite), &tft->cmdWrite);
}

/** @brief Display update priority. */
uint32_t tftUpdate_priority = 4;

/** @brief Display update period. */
TickType_t tftUpdate_period_ms = 16;

/** @brief Display update task. */
cmr_task_t tftUpdate_task;

/**
 * @brief Task for updating the display.
 *
 * @param pvParameters (tft_t *) The display.
 *
 * @return Does not return.
 */
void tftUpdate(void *pvParameters) {
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

    TickType_t lastWakeTime = xTaskGetTickCount();
    cmr_gpioWrite(GPIO_PD_N, 0);
    vTaskDelayUntil(&lastWakeTime, TFT_RESET_MS);
    cmr_gpioWrite(GPIO_PD_N, 1);
    vTaskDelayUntil(&lastWakeTime, TFT_RESET_MS);

    // Initialize the display.
    tftCmd(tft, TFT_CMD_CLKEXT, 0x00);
    tftCmd(tft, TFT_CMD_ACTIVE, 0x00);
    tftCmd(tft, TFT_CMD_ACTIVE, 0x00);

    // Wait for display to initialize.
    vTaskDelayUntil(&lastWakeTime, TFT_INIT_MS);

    uint32_t chipID;
    tftRead(tft, TFT_ADDR_CHIP_ID, sizeof(chipID), &chipID);
    configASSERT(chipID == TFT_CHIP_ID);

    // Initialize registers.
    for (size_t i = 0; i < sizeof(tftInits) / sizeof(tftInits[0]); i++) {
        const tftInit_t *init = tftInits + i;
        tftWrite(tft, init->addr, sizeof(init->val), &init->val);
    }

    tftDisplay(tft, &tftDLStartup);
    vTaskDelayUntil(&lastWakeTime, TFT_STARTUP_MS);

    while (
        vTaskDelayUntil(&lastWakeTime, tftUpdate_period_ms), 1
    ) {
        tftDisplay(tft, &tftDLRTD);
    }
}

/** @brief The display. */
static tft_t tft;

/**
 * @brief Initializes the display.
 */
void tftInit(void) {
    const QSPI_InitTypeDef qspiInit = {
        .ClockPrescaler = 32,
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

    cmr_taskInit(
        &tftUpdate_task, "tftUpdate", tftUpdate_priority,
        tftUpdate, &tft
    );
}

