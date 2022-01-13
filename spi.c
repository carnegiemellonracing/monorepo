/*
 * spi.c
 *
 *  Created on: Sep 7, 2021
 *      Author: vamsi
 */

#include <stdlib.h>
#include <string.h>
#include <CMR/gpio.h>
#include "gpio.h"
#include "spi.h"

/** @brief Voltage/Current Hz TX priority. */
static const uint32_t HVCSpiUpdate_priority = 5;

/** @brief Voltage/Current 1000 Hz TX period (milliseconds). */
static const TickType_t HVCSpiUpdate_period_ms = 1;

/** @brief Voltage/Current 1000 Hz TX task. */
static cmr_task_t HVCSpiUpdate_task;

/** @brief Primary SPI. */
static cmr_spi_t spi;

/** @brief SPI bus parameters for the HVC (see datasheet ADE7912/ADE7913). */
static const SPI_InitTypeDef HVCSpiInit = {
	.Mode = SPI_MODE_MASTER,
	.Direction = SPI_DIRECTION_2LINES,
	.DataSize = SPI_DATASIZE_8BIT,
	.CLKPolarity = SPI_POLARITY_HIGH,
	.CLKPhase = SPI_PHASE_1EDGE,
	.NSS = SPI_NSS_HARD_OUTPUT,
	.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4, // Need to verify this is an ok prescaler
	.FirstBit = SPI_FIRSTBIT_MSB,
	.TIMode = SPI_TIMODE_DISABLE,
	.CRCCalculation = SPI_CRCCALCULATION_DISABLE,
	.CRCPolynomial = 10
};


/** @brief SPI pins for the ADC. */
static const cmr_spiPinConfig_t HVCSpiPins = {
    .mosi = { .port = GPIOA, .pin = GPIO_PIN_7 },
    .miso = { .port = GPIOA, .pin = GPIO_PIN_6 },
    .sck = { .port = GPIOA, .pin = GPIO_PIN_5 },
    .nss = { .port = GPIOA, .pin = GPIO_PIN_4 }
};

// These need to be updated
#define VOLTAGE_RX_LEN 0x03
#define CURRENT_RX_LEN 0x03

// Current average sample count = rate(Hz) * time(s)
static const int16_t numSamplesInstant = 20;//10 * 2;
static const int16_t numSamplesAverage = 3000;//100 * 30;
#define NUM_SAMPLES_AVERAGE 3000

static volatile int32_t currentSingleSample = 0;
static volatile int32_t currentAvg = 0;
static volatile int32_t currentInstant = 0;

static volatile int32_t HighVoltage = 0;

#define ADDR_OFFSET 3
#define READ_OFFSET 2

typedef enum {
    SPI_READ = 0x1,
    SPI_WRITE = 0x0
} spiReadWrite_t;

/**
 * @brief Returns header byte for transmission
 *
 * @param address Register Address within device
 * @param read If want data back from HV Sense
 * @param rxLen Number of bytes to receive
 */
static uint8_t getSPIHeaderByte(hvSenseRegister_t address, spiReadWrite_t read) {
    return ( (address << ADDR_OFFSET) | (read << READ_OFFSET) );
}

/**
 * @brief Reads from the HV Sense
 *
 * @param address Register Address within device
 * @param rxData Pointer to where to write receive data
 * @param rxLen Number of bytes to receive
 */
static void HVSenseRead(hvSenseRegister_t address, uint8_t* rxData, size_t rxLen) {
    uint8_t header = getSPIHeaderByte(address, SPI_READ);
    // Not sure about length so splitting up - TODO check if this works
    cmr_spiTXRX(&spi, &header, NULL, 1);
    cmr_spiTXRX(&spi, NULL, rxData, rxLen);
}

/**
 * @brief Writes to HV Sense
 *
 * @param address Register Address within device
 * @param txData Payload to transmit
 * @param txLen Number of bytes to transmit
 */
static void HVSenseWrite(hvSenseRegister_t address, uint8_t* txData, size_t txLen) {
    uint8_t data[txLen + 1];
    data[0] = getSPIHeaderByte(address, SPI_WRITE);
    memcpy(&(data[1]), txData, txLen);
    cmr_spiTXRX(&spi, &data, NULL, txLen + 1);
}

/**
 * @brief Task for updating the Voltage and Current
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void HVCSpiUpdate(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    // https://www.analog.com/media/en/technical-documentation/data-sheets/ade7912_7913.pdf
    // Read the STATUS0 register until Bit 0 (RESET_ON) is cleared to 0
    uint8_t underReset = 1;
    while (underReset) {
        uint8_t status;
        HVSenseRead(STATUS0, &status, 1);

        underReset = (status & STATUS0_RESET_ON);
    }

    // Initialize the CONFIG register
    uint8_t configuration = ADC_FREQ_1kHz;
    HVSenseWrite(CONFIG, &configuration, 1);

    // Initialize the EMI_CTRL register
    uint8_t emi_config = EMI_CONFIG;
    HVSenseWrite(EMI_CTRL, &emi_config, 1);

    // Set the lock register to 0xCA to protect the user accessible and internal configuration registers.
    uint8_t lock = LOCK_KEY_EN;
    HVSenseWrite(LOCK, &lock, 1);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {

        // Sample HV Bus Voltage
        uint8_t rxVoltage[VOLTAGE_RX_LEN] = {0,0,0};
        HVSenseRead(V1WV, rxVoltage, VOLTAGE_RX_LEN);
        HighVoltage = (int32_t) ((rxVoltage[2] << 16) | (rxVoltage[1] << 8) | rxVoltage[0]);

        // Sample HV Current
        uint8_t rxCurrent[3] = {0,0,0};
        HVSenseRead(IWV, rxCurrent, CURRENT_RX_LEN);
        currentSingleSample = (int32_t) ((rxCurrent[2] << 16) | (rxCurrent[1] << 8) | rxCurrent[0]);

		// Rolling average
        // A single sample is too noisy for an "instant" measurement so do a small average
        currentInstant = (currentInstant*(numSamplesInstant-1) + currentSingleSample) / numSamplesInstant;
        currentAvg = (currentAvg*(numSamplesAverage-1) + currentSingleSample) / numSamplesAverage;

        vTaskDelayUntil(&lastWakeTime, HVCSpiUpdate_period_ms);
    }
}

/**
 * @brief Initializes the HVC SPI interface.
 */
void spiInit(void) {
    cmr_spiInit(
        &spi, SPI1, &HVCSpiInit, &HVCSpiPins,
        DMA2_Stream2, DMA_CHANNEL_3,
        DMA2_Stream3, DMA_CHANNEL_3
    );

    cmr_taskInit(
        &HVCSpiUpdate_task,
        "Voltage and Current Update",
		HVCSpiUpdate_priority,
		HVCSpiUpdate,
        NULL
    );
}


// Accessor/Transfer Functions

// Voltage divider into the input of V1P is 680ohm/(680ohm + 820kohm)
// Inverse of this value is 1206.88 (round to 1207)
// We also need to reverse the polarity of this measurement
int32_t getHVmillivolts() {
    return (int32_t)(HighVoltage * -1207);
}

// V=IR 
// Measure current across shunt resistor (166.6 uohm)
// 1/166.6u = 6002
int32_t getCurrentInstant() {
     return (int32_t)(currentInstant * 6002);
}
 
int32_t getCurrentAverage() {
    return (int32_t)(currentAvg * 6002);
}

