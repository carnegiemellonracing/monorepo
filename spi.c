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
    .CLKPhase = SPI_PHASE_2EDGE,
    .NSS = SPI_NSS_HARD_OUTPUT,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256,
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
#define VOLTAGE_RX_LEN  0x03
#define CURRENT_RX_LEN  0x03
#define BURST_RX_LEN    20

// Current average sample count = rate(Hz) * time(s)
static const int16_t numSamplesInstant = 20;//10 * 2;
static const int16_t numSamplesAverage = 3000;//100 * 30;
#define NUM_SAMPLES_AVERAGE 3000

static volatile int32_t currentSingleSample_ADC = 0;
static volatile int32_t currentAvg_ADC = 0;
static volatile int32_t currentInstant_ADC = 0;

static volatile int32_t HighVoltage_ADC = 0;
static volatile int32_t voltageHV = 0;

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
    uint8_t data[rxLen + 1];
    cmr_spiTXRX(&spi, &header, data, rxLen + 1);
    memcpy(rxData, &(data[1]), rxLen);
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


/** @brief Converts ADC reading into HV voltage */
static inline int32_t ADCtoMV_HVSense (int32_t adc_input) {
    return (int32_t) ((0.1242f * adc_input) - 46767.f);
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
    uint8_t temp = 1;

    // Read the STATUS0 register until Bit 0 (RESET_ON) is cleared to 0
    uint8_t underReset = 1;
    while (underReset) {
        uint8_t status = 1;
        HVSenseRead(STATUS0, &status, 1);

        underReset = (status & STATUS0_RESET_ON);
    }

    // Initialize the CONFIG register
    uint8_t configuration = ADC_FREQ_1kHz;
    HVSenseWrite(CONFIG, &configuration, 1);
    HVSenseRead(CONFIG, &temp, 1);
    configASSERT(temp == configuration);

    // Initialize the EMI_CTRL register
    uint8_t emi_config = EMI_CONFIG;
    HVSenseWrite(EMI_CTRL, &emi_config, 1);
    HVSenseRead(EMI_CTRL, &temp, 1);
    configASSERT(temp == emi_config);

    // Set the lock register to 0xCA to protect the user accessible and internal configuration registers.
    uint8_t lock = LOCK_KEY_EN;
    HVSenseWrite(LOCK, &lock, 1);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        
        // Wait until data is ready
        int dataReady_L = cmr_gpioRead(GPIO_HVSENSE_DRDY_L);
        while (dataReady_L) {
            dataReady_L = cmr_gpioRead(GPIO_HVSENSE_DRDY_L);
        }

        // Sample HV Bus Voltage
        uint8_t rxVoltage[VOLTAGE_RX_LEN] = {0,0,0};
        HVSenseRead(V1WV, rxVoltage, VOLTAGE_RX_LEN);
        HighVoltage_ADC = (int32_t) ((rxVoltage[2]) | (rxVoltage[1] << 8) | (rxVoltage[0] << 16) );
        HighVoltage_ADC = (HighVoltage_ADC << 8) >> 8;
        voltageHV = ADCtoMV_HVSense(HighVoltage_ADC);


        // Sample HV Current
        uint8_t rxCurrent[3] = {0,0,0};
        HVSenseRead(IWV, rxCurrent, CURRENT_RX_LEN);
        currentSingleSample_ADC = (int32_t) ((rxCurrent[2]) | (rxCurrent[1] << 8) | (rxCurrent[0] << 16));
        currentSingleSample_ADC = (currentSingleSample_ADC << 8) >> 8;

        // Rolling average
        // A single sample is too noisy for an "instant" measurement so do a small average
        currentInstant_ADC = (currentInstant_ADC*(numSamplesInstant-1) + currentSingleSample_ADC) / numSamplesInstant;
        currentAvg_ADC = (currentAvg_ADC*(numSamplesAverage-1) + currentSingleSample_ADC) / numSamplesAverage;

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
    // https://www.analog.com/media/en/technical-documentation/data-sheets/ade7912_7913.pdf
//    static const float maxVoltageConverted_mV = 788.f;
//    static const int32_t maxVoltageADCValue = 0x7FFFFF;
//    static const int32_t minVoltageADCValue = 0x800000;
//    static const float sensedToHV = 1207.f;
//
//    // Convert ADC value to Sensed Voltage
//    float sensedVoltage_mV;
//    if (HighVoltage_ADC >= 0) {
//        sensedVoltage_mV = (HighVoltage_ADC - 400000) * maxVoltageConverted_mV / maxVoltageADCValue;
//    } else {
//        sensedVoltage_mV = (HighVoltage_ADC + 400000) * maxVoltageConverted_mV / minVoltageADCValue;
//    }
//
//    // Convert Sensed Voltage to HV Bus Voltage
//    float HV_mV = sensedVoltage_mV * sensedToHV;

    // float HV_mV = (0.1242f * HighVoltage_ADC) - 46767.f;
    return voltageHV;
}

// Convert IP ADC value to Shunt Voltage
float adcToCurrent(int32_t currentADC) {
    static const float maxCurrentConverted_V = 0.04927f;
    static const int32_t maxCurrentADCValue = 0x7FFFFF;
    static const int32_t minCurrentADCValue = 0x800000;

    float sensedCurrent_V;
    if (currentADC >= 0) {
        sensedCurrent_V = currentADC * maxCurrentConverted_V / maxCurrentADCValue;
    } else {
        sensedCurrent_V = currentADC * maxCurrentConverted_V / minCurrentADCValue;
    }

    return sensedCurrent_V;
}

// V=IR 
// Measure current across shunt resistor (166.6 uohm)
// 1/166.6u = 6002
int32_t getCurrentInstant() {
    return (int32_t)(adcToCurrent(currentInstant_ADC) * 6002);
}
 
int32_t getCurrentAverage() {
    return (int32_t)(adcToCurrent(currentAvg_ADC) * 6002);
}

