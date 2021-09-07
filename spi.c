/*
 * spi.c
 *
 *  Created on: Sep 7, 2021
 *      Author: vamsi
 */

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
	.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32, // Need to verify this is an ok prescaler
	.FirstBit = SPI_FIRSTBIT_MSB,
	.TIMode = SPI_TIMODE_DISABLE,
	.CRCCalculation = SPI_CRCCALCULATION_DISABLE,
	.CRCPolynomial = 10
};


/** @brief SPI pins for the IMU. */
static const cmr_spiPinConfig_t HVCSpiPins = {
    .mosi = { .port = GPIOA, .pin = GPIO_PIN_7 },
    .miso = { .port = GPIOA, .pin = GPIO_PIN_6 },
    .sck = { .port = GPIOA, .pin = GPIO_PIN_5 },
    .nss = { .port = GPIOA, .pin = GPIO_PIN_4 }
};

// These need to be updated
static const uint8_t VOLTAGE_TX_BYTE = 0x00;
static const uint8_t VOLTAGE_RX_LEN  = 0x03;
static const uint8_t CURRENT_TX_BYTE = 0x00;
static const uint8_t CURRENT_RX_LEN  = 0x03;

// Current average sample count = rate(Hz) * time(s)
static const int16_t numSamplesInstant = 20;//100 * 2;
static const int16_t numSamplesAverage = 3000;//100 * 30;
#define NUM_SAMPLES_AVERAGE 3000

static volatile int32_t currentSingleSample = 0;
static volatile int32_t currentAvg = 0;
static volatile int32_t currentInstant = 0;

static volatile int32_t HighVoltage = 0;

/**
 * @brief Task for updating the Voltage and Current
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void HVCSpiUpdate(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        uint8_t rxVoltage[3] = {0,0,0};

        cmr_spiTXRX(&spi, &VOLTAGE_TX_BYTE, &rxVoltage, VOLTAGE_RX_LEN);
        HighVoltage = (int32_t) ((rx[2] << 16) | (rx[1] << 8) | rx[0]);

        uint8_t rxCurrent[3] = {0,0,0};
        cmr_spiTXRX(&spi, &VOLTAGE_TX_BYTE, &rxCurrent, VOLTAGE_RX_LEN);
        currentSingleSample = (int32_t) ((rx[2] << 16) | (rx[1] << 8) | rx[0]);

		// Rolling average
        // A single sample is too noisy for an "instant" measurement so do a small average
        currentInstant = (currentInstant*(numSamplesInstant-1) + currentSingleSample) / numSamplesInstant;
        currentAvg = (currentAvg*(numSamplesAverage-1) + currentSingleSample) / numSamplesAverage;

        vTaskDelayUntil(&lastWakeTime, HVCSpiUpdate_period_ms);
    }
}

int cmr_spiTXRX(
    cmr_spi_t *spi, const void *txData, void *rxData, size_t len
);

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

// TO-DO: NEED TO CREATE transfer math
int32_t getHVmillivolts() {
    return HighVoltage;
}

int32_t getCurrentInstant() {
     return currentInstant;
}
 
int32_t getCurrentAverage() {
    return currentAvg;
}

