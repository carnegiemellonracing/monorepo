#include <CMR/spi.h>    // SPI interface
#include <CMR/gpio.h>
#include <CMR/tasks.h>
#include <stdbool.h>

#include "spi.h"

#define SRREAD 0b00010000
#define SRWRITE 0b00001000


cmr_spi_t ADS7038Spi;
cmr_spi_t MCP3202Spi;
uint16_t ppos[2];
uint8_t	swButtons;

static cmr_task_t ADS7038_updateIO_task;
static const TickType_t ADS7038_updateIO_period_ms = 100;
static const uint32_t ADS7038_updateIO_priority = 3;

static const SPI_InitTypeDef ADS7038SpiInit = {
		.Mode = SPI_MODE_MASTER,
		.Direction = SPI_DIRECTION_2LINES,
		.DataSize = SPI_DATASIZE_8BIT,
		.CLKPolarity = SPI_POLARITY_HIGH,
		.CLKPhase = SPI_PHASE_2EDGE,
		.NSS = SPI_NSS_SOFT,
		.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256,
		.FirstBit = SPI_FIRSTBIT_MSB,
		.TIMode = SPI_TIMODE_DISABLE,
		.CRCCalculation = SPI_CRCCALCULATION_DISABLE,
		.CRCPolynomial = 10
};

static const cmr_spiPinConfig_t ADS7038SpiPins = {
    .mosi = { .port = GPIOC, .pin = GPIO_PIN_3 },
    .miso = { .port = GPIOC, .pin = GPIO_PIN_2 },
    .sck = { .port = GPIOB, .pin = GPIO_PIN_13 },
    .nss = { .port = GPIOB, .pin = GPIO_PIN_12 }
};

static const SPI_InitTypeDef MCP3202SpiInit = {
		.Mode = SPI_MODE_MASTER,
		.Direction = SPI_DIRECTION_2LINES,
		.DataSize = SPI_DATASIZE_8BIT,
		.CLKPolarity = SPI_POLARITY_HIGH,
		.CLKPhase = SPI_PHASE_2EDGE,
		.NSS = SPI_NSS_SOFT,
		.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256,
		.FirstBit = SPI_FIRSTBIT_MSB,
		.TIMode = SPI_TIMODE_DISABLE,
		.CRCCalculation = SPI_CRCCALCULATION_DISABLE,
		.CRCPolynomial = 10
};

static const cmr_spiPinConfig_t MCP3202SpiPins = {
	.mosi = { .port = GPIOE, .pin = GPIO_PIN_6 },
	.miso = { .port = GPIOE, .pin = GPIO_PIN_5 },
	.sck = { .port = GPIOE, .pin = GPIO_PIN_2 },
	.nss = { .port = GPIOE, .pin = GPIO_PIN_4 }
};

uint8_t ADS7038_read(uint8_t reg) {
    uint8_t command[3] = {SRREAD, reg, 0};
    uint8_t dummy[3] = {0, 0, 0};
    uint8_t data[3];

    cmr_spiTXRX(&ADS7038Spi, command, NULL, 3);
    for(int i = 0; i < 1000; i++);
    cmr_spiTXRX(&ADS7038Spi, dummy, data, 3);

    return data[0];
}

void ADS7038_write(uint8_t reg, uint8_t data) {
    uint8_t command[3] = {SRWRITE, reg, data};

    cmr_spiTXRX(&ADS7038Spi, command, NULL, 3);
}


static void ADS7038_updateIO(void *pvParameters) {
	(void) pvParameters;

	TickType_t lastWakeTime = xTaskGetTickCount();

	while (1) {
		ADS7038_adcManualRead();
		swButtons = ADS7038_read(0xD);
		vTaskDelayUntil(&lastWakeTime, ADS7038_updateIO_period_ms);
	}
}

uint16_t *ADS7038_getPaddles() {
	return ppos;
}

uint8_t ADS7038_getButtons() {
	return swButtons;
}

void ADS7038Init() {
    cmr_spiInit(
        &ADS7038Spi, SPI2, &ADS7038SpiInit, &ADS7038SpiPins,
        DMA2_Stream2, 0,
        DMA2_Stream3, 0
    );

    ADS7038_read(0x00);
    ADS7038_write(0x02, 0b00010000);
    ADS7038_read(0x02);
    ADS7038_write(0x10, 0b00000000);
    ADS7038_read(0x10);
    ADS7038_write(0x05, 0b00111100);
    ADS7038_read(0x05);
    ADS7038_write(0x07, 0b00000000);
    ADS7038_read(0x07);

}

uint16_t MCP3202_init() {

	SPI_InitTypeDef MCP3202SpiInit = {0};
	MCP3202SpiInit.Mode = SPI_MODE_MASTER;
	MCP3202SpiInit.Direction = SPI_DIRECTION_2LINES;
	MCP3202SpiInit.DataSize = SPI_DATASIZE_8BIT;
	MCP3202SpiInit.CLKPolarity = SPI_POLARITY_HIGH;
	MCP3202SpiInit.CLKPhase = SPI_PHASE_2EDGE;
	MCP3202SpiInit.NSS = SPI_NSS_SOFT;
	MCP3202SpiInit.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	MCP3202SpiInit.FirstBit = SPI_FIRSTBIT_MSB;
	MCP3202SpiInit.TIMode = SPI_TIMODE_DISABLE;
	MCP3202SpiInit.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	MCP3202SpiInit.CRCPolynomial = 0x0;
	MCP3202SpiInit.NSSPMode = SPI_NSS_PULSE_DISABLE;
	MCP3202SpiInit.NSSPolarity = SPI_NSS_POLARITY_LOW;
	MCP3202SpiInit.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	MCP3202SpiInit.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	MCP3202SpiInit.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	MCP3202SpiInit.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	MCP3202SpiInit.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	MCP3202SpiInit.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	MCP3202SpiInit.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	MCP3202SpiInit.IOSwap = SPI_IO_SWAP_DISABLE;

    cmr_spiInit(
        &MCP3202Spi, SPI4, &MCP3202SpiInit, &MCP3202SpiPins,
        DMA2_Stream2, 0,
        DMA2_Stream3, 0
    );
}

uint16_t MCP3202_read(uint8_t channel) {
	uint8_t config;
	if (channel == 1) {
		config = 0b11100000;
	} else if (channel == 0) {
		config = 0b10100000;
	} else {
		return 0;
	}

	HAL_StatusTypeDef status;
	//0000_0001_1110_0000_0000_0000
	//xxxx_xxxx_xxxN_dddd_dddd_dddd
	//000000xxxx_xNdd_dddd_dddd_dd00_0000
	uint8_t command[3] = {1, config, 0};
    uint8_t data[3];
    taskENTER_CRITICAL();
    cmr_spiTXRX(&MCP3202Spi, command, data, 3);
    taskEXIT_CRITICAL();
//    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
//    status = HAL_SPI_TransmitReceive(&hspi4, (void*) command, data, 3, 1);
//    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);

    uint16_t retval = ((data[1] & 0x0f) << 8) | data[2];

    return retval;
}
