#include "ADS7038.h"
#include <CMR/spi.h>    // SPI interface
#include <CMR/gpio.h>
#include <CMR/tasks.h>

cmr_spi_t ADS7038Spi;
uint16_t ppos[2];
uint8_t	swButtons;

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

uint16_t ADS7038_manualRead() {
	uint8_t channel = 0;
	uint16_t adcValues[8];
	while (1) {
		uint8_t command[3] = {SRWRITE, 0x11, channel};
		uint8_t data[3];

		cmr_spiTXRX(&ADS7038Spi, command, data, 3);
		uint16_t adcValue = (data[0] << 4) | (data[1] >> 4);
		uint8_t readChannel = data[1] & 0xF;

		adcValues[readChannel] = adcValue;

		channel = (channel + 1) % 8;
		for(int i = 0; i < 100000; i++);

		(void) adcValue;
		(void) readChannel;
	}
}

void ADS7038_adcManualRead() {
	uint8_t command0[3] = {SRWRITE, 0x11, PPOS_0};
	uint8_t command1[3] = {SRWRITE, 0x11, PPOS_1};
	uint8_t data[3];

	// Set first channel, data received is not meaningful
	cmr_spiTXRX(&ADS7038Spi, command0, data, 3);

	// Set second channel, data received is not meaningful
	cmr_spiTXRX(&ADS7038Spi, command1, data, 3);

	// Receive first channel data
	cmr_spiTXRX(&ADS7038Spi, (uint8_t[]) {0, 0, 0}, data, 3);
	uint16_t adcValue0 = (data[0] << 4) | (data[1] >> 4);
	uint8_t readChannel0 = data[1] & 0xF;
	ppos[0] = adcValue0;

	// Receive second channel data
	cmr_spiTXRX(&ADS7038Spi, (uint8_t[]) {0, 0, 0}, data, 3);
	uint16_t adcValue1 = (data[0] << 4) | (data[1] >> 4);
	uint8_t readChannel1 = data[1] & 0xF;
	ppos[1] = adcValue1;
}

void ADS7038Init() {
    cmr_spiInit(
        &ADS7038Spi, SPI2, &ADS7038SpiInit, &ADS7038SpiPins,
        DMA2_Stream2, DMA_CHANNEL_3,
        DMA2_Stream3, DMA_CHANNEL_3
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

    while (1) {
    	ADS7038_adcManualRead();
    	swButtons = ADS7038_read(0xD);
    }
}
