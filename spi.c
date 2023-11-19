#include "spi.h"
#include "expandersPrivate.h"
#include <CMR/spi.h>    // SPI interface
#include <CMR/gpio.h>
#include <CMR/tasks.h>

cmr_spi_t ADS7038Spi;
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

int ADS7038_read(uint8_t reg,  uint8_t *data) {
    uint8_t command[3] = {RD_REG, reg, 0};
    uint8_t dummy[3] = {0, 0, 0};
	// Initiate Register Read
	int status = 0;
    status = cmr_spiTXRX(&ADS7038Spi, command, NULL, SPI_MSG_LEN); // TODO add check for -1
	if (status < 0){
		return -1;
	}

    for(int i = 0; i < 1000; i++);

	// Read Data from register address
    status = cmr_spiTXRX(&ADS7038Spi, dummy, data, SPI_MSG_LEN);
		if (status < 0){
		return -1;
	}
    return status;
}

int ADS7038_write(uint8_t reg, uint8_t data) {
    uint8_t command[3] = {WR_REG, reg, data};
    return cmr_spiTXRX(&ADS7038Spi, command, NULL, SPI_MSG_LEN);
}

uint16_t ADS7038_manualRead() {
	uint8_t channel = 0;
	uint16_t adcValues[8];
	while (1) {
		uint8_t command[3] = {WR_REG, 0x11, channel};
		uint8_t data[3];

		cmr_spiTXRX(&ADS7038Spi, command, data, SPI_MSG_LEN);
		uint16_t adcValue = (data[0] << 4) | (data[1] >> 4);
		uint8_t readChannel = data[1] & 0xF;

		adcValues[readChannel] = adcValue;

		channel = (channel + 1) % 8;
		for(int i = 0; i < 100000; i++);
	}
}

int ADS7038_adcManualRead(uint16_t *ppos) {
	uint8_t command0[3] = {WR_REG, CHANNEL_SEL_REG, PPOS_0_PORT};
	uint8_t command1[3] = {WR_REG, CHANNEL_SEL_REG, PPOS_1_PORT};
	uint8_t data[3];

	// Set first channel, data received is not meaningful
	cmr_spiTXRX(&ADS7038Spi, command0, NULL, 3);

	// Set second channel, data received is not meaningful
	cmr_spiTXRX(&ADS7038Spi, command1, NULL, 3);

	// Receive first channel data
	if(cmr_spiTXRX(&ADS7038Spi, (uint8_t[]) {0, 0, 0}, data, SPI_MSG_LEN) == 0){
	uint16_t adcValue0 = (data[0] << 4) | (data[1] >> 4);
	uint8_t readChannel0 = data[1] & 0xF;
	ppos[0] = adcValue0;
	}
	else{
		return -1;
	}

	// Receive second channel data
	if(cmr_spiTXRX(&ADS7038Spi, (uint8_t[]) {0, 0, 0}, data, SPI_MSG_LEN) == 0){
	uint16_t adcValue1 = (data[0] << 4) | (data[1] >> 4);
	uint8_t readChannel1 = data[1] & 0xF;
	ppos[1] = adcValue1;
	}
	else{
		return -1;
	}
	return 0;
}

int ADS7038Init() {
    cmr_spiInit(
        &ADS7038Spi, SPI2, &ADS7038SpiInit, &ADS7038SpiPins,
        DMA2_Stream2, DMA_CHANNEL_3,
        DMA2_Stream3, DMA_CHANNEL_3
    );
	int status = 0;
    status |= ADS7038_read(SYSTEM_STATUS_REG,NULL);
    status |= ADS7038_write(DATA_CFG_REG, 0b00010000);
    status |= ADS7038_read(DATA_CFG_REG,NULL);
    status |= ADS7038_write(SEQUENCE_CFG_REG, 0b00000000);
    status |= ADS7038_read(SEQUENCE_CFG_REG,NULL);
    status |= ADS7038_write(PIN_CFG_REG, 0b00111100);
    status |= ADS7038_read(PIN_CFG_REG,NULL);
    status |= ADS7038_write(GPIO_CFG_REG, 0b00000000);
    status |= ADS7038_read(GPIO_CFG_REG,NULL);

	return status;
//     while (1) {
// //    	ADS7038_adcManualRead();
//     	// swButtons = ADS7038_read(GPI_VALUE_REG);
//     }
}
