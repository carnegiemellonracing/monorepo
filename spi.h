#ifndef ADS_H
#define ADS_H

#include <stdbool.h>
#include <stdint.h>
#define SPI_MSG_LEN 3
#define RD_REG      0b00010000 // 0b0001_0000
#define WR_REG      0b00001000 // 0b0000_1000
#define SPI_SET_BIT 0b00011000 // 0b0001_1000
#define SPI_CLR_BIT 0b00100000 // 0b0010_0000
#define PPOS_0_PORT 1
#define PPOS_1_PORT 6

void ADS7038Init();
uint8_t ADS7038_read(uint8_t reg, int* status);
void ADS7038_write(uint8_t reg, uint8_t data);
void ADS7038_adcManualRead(uint16_t *ppos);

#endif
