#ifndef SPI_H
#define SPI_H

#include <stdbool.h>
#include <stdint.h>

#define SRREAD 0b00010000
#define SRWRITE 0b00001000
#define PPOS_0 1
#define PPOS_1 6

void ADS7038Init();
uint8_t ADS7038_read(uint8_t reg);
void ADS7038_write(uint8_t reg, uint8_t data);
static void ADS7038_updateIO(void *pvParameters);
bool ADS7038_adcManualRead();
uint16_t MCP3202_read(uint8_t channel);
uint16_t MCP3202_init();


#endif
