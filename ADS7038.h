#ifndef ADS_H
#define ADS_H

#include <stdbool.h>
#include <stdint.h>

#define SPI_MSG_LEN 3
#define RD_REG 0b00010000       // 0b0001_0000
#define WR_REG 0b00001000       // 0b0000_1000
#define SPI_SET_BIT 0b00011000  // 0b0001_1000
#define SPI_CLR_BIT 0b00100000  // 0b0010_0000

#define PPOS_0_PORT 1 // Paddle Position 0 Port
#define PPOS_1_PORT 6 // Paddle Position 1 Port
#define SW_BUTTON_0 5 // Steering Wheel Button 0
#define SW_BUTTON_1 2 // Steering Wheel Button 1
#define SW_BUTTON_2 3 // Steering Wheel Button 2
#define SW_BUTTON_3 4 // Steering Wheel Button 3


typedef enum {
    SYSTEM_STATUS_REG   = 0x0,          /** System Status Register */
    GENERAL_CFG_REG     = 0x1,          /** General Configuration Register */
    DATA_CFG_REG        = 0x2,          /** Data Configuration Register */
    OSR_CFG_REG         = 0x3,          /** Oversampling Ratio Configuration Register */
    OPMODE_CFG_REG      = 0x4,          /** Operating Mode Configuration Register */
    PIN_CFG_REG         = 0x5,          /** Pin Configuration Register */
    GPIO_CFG_REG        = 0x7,          /** GPIO Configuration Register */
    GPO_VALUE_REG       = 0xB,          /** GPIO Output Value Register */
    GPI_VALUE_REG       = 0xD,          /** GPIO Input Value Register */
    SEQUENCE_CFG_REG    = 0x10,         /** Sequence Configuration Register */
    CHANNEL_SEL_REG     = 0x11,         /** Channel Select Register */

} ADS7038Register_t;

bool ADS7038Init();
int ADS7038Configure();
int ADS7038_read(uint8_t reg, uint8_t* data);
int ADS7038_write(uint8_t reg, uint8_t data);
int ADS7038_adcManualRead(uint16_t* ppos);

#endif
