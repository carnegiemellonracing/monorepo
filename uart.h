/*
 * uart.h
 *
 *  Created on: Jul 26, 2020
 *      Author: vamsi
 */

#ifndef UART_H_
#define UART_H_

// Import library modules
// NONE

// Import custom modules
#include <CMR/uart.h>
#include <stdbool.h>

//-----------------------------------------------------------------------------
// DEFINES                                                                    |
//-----------------------------------------------------------------------------
#define MAX_RESPONSE_LENGTH 128u

//-----------------------------------------------------------------------------
// TYPEDEF ENUMERATIONS                                                       |
//-----------------------------------------------------------------------------
typedef uint8_t Byte;

typedef enum {
  SLAVE_REG_CMD        = 0x02,
  SLAVE_REG_CHANNELS0  = 0x03,
  SLAVE_REG_CHANNELS1  = 0x04,
  SLAVE_REG_CHANNELS2  = 0x05,
  SLAVE_REG_CHANNELS3  = 0x06,
  SLAVE_REG_OVERSMPL   = 0x07,
  SLAVE_REG_ADDR       = 0x0A,
  SLAVE_REG_GROUP_ID   = 0x0B,
  SLAVE_REG_DEV_CTRL   = 0x0C,
  SLAVE_REG_NCHAN      = 0x0D,
  SLAVE_REG_DEVCONFIG  = 0x0E,
  SLAVE_REG_PWRCONFIG  = 0x0F,
  SLAVE_REG_COMCONFIG0 = 0x10,
  SLAVE_REG_COMCONFIG1 = 0x11,
  SLAVE_REG_TXHOLDOFF  = 0x12,
  SLAVE_REG_CBCONFIG   = 0x13,
  SLAVE_REG_CBENBL0    = 0x14,
  SLAVE_REG_CBENBL1    = 0x15,
  SLAVE_REG_TSTCONFIG0 = 0x1E,
  SLAVE_REG_TSTCONFIG1 = 0x1F,
  SLAVE_REG_TESTCTRL0  = 0x20,
  SLAVE_REG_TESTCTRL1  = 0x21,
  SLAVE_REG_TEST_ADC0  = 0x22,
  SLAVE_REG_TEST_ADC1  = 0x23,
  SLAVE_REG_TEST_ADC2  = 0x24,
  SLAVE_REG_TESTAUXPU  = 0x25,
  SLAVE_REG_CTO        = 0x28,
  SLAVE_REG_CTO_CNT0   = 0x29,
  SLAVE_REG_CTO_CNT1   = 0x2A,
  SLAVE_REG_CTO_CNT2   = 0x2B,
  SLAVE_REG_AM_PER     = 0x32,
  SLAVE_REG_AM_CHAN0   = 0x33,
  SLAVE_REG_AM_CHAN1   = 0x34,
  SLAVE_REG_AM_CHAN2   = 0x35,
  SLAVE_REG_AM_CHAN3   = 0x36,
  SLAVE_REG_AM_OSMPL   = 0x37,
  SLAVE_REG_SMPL_DLY1  = 0x3D,
  SLAVE_REG_CELL_SPER  = 0x3E,
  SLAVE_REG_AUX_SPER0  = 0x3F,
  SLAVE_REG_AUX_SPER1  = 0x40,
  SLAVE_REG_AUX_SPER2  = 0x41,
  SLAVE_REG_AUX_SPER3  = 0x42,
  SLAVE_REG_TEST_SPER0 = 0x43,
  SLAVE_REG_TEST_SPER1 = 0x44,
  SLAVE_REG_SHDN_STS   = 0x50,
  SLAVE_REG_STATUS     = 0x51,
  SLAVE_REG_FAULT_SUM0 = 0x52,
  SLAVE_REG_FAULT_SUM1 = 0x52,
  SLAVE_REG_FAULT_UV0  = 0x54,
  SLAVE_REG_FAULT_UV1  = 0x55,
  SLAVE_REG_FAULT_OV0  = 0x56,
  SLAVE_REG_FAULT_OV1  = 0x57,
  SLAVE_REG_FAULT_AUX0 = 0x58,
  SLAVE_REG_FAULT_AUX1 = 0x59,
  SLAVE_REG_FAULT_2UV0 = 0x5A,
  SLAVE_REG_FAULT_2UV1 = 0x5B,
  SLAVE_REG_FAULT_2OV0 = 0x5C,
  SLAVE_REG_FAULT_2OV1 = 0x5D,
  SLAVE_REG_FAULT_COM0 = 0x5E,
  SLAVE_REG_FAULT_COM1 = 0x5F,
  SLAVE_REG_FAULT_SYS  = 0x60,
  SLAVE_REG_FAULT_DEV0 = 0x61,
  SLAVE_REG_FAULT_DEV1 = 0x62,
  SLAVE_REG_FAULT_GPI  = 0x63,
  SLAVE_REG_MASK_COMM0 = 0x68,
  SLAVE_REG_MASK_COMM1 = 0x69,
  SLAVE_REG_MASK_SYS   = 0x6A,
  SLAVE_REG_MASK_DEV0  = 0x6B,
  SLAVE_REG_MASK_DEV1  = 0x6C,
  SLAVE_REG_FO_CTRL0   = 0x6D,
  SLAVE_REG_FO_CTRL1   = 0x6E,
  SLAVE_REG_GPIO_DIR   = 0x78,
  SLAVE_REG_GPIO_OUT   = 0x79,
  SLAVE_REG_GPIO_PU    = 0x7A,
  SLAVE_REG_GPIO_PD    = 0x7B,
  SLAVE_REG_GPIO_IN    = 0x7C,
  SLAVE_REG_GP_FLT_IN  = 0x7D,
  SLAVE_REG_MAGIC1_0   = 0x82,
  SLAVE_REG_MAGIC1_1   = 0x83,
  SLAVE_REG_MAGIC1_2   = 0x84,
  SLAVE_REG_MAGIC1_3   = 0x85,
  SLAVE_REG_COMP_UV    = 0x8C,
  SLAVE_REG_COMP_OV    = 0x8D,
  SLAVE_REG_CELL_UV0   = 0x8E,
  SLAVE_REG_CELL_UV1   = 0x8F,
  SLAVE_REG_CELL_OV0   = 0x90,
  SLAVE_REG_CELL_OV1   = 0x91,
  SLAVE_REG_AUX0_UV0   = 0x92,
  SLAVE_REG_AUX0_UV1   = 0x93,
  SLAVE_REG_AUX0_OV0   = 0x94,
  SLAVE_REG_AUX0_OV1   = 0x95,
  SLAVE_REG_AUX1_UV0   = 0x96,
  SLAVE_REG_AUX1_UV1   = 0x97,
  SLAVE_REG_AUX1_OV0   = 0x98,
  SLAVE_REG_AUX1_OV1   = 0x99,
  SLAVE_REG_AUX2_UV0   = 0x9A,
  SLAVE_REG_AUX2_UV1   = 0x9B,
  SLAVE_REG_AUX2_OV0   = 0x9C,
  SLAVE_REG_AUX2_OV1   = 0x9D,
  SLAVE_REG_AUX3_UV0   = 0x9E,
  SLAVE_REG_AUX3_UV1   = 0x9F,
  SLAVE_REG_AUX3_OV0   = 0xA0,
  SLAVE_REG_AUX3_OV1   = 0xA1,
  SLAVE_REG_AUX4_UV0   = 0xA2,
  SLAVE_REG_AUX4_UV1   = 0xA3,
  SLAVE_REG_AUX4_OV0   = 0xA4,
  SLAVE_REG_AUX4_OV1   = 0xA5,
  SLAVE_REG_AUX5_UV0   = 0xA6,
  SLAVE_REG_AUX5_UV1   = 0xA7,
  SLAVE_REG_AUX5_OV0   = 0xA8,
  SLAVE_REG_AUX5_OV1   = 0xA9,
  SLAVE_REG_AUX6_UV0   = 0xAA,
  SLAVE_REG_AUX6_UV1   = 0xAB,
  SLAVE_REG_AUX6_OV0   = 0xAC,
  SLAVE_REG_AUX6_OV1   = 0xAD,
  SLAVE_REG_AUX7_UV0   = 0xAE,
  SLAVE_REG_AUX7_UV1   = 0xAF,
  SLAVE_REG_AUX7_OV0   = 0xB0,
  SLAVE_REG_AUX7_OV1   = 0xB1,
  SLAVE_REG_LOT_NUM0   = 0xBE,
  SLAVE_REG_LOT_NUM1   = 0xBF,
  SLAVE_REG_LOT_NUM2   = 0xC0,
  SLAVE_REG_LOT_NUM3   = 0xC1,
  SLAVE_REG_LOT_NUM4   = 0xC2,
  SLAVE_REG_LOT_NUM5   = 0xC3,
  SLAVE_REG_LOT_NUM6   = 0xC4,
  SLAVE_REG_LOT_NUM7   = 0xC5,
  SLAVE_REG_SER_NUM0   = 0xC6,
  SLAVE_REG_SER_NUM1   = 0xC7,
  SLAVE_REG_SCRATCH0   = 0xC8,
  SLAVE_REG_SCRATCH1   = 0xC9,
  SLAVE_REG_SCRATCH2   = 0xCA,
  SLAVE_REG_SCRATCH3   = 0xCB,
  SLAVE_REG_SCRATCH4   = 0xCC,
  SLAVE_REG_SCRATCH5   = 0xCD,
  SLAVE_REG_SCRATCH6   = 0xCE,
  SLAVE_REG_SCRATCH7   = 0xCF,
  SLAVE_REG_VSOFFSET   = 0xD2,
  SLAVE_REG_VSGAIN     = 0xD3,
  SLAVE_REG_AX0OFFSET0 = 0xD4,
  SLAVE_REG_AX0OFFSET1 = 0xD5,
  SLAVE_REG_AX1OFFSET0 = 0xD6,
  SLAVE_REG_AX1OFFSET1 = 0xD7,
  SLAVE_REG_AX2OFFSET0 = 0xD8,
  SLAVE_REG_AX2OFFSET1 = 0xD9,
  SLAVE_REG_AX3OFFSET0 = 0xDA,
  SLAVE_REG_AX3OFFSET1 = 0xDB,
  SLAVE_REG_AX4OFFSET0 = 0xDC,
  SLAVE_REG_AX40FFSET1 = 0xDD,
  SLAVE_REG_AX5OFFSET0 = 0xDE,
  SLAVE_REG_AX5OFFSET1 = 0xDF,
  SLAVE_REG_AX6OFFSET0 = 0xE0,
  SLAVE_REG_AX6OFFSET1 = 0xE1,
  SLAVE_REG_AX7OFFSET0 = 0xE2,
  SLAVE_REG_AX7OFFSET1 = 0xE3,
  SLAVE_REG_TSTR_ECC0  = 0xE6,
  SLAVE_REG_TSTR_ECC1  = 0xE7,
  SLAVE_REG_TSTR_ECC2  = 0xE8,
  SLAVE_REG_TSTR_ECC3  = 0xE9,
  SLAVE_REG_TSTR_ECC4  = 0xEA,
  SLAVE_REG_TSTR_ECC5  = 0xEB,
  SLAVE_REG_TSTR_ECC6  = 0xEC,
  SLAVE_REG_TSTR_ECC7  = 0xED,
  SLAVE_REG_CSUM0      = 0xF0,
  SLAVE_REG_CSUM1      = 0xF1,
  SLAVE_REG_CSUM2      = 0xF2,
  SLAVE_REG_CSUM3      = 0xF3,
  SLAVE_REG_CSUM_RSLT0 = 0xF4,
  SLAVE_REG_CSUM_RSLT1 = 0xF5,
  SLAVE_REG_CSUM_RSLT2 = 0xF6,
  SLAVE_REG_CSUM_RSLT3 = 0xF7,
  SLAVE_REG_TEST_CSUM0 = 0xF8,
  SLAVE_REG_TEST_CSUM1 = 0xF9,
  SLAVE_REG_EE_BURN    = 0xFA,
  SLAVE_REG_MAGIC2_0   = 0xFC,
  SLAVE_REG_MAGIC2_1   = 0xFD,
  SLAVE_REG_MAGIC2_2   = 0xFE,
  SLAVE_REG_MAGIC2_3   = 0xFF
} slave_reg_t;

typedef enum {
  RESPONSE = 0b0,
  COMMAND  = 0b1,
} frame_type_t;

typedef enum {
  SINGLE_WRITE_WITH_RESPONSE       = 0b000,
  SINGLE_WRITE_WITHOUT_RESPONSE    = 0b001,
  GROUP_WRITE_WITH_RESPONSE        = 0b010,
  GROUP_WRITE_WITHOUT_RESPONSE     = 0b011,
  BROADCAST_WRITE_WITH_RESPONSE    = 0b110,
  BROADCAST_WRITE_WITHOUT_RESPONSE = 0b111,
} frame_request_t;

// 8-bit register address size is preferred--All USER registers are addressable in 8-bit
typedef enum {
  REGISTER_ADDRESS_8_BIT  = 0b0,
  REGISTER_ADDRESS_16_BIT = 0b1,
} frame_address_size_t;

// Data size of 7 bytes is not supported
typedef enum {
  DATA_SIZE_0_BYTE = 0b000,
  DATA_SIZE_1_BYTE = 0b001,
  DATA_SIZE_2_BYTE = 0b010,
  DATA_SIZE_3_BYTE = 0b011,
  DATA_SIZE_4_BYTE = 0b100,
  DATA_SIZE_5_BYTE = 0b101,
  DATA_SIZE_6_BYTE = 0b110,
  DATA_SIZE_8_BYTE = 0b111,
} frame_data_size_t;

typedef struct {
  frame_type_t frameType;           // Response or command (0 or 1)
  frame_request_t requestType;      // Board communication specifier
  frame_address_size_t addressSize; // The size of the register address (8 or 16)
  frame_data_size_t dataSize;       // Number of bytes of data (0-6, 8)
  uint8_t responseBytes;            // Number of data bytes contained in response frame minus 1
} frame_init_t;

typedef struct {
  frame_init_t *frameInit;
  uint8_t deviceAddress;
  slave_reg_t registerAddress;
  uint8_t data[MAX_RESPONSE_LENGTH];
} uart_command_t;

typedef uart_command_t uart_response_t;

//-----------------------------------------------------------------------------
// STATIC VARIABLE DEFINITIONS                                                |
//-----------------------------------------------------------------------------
static const uint32_t MAX_COMMAND_LENGTH          = 14;  // 1 Frame Init, 1 Device Address, 2 Register Address, 8 Data, 2 CRC
static const uint32_t MAX_RESPONSE_FRAME_LENGTH   = 1;   // Frame initialization is 1 byte
static const uint32_t MAX_REGISTER_ADDRESS_LENGTH = 2;   // Most registers are 1 byte, some are 2 bytes
//static const uint32_t GPIO_UART_OUTPUT_ENABLE_PIN = 80;

typedef enum {
  UART_FAILURE = 0,
  UART_SUCCESS = 1,
} uart_result_t;

//-----------------------------------------------------------------------------
// GLOBAL INTERFACE FUNCTION PROTOTYPES                                       |
//-----------------------------------------------------------------------------
void uart_init(void);
uart_result_t uart_sendCommand(const uart_command_t *command);
uart_result_t uart_receiveResponse(uart_response_t *response);


#endif /* UART_H_ */
