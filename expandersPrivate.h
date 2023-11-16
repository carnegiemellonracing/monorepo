/**
 * @file expandersPrivate.h
 * @brief Private GPIO/ADC expanders interface
 *
 * @author Carnegie Mellon Racing
 */

#ifndef EXPANDERS_PRIVATE_H
#define EXPANDERS_PRIVATE_H

#include "expanders.h"

// /** @brief Represents a PCA9555 command byte. */
// typedef enum {
//     PCA9555_INPUT_PORT_0 = 0x00,    /** Read input from port 0 */
//     PCA9555_INPUT_PORT_1,           /** Read input from port 1 */
//     PCA9555_OUTPUT_PORT_0,          /** Write output to port 0 */
//     PCA9555_OUTPUT_PORT_1,          /** Write output to port 1 */
//     PCA9555_POL_INV_PORT_0,         /** Write polarity inversion to port 0 */
//     PCA9555_POL_INV_PORT_1,         /** Write polarity inversion to port 1 */
//     PCA9555_CONFIG_PORT_0,          /** Write I/O direction config to port 0 */
//     PCA9555_CONFIG_PORT_1           /** Write I/O direction config to port 0 */
// } pca9555Cmd_t;

typedef enum {
    PCA9554_INPUT_PORT = 0x00,    /** Read input */
    PCA9554_OUTPUT_PORT,          /** Write output */
    PCA9554_POL_INV_PORT,         /** Write polarity inversion */
    PCA9554_CONFIG_PORT,          /** Write I/O direction config */
} pca9554Cmd_t;

// TODO: Inconsistent datasheet with pointer byte?
typedef enum {
    AD5593R_POINTER_CONFIG_MODE = 0x00,     /** Configuration mode */
    AD5593R_POINTER_DAC_WR = 0x10,          /** DAC write mode */
    AD5593R_POINTER_ADC_RD = 0x40,          /** ADC readback mode */
    AD5593R_POINTER_DAC_RD = 0x50,          /** DAC readback mode */
    AD5593R_POINTER_GPIO_RD = 0x60,         /** GPIO readback mode */
    AD5593R_POINTER_REG_RD = 0x70,          /** Register readback mode */
} ad5593RPointerByte_t;

typedef enum {
    AD5593R_CTRL_REG_NOP = 0x0,                     /** NOP */
    AD5593R_CTRL_REG_ADC_SEQ = 0x2,                 /** ADC Sequence Register */
    AD5593R_CTRL_REG_GEN = 0x3,                     /** General-Purpose Control Register */
    AD5593R_CTRL_REG_ADC_CONFIG = 0x4,              /** ADC Pin Configuration Register */
    AD5593R_CTRL_REG_DAC_CONFIG = 0x5,              /** DAC Pin Configuration Register */
    AD5593R_CTRL_REG_PULLDWN_CONFIG = 0x6,          /** Pull-Down Configuration Register */
    AD5593R_CTRL_REG_LDAC_MODE = 0x7,               /** LDAC Mode Register */
    AD5593R_CTRL_REG_GPIO_CONFIG = 0x8,             /** GPIO Write Configuration Register */
    AD5593R_CTRL_REG_GPIO_OUTPUT = 0x9,             /** GPIO Write Data Register */
    AD5593R_CTRL_REG_GPIO_INPUT = 0xA,              /** GPIO Read Configuration Register */
    AD5593R_CTRL_REG_PD_REF = 0xB,                  /** Power-Down/Reference Control Register */
    AD5593R_CTRL_REG_GPIO_OPENDRAIN_CONFIG = 0xC,   /** GPIO Open-Drain Configuration Register */
    AD5593R_CTRL_REG_IO_TS_CONFIG = 0xD,            /** Three-State Configuration Register */
    AD5593R_CTRL_REG_SW_RESET = 0xF,                /** Software Reset */
    AD5593R_CTRL_REG_DAC_WR = 0x10                  /** DAC Write Register */
} ad5593RControlRegister_t;

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
typedef struct {
    uint16_t expanderAddress;
    uint8_t port;
    uint8_t pin;
} expanderPinConfig_t;

// typedef struct {
//     expanderPinConfig_t pins[ROTARY_POS_LEN];
// } expanderRotaryConfig_t;

#endif /* EXPANDERS_H */

