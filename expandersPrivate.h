/**
 * @file expandersPrivate.h
 * @brief Private GPIO expanders interface
 *
 * @author Carnegie Mellon Racing
 */

#ifndef EXPANDERS_PRIVATE_H
#define EXPANDERS_PRIVATE_H

/** @brief Represents a PCA9555 command byte. */
typedef enum {
    PCA9555_INPUT_PORT_0 = 0x00,    /** Read input from port 0 */
    PCA9555_INPUT_PORT_1,           /** Read input from port 1 */
    PCA9555_OUTPUT_PORT_0,          /** Write output to port 0 */
    PCA9555_OUPTUT_PORT_1,          /** Write output to port 1 */
    PCA9555_POL_INV_PORT_0,         /** Write polarity inversion to port 0 */
    PCA9555_POL_INV_PORT_1,         /** Write polarity inversion to port 1 */
    PCA9555_CONFIG_PORT_0,          /** Write I/O direction config to port 0 */
    PCA9555_CONFIG_PORT_1           /** Write I/O direction config to port 0 */
} pca9555Cmd_t;

typedef enum {
    PCA9554_INPUT_PORT = 0x00,    /** Read input */
    PCA9554_OUTPUT_PORT,          /** Write output */
    PCA9554_POL_INV_PORT,         /** Write polarity inversion */
    PCA9554_CONFIG_PORT,          /** Write I/O direction config */
} pca9554Cmd_t;

typedef struct {
    uint16_t expanderAddr;
    uint8_t port;
    uint8_t pin;
    bool value;
} expanderButtonState_t;


#endif /* EXPANDERS_H */

