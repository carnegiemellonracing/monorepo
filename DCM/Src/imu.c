/**
 * @file imu.c
 * @brief LIS2DW Interface
 *
 * @author Griffin
 */

#include "imu.h"

/** @brief SPI object for the IMU. */
static cmr_spi_t imuSpi;

/** @brief Current pose of the IMU. */
static imuSensorPose_t sensorPose;

/** @brief Current calculated pose of the car. */
static imuCarPose_t carPose;

/** @brief SPI bus parameters for the IMU (see datasheet). */
static const SPI_InitTypeDef imuSpiInit = {
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

/** @brief SPI pins for the IMU. */
static const cmr_spiPinConfig_t imuSpiPins = {
    .mosi = { .port = GPIOA, .pin = GPIO_PIN_7 },
    .miso = { .port = GPIOA, .pin = GPIO_PIN_6 },
    .sck = { .port = GPIOA, .pin = GPIO_PIN_5 },
    .nss = { .port = GPIOA, .pin = GPIO_PIN_4 }
};

/**
 * @brief Read the acceleration value from one of three
 * axis as specified by the enum imuAxisDesignator_t.
 *
 * @note See datasheet pg. 33
 */
int16_t imuReadAccData(imuAxisDesignator_t axis){
	uint8_t addr_l = 0x28 + (axis << 1);
	uint8_t addr_h = addr_l + 1;

	uint16_t val_l = imuRegisterRead(addr_l);
	uint16_t val_h = imuRegisterRead(addr_h);

	return (int16_t)((val_h << 8) + val_l);
}

/** @brief Perform a block read of the acceleration data. */
void imuUpdateSensorPose(){
	uint8_t rx[7] = {0,0,0,0,0,0,0};
	uint8_t reg = 0x28; // Address of first data register.
	reg |= 0x80; // Indicate to IMU this is a read operation.
	uint8_t tx[7] = {reg,0,0,0,0,0,0};
	cmr_spiTXRX(&imuSpi, &tx, &rx, 7);

	sensorPose.acc_x = (int16_t) ((((uint16_t)rx[2]) << 8) + rx[1]);
	sensorPose.acc_y = (int16_t) ((((uint16_t)rx[4]) << 8) + rx[3]);
	sensorPose.acc_z = (int16_t) ((((uint16_t)rx[6]) << 8) + rx[5]);
}

/** @brief Write a value to the specified register. */
void imuRegisterWrite(uint8_t reg, uint8_t val){
	uint8_t tx[2] = {reg, val};
	cmr_spiTXRX(&imuSpi, &tx, NULL, 2);
}

/** @brief Read the specified register. */
uint8_t imuRegisterRead(uint8_t reg){
	uint8_t rx[2] = {0,0};
	reg |= 0x80; // Indicate to IMU this is a read operation.
	uint8_t tx[2] = {reg, 0};
	cmr_spiTXRX(&imuSpi, &tx, &rx, 2);
	return rx[1];
}

/** @brief Retrieve a copy of the current car pose. */
void imuGetCarPose(imuCarPose_t *pose){
	*pose = carPose;
}

/** @brief Retrieve a copy of the current sensor pose. */
void imuGetSensorPose(imuSensorPose_t *pose){
	*pose = sensorPose;
}

/** @brief Pose 1000 Hz TX priority. */
static const uint32_t imuUpdatePose_priority = 7;

/** @brief Pose 1000 Hz TX period (milliseconds). */
static const TickType_t imuUpdatePose_period_ms = 1;

/** @brief Pose 1000 Hz TX task. */
static cmr_task_t imuUpdatePose_task;

/**
 * @brief Task for updating the IMU Pose estimates.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void imuUpdatePose(void *pvParameters) {
    (void) pvParameters;    // Placate compiler.
    imuStart();

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
    	uint8_t drdy = imuRegisterRead(0x27) & 1;
    	if (!drdy) continue;

    	imuUpdateSensorPose();

    	/* Car lateral acceleration is aligned with sensor Y-axis. */
    	carPose.lateral_acc = sensorPose.acc_y;

    	/* Car longitudinal acceleration is a composite
    	 * of the -Z and +X axis acceleration. */
    	carPose.longitude_acc = (int16_t)((float)sensorPose.acc_x * IMU_COS_ANG -
    		(float)sensorPose.acc_z * IMU_SIN_ANG);

    	/* Car vertical acceleration is a composite
		 * of the +Z and -X axis acceleration. */
		carPose.vertical_acc = (int16_t)((float)sensorPose.acc_x * IMU_SIN_ANG +
			(float)sensorPose.acc_z * IMU_COS_ANG);


        vTaskDelayUntil(&lastWakeTime, imuUpdatePose_period_ms);
    }
}

/**
 * @brief Start the IMU. Note that this must be called after
 * init, but from within a task.
 **/
void imuStart(void){
	uint8_t who = imuRegisterRead(0x0F);
	(void) who; // Placate compiler

	// Enable Block Data Update for 'atomic' 16-bit data reads.
	// Disable I2C communication.
	// Enable consecutive register reads.
	imuRegisterWrite(0x21, 0x0E);
	// Set Full-scale values to be +/- 2g with cutoff of (1600/2)Hz
	imuRegisterWrite(0x25, 0x00);
	// Disable the FIFO
	imuRegisterWrite(0x2e, 0x00);
	// Enable High-performance @ 1600Hz
	imuRegisterWrite(0x20, 0x94);
	// Enable Self-Test
	//imuRegisterWrite(0x22, 0x80);
}

/** @brief Initialize the IMU SPI. */
void imuInit(void){
    cmr_spiInit(
        &imuSpi, SPI1, &imuSpiInit, &imuSpiPins,
        NULL, NULL,
        NULL, NULL
    );

    cmr_taskInit(
        &imuUpdatePose_task,
        "IMU Pose Update",
		imuUpdatePose_priority,
		imuUpdatePose,
        NULL
    );
}
