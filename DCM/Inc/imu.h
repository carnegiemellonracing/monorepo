/**
 * @file imu.h
 * @brief LIS2DW Interface
 *
 * https://www.st.com/resource/en/datasheet/lis2dw12.pdf
 *
 * @author Griffin
 */

#ifndef IMU_H
#define IMU_H

#include <CMR/spi.h>    // SPI interface
#include <CMR/tasks.h>  // Task interface

/** @brief cos(63 deg). */
#define IMU_COS_ANG 	0.45399049974

/** @brief sin(63 deg). */
#define IMU_SIN_ANG 	0.89100652419

/**
 * @brief IMU (accel, gyro, magneto) axis labels
 *
 */
typedef enum {
    IMU_AXIS_X = 0, /**<@brief Arbitrary basis vec */
    IMU_AXIS_Y = 1, /**<@brief Arbitrary basis vec */
    IMU_AXIS_Z = 2 /**<@brief Arbitrary basis vec */
} imuAxisDesignator_t;

/**
 * @brief Sensed acceleration values. See the following image for a
 * quick reference orientation:
 * \image html ls2dw_orientation.png
 *
 */
typedef struct {
	int16_t acc_x; /**<@brief Sensed acceleration along x */
	int16_t acc_y; /**<@brief Sensed acceleration along y */
	int16_t acc_z; /**<@brief Sensed acceleration along z */
} imuSensorPose_t;

/**
 * @brief Inferred car accelerations (post-transform)
 *
 */
typedef struct {
	int16_t longitude_acc; /**<@brief (front-back) */
	int16_t lateral_acc; /**<@brief (side-side) */
	int16_t vertical_acc; /**<@brief (up-down) */
} imuCarPose_t;

void imuInit();
void imuStart(void);
uint8_t imuRegisterRead(uint8_t reg);
void imuRegisterWrite(uint8_t reg, uint8_t val);
int16_t imuReadAccData(imuAxisDesignator_t axis);
void imuGetCarPose(imuCarPose_t *pose);

#endif /* IMU_H */

