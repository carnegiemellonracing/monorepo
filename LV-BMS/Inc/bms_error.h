/*
 * bms_error.h
 *
 *  Created on: Jul 30, 2026
 *      Author: anvitaa 
 */

#ifndef BMS_ERROR_H_
#define BMS_ERROR_H_

typedef enum {
	BMB_NO_ERR = 0,

	BMB_VOLTAGE_READ_ERROR,
	BMB_TEMP_READ_ERROR
} BMB_UART_ERRORS;

#endif /* BMS_ERROR_H_ */
