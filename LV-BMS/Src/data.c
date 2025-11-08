/**
 * @file data.h
 * @brief Implements LV-BMS CAN transmission functions
 *
 * @author Carnegie Mellon Racing
 */

#include "data.h"

#include <CMR/tasks.h>
#include <CMR/uart.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "adc.h"
#include "bq_interface.h"
#include "can.h"
#include "gpio.h"
#include "i2c.h"
#include "uart.h"

#define CELL_NUM 7
#define VSENSE_CHANNELS 14
#define TEMP_CHANNELS 14
#define TOP_CELL VCELL14_HI
#define NUM_GPIO_CHANNELS 4

uint16_t sendVolt[CELL_NUM];
float rawCellVolts[CELL_NUM];
float cellVoltages[CELL_NUM];
uint8_t cellNum[CELL_NUM];
signed char offset_corr[CELL_NUM];
signed char gain_corr[CELL_NUM];
unsigned int vref_corr;
uint16_t adc_sensen;
uint16_t cellTemps[CELL_NUM];

bool setup = false;


static uint16_t calculateVoltage(uint8_t msb, uint8_t lsb) {
	//formula from TI's code
	//Bitwise OR high byte shifted by 8 and low byte, apply scaling factor

	return (uint16_t) (0.19073*((((uint16_t)msb << 8) | lsb)));
}


uint8_t getVoltages(void) {
    TickType_t time_prev = xTaskGetTickCount();
     uart_command_t read_voltage = {
			.readWrite = SINGLE_READ,
			.dataLen = 1,
			.deviceAddress = 0xFF, //not used!
			.registerAddress = TOP_CELL,
			.data = {VSENSE_CHANNELS*2-1}, //reading high and low for cell 0-VSENSE_CHANNELS
			.crc = {0xFF, 0xFF}
		};
    
    taskENTER_CRITICAL();

    uart_sendCommand(&read_voltage);
    uart_response_t response;

    uint8_t status = uart_receiveResponse(&response, 27);
    if(status != 0) {
        return 1;
        taskEXIT_CRITICAL();
    }

    taskEXIT_CRITICAL();


    for(uint8_t j = 0; j < VSENSE_CHANNELS; j++) {
        uint8_t high_byte_data = response.data[2*j];
        uint8_t low_byte_data = response.data[2*j+1];
        cellVoltages[VSENSE_CHANNELS-j-1] = calculateVoltage(high_byte_data, low_byte_data);
    }

    vTaskDelayUntil(&time_prev, 32);

    sendOvervoltageFlags(cellVoltages);
    
    cmr_canLVBMS_Voltage cell1_4;
    cmr_canLVBMS_Voltage cell5_7;
    cell1_4.cell1 = sendVolt[0];
    cell1_4.cell2 = sendVolt[1];
    cell1_4.cell3 = sendVolt[2];
    cell1_4.cell4 = sendVolt[3];
    cell5_7.cell1 = sendVolt[4];
    cell5_7.cell2 = sendVolt[5];
    cell5_7.cell3 = sendVolt[6];

    canTX(CMR_CANID_LVBMS_CELL_VOLTAGE_1_4, &cell1_4, sizeof(cell1_4), canTX10Hz_period_ms);
    canTX(CMR_CANID_LVBMS_CELL_VOLTAGE_5_7, &cell5_7, sizeof(cell5_7), canTX10Hz_period_ms);
    return 0;
}


// Sends overvoltage flags
void sendOvervoltageFlags(uint16_t voltages[CELL_NUM]) {
    uint8_t flag = 0;
    uint8_t overVolt = 0; // TBD

    for (int i = 0; i < CELL_NUM; i++) {
        if (voltages[i] > overVolt) flag |= (1 << i);
    }

    canTX(CMR_CANID_LVBMS_CELL_OVERVOLTAGE, &flag, sizeof(flag), canTX10Hz_period_ms);
}

// Sends bus voltage derived from cell voltages
void sendBusVoltage(uint16_t voltages[CELL_NUM]) {
    uint16_t totalVoltage = 0;

    for (int i = 0; i < CELL_NUM; i++) {
        totalVoltage += voltages[i];
    }

    canTX(CMR_CANID_LVBMS_BUS_VOLTAGE, &totalVoltage, sizeof(totalVoltage), canTX10Hz_period_ms);
}

static uint32_t vtherm_read_index(uint16_t vtherm_index) {

	int map[] = {4, 6, 7, 5, 2, 1, 0, 3, 3, 0, 1, 2, 5, 4, 2, 1}; //based on schematic
	int sel_index = map[vtherm_index];

	cmr_gpioWrite(GPIO_VTHERM_SEL0, sel_index & 0x1);
	cmr_gpioWrite(GPIO_VTHERM_SEL1, sel_index & 0x2);
	cmr_gpioWrite(GPIO_VTHERM_SEL2, sel_index & 0x4);
	if(vtherm_index < 0x8)
		return adc_read(ADC_VTHERM_PIN1);
	return adc_read(ADC_VTHERM_PIN2);
}

/*static void vtherm_read(const adc_channel_t ch) {
	TickType_t time_prev = xTaskGetTickCount();
	while(true) {
		for(uint32_t i = 0; i < VTHERM_NUM; i++) {
			int temp = vtherm_read_index(i);
			(void *)temp;
		}
		int temp = vtherm_read_index(7);
		int temp2 = vtherm_read_index(8);
		temp + temp2;
		vTaskDelayUntil(&time_prev, vtherm_read_period_ms);
	}
}*/
//not sure how temp conversion works rn.
uint16_t tempConvert(uint16_t adc_value) {
    float voltage = (adc_value * VREF_THERM) / ADC_COUNT;
    float resistance = (VREF_THERM * RESISTOR) / voltage - RESISTOR;

    // Steinhart-Hart equation for NTC thermistor
    //float temperature = 1.0 / (A + B * log(resistance) + C * pow(log(resistance), 3)) - 273.15;
    float temperature = 30; //placeholder
    return (uint16_t)(temperature * 100.0f);  //1/100 degree C
}




uint8_t getTemps(int channel) {
    uart_command_t read_therms = {
		.readWrite = SINGLE_READ,
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = GPIO5_HI,
		.data = {0x07},
		.crc = {0xFF, 0xFF}
	};

    taskENTER_CRITICAL();
	uart_sendCommand(&read_therms);

    uart_response_t response;

    if(uart_receiveResponse(&response, CELL_NUM) == UART_FAILURE) {
        return 1;
    }

	taskEXIT_CRITICAL();

    for(uint8_t k = 0; k < NUM_GPIO_CHANNELS; k++) {
        uint8_t high_byte_data = response.data[2*k];
        uint8_t low_byte_data = response.data[2*k+1];
        size_t index = (4*channel) + k;
        //TODO: make sure this is matching the thermistor indices properly

        cellTemps[index] = calculateTemp(high_byte_data, low_byte_data);
    }

    uint16_t data1[4], data2[3];

    // Split temperatures into two groups
    memcpy(data1, cellTemps, 4 * sizeof(uint16_t));     // Temps 1-4
    memcpy(data2, &cellTemps[4], 3 * sizeof(uint16_t)); // Temps 5-7

    canTX(CMR_CANID_LVBMS_CELL_TEMP_1_4, data1, sizeof(data1), canTX10Hz_period_ms);
    canTX(CMR_CANID_LVBMS_CELL_TEMP_5_7, data2, sizeof(data2), canTX10Hz_period_ms);
 
    return 0;
}

// Sends overtemperature flags (derived from temperature data)
void sendOvertempFlags(uint16_t temps[8]) {
    uint8_t flag = 0;

    for (int i = 0; i < 8; i++) {
        if (temps[i] > 6000) flag |= (1 << i);
    }

    // Send CAN messages
    canTX(CMR_CANID_LVBMS_CELL_OVERTEMP, &flag, sizeof(flag), canTX10Hz_period_ms);
}

// Sends the bus current
void sendCurrent(void) {
    uint16_t sensep = adc_read(ADC_AFE_VIOUT);
    float current = float_to_uint16((sensep - adc_sensen)*vref_corr/(ADC_COUNT*GVCOUT*1e3));
    canTX(CMR_CANID_LVBMS_CURRENT, &current, sizeof(current), canTX10Hz_period_ms);
}