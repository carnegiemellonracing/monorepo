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

uint16_t cellVoltages[CELL_NUM];
uint16_t cellTemps[CELL_NUM];
signed char offset_corr[CELL_NUM];
signed char gain_corr[CELL_NUM];
unsigned int vref_corr;
uint16_t adc_sensen;

bool setup = false;

uint16_t getTempData(uint8_t index) {
	return cellTemps[index];
}

uint16_t getVoltageData(uint8_t index) {
	return cellVoltages[index];
}


static uint16_t calculateVoltage(uint8_t msb, uint8_t lsb) {
	//formula from TI's code
	//Bitwise OR high byte shifted by 8 and low byte, apply scaling factor

	return (uint16_t) (0.19073*((((uint16_t)msb << 8) | lsb)));
}


uint8_t getVoltages(void) {
    TickType_t time_prev = xTaskGetTickCount();
    uart_command_t read_voltage = {
			.readWrite = BROADCAST_READ,
			.dataLen = 1,
			.deviceAddress = 0xFF, //not used!
			.registerAddress = TOP_CELL,
			.data = {CELL_NUM*2-1}, //reading high and low for cell 0-CELL_NUM
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


    for(uint8_t j = 0; j < CELL_NUM; j++) {
        uint8_t high_byte_data = response.data[2*j];
        uint8_t low_byte_data = response.data[2*j+1];
        cellVoltages[CELL_NUM-j-1] = calculateVoltage(high_byte_data, low_byte_data);
    }

    sendOvervoltageFlags(cellVoltages);

    return 0;
}


// Sends overvoltage flags
void sendOvervoltageFlags(uint16_t voltages[CELL_NUM]) {
    uint8_t flag = 0;
    uint8_t overVolt = 0; // TBD

    for (int i = 0; i < CELL_NUM; i++) {
        if (cellVoltages[i] > overVolt) flag |= (1 << i);
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
		.readWrite = BROADCAST_READ,
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = GPIO1_HI,
		.data = {0x03},
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
        size_t index = (4*k) + channel;
       
        //TODO: make sure this is matching the thermistor indices properly
        if (index < CELL_NUM) {
            cellTemps[index] = calculateTemp(high_byte_data, low_byte_data);
        }
        
    }
 
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


// Main sample task entry point for BMS
void vBMBSampleTask(void *pvParameters) {
    bool ledToggle = false;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	// Main BMS control loop
	while (1) {
		// Loop through the 4 different MUX channels and select a different one
		// We still monitor all voltages each channel switch
		for(uint8_t j = 0; j < 4; j++) {
			setMuxOutput(j);
			xLastWakeTime = xTaskGetTickCount();
			vTaskDelayUntil(&xLastWakeTime, 10);
			pollAllTemperatureData(j);
		}

		// uint8_t err = pollAllVoltageData();
        writeLED(ledToggle);
		ledToggle = !ledToggle;
        vTaskDelayUntil(&xLastWakeTime, 100);

	}
}



/** @brief Sample Task Priority priority. */
static const uint32_t sampleTaskPriority = 2;

/** @brief Sample Task period (milliseconds). */
static const TickType_t sampleTaskPeriod_ms = 100;

/** @brief Sample task. */
static cmr_task_t sampleTask;

void sampleInit(){
    cmr_taskInit(
        &sampleTask,
        "BMS Sample Init",
        sampleTaskPriority,
        vBMBSampleTask,
        NULL
    );
}