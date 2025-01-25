/**
 * @file data.h
 * @brief Implements LV-BMS CAN transmission functions
 *
 * @author Carnegie Mellon Racing
 */

#include <stdint.h>
#include <can.c>
#include <i2c.h>
#include <data.h>
#include <adc.h>
#include <CMR/tasks.h>

uint16_t cellVoltages[6];
uint16_t cellTemps[8];
signed char offset_corr[7];
signed char offset_corr[7];
unsigned int vref_corr;
uint16_t adc_sensen;

bool setup = false;


void AFE_SETUP(void){
    if(!setup){
        i2c_write_register(POWER_CTL, 0x0D); //should probbaly throw an error if i2c fails. 
                                               //This turns on the things we need to use btw(vcout,viout).
        adc_sensen = adcRead(ADC_AFE_VIOUT);
        i2c_write_register(CONFIG_1, 0x05); //sets gain to 8. put at 0x00 for 4. swtiches to sensep

        uint8_t reg_value;

        for (index = 0; index < 7; index++) {
            i2c_read_register(VREF_CAL + index, &reg_value) == 0; 
            offset_corr[index] = reg_value >> 4;   // Extract the upper 4 bits for offset
            gain_corr[index] = reg_value & 0x0F;  // Extract the lower 4 bits for gain
        }

        // Read MSBs for VREF offset and gain corrections
        i2c_read_register(VREF_CAL_EXT, &reg_value);
        offset_corr[0] |= (((reg_value & 0x06) << 3) ^ 0x20) - 0x20;
        gain_corr[0]   |= (((reg_value & 0x01) << 4) ^ 0x10) - 0x10;

        // Read MSBs for VC1 and VC2 offset and gain corrections
        i2c_read_register(VC_CAL_EXT_1, &reg_value);
   
        offset_corr[1] |= (((reg_value & 0x80) >> 3) ^ 0x10) - 0x10;
        gain_corr[1]   |= (((reg_value & 0x40) >> 2) ^ 0x10) - 0x10;

        offset_corr[2] |= (((reg_value & 0x20) >> 1) ^ 0x10) - 0x10;
        gain_corr[2]   |= (((reg_value & 0x10)) ^ 0x10) - 0x10;

        // Read MSBs for VC3, VC4, VC5, and VC6 offset and gain corrections
        i2c_read_register(VC_CAL_EXT_2, &reg_value);
        offset_corr[3] |= (((reg_value & 0x80) >> 3) ^ 0x10) - 0x10;
        gain_corr[3]   |= (((reg_value & 0x40) >> 2) ^ 0x10) - 0x10;

        offset_corr[4] |= (((reg_value & 0x20) >> 1) ^ 0x10) - 0x10;
        gain_corr[4]   |= (((reg_value & 0x10)) ^ 0x10) - 0x10;     

        offset_corr[5] |= (((reg_value & 0x08) << 1) ^ 0x10) - 0x10; 
        gain_corr[5]   |= (((reg_value & 0x04) << 2) ^ 0x10) - 0x10; 

        offset_corr[6] |= (((reg_value & 0x02) << 3) ^ 0x10) - 0x10; 
        gain_corr[6]   |= (((reg_value & 0x01) << 4) ^ 0x10) - 0x10; 

        vref_corr = VREF_NOM * (1000L + gain_corr[0]) + offset_corr[0];
        adc_sensen = adcRead(ADC_AFE_VIOUT);

        setup = true;
    }
}



void getVoltages(void) {
    for (uint8_t i = 0; i < 6; i++) {
        i2c_write_and_validate(CELL_CTL, 0x10 | index);  
        vTaskDelay(pdMS_TO_TICKS(32));
            //reference data sheet for formula
        cellVoltages[i] = float_to_uint16(((adcRead(ADC_AFE_VCOUT) * vref_corr + ADC_COUNT * offset_corr[index+1]) \
                       * (1000L + gain_corr[index+1])) /(GVCOUT * ADC_COUNT * 1e6));
                       
    }
    sendOvervoltageFlags(cellVoltages);
    sendVoltages(cellVoltages);
   
}

// Sends cell voltages (1-6) split into two CAN messages
void sendVoltages(uint16_t voltages[6]) {
    uint16_t data1[3], data2[3];

    // Split voltages into two groups
    memcpy(data1, voltages, 3 * sizeof(uint16_t));     // Voltages 1-3
    memcpy(data2, &voltages[3], 3 * sizeof(uint16_t)); // Voltages 4-6

    canTX(CAN_ID_LV_BMS_CELL_VOLTAGE_1_3, data1, sizeof(data1), canTX10Hz_period_ms);
    canTX(CAN_ID_LV_BMS_CELL_VOLTAGE_4_6, data2, sizeof(data2), canTX10Hz_period_ms);
}

// Sends overvoltage flags
void sendOvervoltageFlags(uint16_t voltages[6]) {
    uint8_t flag = 0;
    uint8_t overVolt = 0; // TBD

    for (int i = 0; i < 6; i++) {
        if (voltages[i] > overVolt) flag |= (1 << i);
    }

    canTX(CAN_ID_LV_BMS_CELL_OVERVOLTAGE, &flag, sizeof(flag), canTX10Hz_period_ms);
}

// Sends bus voltage derived from cell voltages
void sendBusVoltage(uint16_t voltages[6]) {
    uint16_t totalVoltage = 0;

    for (int i = 0; i < 6; i++) {
        totalVoltage += voltages[i];
    }

    canTX(CAN_ID_LV_BMS_BUS_VOLTAGE, &totalVoltage, sizeof(totalVoltage), canTX10Hz_period_ms);
}

static uint32_t vtherm_read_index(uint32_t vtherm_index) {

	int map[] = {4, 6, 7, 5, 2, 1, 0, 3, 3, 0, 1, 2, 5, 4, 2, 1};
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

void getTemps(void) {
    // Implement
    for(uint32_t i = 0; i < VTHERM_NUM; i++) {
			int temp = vtherm_read_index(i);
			(void *)temp;
		}
		int temp = vtherm_read_index(7);
		int temp2 = vtherm_read_index(8);
		int final = temp + temp2;
		vTaskDelayUntil(&time_prev, vtherm_read_period_ms);
}

// Sends cell temperatures (1-8) split into two CAN messages
void sendTemps(uint16_t temps[8]) {
    uint16_t data1[4], data2[4];

    // Split temperatures into two groups
    memcpy(data1, temps, 4 * sizeof(uint16_t));     // Temps 1-4
    memcpy(data2, &temps[4], 4 * sizeof(uint16_t)); // Temps 5-8

    canTX(CAN_ID_LV_BMS_CELL_TEMP_1_4, data1, sizeof(data1), canTX10Hz_period_ms);
    canTX(CAN_ID_LV_BMS_CELL_TEMP_5_8, data2, sizeof(data2), canTX10Hz_period_ms);
}

// Sends overtemperature flags (derived from temperature data)
void sendOvertempFlags(uint16_t temps[8]) {
    uint8_t flag = 0;
    
    for (int i = 0; i < 8; i++) {
        if (temps[i] > 6000) flag |= (1 << i);
    }

    // Send CAN messages
    canTX(CAN_ID_LV_BMS_CELL_OVERTEMP, &flag, sizeof(flag), canTX10Hz_period_ms);
}

// Sends the bus current
void sendCurrent(void) {
    // Implement
    uint16_t sensep = ADC_read(ADC_AFE_VIOUT);
    float current = (sensep - adc_sensen)*vref_corr/(ADC_COUNT*GVCOUT*1e3);
    canTX(CAN_ID_LV_BMS_CURRENT, &current, sizeof(current), canTX10Hz_period_ms);
}