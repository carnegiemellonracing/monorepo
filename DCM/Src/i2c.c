/**
 * @file i2c.c
 * @brief All I2C functionality on the CDC (i.e. the RTC and FRAM)
 *
 * @author Carnegie Mellon Racing
 */

#include <string.h>

#include <stm32h7xx_hal.h>  // HAL interface
#include <CMR/tasks.h>  // Task interface
#include <CMR/gpio.h>
#include <CMR/panic.h>
#include <CMR/config_screen_helper.h>
#include <CMR/can_ids.h>

#include "i2c.h"
#include "gpio.h"
#include "can.h"

#define SIZE_PER_DRIVER (sizeof(uint8_t) * MAX_MENU_ITEMS)
#define FRAM_ODOMETER_CONFIG_ADDRESS num_values_driver_enum

/** @brief MCU I2C address */
static uint32_t ownAddress = 0x00;

/** @brief FRAM I2C Address */
static uint16_t framAddress = 0b1010000;

/** @brief RTC I2C address */
static uint32_t rtcAddress = 0x68; 

/** @brief FRAM period */
static const uint32_t FRAM_TIMEOUT = 1;

/** @brief Primary, shared I2C interface */
static cmr_i2c_t i2c;

static void rtcInit(void);
static void framInit(void);
static void PollRTC(void *pvParameters);

static cmr_task_t FRAM_task;

static const TickType_t i2cTaskUpdatePeriod_ms = 10;

volatile cmr_can_rtc_data_t time;

volatile float odometer_km;

extern volatile uint8_t parametersFromDIM[];
extern volatile cmr_driver_profile_t currentDriver;
extern volatile bool framWrite_flag;

//forward declaration
static void framUpdate(void *pvParameters);

/**********
 * COMMON *
 **********/

/** @brief Initializes I2C stuff for the CDC */
void i2cInit() {
    cmr_i2cInit(
        &i2c, I2C4,                // TODO: Increase Clock Speed if can't hit deadlines
        0, 0, /* Clock Speed and own address */
        GPIOF, GPIO_PIN_14,         /* Clock Port/Pin */
        GPIOF, GPIO_PIN_15          /* Data Port/Pin */
    );

    cmr_taskInit(
        &FRAM_task,
        "FRAM Task",
        1, /* TODO: magic number */
		framUpdate,
        NULL
    );

    framInit();
}

/********
 * FRAM *
 ********/

static int framRandomRead(cmr_i2c_t *i2c, uint16_t deviceAddress, uint16_t readAddress, uint8_t *readData, uint16_t readDataLength);

/** @brief Initialize the configuration of variables stored in FRAM
 *  FRAM has 32,768 words x 8 bits
 *  https://www.fujitsu.com/us/Images/MB85RC256V-DS501-00017-3v0-E.pdf
 *
 *  @warning Be sure that none of the variables overlap in FRAM memory
 *  Adding one so that odometer is tacked onto the end of FRAM
 */
static cmr_framVariable_t framVarsConfig[num_values_driver_enum + 1];

volatile uint8_t currentParameters[MAX_MENU_ITEMS];

/** @brief Read data from FRAM
 *  @param variable The enum corresponding to the variable name
 *  @param data Pointer to where data should be written to (same size as dataLength)
 */
int framRead(framVariable_t variable, uint8_t *data)
{
    if (variable >= num_values_driver_enum + 1 || data == NULL)
    {
        return -1;
    }

    uint16_t startAddress = framVarsConfig[variable].startAddress;
    size_t dataLength = framVarsConfig[variable].dataLength;
    int ret = 0;

    taskENTER_CRITICAL();
    ret |= cmr_i2cMemRX(&i2c, framAddress, startAddress, 2, data, dataLength, HAL_MAX_DELAY);
    taskEXIT_CRITICAL();

    return ret;
}

/** @brief Write data to FRAM */
int framWrite(framVariable_t variable, uint8_t *data)
{
    if (variable >= num_values_driver_enum + 1)
    {
        return -1;
    }

    int retv_total = 0;

    if (variable == FRAM_ODOMETER_CONFIG_ADDRESS) {
        for (int i = 0; i < framVarsConfig[variable].dataLength; i++) {
            uint16_t address = framVarsConfig[variable].startAddress + i;
            uint8_t command[3] = {
                // First send upper 8 bits of address
                (address >> 8) & 0xFF,
                // Then send lower 8 bits of address
                address & 0xFF,
                // Followed by data
                data[i]
            };
            taskENTER_CRITICAL();
            int ret = cmr_i2cTX(&i2c, framAddress, command,
                                3, 1);
            taskEXIT_CRITICAL();
            retv_total |= ret;
        }
        return retv_total;
    }


    for (int i = 0; i < 17; i++){
        // Add 2 to data length for starting address
        const size_t commandLength = framVarsConfig[variable].dataLength + 2;
        uint8_t command[commandLength];

        uint16_t address = framVarsConfig[variable].startAddress + i;
        // First send upper 8 bits of address
        command[0] = (address >> 8) & 0xFF;
        // Then send lower 8 bits of address
        command[1] = address & 0xFF;
        command[2] = data[i];


        // Send the command
        taskENTER_CRITICAL();
        int ret = cmr_i2cTX(&i2c, framAddress, command,
                            3, 1);
        taskEXIT_CRITICAL();
        retv_total = ret | retv_total;

    }
    return retv_total;
}

/* NOTE: i2cInit() must also be called */
static void framInit()
{
	// TODO: add call to set fram write protect
    // Set Addresses in FRAM Space
    for (framVariable_t i = 0; i < num_values_driver_enum; i++)
    {
        int16_t startAddress = i * SIZE_PER_DRIVER;
        if (startAddress < 0)
        {
            cmr_panic("Trying to use too much FRAM space");
        }
        framVarsConfig[i].startAddress = startAddress;
        framVarsConfig[i].dataLength = SIZE_PER_DRIVER;
    }
    framVarsConfig[FRAM_ODOMETER_CONFIG_ADDRESS].startAddress = FRAM_ODOMETER_CONFIG_ADDRESS * SIZE_PER_DRIVER;
    framVarsConfig[FRAM_ODOMETER_CONFIG_ADDRESS].dataLength = sizeof(float);    // storing odometer in km as a float

    // Enable Write Protection
    //    cmr_gpioWrite(GPIO_FRAM_WP, 1);

// 	  Read the driver's default values into the main_menu array
    int retv = framRead(Default, currentParameters);
	// flush the currentParams into the main_menu_array
    for (int i = 0; i < MAX_MENU_ITEMS; i++){
		config_menu_main_array[i].value.value = currentParameters[i];
	}

    // Read odometer
    retv = framRead(FRAM_ODOMETER_CONFIG_ADDRESS, (uint8_t *)&odometer_km);

    //Use the below to flash FRAM default params the first time the CDC is setup
     for (int i = 0; i < MAX_MENU_ITEMS; i++) {
     	currentParameters[i] = config_menu_main_array[i].value.value;
     }
     for (framVariable_t var = Default; var < num_values_driver_enum; var++)
     {
         uint8_t arr[MAX_MENU_ITEMS] = {0};
         currentParameters[0] = var;
         int retv5 = framWrite(var, currentParameters);
         int retv6 = framRead(var, arr);
         if (memcmp(arr, currentParameters, MAX_MENU_ITEMS) != 0)
         {
             cmr_panic("FRAM init failed");
         }
     }
     odometer_km = 0.0f;
}

/*******
 * RTC *
 *******/

/* i2cInit() must also be called */
static void rtcInit(void) {}

static void framUpdate(void *pvParameters) {
    /* The FRAM Logic is handled below here, keeps i2c functionality together in the
     * same task.
     */
	(void) pvParameters; // Placate compiler.

	TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
		if(framWrite_flag) {
			// FramWrite new driver data
			int retv = framWrite(currentDriver, parametersFromDIM);
			if (retv != 0) cmr_panic("Fram failed");

			// give the hal enough time to flush i2c data
			TickType_t lastWakeTime = xTaskGetTickCount();
			vTaskDelayUntil(&lastWakeTime, 50);

			uint8_t temp_arr[MAX_MENU_ITEMS] = {0};
			framRead(currentDriver, temp_arr);

			// Read to currentParameters to ensure Fram is updated
			framRead(currentDriver, currentParameters);

			if (memcmp(parametersFromDIM, currentParameters, MAX_MENU_ITEMS) == 0){
				// reset the flag
				framWrite_flag = false;
			} else {
				cmr_panic("Fram read and write disagreed");
			}

		}

		volatile cmr_canDIMRequest_t *dimRequest = (volatile cmr_canDIMRequest_t *) canVehicleGetPayload(CANRX_VEH_REQUEST_DIM);
		cmr_driver_profile_t fram_requested_driver = dimRequest->requestedDriver;

		// new driver has been requested
		if(currentDriver != fram_requested_driver){
			// Read the current driver's parameters
			framRead(fram_requested_driver, currentParameters);

			// Update the config_menu_main_array
			for (int i = 0; i < MAX_MENU_ITEMS; i++) {
				config_menu_main_array[i].value.value = currentParameters[i];
			}

			// Switch drivers
			currentDriver = fram_requested_driver;
		}

		// Write odometer
		framWrite(FRAM_ODOMETER_CONFIG_ADDRESS, (uint8_t *)&odometer_km);
		vTaskDelayUntil(&lastWakeTime, i2cTaskUpdatePeriod_ms);
    }
}
