/**
 * @file gpio.c
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#include "gpio.h"  // Interface to implement
#include "adc.h"

#include <CMR/gpio.h>   // GPIO interface
#include <stm32f4xx_hal.h>  // HAL interface

#include "state.h"

bool gpioButtonStates[NUM_BUTTONS];

bool canLRUDStates[4];
bool gpioLRUDStates[4];

static const uint32_t gpioReadButtons_priority = 5;


/** @brief Button input task task. */
static cmr_task_t gpioReadButtons_task;

bool canButtonStates[NUM_BUTTONS];

/**
 * @brief Board-specific pin configuration.
 *
 * Replace/add more pin configurations here as appropriate. Each enumeration
 * value of `gpio_t` should get a configuration.
 *
 * @see `stm32f4xx_hal_gpio.h` for various initialization values.
 */
static const cmr_gpioPinConfig_t gpioPinConfigs[GPIO_LEN] = {
    // Change LRUD
	//Delete LRUD
	[GPIO_BUTTON_A] = { .port = GPIOC, .init = { .Pin = GPIO_PIN_6,
					.Mode = GPIO_MODE_IT_RISING_FALLING, .Pull = GPIO_PULLUP,
					.Speed = GPIO_SPEED_FREQ_LOW } },
	[GPIO_BUTTON_B] = { .port = GPIOC, .init = { .Pin = GPIO_PIN_7,
				.Mode = GPIO_MODE_IT_RISING_FALLING, .Pull = GPIO_PULLUP,
				.Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_BUTTON_SW1] = { .port = GPIOB, .init = { .Pin = GPIO_PIN_4,
                        .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP,
                        .Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_BUTTON_SW2] = { .port = GPIOB, .init = { .Pin = GPIO_PIN_5,
                        .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP,
                        .Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_BUTTON_PUSH] = { .port = GPIOC, .init = { .Pin = GPIO_PIN_4,
                        .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP,
                        .Speed = GPIO_SPEED_FREQ_LOW } },
	[GPIO_LED_AMS] = { .port = GPIOB, .init = { .Pin = GPIO_PIN_14,
                       .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL,
                       .Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_LED_IMD] = { .port = GPIOB, .init = { .Pin = GPIO_PIN_15,
                       .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL,
                       .Speed = GPIO_SPEED_FREQ_LOW } },
    [GPIO_LED_BSPD] = { .port = GPIOB, .init = { .Pin = GPIO_PIN_13,
                        .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL,
                        .Speed = GPIO_SPEED_FREQ_LOW } },
	[GPIO_LED_STATUS] = { .port = GPIOC, .init = { .Pin = GPIO_PIN_9,
					.Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL,
					.Speed = GPIO_SPEED_FREQ_LOW } },
	[GPIO_PD_N] = { .port = GPIOA, .init = { .Pin = GPIO_PIN_5,
					.Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL,
					.Speed = GPIO_SPEED_FREQ_LOW } }
};


//Define the two variables that tracks rotary input
volatile int RotaryA = 0;
volatile int RotaryB = 0;
/**
* @brief Adds Interrupt and Programs Callback Function
*/
static volatile int rotaryPosition = 0; //This keeps track of rotary position, the important variable, mod 8
static volatile int pastRotaryPosition = 0; //Keeps track of past rotary position, mod 8


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// Only handle GPIO_PIN_0 interrupts
	// if (GPIO_Pin != GPIO_PIN_6) {
	// 	return;
	// }

	// Save current rotary position
	pastRotaryPosition = rotaryPosition;

	// Read current state of both encoder pins
	//AB CW
	//BA CCW
	// int stateA = cmr_gpioRead(GPIO_BUTTON_A);
	int stateB = cmr_gpioRead(GPIO_BUTTON_B);
	// int oldB = ;
	 
	// if 

	// Update position based on encoder states
	// rotaryPosition += (stateA == stateB) ? 1 : -1;
	rotaryPosition = stateB;

	// Request gear change
	if(getCurrState() != CONFIG) {
		reqGear();
	}
	else
	{
		config_increment_up_requested = true;
		config_increment_down_requested = false;
	}
}

//declaration for use
static void XYActivate(void);

/*
debouncing for button presses for LRUD
 */
# define DEBOUNCE_DELAY 50

static uint32_t lastPress[LRUD_LEN] = {0};

static bool lastState[LRUD_LEN] = {false};

// master function for detecting and changing can button states, will be in gpio loop
void canLRUDDetect(void){

	XYActivate();

	for(int i = 0; i < LRUD_LEN; i++){
		bool pressed = true;
		if(gpioLRUDStates[i]){
			bool pressed = true;
			if(lastState[i] == false)
				//new press
				if(getCurrState() != CONFIG)
				{	
					TickType_t press = xTaskGetTickCount();
					while(xTaskGetTickCount() - press <= 1000){
						if(!gpioLRUDStates[i]) {
							lastState[i] = false;
							return;
						}
					}
				}
				lastState[i] = true;
				lastPress[i] = xTaskGetTickCount();
		}
		else {
			if(lastState[i]){
				//pressed and release
				canLRUDStates[i] = (xTaskGetTickCount() - lastPress[i] >= DEBOUNCE_DELAY);
				lastState[i] = false;
			} else {
				// canLRUDStates[i] = false;
			}
		}
	}
}

/**
 * @brief Converts analog ADC to 0-3.3V, then scales that to 0-5V
 *
 * @param analog ADC value between 0-4096. Helper function for adcToXY
 *
 * @return a float between 0-5V.
 */

static float adcToVoltage(uint32_t analog){

	//error check
	if(analog > 4096){
		return -1;
	}else{
		//Scale analog to voltage between 0-5V
		float finalVolt = ((float)analog/4096.0f) * 5.0f;

		return finalVolt;
	}
}


/* Reads ADC input and switch case based on voltage values and has corresponding states boolean variables
		*Case 1: below 0.5V both
		*Case 2: X between 0.5V and 4.5V and Y below 0.5V
		*Case 3: Y between 0.5V and 4.5V and X below 0.5V
		*Case 4: above 4.5V both
		*/
//add can buttons to this
static void XYActivate(void){
	float sensorX = adcToVoltage(cmr_sensorListGetValue(&sensorList, SENSOR_CH_X));
	float sensorY = adcToVoltage(cmr_sensorListGetValue(&sensorList, SENSOR_CH_Y));
	// Both sensors less than 0.5V
    // LEFT
	gpioLRUDStates[RIGHT]  = (sensorX >= 4.5f);
	gpioLRUDStates[LEFT] = (sensorX <= 1.0f);
	gpioLRUDStates[DOWN]    = (sensorY >= 4.5f);
	gpioLRUDStates[UP]  = (sensorY <= 1.0f);
}


/**
 * @brief reads state of all buttons
 */
static void gpioReadButtons(void *pvParameters) {
    (void)pvParameters;
	TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
		uint8_t paddle = adcRead(ADC_PADDLE);
		if (paddle > 50 ) config_increment_up_requested = true;
        // Direct assignment for CAN buttons
        for(int i=0; i<NUM_BUTTONS; i++){
			// Active Low
			canButtonStates[i] = !cmr_gpioRead(i);
        }
		canLRUDDetect();
		vTaskDelayUntil(&lastWakeTime, 100);
    }
}

/**
* @brief Gets rotary position
*/
int getRotaryPosition(){
	return rotaryPosition;
}

/**
* @brief: Gets past rotary position
*/
int getPastRotaryPosition(){
	return pastRotaryPosition;
}


/**
 * @brief Initializes the GPIO interface.
 */
void gpioInit(void) {
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    cmr_gpioPinInit(
        gpioPinConfigs, sizeof(gpioPinConfigs) / sizeof(gpioPinConfigs[0]));
    cmr_taskInit(
        &gpioReadButtons_task,
        "gpioReadButtons",
        gpioReadButtons_priority,
        gpioReadButtons,
        NULL);
}