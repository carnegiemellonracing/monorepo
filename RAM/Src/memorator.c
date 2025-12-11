/**
 * @file  memorator.c
 * @brief Acts as CAN memorator
 *
 * @author Ayush Garg
 */

#include "memorator.h"  // interface to implement
#include <CMR/tasks.h>  // Task interface
#include <stdint.h>     /* integer types */
#include <string.h>     /* memcpy */
#include <stm32f4xx_hal.h>  // HAL interface



/** @brief Status LED priority. */
static const uint32_t memorator_priority = 3;

/** @brief memorator period (milliseconds). */
static const TickType_t memorator_period_ms = 10;

/** @brief Status LED task. */
static cmr_task_t memoratorTask;

/** @brief HAL SD Card Handle */
SD_HandleTypeDef hsd;


// CAN can at most transmit 62500 bytes/second. Since we have 3 CAN buses this 
// yields 1875 bytes/second that can be transmitted to us for any given 
// 10ms interval. Therefore we give a buffer of 4096 bytes for headroom
#define BUFFER_SIZE 512
static uint8_t buffer[BUFFER_SIZE];
static uint16_t bufferLocation = 0;


/**
 * @brief Serialize and write a record into the memorator buffer.
 *
 * This function writes the fields into a global buffer at the current 
 * write offset. Writes occur inside a FreeRTOS critical section to ensure
 * atomicity. If there is not enough space remaining in the buffer,
 * the function exits without writing anything.
 *
 * @param ID           16-bit CAN ID
 * @param timeStamp    32-bit timestamp for the record.
 * @param dataLength   Length of the data payload in bytes.
 * @param data         Pointer to the data payload to write.
 *
 * @return void        Does not return a value.
 */
void memoratorWrite(uint16_t ID, uint32_t timeStamp, uint8_t dataLength,  uint8_t* data) {

    uint8_t toWrite = sizeof(ID) + sizeof(timeStamp) + sizeof(dataLength) + dataLength;

    if (bufferLocation + toWrite >= BUFFER_SIZE){
        return;
    }

    //writes to buffer
    memcpy(&buffer[bufferLocation], &ID, sizeof(ID));
    bufferLocation += sizeof(ID);
    memcpy(&buffer[bufferLocation], &timeStamp, sizeof(timeStamp));
    bufferLocation += sizeof(timeStamp);
    memcpy(&buffer[bufferLocation], &dataLength, sizeof(dataLength));
    bufferLocation += sizeof(dataLength);
    memcpy(&buffer[bufferLocation], data, dataLength);
    bufferLocation += dataLength;

}


static void writeToSDCard(void *pvParameters) 
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_SDIO_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**SDIO GPIO Configuration
    PC8     ------> SDIO_D0
    PC9     ------> SDIO_D1
    PC10     ------> SDIO_D2
    PC11     ------> SDIO_D3
    PC12     ------> SDIO_CK
    PD2     ------> SDIO_CMD
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    TickType_t lastWakeTime = xTaskGetTickCount();


    hsd.Instance = SDIO;
    hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv = 2;
    if (HAL_SD_Init(&hsd) != HAL_OK)
    {
        cmr_panic("SD Card Init Failure");
    }
    // I was unable to get 4 bit wide working :( 
    // Prehaps an issue in hardware
    // if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
    // {
    //     cmr_panic("SD Card 4 Byte Failing");
    // }

    uint32_t currentBlock = 0;
    while (1){
        // Write block 0
        if (HAL_SD_WriteBlocks(&hsd, buffer, currentBlock, BUFFER_SIZE/512, HAL_MAX_DELAY) != HAL_OK){
            cmr_panic("Write Fail");
        }
        currentBlock += 16;
        bufferLocation = 0;
        vTaskDelayUntil(&lastWakeTime, 100);
    }

}

void memoratorInit(){
    bufferLocation = 0;
    cmr_taskInit(
        &memoratorTask,
        "memorator",
        memorator_priority,
        writeToSDCard,
        NULL
    );
}




