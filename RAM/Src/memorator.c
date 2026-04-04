/**
 * @file  memorator.c
 * @brief Acts as CAN memorator
 *
 * @author Ayush Garg
 */

#include "memorator.h"  // interface to implement
#include <CMR/tasks.h>  // Task interface
#include <CMR/sdio.h>  // Task interface
#include <stdint.h>     /* integer types */
#include "fatfs.h"
#include "string.h" // memcpy
#include <stdbool.h>


/** @brief memorator priority. */
static const uint32_t memorator_priority = 1;

/** @brief memorator period (milliseconds). */
static const TickType_t memorator_period_ms = 40;

/** @brief Status LED task. */
static cmr_task_t memoratorTask;

/** @brief File Obj */
static FIL filObj;

/** @brief file to write */
char filename[64];

extern char testID_name[9]; 
char prev_testID_name[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; 
bool prev_testID_blank;
int blank_num = 0;  

//forward declarations
static void switchBuffer(void);
static void check_new_testID(); 

static const cmr_gpioPin_t clockPin = (cmr_gpioPin_t){
    .port = GPIOC,
    .pin = GPIO_PIN_12
};

static const cmr_gpioPin_t cmdPin = (cmr_gpioPin_t){
    .port = GPIOD,
    .pin = GPIO_PIN_2
};

static const cmr_gpioPin_t dataPins[4] = {
    [0] = {
        .port = GPIOC,
        .pin = GPIO_PIN_8
    },
    [1] = {
        .port = GPIOC,
        .pin = GPIO_PIN_9
    },
    [2] = {
        .port = GPIOC,
        .pin = GPIO_PIN_10
    },
    [3] = {
        .port = GPIOC,
        .pin = GPIO_PIN_11
    }
};

static const cmr_sdioPinConfig_t sdioPinConfig= {
    .clockPin = clockPin,
    .cmdPin = cmdPin,
    .dataPins = dataPins,
    .dataPinLength = sizeof(dataPins)/sizeof(cmr_gpioPin_t)
};

// CAN can at most transmit 62500 bytes/second. Since we have 3 CAN buses this 
// yields 7500 bytes/second that can be transmitted to us for any given 
// 40ms interval. Therefore we give a buffer of 8192 bytes for headroom
#define BUFFER_SIZE 8192
static uint8_t bufferA[BUFFER_SIZE];
static uint8_t bufferB[BUFFER_SIZE];
static uint16_t bufferLocation = 0;
static uint8_t* txBuffer = bufferA;
static uint8_t* rxBuffer = bufferB;


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
void memoratorWrite(uint16_t ID, RTC_TimeTypeDef timeStamp, uint8_t dataLength,  uint8_t* data) {

    uint8_t toWrite = sizeof(ID) + sizeof(timeStamp) + sizeof(dataLength) + dataLength;

    if (bufferLocation + toWrite >= BUFFER_SIZE)
        return;
    
    //writes to buffer
    memcpy(&rxBuffer[bufferLocation], &ID, sizeof(ID));
    bufferLocation += sizeof(ID);
    memcpy(&rxBuffer[bufferLocation], &(timeStamp.Hours), sizeof(timeStamp.Hours));
    bufferLocation += sizeof(timeStamp.Hours);
    memcpy(&rxBuffer[bufferLocation], &(timeStamp.Minutes), sizeof(timeStamp.Minutes));
    bufferLocation += sizeof(timeStamp.Minutes);
    memcpy(&rxBuffer[bufferLocation], &(timeStamp.Seconds), sizeof(timeStamp.Seconds));
    bufferLocation += sizeof(timeStamp.Seconds);
    memcpy(&rxBuffer[bufferLocation], &(timeStamp.SubSeconds), sizeof(timeStamp.SubSeconds));
    bufferLocation += sizeof(timeStamp.SubSeconds);
    memcpy(&rxBuffer[bufferLocation], &dataLength, sizeof(dataLength));
    bufferLocation += sizeof(dataLength);
    memcpy(&rxBuffer[bufferLocation], data, dataLength);
    bufferLocation += dataLength;

}

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
static void writeToSDCard(void *pvParameters) 
{
    // If needed we can improve error handling here
    static uint16_t oldBufferLocation;
    uint8_t res;

    cmr_sdioInit(&sdioPinConfig);
    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1){
        oldBufferLocation = bufferLocation;
        switchBuffer();
        check_new_testID(); 
        cmr_SDIO_mount();
        //check_new_testID(); //write new names from DAQ live 
        res = cmr_SDIO_openFile(&filObj, filename);
        if (!res){
            cmr_SDIO_write(&filObj, txBuffer, oldBufferLocation);
            cmr_SDIO_closeFile(&filObj);
        }
        cmr_SDIO_unmount();
        vTaskDelayUntil(&lastWakeTime, memorator_period_ms);
    }

}

/**
 * @brief Switches the RX and TX buffer
 *
 * @note This must be a critical section so that way we don't lose CAN messages
 *
 * @return void        Does not return a value.
 */
static void switchBuffer(){
    // Critical Section to ensure atomicity when switching buffers 
    taskENTER_CRITICAL();

    if (rxBuffer == bufferA){
        rxBuffer = bufferB;
        txBuffer = bufferA;
    }
    else{
        rxBuffer = bufferA;
        txBuffer = bufferB;
    }
    bufferLocation = 0;

    taskEXIT_CRITICAL();
}
 
/**
 * @brief checks testID and creates new file if new name 
 *
 * @return void        Does not return a value.
 */
static void check_new_testID(){
    uint8_t count_zeros_testID = 0;
    bool new_name = false; 
    //check testID from DAQ Live against current filename 
    for(int i = 0; i<8; i++){
        if (testID_name[i] != prev_testID_name[i]) {
            new_name = true; 
        }
        if (testID_name[i] == 0) {
            count_zeros_testID++; 
        }
        prev_testID_name[i] = testID_name[i]; 
    }
    //testID is newly blank 
    if (count_zeros_testID == 8 && !prev_testID_blank) {
        RTC_DateTypeDef curDate = getRTCDate();
        RTC_TimeTypeDef curTime = getRTCTime();
        snprintf(filename, sizeof(filename), "%u-%u-%u_%u-%u-%u-%lu.bin", curDate.Month, curDate.Date, curDate.Year, curTime.Hours, curTime.Minutes, curTime.Seconds, curTime.SubSeconds);
        prev_testID_blank = true; 
    } else if (count_zeros_testID != 8 && new_name) {
        snprintf(filename, sizeof(filename), "%s.bin", testID_name); 
        prev_testID_blank = false; 
    }

}

void memoratorInit(){
    bufferLocation = 0;
    RTC_DateTypeDef curDate = getRTCDate();
    RTC_TimeTypeDef curTime = getRTCTime();
    prev_testID_blank = false; //assume we start with no testID from DAQ Live 
    snprintf(filename, sizeof(filename), "%u-%u-%u_%u-%u-%u-%lu.bin", curDate.Month, curDate.Date, curDate.Year, curTime.Hours, curTime.Minutes, curTime.Seconds, curTime.SubSeconds);
    cmr_taskInit(
        &memoratorTask,
        "memorator",
        memorator_priority,
        writeToSDCard,
        NULL
    );
}