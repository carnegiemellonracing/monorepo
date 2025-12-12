/**
 * @file  memorator.c
 * @brief Acts as CAN memorator
 *
 * @author Ayush Garg
 */

#include "memorator.h"  // interface to implement
#include <CMR/tasks.h>  // Task interface
#include <CMR/dma.h>  // Task interface
#include <stdint.h>     /* integer types */
#include <stm32f4xx_hal.h>  // HAL interface
#include "fatfs.h"
#include <stdio.h>
#include <string.h>

/** @brief Status LED priority. */
static const uint32_t memorator_priority = 3;

/** @brief memorator period (milliseconds). */
static const TickType_t memorator_period_ms = 10;

/** @brief Status LED task. */
static cmr_task_t memoratorTask;

int randomBS(void);
void Error_Handler(void);

// CAN can at most transmit 62500 bytes/second. Since we have 3 CAN buses this 
// yields 1875 bytes/second that can be transmitted to us for any given 
// 10ms interval. Therefore we give a buffer of 4096 bytes for headroom
#define BUFFER_SIZE 4096
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
void memoratorWrite(uint16_t ID, uint32_t timeStamp, uint8_t dataLength,  uint8_t* data) {

    uint8_t toWrite = sizeof(ID) + sizeof(timeStamp) + sizeof(dataLength) + dataLength;

    if (bufferLocation + toWrite >= BUFFER_SIZE){
        return;
    }
    //writes to buffer
    memcpy(&rxBuffer[bufferLocation], &ID, sizeof(ID));
    bufferLocation += sizeof(ID);
    memcpy(&rxBuffer[bufferLocation], &timeStamp, sizeof(timeStamp));
    bufferLocation += sizeof(timeStamp);
    memcpy(&rxBuffer[bufferLocation], &dataLength, sizeof(dataLength));
    bufferLocation += sizeof(dataLength);
    memcpy(&rxBuffer[bufferLocation], data, dataLength);
    bufferLocation += dataLength;

}

static void writeToSDCard(void *pvParameters) 
{
    
    randomBS();
    // uint32_t currentBlock = 0;
    // MX_GPIO_Init();
    // MX_DMA_Init();
    // MX_SDIO_SD_Init();
    // MX_FATFS_Init();
    // HAL_SD_MspInit(&hsd);
    // cmr_dmaInit(&hdma_sdio_rx);
    // cmr_dmaInit(&hdma_sdio_tx);
    // SDIO_SDCard_Test();
    // TickType_t lastWakeTime = xTaskGetTickCount();

    while (1){
        
        // // Write block 0
        // if (HAL_SD_WriteBlocks_DMA(&hsd, buffer, currentBlock, BUFFER_SIZE/512) != HAL_OK){
        //     cmr_panic("Write Fail");
        // }
        // currentBlock += 16;
        // bufferLocation = 0;
        // vTaskDelayUntil(&lastWakeTime, 100);
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

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

static void MX_SDIO_SD_Init(void);
static void SDIO_SDCard_Test(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int randomBS(void)
{
    MX_SDIO_SD_Init();
    HAL_SD_MspInit(&hsd);
    cmr_dmaInit(&hdma_sdio_rx);
    cmr_dmaInit(&hdma_sdio_tx);
    MX_FATFS_Init();
    // Test The SDIO SD Card Interface
    SDIO_SDCard_Test();

  while (1)
  {

  }
}

static void SDIO_SDCard_Test(void)
{
  FATFS FatFs;
  FIL Fil;
  FRESULT FR_Status;
  FATFS *FS_Ptr;
  UINT RWC, WWC; // Read/Write Word Counter
  DWORD FreeClusters;
  uint32_t TotalSize, FreeSpace;
  char RW_Buffer[200];
  do
  {
    //------------------[ Mount The SD Card ]--------------------
    FR_Status = f_mount(&FatFs, SDPath, 1);
    if (FR_Status != FR_OK)
    {
      break;
    }
    //------------------[ Get & Print The SD Card Size & Free Space ]--------------------
    f_getfree("", &FreeClusters, &FS_Ptr);
    TotalSize = (uint32_t)((FS_Ptr->n_fatent - 2) * FS_Ptr->csize * 0.5);
    FreeSpace = (uint32_t)(FreeClusters * FS_Ptr->csize * 0.5);
    //------------------[ Open A Text File For Write & Write Data ]--------------------
    //Open the file
    FR_Status = f_open(&Fil, "praket.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
    if(FR_Status != FR_OK)
    {
      break;
    }
    // (1) Write Data To The Text File [ Using f_puts() Function ]
    f_puts("Hello! Anvita How Are you doing)\n", &Fil);
    // (2) Write Data To The Text File [ Using f_write() Function ]
    strcpy(RW_Buffer, "Hello! From STM32 To SD Card Over SDIO + DMA, Using f_write()\r\n");
    f_write(&Fil, RW_Buffer, strlen(RW_Buffer), &WWC);
    // Close The File
    f_close(&Fil);
    //------------------[ Open A Text File For Read & Read Its Data ]--------------------
    // Open The File
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_READ);
    if(FR_Status != FR_OK)
    {
      break;
    }
    // (1) Read The Text File's Data [ Using f_gets() Function ]
    f_gets(RW_Buffer, sizeof(RW_Buffer), &Fil);
    // (2) Read The Text File's Data [ Using f_read() Function ]
    f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC);
    // Close The File
    f_close(&Fil);
    //------------------[ Open An Existing Text File, Update Its Content, Read It Back ]--------------------
    // (1) Open The Existing File For Write (Update)
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_OPEN_EXISTING | FA_WRITE);
    FR_Status = f_lseek(&Fil, f_size(&Fil)); // Move The File Pointer To The EOF (End-Of-File)
    if(FR_Status != FR_OK)
    {
      break;
    }
    // (2) Write New Line of Text Data To The File
    FR_Status = f_puts("This New Line Was Added During File Update!\r\n", &Fil);
    f_close(&Fil);
    memset(RW_Buffer,'\0',sizeof(RW_Buffer)); // Clear The Buffer
    // (3) Read The Contents of The Text File After The Update
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_READ); // Open The File For Read
    f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC);
    f_close(&Fil);
    //------------------[ Delete The Text File ]--------------------
    // Delete The File
    /*
    FR_Status = f_unlink(MyTextFile.txt);
    if (FR_Status != FR_OK){
        sprintf(TxBuffer, "Error! While Deleting The (MyTextFile.txt) File.. \r\n");
        USC_CDC_Print(TxBuffer);
    }
    */
  } while(0);
  //------------------[ Test Complete! Unmount The SD Card ]--------------------
  FR_Status = f_mount(NULL, "", 0);
}

/**
  * @brief SDIO Initialization Function. This function is call by CMR then 
  * by the HAL separately
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  // TODO Play around with this number and see what works
  hsd.Init.ClockDiv = 118;
  // TODO see if 4b is actually working??

}

/**
* @brief SD MSP Initialization. This function is called by the HAL when first
* writing to an SD card and by us to configure the DMAs
*
* @param hsd: SD handle pointer
* @retval None
*/
void HAL_SD_MspInit(SD_HandleTypeDef* hsd)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(hsd->Instance==SDIO)
    {

         /* Peripheral clock enable */
        __HAL_RCC_SDIO_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        /**SDIO GPIO Configuration
        PD2     ------> SDIO_CMD
        PC12     ------> SDIO_CK
        PC8     ------> SDIO_D0
        */
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* SDIO DMA Init */
        /* SDIO_RX Init */
        hdma_sdio_rx.Instance = DMA2_Stream3;
        hdma_sdio_rx.Init.Channel = DMA_CHANNEL_4;
        hdma_sdio_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_sdio_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_sdio_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_sdio_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_sdio_rx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
        hdma_sdio_rx.Init.Mode = DMA_PFCTRL;
        hdma_sdio_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_sdio_rx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
        hdma_sdio_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
        hdma_sdio_rx.Init.MemBurst = DMA_MBURST_INC4;
        hdma_sdio_rx.Init.PeriphBurst = DMA_PBURST_INC4;
        if (HAL_DMA_Init(&hdma_sdio_rx) != HAL_OK)
            cmr_panic("SD Card Tx DMA setup failed!");

        __HAL_LINKDMA(hsd,hdmarx,hdma_sdio_rx);

        /* SDIO_TX Init */
        hdma_sdio_tx.Instance = DMA2_Stream6;
        hdma_sdio_tx.Init.Channel = DMA_CHANNEL_4;
        hdma_sdio_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_sdio_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_sdio_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_sdio_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_sdio_tx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
        hdma_sdio_tx.Init.Mode = DMA_PFCTRL;
        hdma_sdio_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
        hdma_sdio_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
        hdma_sdio_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
        hdma_sdio_tx.Init.MemBurst = DMA_MBURST_INC4;
        hdma_sdio_tx.Init.PeriphBurst = DMA_PBURST_INC4;
        if (HAL_DMA_Init(&hdma_sdio_tx) != HAL_OK)
            cmr_panic("SD Card Tx DMA setup failed!");

        __HAL_LINKDMA(hsd,hdmatx,hdma_sdio_tx);

        /* SDIO interrupt Init */
        HAL_NVIC_SetPriority(SDIO_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(SDIO_IRQn);
  }
}


/**
  * @brief This function handles SDIO global interrupt.
  */
void SDIO_IRQHandler(void)
{
    HAL_SD_IRQHandler(&hsd);
}