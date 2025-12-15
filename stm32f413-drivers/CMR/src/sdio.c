/**
 * @file sdio.c
 * @brief simple wrapper for SDIO that lets you write to an FATFS SD Card 
 * using an SDIO DMA. Note we choose not to implement reading from SD card 
 * as we are unsure of desired interface of future users.
 *
 * @note We largely follow this guide: 
 *   https://deepbluembedded.com/stm32-sdio-dma-example/
 *
 * @author Ayush Garg
 */

#include "string.h" 

#include <CMR/dma.h>  // Interface to implement
#include "CMR/panic.h"  // cmr_panic()
#include <CMR/rcc.h>  // Clock Enable
#include <CMR/sdio.h>  // Interface to implement
#include <stm32f4xx_hal.h>  // HAL interface


#ifdef HAL_SD_MODULE_ENABLED
#ifdef HAL_DMA_MODULE_ENABLED
// Code should be reasonable easy to port (and might just work) for H7 but 
// untested so we have this include guard to draw attention to it
#ifdef F413 


SD_HandleTypeDef hsd;
uint8_t cmr_SDIO_pinCount;
static DMA_HandleTypeDef hdma_sdio_rx;
static DMA_HandleTypeDef hdma_sdio_tx;
static FATFS FatFs;


//forward declarations
static void cmr_SDIO_pinConfig(cmr_sdioPinConfig_t* pinConfig);
static void cmr_SDIO_dmaConfig();
static void MX_SDIO_SD_Init();


void cmr_sdioInit(cmr_sdioPinConfig_t* pins) {
    cmr_rccSDIOClockEnable();
    cmr_SDIO_pinConfig(pins);
    cmr_SDIO_dmaConfig();
    MX_SDIO_SD_Init();
    cmr_SDIO_dmaConfig();
    cmr_dmaInit(&(hdma_sdio_rx));
    cmr_dmaInit(&(hdma_sdio_tx));
    MX_FATFS_Init();
}

/**
* @brief SDIO Initialization Function. This function is call by the FATFS Lib
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
    hsd.Init.ClockDiv = 0;
}

static void cmr_SDIO_pinConfig(cmr_sdioPinConfig_t* pinConfig){

    //Would reccomend changing the speeds to very high with external pullups
    const cmr_gpioPinConfig_t clockPin = {
        .port = pinConfig->clockPin.port,
        .init = {
            .Pin = pinConfig->clockPin.pin,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF12_SDIO
        }
    };

    const cmr_gpioPinConfig_t cmdPin = {
        .port = pinConfig->cmdPin.port,
        .init = {
            .Pin = pinConfig->cmdPin.pin,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_PULLUP,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF12_SDIO
        }
    };

    cmr_singleGpioPinInit(&clockPin);
    cmr_singleGpioPinInit(&cmdPin);

    // Inits the Data Lines
    cmr_gpioPinConfig_t dataPin;
    for (uint8_t i=0; i<pinConfig->dataPinLength; i++){
        dataPin = (cmr_gpioPinConfig_t) {
            .port = pinConfig->dataPins[i].port,
            .init = {
                .Pin = pinConfig->dataPins[i].pin,
                .Mode = GPIO_MODE_AF_PP,
                .Pull = GPIO_PULLUP,
                .Speed = GPIO_SPEED_FREQ_HIGH,
                .Alternate = GPIO_AF12_SDIO
            }
        };
        cmr_singleGpioPinInit(&dataPin);
    }
    cmr_SDIO_pinCount = pinConfig->dataPinLength;
}   

static void cmr_SDIO_dmaConfig(){

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

    __HAL_LINKDMA(&hsd,hdmarx,hdma_sdio_rx);

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

    __HAL_LINKDMA(&hsd,hdmatx,hdma_sdio_tx);

    /* SDIO interrupt Init */
    HAL_NVIC_SetPriority(SDIO_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SDIO_IRQn);

}

/**
* @brief This function handles SDIO global interrupt.
*/
void SDIO_IRQHandler(void)
{
    HAL_SD_IRQHandler(&hsd);
}

bool cmr_SDIO_mount(void){
    FRESULT FR_Status = f_mount(&FatFs, SDPath, 1);
    return FR_Status != FR_OK;
}

uint32_t cmr_SDIO_remainingSpace(){
    static FATFS* FS_Ptr;
    static DWORD FreeClusters;

    f_getfree("", &FreeClusters, &FS_Ptr);
    uint32_t freeSpace = (uint32_t)(FreeClusters * FS_Ptr->csize * 0.5);
    return freeSpace;
}

bool cmr_SDIO_openFile(FIL* filObj, char* path){
    FRESULT FR_Status = f_open(filObj, path, FA_WRITE | FA_OPEN_APPEND);

    return FR_Status != FR_OK;
}

bool cmr_SDIO_write(FIL* filObj, void* data, uint16_t dataLen){
    UINT WWC; 
    FRESULT FR_Status = f_write(filObj, data, dataLen, &WWC);
    if (FR_Status != FR_OK)
        return true;

    FR_Status = f_sync(filObj);
    return FR_Status != FR_OK;
}

bool cmr_SDIO_closeFile(FIL* fileObj){
    FRESULT FR_Status = f_close(fileObj);
    return FR_Status == FR_OK;
}

bool cmr_SDIO_unmount(){
    FRESULT FR_Status = f_mount(NULL, "", 0);
    return FR_Status == FR_OK;
}


#endif /* F413 */
#endif /* HAL_DMA_MODULE_ENABLED */
#endif /* HAL_SD_MODULE_ENABLED */