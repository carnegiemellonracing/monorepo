/**
 ******************************************************************************
 * @file    main.c
 * @brief   Bare-bones STM32F413 example: ITM (SWO) char output + PB7 LED blink
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"
#include <CMR/panic.h>

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/can.h>    // CAN interface
#include <CMR/adc.h>    // ADC interface
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/tasks.h>  // Task interface

#include "adc.h"        // Board-specific ADC interface
#include "assi.h"
#include "can.h"        // Board-specific CAN interface
#include "dac.h"        // Board-specific DAC interface
#include "error.h"
#include "gpio.h"       // Board-specific GPIO interface
#include "sensors.h"    // Board-specific sensors interface
#include "statusLED.h"  // Status LED
#include "state.h"      // stateInit()
#include "pwm.h"
#include "tssi.h"       // TSSI control 

/* Nucleo user LED (LD2) is on PB7 for some F413 nucleo boards -- adjust if needed */
#define LED_GPIO_PORT   GPIOB
#define LED_GPIO_PIN    GPIO_PIN_7

static void SystemClock_Config(void);
static void ITM_Init(void);
static void GPIO_Init(void);
static void ITM_SendChar_Blocking(uint8_t ch);

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    // Peripheral configuration.
    ITM->LAR = 0xC5ACCE55;
    DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN; 

  // Clear the fields we're about to modify
    DWT->CTRL &= ~(
        (0xFu << DWT_CTRL_POSTPRESET_Pos) |
        DWT_CTRL_CYCTAP_Msk |
        DWT_CTRL_PCSAMPLENA_Msk
    );

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    
    TPI->SPPR = 2;                  // NRZ
    TPI->ACPR = 47;                 // 96 MHz -> 2 MHz
    TPI->FFCR = 0;
    

    ITM->TCR = ITM_TCR_ITMENA_Msk | ITM_TCR_SWOENA_Msk | ITM_TCR_DWTENA_Msk | ITM_TCR_SYNCENA_Msk ;
    ITM->TER = 0;

    // POSTPRESET = 3
    DWT->CTRL &= ~(0xFu << DWT_CTRL_POSTPRESET_Pos);
    DWT->CTRL |= (0xFu << DWT_CTRL_POSTPRESET_Pos);

    // Sample using CYCCNT bit 10
    DWT->CTRL |= DWT_CTRL_CYCTAP_Msk;

    // Enable the cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Sync packet every 2^23 cycles (~87ms @ 96MHz) — use 3 (2^27, ~1.4s) if you want fewer syncs once things work
    DWT->CTRL |= (0x1u << DWT_CTRL_SYNCTAP_Pos);

    // Enable PC sampling
    DWT->CTRL |= DWT_CTRL_PCSAMPLENA_Msk;

    gpioInit();
    
    pwmInit();
    canInit();
    adcInit();
    sensorsInit();
    stateInit();
    tssiInit();
    assiInit();

    statusLEDInit();

    vTaskStartScheduler();
    cmr_panic("vTaskStartScheduler returned!");
}

/**
 * @brief Enable ITM stimulus port 0 for SWO output.
 */
static void ITM_Init(void)
{
    /* Enable trace pin (SWO) output */
    DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN;

    /* Enable trace subsystem (DWT, ITM) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Enable DWT cycle counter -- orbstat uses this for its timestamp field */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    /* Enable stimulus port 0 (general chars) and port 1 (profiling data) */
    ITM->TER |= (1UL << 0) | (1UL << 1);

    /* Enable ITM itself */
    ITM->TCR |= ITM_TCR_ITMENA_Msk;
}


/**
 * @brief Send one character on ITM stimulus port 0, waiting if FIFO is busy.
 */
static void ITM_SendChar_Blocking(uint8_t ch)
{
    /* Only attempt to send if ITM/port enabled and a debugger has SWO attached */
    if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&
        ((ITM->TER & (1UL << 0)) != 0UL))
    {
        while (ITM->PORT[0].u32 == 0UL)
        {
            /* wait for FIFO ready */
        }
        ITM->PORT[0].u8 = ch;
    }
}

/**
 * @brief GPIO init for PB7 (Nucleo LED) as push-pull output.
 */
static void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin   = LED_GPIO_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);
}

/**
 * @brief System Clock Configuration (as provided).
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /* Configure the main internal regulator output voltage */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 96;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        cmr_panic("HAL_RCC_OscConfig() failed!");
    }

    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
        cmr_panic("HAL_RCC_ClockConfig() failed!");
    }
}


void __cyg_profile_func_enter(void *this_fn, void *call_site)
    __attribute__((no_instrument_function));
void __cyg_profile_func_exit(void *this_fn, void *call_site)
    __attribute__((no_instrument_function));
    
#define TRACE_CHANNEL     0U       /* orbstat listens on this ITM channel for profile data */
#define ITM_TIMEOUT_LOOPS 2000UL   /* bounded spin count, not a hang */

static inline int ITM_WriteWord_Timeout(uint32_t channel, uint32_t word)
{
    volatile uint32_t timeout = ITM_TIMEOUT_LOOPS;

    while ((ITM->PORT[channel].u32 == 0UL) && (--timeout))
    {
        /* bounded wait for FIFO slot */
    }

    if (timeout == 0UL)
    {
        return 0; /* gave up -- no capture session draining the FIFO */
    }

    ITM->PORT[channel].u32 = word;
    return 1;
}

void __cyg_profile_func_enter(void *this_fn, void *call_site)
{
    if (!(ITM->TCR & ITM_TCR_ITMENA_Msk)) return;
    if (!(ITM->TER & (1UL << TRACE_CHANNEL))) return;

    uint32_t oldIntStat = __get_PRIMASK();
    __disable_irq();

    /* Tag 0x4 = enter, upper nibble is orbstat's packet-type marker */
    if (ITM_WriteWord_Timeout(TRACE_CHANNEL, (DWT->CYCCNT & 0x03FFFFFFUL) | 0x40000000UL))
    {
        ITM_WriteWord_Timeout(TRACE_CHANNEL, (uint32_t)call_site & 0xFFFFFFFEUL);
        ITM_WriteWord_Timeout(TRACE_CHANNEL, (uint32_t)this_fn  & 0xFFFFFFFEUL);
    }

    __set_PRIMASK(oldIntStat);
}

void __cyg_profile_func_exit(void *this_fn, void *call_site)
{
    if (!(ITM->TCR & ITM_TCR_ITMENA_Msk)) return;
    if (!(ITM->TER & (1UL << TRACE_CHANNEL))) return;

    uint32_t oldIntStat = __get_PRIMASK();
    __disable_irq();

    /* Tag 0x5 = exit */
    if (ITM_WriteWord_Timeout(TRACE_CHANNEL, (DWT->CYCCNT & 0x03FFFFFFUL) | 0x50000000UL))
    {
        ITM_WriteWord_Timeout(TRACE_CHANNEL, (uint32_t)call_site & 0xFFFFFFFEUL);
        ITM_WriteWord_Timeout(TRACE_CHANNEL, (uint32_t)this_fn  & 0xFFFFFFFEUL);
    }

    __set_PRIMASK(oldIntStat);
}