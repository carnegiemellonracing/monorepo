#include <CMR/timer.h>
#include <stm32f4xx_hal.h>

static TIM_HandleTypeDef htim2;

void microsecond_timer_init(void)
{
    // Enable TIM2 clock
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 95;  // 96 MHz / 96 = 1 MHz (1 count = 1 µs)
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF;  // Max value for 32-bit timer
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    HAL_TIM_Base_Init(&htim2);
    
    HAL_TIM_Base_Start(&htim2);

}

uint32_t get_time_us(void)
{
    return TIM2->CNT;
}


