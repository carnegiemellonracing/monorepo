/**
 * @file dwt.h
 * @brief Interface of DWT
 *
 * @author Ayush Garg and Yi-An Liao
 */

uint32_t DWT_Delay_Init(void);
void  DWT_Delay_us(volatile uint32_t au32_microseconds);
void DWT_Delay_ms(volatile uint32_t au32_milliseconds);