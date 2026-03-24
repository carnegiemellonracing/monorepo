#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

/**
 * @brief Initialize TIM2 as a microsecond counter
 * @note Must be called after clock configuration (96 MHz system clock)
 *       Timer will overflow after ~71.6 minutes (4,294,967,295 µs)
 */
void microsecond_timer_init(void);

/**
 * @brief Get current microsecond count
 * @return Current timer value in microseconds
 */
uint32_t get_time_us(void);

#endif //TIMER_H
