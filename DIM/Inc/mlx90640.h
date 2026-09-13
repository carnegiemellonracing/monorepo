/**
 * @file mlx90640.h
 * @brief MLX90640 32x24 IR thermal array interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef MLX90640_H
#define MLX90640_H

#include <stdint.h>

#define MLX_COLS 32

void mlx90640Init(void);
void mlx90640GetRow(uint8_t row, int16_t *out);

#endif /* MLX90640_H */
