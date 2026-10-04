/**
 * @file canError.h
 * @brief CAN error logging and summary broadcast.
 *
 * Every CAN IRQ is intercepted (via `-Wl,--wrap=HAL_CAN_IRQHandler`) so bxCAN
 * errors can be recorded before HAL clears them. A per-bus summary is then
 * broadcast at 1 Hz on every bus, so a dead bus is still reported elsewhere.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CAN_ERROR_H
#define CAN_ERROR_H

void canErrorInit(void);

#endif /* CAN_ERROR_H */
