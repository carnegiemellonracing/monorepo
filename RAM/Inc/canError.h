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

#include <stdint.h>     // uint8_t

/**
 * @brief bxCAN last error code (ESR.LEC) values.
 *
 * @note Indexes `canErrorLog_t.lec`. See RM0430 CAN_ESR.
 */
typedef enum {
    CAN_ERROR_LEC_NONE = 0,         /**< @brief No error. */
    CAN_ERROR_LEC_STUFF,            /**< @brief Stuff error. */
    CAN_ERROR_LEC_FORM,             /**< @brief Form error. */
    CAN_ERROR_LEC_ACK,              /**< @brief Acknowledgment error. */
    CAN_ERROR_LEC_BIT_RECESSIVE,    /**< @brief Bit recessive error. */
    CAN_ERROR_LEC_BIT_DOMINANT,     /**< @brief Bit dominant error. */
    CAN_ERROR_LEC_CRC,              /**< @brief CRC error. */
    CAN_ERROR_LEC_SOFTWARE,         /**< @brief Set by software (unused). */
    CAN_ERROR_LEC_LEN
} canErrorLEC_t;

/**
 * @brief CAN error summary IDs; the bus being summarized is added as an offset
 * (`CAN_ERROR_SUMMARY_ID_BASE + cmr_canBusID_t`).
 *
 * @todo Move into `CMR/can_ids.h` once the format settles.
 */
#define CAN_ERROR_SUMMARY_ID_BASE 0x7C0

/** @brief Error summary flag bits (`canErrorSummary_t.flags`). */
typedef enum {
    CAN_ERROR_SUMMARY_WARNING       = (1 << 0), /**< @brief Currently error warning. */
    CAN_ERROR_SUMMARY_PASSIVE       = (1 << 1), /**< @brief Currently error passive. */
    CAN_ERROR_SUMMARY_BUS_OFF       = (1 << 2), /**< @brief Currently bus-off. */
    CAN_ERROR_SUMMARY_WARNING_ENTER = (1 << 3), /**< @brief Entered error warning this period. */
    CAN_ERROR_SUMMARY_PASSIVE_ENTER = (1 << 4), /**< @brief Entered error passive this period. */
    CAN_ERROR_SUMMARY_BUS_OFF_ENTER = (1 << 5), /**< @brief Entered bus-off this period. */
    CAN_ERROR_SUMMARY_SATURATED     = (1 << 6), /**< @brief A count below hit 255. */
} canErrorSummaryFlags_t;

/**
 * @brief Per-bus CAN error summary message.
 *
 * @note Counts are errors since the previous summary, saturating at 255.
 */
typedef struct {
    uint8_t flags;              /**< @brief `canErrorSummaryFlags_t` bits. */
    uint8_t tec;                /**< @brief Current transmit error counter. */
    uint8_t rec;                /**< @brief Current receive error counter. */
    uint8_t txArbitrationLost;  /**< @brief TX arbitration lost, all mailboxes. */
    uint8_t txError;            /**< @brief TX errors, all mailboxes. */
    uint8_t rxOverrun;          /**< @brief RX FIFO overruns, both FIFOs. */
    uint8_t protocolErrors;     /**< @brief LEC errors (stuff/form/ack/bit/CRC). */
    uint8_t lastLEC;            /**< @brief Most recent `canErrorLEC_t` ever seen. */
} canErrorSummary_t;
_Static_assert(sizeof(canErrorSummary_t) == 8, "Summary must fit one CAN frame");

void canErrorInit(void);

#endif /* CAN_ERROR_H */
