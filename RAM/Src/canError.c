/**
 * @file canError.c
 * @brief CAN error logging and summary broadcast implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <stdbool.h>    // bool

#include <CMR/tasks.h>  // Task interface

#include "canError.h"   // Interface to implement
#include "can.h"        // canTX(), cmr_canBusID_t

/** @brief Number of TX mailboxes in a bxCAN peripheral. */
#define CAN_ERROR_TX_MAILBOXES 3

/** @brief Number of RX FIFOs in a bxCAN peripheral. */
#define CAN_ERROR_RX_FIFOS 2

/** @brief Per-bus CAN error log, broken down by where in bxCAN it occurred. */
typedef struct {
    uint32_t total;             /**< @brief Total error events recorded. */

    /** @brief TX mailbox arbitration lost (TSR.ALSTx), per mailbox. */
    uint32_t txArbitrationLost[CAN_ERROR_TX_MAILBOXES];
    /** @brief TX mailbox transmission error (TSR.TERRx), per mailbox. */
    uint32_t txError[CAN_ERROR_TX_MAILBOXES];
    /** @brief RX FIFO overrun (RFxR.FOVRx), per FIFO. */
    uint32_t rxOverrun[CAN_ERROR_RX_FIFOS];

    uint32_t errorWarning;      /**< @brief Entries into error warning (ESR.EWGF). */
    uint32_t errorPassive;      /**< @brief Entries into error passive (ESR.EPVF). */
    uint32_t busOff;            /**< @brief Entries into bus-off (ESR.BOFF). */

    /** @brief Protocol errors (ESR.LEC), indexed by `canErrorLEC_t`. */
    uint32_t lec[CAN_ERROR_LEC_LEN];
    uint8_t lastLEC;            /**< @brief Most recent nonzero `canErrorLEC_t`. */

    uint8_t tecMax;            /**< @brief Peak transmit error counter (ESR.TEC). */
    uint8_t recMax;             /**< @brief Peak receive error counter (ESR.REC). */
    uint32_t lastESR;           /**< @brief Raw ESR at the most recent error. */
    uint32_t lastTSR;           /**< @brief Raw TSR at the most recent error. */
    TickType_t lastError_ms;    /**< @brief Timestamp of the most recent error. */

    uint32_t prevESRFlags;      /**< @brief EWGF/EPVF/BOFF at last IRQ (edge detection). */
} canErrorLog_t;

/**
 * @brief CAN error logs, indexed by `cmr_canBusID_t`.
 *
 * @note Only written from CAN IRQs, which all share one NVIC priority and
 * therefore never preempt each other.
 */
static volatile canErrorLog_t canErrorLog[CMR_CAN_BUS_NUM];

/** @brief Real HAL IRQ handler (see `-Wl,--wrap=HAL_CAN_IRQHandler`). */
void __real_HAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan);

/** @brief bxCAN peripheral for each bus (must match `canInit()`). */
static CAN_TypeDef *const canErrorInstance[CMR_CAN_BUS_NUM] = {
    [CMR_CAN_BUS_VEH] = CAN3,
    [CMR_CAN_BUS_DAQ] = CAN2,
    [CMR_CAN_BUS_TRAC] = CAN1,
};

/**
 * @brief Maps a bxCAN instance to its bus.
 *
 * @param instance The CAN peripheral.
 *
 * @return The bus index, or `CMR_CAN_BUS_NUM` if unknown.
 */
static cmr_canBusID_t canErrorBus(const CAN_TypeDef *instance) {
    for (cmr_canBusID_t bus = 0; bus < CMR_CAN_BUS_NUM; bus++) {
        if (canErrorInstance[bus] == instance) {
            return bus;
        }
    }
    return CMR_CAN_BUS_NUM;
}

/**
 * @brief Records CAN errors from a register snapshot.
 *
 * Mirrors the error decoding in `HAL_CAN_IRQHandler()`, but keeps which
 * mailbox/FIFO/protocol stage the error came from instead of collapsing it.
 *
 * @warning Called from an interrupt handler!
 */
static void canErrorRecord(
    volatile canErrorLog_t *log,
    uint32_t ier, uint32_t msr, uint32_t tsr,
    uint32_t rf0r, uint32_t rf1r, uint32_t esr
) {
    uint32_t events = 0;

    // TX mailboxes: request completed without TXOK.
    if (ier & CAN_IER_TMEIE) {
        static const uint32_t rqcp[] = { CAN_TSR_RQCP0, CAN_TSR_RQCP1, CAN_TSR_RQCP2 };
        static const uint32_t txok[] = { CAN_TSR_TXOK0, CAN_TSR_TXOK1, CAN_TSR_TXOK2 };
        static const uint32_t alst[] = { CAN_TSR_ALST0, CAN_TSR_ALST1, CAN_TSR_ALST2 };
        static const uint32_t terr[] = { CAN_TSR_TERR0, CAN_TSR_TERR1, CAN_TSR_TERR2 };

        for (size_t i = 0; i < CAN_ERROR_TX_MAILBOXES; i++) {
            if (!(tsr & rqcp[i]) || (tsr & txok[i])) {
                continue;
            }
            if (tsr & alst[i]) {
                log->txArbitrationLost[i]++;
                events++;
            } else if (tsr & terr[i]) {
                log->txError[i]++;
                events++;
            }
        }
    }

    // RX FIFO overruns (flag only cleared by HAL when the IRQ is enabled).
    if ((ier & CAN_IER_FOVIE0) && (rf0r & CAN_RF0R_FOVR0)) {
        log->rxOverrun[0]++;
        events++;
    }
    if ((ier & CAN_IER_FOVIE1) && (rf1r & CAN_RF1R_FOVR1)) {
        log->rxOverrun[1]++;
        events++;
    }

    // Error state flags are levels; count rising edges.
    const uint32_t stateMask = CAN_ESR_EWGF | CAN_ESR_EPVF | CAN_ESR_BOFF;
    uint32_t rising = (esr & stateMask) & ~log->prevESRFlags;
    log->prevESRFlags = esr & stateMask;
    if (rising & CAN_ESR_EWGF) {
        log->errorWarning++;
        events++;
    }
    if (rising & CAN_ESR_EPVF) {
        log->errorPassive++;
        events++;
    }
    if (rising & CAN_ESR_BOFF) {
        log->busOff++;
        events++;
    }

    // Protocol error (LEC); HAL clears LEC after handling ERRI.
    if ((ier & CAN_IER_ERRIE) && (ier & CAN_IER_LECIE) && (msr & CAN_MSR_ERRI)) {
        uint32_t lec = (esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos;
        if (lec != CAN_ERROR_LEC_NONE) {
            log->lec[lec]++;
            log->lastLEC = (uint8_t) lec;
            events++;
        }
    }

    uint8_t tec = (uint8_t) ((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
    uint8_t rec = (uint8_t) ((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
    if (tec > log->tecMax) log->tecMax = tec;
    if (rec > log->recMax) log->recMax = rec;

    if (events != 0) {
        log->total += events;
        log->lastESR = esr;
        log->lastTSR = tsr;
        log->lastError_ms = xTaskGetTickCountFromISR();
    }
}

/**
 * @brief Wraps the HAL CAN IRQ handler to log errors before HAL clears them.
 *
 * Linked in place of `HAL_CAN_IRQHandler` via `-Wl,--wrap`, so the CMR
 * driver's IRQ vectors and error callback are left untouched.
 *
 * @warning Called from an interrupt handler!
 */
void __wrap_HAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan) {
    cmr_canBusID_t bus = canErrorBus(hcan->Instance);
    if (bus < CMR_CAN_BUS_NUM) {
        CAN_TypeDef *instance = hcan->Instance;
        canErrorRecord(
            &canErrorLog[bus],
            READ_REG(instance->IER), READ_REG(instance->MSR),
            READ_REG(instance->TSR), READ_REG(instance->RF0R),
            READ_REG(instance->RF1R), READ_REG(instance->ESR)
        );
    }

    __real_HAL_CAN_IRQHandler(hcan);
}

/** @brief Cumulative counts at the previous summary, for computing deltas. */
typedef struct {
    uint32_t txArbitrationLost;
    uint32_t txError;
    uint32_t rxOverrun;
    uint32_t protocolErrors;
    uint32_t errorWarning;
    uint32_t errorPassive;
    uint32_t busOff;
} canErrorTotals_t;

/** @brief Error summary task priority. */
static const uint32_t canErrorSummary_priority = 2;

/** @brief Error summary period (milliseconds). */
static const TickType_t canErrorSummary_period_ms = 1000;

/** @brief Error summary TX timeout (milliseconds). */
static const TickType_t canErrorSummary_timeout_ms = 10;

/** @brief Error summary task. */
static cmr_task_t canErrorSummary_task;

/**
 * @brief Sums the cumulative error counts for a bus.
 *
 * @note Each field is read atomically, but the ISR may update the log
 * between reads; a count landing in the next summary instead is fine.
 */
static canErrorTotals_t canErrorTotals(const volatile canErrorLog_t *log) {
    canErrorTotals_t totals = {
        .errorWarning = log->errorWarning,
        .errorPassive = log->errorPassive,
        .busOff = log->busOff,
    };
    for (size_t i = 0; i < CAN_ERROR_TX_MAILBOXES; i++) {
        totals.txArbitrationLost += log->txArbitrationLost[i];
        totals.txError += log->txError[i];
    }
    for (size_t i = 0; i < CAN_ERROR_RX_FIFOS; i++) {
        totals.rxOverrun += log->rxOverrun[i];
    }
    for (size_t i = CAN_ERROR_LEC_STUFF; i <= CAN_ERROR_LEC_CRC; i++) {
        totals.protocolErrors += log->lec[i];
    }
    return totals;
}

/**
 * @brief Saturating delta between two cumulative counts.
 *
 * @param saturated Set to true if the delta did not fit.
 */
static uint8_t canErrorDelta(uint32_t now, uint32_t prev, bool *saturated) {
    uint32_t delta = now - prev;
    if (delta > UINT8_MAX) {
        *saturated = true;
        return UINT8_MAX;
    }
    return (uint8_t) delta;
}

/**
 * @brief Broadcasts every bus's CAN error summary on every bus at 1 Hz.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void canErrorSummary(void *pvParameters) {
    (void) pvParameters;

    canErrorTotals_t prev[CMR_CAN_BUS_NUM] = { 0 };
    canErrorSummary_t summaries[CMR_CAN_BUS_NUM];

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&lastWakeTime, canErrorSummary_period_ms);

        for (cmr_canBusID_t bus = 0; bus < CMR_CAN_BUS_NUM; bus++) {
            canErrorTotals_t now = canErrorTotals(&canErrorLog[bus]);
            uint32_t esr = READ_REG(canErrorInstance[bus]->ESR);

            bool saturated = false;
            canErrorSummary_t summary = {
                .tec = (uint8_t) ((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos),
                .rec = (uint8_t) ((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos),
                .txArbitrationLost = canErrorDelta(
                    now.txArbitrationLost, prev[bus].txArbitrationLost, &saturated
                ),
                .txError = canErrorDelta(now.txError, prev[bus].txError, &saturated),
                .rxOverrun = canErrorDelta(now.rxOverrun, prev[bus].rxOverrun, &saturated),
                .protocolErrors = canErrorDelta(
                    now.protocolErrors, prev[bus].protocolErrors, &saturated
                ),
                .lastLEC = canErrorLog[bus].lastLEC,
            };

            if (esr & CAN_ESR_EWGF) summary.flags |= CAN_ERROR_SUMMARY_WARNING;
            if (esr & CAN_ESR_EPVF) summary.flags |= CAN_ERROR_SUMMARY_PASSIVE;
            if (esr & CAN_ESR_BOFF) summary.flags |= CAN_ERROR_SUMMARY_BUS_OFF;
            if (now.errorWarning != prev[bus].errorWarning) {
                summary.flags |= CAN_ERROR_SUMMARY_WARNING_ENTER;
            }
            if (now.errorPassive != prev[bus].errorPassive) {
                summary.flags |= CAN_ERROR_SUMMARY_PASSIVE_ENTER;
            }
            if (now.busOff != prev[bus].busOff) {
                summary.flags |= CAN_ERROR_SUMMARY_BUS_OFF_ENTER;
            }
            if (saturated) summary.flags |= CAN_ERROR_SUMMARY_SATURATED;

            prev[bus] = now;
            summaries[bus] = summary;
        }

        // Send every summary on every bus, so a dead bus is still reported.
        for (cmr_canBusID_t txBus = 0; txBus < CMR_CAN_BUS_NUM; txBus++) {
            for (cmr_canBusID_t bus = 0; bus < CMR_CAN_BUS_NUM; bus++) {
                canTX(
                    txBus, CAN_ERROR_SUMMARY_ID_BASE + bus,
                    &summaries[bus], sizeof(summaries[bus]),
                    canErrorSummary_timeout_ms
                );
            }
        }
    }
}

/**
 * @brief Initializes CAN error logging and the summary task.
 *
 * @warning Must be called after `canInit()`.
 */
void canErrorInit(void) {
    // CMR driver doesn't enable RX FIFO overrun IRQs; enable them for logging.
    for (cmr_canBusID_t bus = 0; bus < CMR_CAN_BUS_NUM; bus++) {
        SET_BIT(
            canErrorInstance[bus]->IER,
            CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO1_OVERRUN
        );
    }

    cmr_taskInit(
        &canErrorSummary_task,
        "canErrorSummary",
        canErrorSummary_priority,
        canErrorSummary,
        NULL
    );
}
