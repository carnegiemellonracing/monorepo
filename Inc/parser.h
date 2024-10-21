/**
 *  @file parser.h
 *  @author Zach Pomper (zbp@cmu.edu)
 *  @brief CAN <-> JSON formatting <-> CBOR transmission
 */

#ifndef _PARSER_H_
#define _PARSER_H_

#include <stdint.h>     /* integer types */
#include <stdlib.h>     /* size_t */
#include <stdbool.h>    /* bool */
#include <cJSON.h>                          /* JSON parsing */

#define MAX_SIGNALS 100

/**
 * @brief Maximum length of a signal name in bytes.
 * @note Name is silently truncated on overrun.
 */
#define MAX_SIGNAL_NAME 30

/**
 * @brief Data type encapsulation for re-interpretation
 * (e.g. you specify f16 in the JSON config, but the incoming
 * signal sends as a u32).
 */
enum data_type {
    DT_INT8,            /**< @brief i8 datatype */
    DT_INT16,           /**< @brief i16 datatype */
    DT_INT32,           /**< @brief i32 datatype */
    DT_INT64,           /**< @brief i64 datatype */
    DT_UINT8,           /**< @brief u8 datatype */
    DT_UINT16,          /**< @brief u16 datatype */
    DT_UINT32,          /**< @brief u32 datatype */
    DT_UINT64,          /**< @brief u64 datatype */
    DT_FLOAT16,         /**< @brief f16 datatype */
    DT_FLOAT32,         /**< @brief f32 datatype */
    DT_FLOAT64,         /**< @brief f64 datatype */
    DT_UNK,             /**< @brief If you see this, something is wrong.*/
    DT_NUM              /**< @brief Number of data types. */
};

/** @brief Some type on which we can apply any conversion gain/bias. It would
 * probably be prudent to make this a floating type. */
typedef double sig_intermediary_val_t;

struct signal {
    uint32_t id;                        /**< @brief Backing can message ID */
    uint32_t bus;                       /**< @brief Backing bus ID */
    size_t offset;                      /**< @brief Backing offset within
                                         * the relevant message */
    size_t out_len;                     /**< @brief Length of each sample
                                         * (in bytes) */
    size_t in_len;                      /**< @brief Length raw received data. */
    char name[MAX_SIGNAL_NAME];         /**< @brief Signal name
                                         * (mostly useful for debugging
                                         * right now, but we have it anyways) */
    enum data_type dt_in;               /**< @brief Type to reinterpret
                                         * input data */
    enum data_type dt_out;              /**< @brief Type to reinterpret
                                         * output data */
    uint32_t kind;                      /**< @brief Index in the configured
                                         * signal vector, unique among
                                         * all signals parsed. */
    sig_intermediary_val_t factor;      /**< @brief Parsed conversion term.
                                         * 1. If not found. */
    sig_intermediary_val_t bias;        /**< @brief Parsed term term.
                                         * 0. If not found. */
    bool enabled;                       /**< @brief Whether this signal
                                         * is enabled for transmission. */
};

void setSignalEnable(uint32_t kind, bool is_enabled);
int parseData(uint32_t bus, uint16_t id, const uint8_t msg[], size_t len);
void parserInit(void);

/**
 * @brief Hard maximum on the length of the signal vector produced by
 * the JSON configuration. (Used by the sampler).
 */

extern int signals_parsed;

#endif  /* _PARSER_H_ */