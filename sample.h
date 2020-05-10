/**
 * @file sample.h
 * @brief Parsed sample data to off-board CBOR conversion
 * How to use this interface:
 * Call addSample() for each sample to enqueue.
 * Periodically call sampleFmtMsg, use the raw_msg handle to send the
 * message out.
 * @todo downsample explanation
 * Call sampleClearMsg after consuming the message.
 *
 * @warning must be protected against multiple access externally
 * (most likely between parseData and sampleFmtMsg)
 *
 * @author Carnegie Mellon Racing
 */

#ifndef _SAMPLE_H_
#define _SAMPLE_H_

#include <stdint.h>         /* usual suspects */
#include <unistd.h>         /* ssize_t */
#include <arm_neon.h>       /* floating types */

/**
 * @brief Maximum number of signals subscribing to a given can message.
 * For example, VSM_Heartbeat might be used by VSM_errorvec and VSM_status.
 * This is only really a limit for the sake of stack usage, so it's
 * probably fine to extend this as needed.
 */
#define MAX_VAL_PER_SIG 8

/**
 * @brief Real sample values.
 */
union sample_value {
	float16_t f16;      /**< @brief Sample value. */
    float32_t f32;      /**< @brief Sample value. */
    double    f64;      /**< @brief Sample value. */
    int32_t   i32;      /**< @brief Sample value. */
    uint32_t  u32;      /**< @brief Sample value. */
    uint16_t  u16;      /**< @brief Sample value. */
    int64_t   i64;      /**< @brief Sample value. */
    int64_t   u64;      /**< @brief Sample value. */
    int16_t   i16;      /**< @brief Sample value. */
    uint8_t   u8;       /**< @brief Sample value. */
    int8_t    i8;       /**< @brief Sample value. */
};

/**
 * @brief Sample ingestation
 */
struct sample {
    /**
     * @brief Configuered sample info from parser.
     */
    struct sample_info {
        size_t len;         /**< @brief Length (in bytes) */
        uint32_t kind;      /**< @brief Signal ID */
    } sig[MAX_VAL_PER_SIG]; /**< @brief Information relevant
                             * to each subscribed of the signal. */
    union sample_value v[MAX_VAL_PER_SIG];  /**< @brief Values for each
                                             * subscriber to the signal. */
    int values_parsed;      /**< @brief Number of signals relevant to
                             * this sample.*/
};

void sampleClearMsg(void);
ssize_t sampleFmtMsg(void);
void addSample(struct sample *s);
void sampleInit(void);

/**
 * @brief Maximum length of the outgoing message.
 * @note This is driven by the particle's maximum transmit unit and bandwidth.
 * As we send at 1Hz and a maximum of ~620 b64 bytes, this corresponds
 * to around 450 CBOR bytes @ 1Hz.
 */
#define MAX_MESSAGE_LEN 450
/* The formatted message formatted for transmission */
extern uint8_t raw_msg[MAX_MESSAGE_LEN];

#endif /* _SAMPLE_H_ */
