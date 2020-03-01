/**
 * @file sample.c
 * @brief Tracking of and management for outgoing samples.
 *
 * @note See sample.h for how to use this.
 * @todo downsample as we go
 *
 * @author Carnegie Mellon Racing
 */

#include "sample.h"                         /* Interface */
#include "parser.h"                         /* MAX_SIGNALS */
#include <stdlib.h>                         /* NULL */
#include <stdint.h>                         /* Usual suspects */
#include <string.h>                         /* memory calls */
#include <cn-cbor/cn-cbor.h>                /* CBOR encoding */
#include "FreeRTOSConfig.h"                 /* configASSERT */

/**
 * @brief Maximum number of queuable sample bytes before coding/
 * downsampling. For practical messages @<= 100Hz and <=4 bytes
 * this is reasonable. In future, we aim to establish 'reasonable'
 * bounds on input frequencies to be able to configure downsampling
 * for signals outside of this 'reasonable' bound,
 * e.g. downsampling a 1 byte message sent at 1KHz to 10Hz.
 * (As of now we would drop the 600/1000 of these sample bytes).
 */
#define MAX_SAMPLEVEC_LEN 500

/**
 * @brief All of the tracked sample data. Fill between transmit periods,
 * empty at the end of one.
 */
static struct sample_data {
    uint8_t count;      /**< @brief Number of samples on this signal */
    uint8_t len;        /**< @brief Length of each sample */
    uint8_t values[MAX_SAMPLEVEC_LEN];  /**< @brief Raw sample values */
} raw_sample_data[MAX_SIGNALS];

/**
 * @brief Encoded message after sampleFmtMsg(). Callers are responsible
 * for protecting against producer/consumer multiple access.
 */
uint8_t raw_msg[MAX_MESSAGE_LEN];

/**
 * @brief CBOR map for encoding
 */
static cn_cbor *msg;
/**
 * @brief This is required by cbor, but we don't care about it
 * (outside of debugging)
 */
static cn_cbor_errback err;

/**
 * @brief Clear the outgoing (raw) message buffer
 */
void sampleClearMsg(void) {
    memset(raw_sample_data, 0, sizeof(raw_sample_data));
    memset(raw_msg, 0, sizeof(raw_msg));
}

/**
 * @brief Encode raw_sample_data into raw_msg.
 * @warning Must clear after using raw_msg (externally)
 * @return ssize_t The (new) length of raw_msg
 */
ssize_t sampleFmtMsg(void) {
    for (size_t i = 0; i < MAX_SIGNALS; i++) {
        if (raw_sample_data[i].count) {
            cn_cbor *data = cn_cbor_data_create(
                raw_sample_data[i].values,
                raw_sample_data[i].count * raw_sample_data[i].len,
                &err
            );

            /* TODO handle properly */
            configASSERT(data != NULL);
            int64_t kind = (int64_t) i;

            /* Key is unique by virtue of monotonicity of i */
            bool ret = cn_cbor_mapput_int(msg, kind, data, &err);

            /* TODO handle properly */
            configASSERT(ret);
        }
    }

    ssize_t ret = cn_cbor_encoder_write(
        raw_msg, 0,
        sizeof(raw_msg) / sizeof(raw_msg[0]),
        msg
    );

    /* Map is now useless, free all of the data points we mapped in
     * and realloc it. */
    cn_cbor_free(msg);
    msg = cn_cbor_map_create(&err);
    return ret;
}

/**
 * @brief Read a sample into the outgoing message queue to be encoded later.
 * @param s The sample to queue.
 */
void addSample(struct sample *s) {
    for (int i = 0; i < s->values_parsed; i++) {
        struct sample_data *store = &raw_sample_data[s->sig[i].kind];
        if (store->count >= MAX_SAMPLEVEC_LEN / s->sig[i].len) {
            /* Uh oh */
            return;
        }

        store->len = s->sig[i].len;
        void *data_fill_pt = &store->values[store->count * store->len];
        memcpy(data_fill_pt, &s->v[i], s->sig[i].len);
        store->count++;
    }
}

/**
 * @brief Initialize the sample manager (CBOR things)
 */
void sampleInit(void) {
    msg = cn_cbor_map_create(&err);
}