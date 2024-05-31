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
#include "uart.h"                           /* transmit freq */
#include "config.h"                         /* downsampling params */
#include <stdlib.h>                         /* NULL */
#include <stdint.h>                         /* Usual suspects */
#include <string.h>                         /* memory calls */
#include <math.h>                           /* round() */
#include <cn-cbor/cn-cbor.h>                /* CBOR encoding */
#include "FreeRTOSConfig.h"                 /* configASSERT */


/**
 *  @brief Look up table for the maximum send counts on each signal according to
 *  the enumerated config. Unfortunately depends implictly on the transfer
 *  frequency
 *
 */
const int count_freq_map[SAMPLE_NUM_FREQS] = {
    [SAMPLE_0HZ] = 0,
    [SAMPLE_1HZ] = 1,
    [SAMPLE_5HZ] = 5,
    [SAMPLE_10HZ] = 10,
    [SAMPLE_50HZ] = 50,
    [SAMPLE_100HZ] = 100,
};

sample_data_t raw_sample_data[MAX_SIGNALS];

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

static ssize_t pack_msg(void) {
    return cn_cbor_encoder_write(
        raw_msg, 0,
        sizeof(raw_msg),
        msg
    );
}

size_t worst_case_length(enum signal_sample_freq *allowance) {
    size_t byte_string_overhead = 3; /* This is the case for CBOR
                                      * byte maps with keys <= 255
                                      * (probably always
                                      * going to be the case). */
   size_t packed_len = 2;           /* CBOR starts with a
                                     * 2-header byte sequence */
   for (int i = 0; i < MAX_SIGNALS; i++) {
        /* For our message structure, we send  */
        size_t availible_length = \
            raw_sample_data[i].count * raw_sample_data[i].len;
        size_t capped_length = count_freq_map[allowance[i]] * \
            raw_sample_data[i].len;

        if (availible_length == 0) {
            /* Don't need to pay overhead for 0 byte vectors */
            continue;
        }

        /* There is a hard limit imposed on this signal's representation in the
         * sample vector, but we might not yet meet that; Use whichever is
         * smaller to calculate the length thus far. */
        if (availible_length <= capped_length) {
            packed_len += availible_length;
        } else {
            packed_len += capped_length;
        }

        /* Factor in the overhead for a given integer key:bytestring value
         * pair. */
        packed_len += byte_string_overhead;
   }

    return packed_len;
}

/**
 * @brief Downsample received messages according to configured cutoff
 * frequencies. The JSON configuration dictates the maximum receive frequency
 * for a given signal (beyond which we truncate),
 * but this is not nearly sufficient to pack all received samples
 * into a packet for the particle.
 * To do this last part, we first aggressively downsample
 * according to our configuration. This may still not be enough, so beyond
 * this point we employ the following heuristic to further cull messages:
 * -Among all signals, choose those with the highest configured send
 * frequency cutoffs. Among these, drop all cutoff one frequency 'tier.'
 * (enumerated in config.h). Note that the lowest cutoff frequency in
 * this search is 0Hz, i.e. we reject the signal entirely.
 * -Repeat until all signals fit
 *
 * @note In this traversal, any ties for which signal to cull
 * will be decided arbitrarily.
 *
 * @warning If you update signal_sample_freq, this will need to change
 * accordingly.
 */
void downsample(void) {
    /* Assume we clear the send buffer at this frequency */
    const int tx_freq_dhz = 10000 / boron_tx_period_ms;
    /* Can't have this spilling 0 */
    configASSERT(boron_tx_period_ms <= 10000);

    /* Current cutoff for each signal being sampled in this packet */
    enum signal_sample_freq sample_freq_allowance[MAX_SIGNALS];
    for (size_t i = 0; i < MAX_SIGNALS; i++) {
        enum signal_sample_freq sig_cutoff = (enum signal_sample_freq) \
                current_settings.signal_cfg[i].sample_cutoff_freq;
        configASSERT(sig_cutoff < SAMPLE_NUM_FREQS);
        /* Update the allowance */
        sample_freq_allowance[i] = sig_cutoff;
    }

    /* Try to propose a fair allowance of samples within the message
     * until we can fit them all into this outgoing message. */
    /* Start at the highest frequency we can ingest at.
     * @warning this does not automatically reflect its true value,
     * MAX_SAMPLEVEC_LEN/MIN_SAMPLE_LENGTH/tx_freq_dhz */
    for (
        enum signal_sample_freq current_level = SAMPLE_100HZ;
        current_level > SAMPLE_0HZ;
        current_level--
    ) {
        for (size_t i = 0; i < MAX_SIGNALS; i++) {
            if (worst_case_length(sample_freq_allowance) < MAX_MESSAGE_LEN) {
                break;
            } else if (sample_freq_allowance[i] == SAMPLE_0HZ) {
                /* No use drawing blood from a stone */
                continue;
            }

            enum signal_sample_freq sig_cutoff = (enum signal_sample_freq) \
                current_settings.signal_cfg[i].sample_cutoff_freq;
            configASSERT(sig_cutoff < SAMPLE_NUM_FREQS);

            /* Math on enums is pretty terrible, but it makes sense here. */
            if (sig_cutoff < current_level) {
                /* Signal cutoff is below this level of the search */
                continue;
            }

            int sample_freq_hz = raw_sample_data[i].count * tx_freq_dhz / 10;
            if (sample_freq_hz >= count_freq_map[current_level]) {
                /* Found a candidate for downsampling.
                 * Knock this signal down a cutoff level and continue
                 * search. */
                /* Math on enums is pretty terrible, but it makes sense here. */
                sample_freq_allowance[i]--;
            }
        }
    }

    /* We can now pack everything down according to sample_freq_allowance */
    for (size_t i = 0; i < MAX_SIGNALS; i++) {
        struct sample_data *sample = &raw_sample_data[i];

        /* If we have 9 samples and we're downsampling to 5 samples,
         * should throw away every other sample. Need to take the ceiling
         * of the real ratio, not the floor */
        int count_allowance = count_freq_map[sample_freq_allowance[i]];
        if (count_allowance == 0 || sample->count == 0) {
            /* Throw them all out now, no need to do anything fancy. */
            sample->count = 0;
            continue;
        }

        /* To actually downsample to the desired frequency, Count along the
         * vector and throw out floating point indices. For example, If we have
         * 210 samples and are targeting 50Hz @ 1Hz transmit frequency, we would
         * keep index 0, increment to index 210/50, round this to get the index
         * to actually keep, and continue. At the end, we'll end up with very
         * close to (but never more than) 50 samples. */
        float float_increment = ((float) sample->count) / ((float) count_allowance);

        /* If our allowance was too generous, use real_index~~float_index */
        if (float_increment < 1.f) {
            float_increment = 1.f;
        }

        /* Time to actually cull samples. Outputs are in the reduced
         * sample->count's */
        /* Could memmove as we go, but I think this is simpler to understand. */
        uint8_t temp_samplevec[MAX_SAMPLEVEC_LEN];
        int new_samplevec_len = 0;
        for (
            float fidx = 0.f;
            ;
            fidx += float_increment
        ) {
            /* Get the adjusted floating value. */
            int real_idx = (int) round(fidx);

            /* We may or may not have exactly the right number of samples,
             * But we'll a) be pretty close, and b) certainly not go over. */
            if (real_idx >= sample->count) {
                break;
            }

            /* Copy this sample value over to save it */
            memcpy(
                &temp_samplevec[sample->len * new_samplevec_len],
                &sample->values[sample->len * real_idx],
                sample->len
            );
            new_samplevec_len++;
        }

        /* Update the backing sample */
        sample->count = new_samplevec_len;
        memcpy(sample->values, temp_samplevec, sample->len * new_samplevec_len);
    }
}

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
//    downsample();

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

    ssize_t ret = pack_msg();

    /* Map is now useless, free all of the data points we mapped in
     * and realloc it. */
    cn_cbor_free(msg);
    msg = cn_cbor_map_create(&err);
    configASSERT(msg != NULL);
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

        void *data_fill_pt = &store->values[store->count * s->sig[i].len];
        memcpy(data_fill_pt, &s->v[i], s->sig[i].len);
        store->len = s->sig[i].len;
        store->count++;
    }
}

/**
 * @brief Initialize the sample manager (CBOR things)
 */
void sampleInit(void) {
    msg = cn_cbor_map_create(&err);
    configASSERT(msg != NULL);
}
