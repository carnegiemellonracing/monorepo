#include "parser.h"
#include <cJSON.h>
#include <CMR/can_types.h>
#include <CMR/can_ids.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <float.h>
#include <string.h>
#include <cn-cbor/cn-cbor.h>
#include <assert.h>

enum data_type {
    DT_INT8,
    DT_INT16,
    DT_INT32,
    DT_INT64,
    DT_UINT8,
    DT_UINT16,
    DT_UINT32,
    DT_UINT64,
    DT_FLOAT16,
    DT_FLOAT32,
    DT_FLOAT64,
    DT_UNK,
    DT_NUM
};

union sample_value {
    // __fp16 fp16;
    int32_t i32;
    // float32_t f32;
    uint32_t u32;
    uint16_t u16;
    int64_t i64;
    int64_t u64;
    int16_t i16;
    uint8_t u8;
    uint8_t i8;
};

#define MAX_SIGNAL_NAME 30

typedef struct signal {
    uint32_t id;
    size_t offset;
    size_t len;
    char name[MAX_SIGNAL_NAME];
    enum data_type dt;
    uint32_t kind;
} json_signal_t;

#define MAX_VAL_PER_SIG 8

struct sample {
    struct signal sig[MAX_VAL_PER_SIG];
    union sample_value v[MAX_VAL_PER_SIG];
    int values_parsed;
};

static const char JSON_STRING[] = {
  #include "can_fmt.rawh"
};


/* Data type names are <= 9 bytes  */
#define DT_NAME_MAX 10
static const char dtNameMap[][DT_NAME_MAX] = {
    [DT_INT16]   = "i16",
    [DT_INT32]   = "i32",
    [DT_INT64]   = "i64",
    [DT_UINT8]   = "u8",
    [DT_UINT16]  = "u16",
    [DT_UINT32]  = "u32",
    [DT_UINT64]  = "u64",
    [DT_FLOAT16] = "f16",
    [DT_FLOAT32] = "f32",
};

enum data_type dtLookupStr(const char *s) {
    for (int i = 0; i < DT_NUM; i++) {
        if (!strncmp(dtNameMap[i], s, DT_NAME_MAX)) {
            return (enum data_type) i;
        }
    }
    return DT_UNK;
}

#define MAX_SAMPLES 50
#define MAX_SAMPLEVEC_LEN 400
#define MAX_MESSAGE_LEN 2000
static int samples_parsed = 0;
static struct signal sample_map[MAX_SAMPLES];

struct sample_data{
    uint8_t count;
    uint8_t len;
    uint8_t values[MAX_SAMPLEVEC_LEN];
} raw_sample_data[MAX_SAMPLES];

uint8_t raw_msg[MAX_MESSAGE_LEN];

static cn_cbor *msg;
static cn_cbor_errback err;

void parserClearMsg(void) {
    memset(raw_sample_data, 0, sizeof(raw_sample_data));
    memset(raw_msg, 0, sizeof(raw_msg));
}

ssize_t parserFmtMsg(void) {
    for (size_t i = 0; i < MAX_SAMPLES; i++) {
        if (raw_sample_data[i].count) {
            cn_cbor *data = cn_cbor_data_create(
                raw_sample_data[i].values,
                raw_sample_data[i].count * raw_sample_data[i].len,
                &err
            );
            assert(data != NULL);
            int64_t kind = (int64_t) i;

            /* Key is unique by virtue of monotonicity of i */
            bool ret = cn_cbor_mapput_int(msg, kind, data, &err);
            assert(ret);
        }
    }

    ssize_t ret = cn_cbor_encoder_write(
        raw_msg, 0,
        sizeof(raw_msg) / sizeof(raw_msg[0]),
        msg
    );
    cn_cbor_free(msg);
    msg = cn_cbor_map_create(&err);
    return ret;
}

static void add_sample(struct sample *s) {
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

int parseData(uint16_t id, uint8_t msg[], size_t len) {
    struct sample sample;
    struct sample *s = &sample;

    struct signal *sigv[MAX_VAL_PER_SIG];
    int relevant_sigs = 0;
    for (int i = 0; i < samples_parsed; i++) {
        if (relevant_sigs >= MAX_VAL_PER_SIG) {
            /* Uh oh */
            return -1;
        }
        if (sample_map[i].id == id) {
            sigv[relevant_sigs++] = &sample_map[i];
        }
    }
    if (!relevant_sigs) {
        /* Uh oh */
        return -1;
    }

    for (size_t i = 0; i < relevant_sigs; i++) {
        struct signal *sig = sigv[i];

        if (sig->offset + sig->len > len) {
            /* Uh oh */
            return -1;
        }

        void *v_pt = msg + sig->offset;
        switch(sig->dt) {
        case DT_INT8:
            s->v[i].i8 = *(int8_t *) v_pt;
            break;
        case DT_INT16:
            s->v[i].i16 = *(int16_t *) v_pt;
            break;
        case DT_INT32:
            s->v[i].i32 = *(int32_t *) v_pt;
            break;
        case DT_INT64:
            s->v[i].i64 = *(int64_t *) v_pt;
            break;
        case DT_UINT8:
            s->v[i].u8 = *(uint8_t *) v_pt;
            break;
        case DT_UINT16:
            s->v[i].u16 = *(uint16_t *) v_pt;
            break;
        case DT_UINT32:
            s->v[i].u32 = *(uint32_t *) v_pt;
            break;
        case DT_UINT64:
            s->v[i].u64 = *(uint64_t *) v_pt;
            break;
        case DT_FLOAT16:
            /* Uh oh */
            return -1;
        case DT_FLOAT32:
            /* This should be a float32_t */
            // s->v[i].f32 = *(float *) v_pt;
            return -1;
        case DT_FLOAT64:
            /* Uh oh */
            return -1;
        default:
            /* Not entirely sure what the best course of action is here. */
            return -1;
        }
        s->sig[i] = *sig;
    }

    s->values_parsed = relevant_sigs;
    add_sample(s);
    return 0;
}

int parserInit(void) {
    /* CBOR things */
    msg = cn_cbor_map_create(&err);

    cJSON *json = cJSON_Parse(JSON_STRING);

    const cJSON *name_pt = cJSON_GetObjectItemCaseSensitive(json, "signals");

    uint32_t kind_count = 0;
    struct cJSON *cur;
    cJSON_ArrayForEach(cur, name_pt) {
        struct signal s = {
            .id     = 0,
            .len    = 0,
            .name   = "UNKNOWN_SIGNAL",
            .dt     = DT_UNK,
            .offset = 0,
            s.kind  = kind_count++,
        };
        if (cJSON_IsObject(cur)) {
            struct cJSON *id, *len, *name, *type, *offset;
            id   = cJSON_GetObjectItem(cur, "id");
            len  = cJSON_GetObjectItem(cur, "length");
            name = cJSON_GetObjectItem(cur, "name");
            type = cJSON_GetObjectItem(cur, "type");
            offset = cJSON_GetObjectItem(cur, "offset");
            if (cJSON_IsNumber(id)) {
                s.id = id->valueint;
            }

            if (cJSON_IsNumber(len)) {
                s.len = len->valueint;
            }

            if (cJSON_IsNumber(offset)) {
                s.offset = offset->valueint;
            }

            if (cJSON_IsString(name) && (name->valuestring != NULL)) {
                strncpy(s.name, name->valuestring, sizeof(s.name));
            }

            if (cJSON_IsString(type) && (type->valuestring != NULL)) {
                s.dt = dtLookupStr(type->valuestring);
            }

            sample_map[samples_parsed++] = s;
        }
    }

    cJSON_Delete(json);

    return 0;
}