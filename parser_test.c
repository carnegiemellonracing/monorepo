#include "parser.h"
#include <CMR/can_types.h>
#include <CMR/can_ids.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <cn-cbor/cn-cbor.h>

static cn_cbor *msg;
static cn_cbor_errback err;

int main() {
    (void) parserInit();

    /* Attempt to parse a sequence of messages */
    cmr_canHeartbeat_t vsm_msg = {
        .state = CMR_CAN_GLV_ON,
        .error = {0x00,0xF0},
        .warning = {0,0},
    };

    int ret;
    const int num_heartbeats = 10;
    for (int i = 0; i < num_heartbeats; i++) {
        ret = parseData(
            CMR_CANID_HEARTBEAT_VSM, (uint8_t *) &vsm_msg, sizeof(vsm_msg)
        );

        if (ret != 0) {
            printf("Test failed\n");
            exit(-1);
        }
    }

    ssize_t msg_len = parserFmtMsg();
    if (msg_len <= 0) {
        printf("Parser formatting failed\n");
        exit(-1);
    }

    msg = cn_cbor_decode(raw_msg, (size_t) msg_len, &err);

    /* Signal 42 in the current signal vector is the VSM's error vec,
     * try to parse and validate */
    cn_cbor *signal = cn_cbor_mapget_int(msg, 41);
    if (signal == NULL || signal->type != CN_CBOR_BYTES) {
        printf("Didn't find VSM messages in the parsed stream\n");
        exit(-1);
    }

    for (int i = 0; i < num_heartbeats; i++) {
        /* Each sample of the heartbeat should be 4 bytes long */
        const size_t errorvec_len = 4;
        const uint8_t *v_pt = signal->v.bytes + (i * errorvec_len);
        if (
            v_pt[0] != vsm_msg.error[0] ||
            v_pt[1] != vsm_msg.error[1] ||
            v_pt[2] != vsm_msg.warning[0] ||
            v_pt[3] != vsm_msg.warning[1]
        ) {
            printf("Message scrambled while parsing\n");
            exit(-1);
        }
    }

    printf("All tests passed!\n");
    cn_cbor_free(msg);
    return 0;
}
