// Copyright 2026 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once
/*
Left nRF blob read-back: the answer to ORYX_CMD_GET_BLOB, param[0] the blob
(ORYX_BLOB_FREEZE, ORYX_BLOB_EVENTS), param[1..2] the byte offset, little
endian.

The answer is one ORYX_EVT_BLOB report carrying an oryx_blob_t in bytes
1..31. It may come later than the command: the board relays the read to the
left nRF and answers when it replies or after a timeout. data holds len bytes
from offset, zero after; total_len is the blob's whole size.

ORYX_CMD_FREEZE_SEEN clears the freeze record's unread flag on the left nRF.
Answered at once with one ORYX_EVT_FREEZE_SEEN report, byte 1 a status:
ORYX_BLOB_OK when the command went out, ORYX_BLOB_NO_LINK otherwise.
*/

#include <stdint.h>
#include "oryx.h"

#define ORYX_BLOB_FREEZE 0 /* last stored freeze/fatal record */
#define ORYX_BLOB_EVENTS 1 /* live event history */

#define ORYX_BLOB_CHUNK 24

/* oryx_blob_t.status; values below 0x80 are the left nRF's own */
#define ORYX_BLOB_OK         0
#define ORYX_BLOB_NONE       1    /* no such blob */
#define ORYX_BLOB_BAD_OFFSET 2    /* offset past the blob */
#define ORYX_BLOB_TIMEOUT    0x80 /* the left nRF did not answer */
#define ORYX_BLOB_NO_LINK    0x81 /* no link to the left nRF */
#define ORYX_BLOB_BUSY       0x82 /* another read is in flight */

typedef struct __attribute__((packed)) {
    uint8_t  which;     /* the blob asked for */
    uint8_t  status;    /* ORYX_BLOB_* */
    uint16_t total_len; /* whole blob, little endian */
    uint16_t offset;    /* the offset asked for, little endian */
    uint8_t  len;       /* 0..ORYX_BLOB_CHUNK */
    uint8_t  data[ORYX_BLOB_CHUNK];
} oryx_blob_t;

_Static_assert(sizeof(oryx_blob_t) == 31, "blob chunk is a wire layout");

/* Weak hook, overridden by the wireless module: answer now or later with
 * oryx_blob_send(from, ...). The default answers at once with
 * ORYX_BLOB_NO_LINK. */
void oryx_blob_request(oryx_transport_t from, uint8_t which, uint16_t offset);

/* Sends one ORYX_EVT_BLOB to the given transport. */
void oryx_blob_send(oryx_transport_t to, const oryx_blob_t *blob);

/* Weak hook, overridden by the wireless module: sends the left nRF its
 * FREEZE_SEEN. Returns ORYX_BLOB_OK or ORYX_BLOB_NO_LINK. The default
 * returns ORYX_BLOB_NO_LINK. */
uint8_t oryx_freeze_seen(void);
