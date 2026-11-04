// Copyright 2026 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once
/*
Radio firmware per half: the answer to ORYX_CMD_GET_RADIO_INFO.

The answer is one ORYX_EVT_RADIO_INFO report carrying an oryx_radio_info_t in
bytes 1..26, zero after. It may come later than the command: the board asks
the left nRF and answers when it replies or after a timeout, with flags 0 when
it never did. A board with no radio answers at once with flags 0. Wired, when
the left has no current right tag, the board reads the right half's own
running hash and confirm state over the cable before answering.

Hashes are the leading bytes of the running image's MCUboot hash TLV (what the
SMP image list reports), so a host can compare both halves against a bundle.
*/

#include <stdint.h>
#include "oryx.h"

#define ORYX_RADIO_INFO_FORMAT 1

/* oryx_radio_info_t.flags */
#define ORYX_RADIO_INFO_F_LEFT_VALID      0x01 /* left version and hash read */
#define ORYX_RADIO_INFO_F_LEFT_CONFIRMED  0x02 /* MCUboot has the left image confirmed */
#define ORYX_RADIO_INFO_F_RIGHT_VALID     0x04 /* right_tag is the right's current report */
#define ORYX_RADIO_INFO_F_RIGHT_CONFIRMED 0x08 /* the right reports its image confirmed */

#define ORYX_RADIO_INFO_HASH_LEN 8

typedef struct __attribute__((packed)) {
    uint8_t  format; /* ORYX_RADIO_INFO_FORMAT */
    uint8_t  flags;  /* ORYX_RADIO_INFO_F_* */
    uint8_t  left_major; /* left MCUboot header version, slot 0 */
    uint8_t  left_minor;
    uint16_t left_revision;
    uint32_t left_build_num;
    uint8_t  left_hash[ORYX_RADIO_INFO_HASH_LEN];
    uint8_t  right_tag[ORYX_RADIO_INFO_HASH_LEN]; /* zero unless RIGHT_VALID */
} oryx_radio_info_t;

_Static_assert(sizeof(oryx_radio_info_t) == 26, "radio info is a wire layout");

/* Weak hook, overridden by the wireless module: answer now or later with
 * oryx_radio_info_send(from, ...). The default answers at once with flags 0. */
void oryx_radio_info_request(oryx_transport_t from);

/* Sends one ORYX_EVT_RADIO_INFO to the given transport. */
void oryx_radio_info_send(oryx_transport_t to, const oryx_radio_info_t *info);
