// Copyright 2026 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once
/*
One Bluetooth host slot: the answer to ORYX_CMD_GET_HOST_SLOTS, param[0] the
slot index (0-based).

The answer is one ORYX_EVT_HOST_SLOTS report carrying an oryx_host_slot_t in
bytes 1..10, zero after. It may come later than the command: the board asks
the left nRF and answers when it replies or after a timeout. slot_count 0
means nothing is known: no radio, or the nRF never answered.

addr is the board's own address on that slot's identity, least significant
byte first. addr_type ORYX_HOST_SLOT_ADDR_UNKNOWN means it is unknown.
*/

#include <stdint.h>
#include "oryx.h"

/* oryx_host_slot_t.flags */
#define ORYX_HOST_SLOT_F_PAIRED    0x01 /* the slot holds a host */
#define ORYX_HOST_SLOT_F_CONNECTED 0x02 /* the slot's host is connected now */

#define ORYX_HOST_SLOT_ADDR_UNKNOWN 0xFF

typedef struct __attribute__((packed)) {
    uint8_t slot;       /* the slot asked for */
    uint8_t slot_count; /* host slots the board keeps, 0 = nothing known */
    uint8_t flags;      /* ORYX_HOST_SLOT_F_* */
    uint8_t addr_type;  /* 0 public, 1 random, ORYX_HOST_SLOT_ADDR_UNKNOWN */
    uint8_t addr[6];    /* least significant byte first */
} oryx_host_slot_t;

_Static_assert(sizeof(oryx_host_slot_t) == 10, "host slot is a wire layout");

/* Weak hook, overridden by the wireless module: answer now or later with
 * oryx_host_slot_send(from, ...). The default answers at once with
 * slot_count 0. */
void oryx_host_slots_request(oryx_transport_t from, uint8_t slot);

/* Sends one ORYX_EVT_HOST_SLOTS to the given transport. */
void oryx_host_slot_send(oryx_transport_t to, const oryx_host_slot_t *info);
