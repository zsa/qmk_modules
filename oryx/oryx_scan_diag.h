// Copyright 2026 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once
/*
Main-loop and link counters: the answer to ORYX_CMD_GET_SCAN_DIAG.

The block is a byte-for-byte copy of struct zsa_scan_diag from the wireless
module's zsa_uart_proto.h, re-declared here so the oryx module does not
depend on the wireless module. wireless.c static-asserts the two sizes match
and fills the block; the default hook leaves it zeroed. The same block goes
to the left nRF as ZSA_MSG_CMD_SCAN_DIAG.

Adding a field: append it where reserved[] starts, shrink reserved[] by the
same size, raise ORYX_SCAN_DIAG_LEN. Raise ORYX_SCAN_DIAG_VERSION only when a
shipped field changes meaning. Never renumber or reuse an offset.

version 0 = no window finished yet; the counters are then meaningless.
*/

#include <stdint.h>
#include "oryx_telemetry.h" // RAW_EPSIZE

/* Bumped only when a shipped field changes meaning. See ADDING A FIELD. */
#define ORYX_SCAN_DIAG_VERSION 1

/* Defined bytes after the `len` byte itself. Grows as fields are appended. */
#define ORYX_SCAN_DIAG_LEN 14

/* Same values as ZSA_SCAN_DIAG_FLAG_* in zsa_uart_proto.h. */
#define ORYX_SCAN_DIAG_FLAG_WIRED_RIGHT 0x01 /* the STM32 owns the right bus (cable) */
#define ORYX_SCAN_DIAG_FLAG_USB_HOST    0x02 /* reports go to a USB host */

/* Main-loop and link counters. Layout-identical to struct zsa_scan_diag.
 * Window fields cover the last completed window; cumulative fields wrap at
 * 65535 and are compared as differences. */
typedef struct __attribute__((packed)) {
    uint8_t  version;           /* ORYX_SCAN_DIAG_VERSION, 0 = nothing measured yet */
    uint8_t  len;               /* defined bytes after this one; ORYX_SCAN_DIAG_LEN */
    uint16_t scans;             /* passes in the window */
    uint16_t max_gap_ms;        /* longest gap between two passes in the window */
    uint16_t gaps_over_5ms;     /* gaps > 5 ms in the window */
    uint16_t gaps_over_20ms;    /* gaps > 20 ms in the window */
    uint16_t right_read_errors; /* cumulative right-half expander read failures */
    uint16_t kbd_reports;       /* cumulative keyboard reports sent to the nRF */
    uint16_t flags;             /* ORYX_SCAN_DIAG_FLAG_* */
} oryx_scan_diag_block_t;

_Static_assert(sizeof(oryx_scan_diag_block_t) == 16, "scan diag block is a wire layout");

/* Fills bytes 1..31 of the 32-byte response report, byte 0 being
 * ORYX_EVT_SCAN_DIAG. */
typedef struct __attribute__((packed)) {
    oryx_scan_diag_block_t diag;
    uint8_t                reserved[15]; /* zero; room for more fields */
} oryx_scan_diag_t;

_Static_assert(sizeof(oryx_scan_diag_t) == RAW_EPSIZE - 1, "scan diag payload must fill the report");

/* A block that claims nothing: version 0. */
static inline void oryx_scan_diag_reset(oryx_scan_diag_t *d) {
    for (uint8_t i = 0; i < sizeof(*d); i++) {
        ((uint8_t *)d)[i] = 0;
    }
}

/* Weak hook. The oryx module owns the wire format but knows none of the
 * answers; a board that has them (Wireless Voyager: the wireless module) overrides
 * this. The default leaves the block zeroed. */
void oryx_scan_diag_fill(oryx_scan_diag_t *d);
