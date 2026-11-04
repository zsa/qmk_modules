// Copyright 2026 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once
/*
Field diagnostic records: the answer to ORYX_CMD_GET_DIAG.

Request: byte 1 = record (ORYX_DIAG_REC_*), byte 2 = page, byte 3 = flags
(ORYX_DIAG_FLAG_*). The answer is one ORYX_EVT_DIAG report carrying an
oryx_diag_t in bytes 1..31: one ORYX_DIAG_PAGE_DATA-byte page of the record.
Page n carries record bytes [n * 24, n * 24 + 24).

The STM32 records are built on the spot. The left nRF records are served from
a copy the board fetched over UART: ORYX_DIAG_FLAG_FETCH starts a fetch, and
age_ms says how long ago each page arrived, so a host that sets the flag,
waits, and reads again can tell a fresh page from an old one.

The record layouts belong to the board (Wireless Voyager: struct zsa_stm32_diag in
the wireless module, struct zsa_left_diag in zsa_uart_proto.h). The default
hook answers version 0: no diagnostics.
*/

#include <stdint.h>
#include "oryx_telemetry.h" // RAW_EPSIZE

/* Bumped only when a shipped field changes meaning. */
#define ORYX_DIAG_VERSION 1

#define ORYX_DIAG_PAGE_DATA 24

/* Request byte 1. */
enum oryx_diag_record {
    ORYX_DIAG_REC_STM32      = 0, /* keyboard MCU, this boot */
    ORYX_DIAG_REC_STM32_PREV = 1, /* keyboard MCU, the boot before */
    ORYX_DIAG_REC_LEFT       = 2, /* left nRF, this boot */
    ORYX_DIAG_REC_LEFT_PREV  = 3, /* left nRF, the boot before */
    ORYX_DIAG_REC_COUNT,
};

/* Request byte 3. */
#define ORYX_DIAG_FLAG_FETCH 0x01 /* fetch both left nRF records again */

/* oryx_diag_t.status */
#define ORYX_DIAG_OK    0 /* data holds the page */
#define ORYX_DIAG_NONE  1 /* no such record, or never received from the nRF */
#define ORYX_DIAG_RANGE 2 /* unknown record or page past the end */

/* Unknown or at least this old. */
#define ORYX_DIAG_AGE_MAX 0xFFFF

typedef struct __attribute__((packed)) {
    uint8_t  version;   /* ORYX_DIAG_VERSION, 0 = no diagnostics */
    uint8_t  record;    /* as requested */
    uint8_t  page;      /* as requested */
    uint8_t  status;    /* ORYX_DIAG_* */
    uint8_t  total_len; /* record size in bytes; 0 unless status is OK */
    uint16_t age_ms;    /* left records: since the page arrived; 0 for the STM32 */
    uint8_t  data[ORYX_DIAG_PAGE_DATA];
} oryx_diag_t;

_Static_assert(sizeof(oryx_diag_t) == RAW_EPSIZE - 1, "diag payload must fill the report");

static inline void oryx_diag_reset(oryx_diag_t *d) {
    for (uint8_t i = 0; i < sizeof(*d); i++) {
        ((uint8_t *)d)[i] = 0;
    }
}

/* Weak hook, overridden by the wireless module. The default leaves the block
 * zeroed. */
void oryx_diag_fill(oryx_diag_t *d, uint8_t record, uint8_t page, uint8_t flags);
