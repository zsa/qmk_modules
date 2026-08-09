// Copyright 2026 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once
/*
What the board says about itself when a host asks over the wire.

===========================================================================
WHY THIS EXISTS
===========================================================================

Over USB there was no way to ask a keyboard anything about its state. Battery
in particular: a plugged-in board's right half was not reachable by any host at
all, because the only path that carried it was Bluetooth. This is the answer to
ORYX_CMD_GET_TELEMETRY, and it is expected to keep growing.

===========================================================================
ADDING A FIELD
===========================================================================

Append it where reserved[] starts, shrink reserved[] by the same size, and
raise ORYX_TLM_LEN. Leave ORYX_TLM_VERSION alone: `len` is what a host tests,
so an old host reading a new firmware stops at the fields it knows, and a new
host reading an old firmware sees a short `len` and calls the fields it did not
get unknown instead of reading whatever happened to be there. Raise the version
only when a field that has shipped changes meaning -- the one thing `len`
cannot express. Never renumber an offset and never reuse one.

Then mirror the field in zapp at zapp-hid-proto/src/lib.rs, which decodes by
the same rule.

===========================================================================
UNKNOWN IS NOT ZERO
===========================================================================

Every field carries a value for "we could not find out". A battery level we
failed to read must never arrive as 0 -- that reads as a flat battery, and
telling those two apart is most of the point of this payload. Same for the nav
masks: 0 means "nothing plugged in", ORYX_TLM_NAV_UNKNOWN means "we cannot
tell". Flags carry ORYX_TLM_BAT_KNOWN because a clear bit otherwise reads as a
confident "no".
*/

#include <stdint.h>
#include "oryx.h" // RAW_EPSIZE, the opcodes this payload answers

/* Bumped only when a shipped field changes meaning. See ADDING A FIELD. */
#define ORYX_TLM_VERSION 1

/* Defined bytes after the `len` byte itself. Grows as fields are appended. */
#define ORYX_TLM_LEN 16

/* Battery level we could not read. Never 0: that is a flat battery. */
#define ORYX_TLM_LEVEL_UNKNOWN 0xFF

/* Nav bitmask bits. Deliberately the same values as ZSA_NAV_CAP_TRACKBALL /
 * ZSA_NAV_CAP_TRACKPAD in zsa_uart_proto.h, so the compiled-in mask the
 * wireless module already announces to the nRF copies straight across. */
#define ORYX_TLM_NAV_TRACKBALL 0x01
#define ORYX_TLM_NAV_TRACKPAD  0x02
/* A half whose nav we cannot determine, as opposed to 0 = nothing plugged in. */
#define ORYX_TLM_NAV_UNKNOWN 0xFF

/* Battery flag bits. */
#define ORYX_TLM_BAT_CHARGING 0x01
#define ORYX_TLM_BAT_USB      0x02
/* Clear means the other bits are unknown, not false. */
#define ORYX_TLM_BAT_KNOWN 0x80

/* What the charging policy is doing, as against what the cell is doing --
 * battery_*_flags already says whether current is flowing. This says why. */
#define ORYX_TLM_CHG_UNKNOWN  0xFF /* nothing has told us; e.g. no nRF link */
#define ORYX_TLM_CHG_BATTERY  0    /* no source attached: running off the cells */
#define ORYX_TLM_CHG_FLOOR    1    /* a source is attached but has not identified
                                    * itself, so we are still holding at the
                                    * pre-enumeration 100 mA */
#define ORYX_TLM_CHG_CHARGING 2    /* charging at the rate the source allows */
#define ORYX_TLM_CHG_FULL     3    /* source attached, charger has terminated */

/* A current we could not find out. Zero, because no limit and no charge rate
 * this board can be set to is zero -- the input floor alone is 100 mA. */
#define ORYX_TLM_CURRENT_UNKNOWN 0

/* Where a level came from, so a host can tell a live reading from a relayed
 * one. The distinction matters: the ESB relay keeps its last value when the
 * radio link stops, and a stale number looks exactly like a fresh one. */
#define ORYX_TLM_SRC_UNKNOWN 0 /* nothing produced this reading */
#define ORYX_TLM_SRC_NRF     1 /* the left nRF, over the STM32<->nRF UART link */
#define ORYX_TLM_SRC_ESB     2 /* the right half over ESB, relayed by the left nRF */
#define ORYX_TLM_SRC_WIRED   3 /* the right half, read directly over the TRRS I2C bus */

/* Fills bytes 1..31 of the 32-byte response report, byte 0 being
 * ORYX_EVT_TELEMETRY. */
typedef struct __attribute__((packed)) {
    uint8_t version;              /* ORYX_TLM_VERSION */
    uint8_t len;                  /* defined bytes after this one; ORYX_TLM_LEN */
    uint8_t battery_left;         /* 0-100, or ORYX_TLM_LEVEL_UNKNOWN */
    uint8_t battery_left_flags;   /* ORYX_TLM_BAT_* */
    uint8_t battery_left_source;  /* ORYX_TLM_SRC_* */
    uint8_t battery_right;        /* 0-100, or ORYX_TLM_LEVEL_UNKNOWN */
    uint8_t battery_right_flags;  /* ORYX_TLM_BAT_* */
    uint8_t battery_right_source; /* ORYX_TLM_SRC_* */
    uint8_t nav_build;            /* ORYX_TLM_NAV_* this build can drive; compile-time, always known */
    uint8_t nav_left;             /* ORYX_TLM_NAV_* attached left, or ORYX_TLM_NAV_UNKNOWN */
    uint8_t nav_right;            /* ORYX_TLM_NAV_* attached right, or ORYX_TLM_NAV_UNKNOWN */
    /* Charging. LE16 rather than one byte each: these are milliamps up to
     * 1500, and rounding them to fit a byte would hide exactly the difference
     * between tiers this payload exists to show. */
    uint16_t power_budget_ma;     /* what the source advertised; 0 = none/unidentified */
    uint16_t charge_limit_ma;     /* input current limit in force, or ORYX_TLM_CURRENT_UNKNOWN */
    uint16_t charge_current_ma;   /* charge current in force, or ORYX_TLM_CURRENT_UNKNOWN */
    uint8_t  charge_state;        /* ORYX_TLM_CHG_* */
    uint8_t reserved[13];         /* zero; room for thirteen more one-byte fields */
} oryx_telemetry_t;

_Static_assert(sizeof(oryx_telemetry_t) == RAW_EPSIZE - 1, "telemetry payload must fill the report");

/* Everything unknown, ready for whoever can actually answer. Not memset(0):
 * zero is a real answer for most of these fields. */
static inline void oryx_telemetry_reset(oryx_telemetry_t *tlm) {
    for (uint8_t i = 0; i < sizeof(*tlm); i++) {
        ((uint8_t *)tlm)[i] = 0;
    }
    tlm->version              = ORYX_TLM_VERSION;
    tlm->len                  = ORYX_TLM_LEN;
    tlm->battery_left         = ORYX_TLM_LEVEL_UNKNOWN;
    tlm->battery_right        = ORYX_TLM_LEVEL_UNKNOWN;
    tlm->battery_left_source  = ORYX_TLM_SRC_UNKNOWN;
    tlm->battery_right_source = ORYX_TLM_SRC_UNKNOWN;
    tlm->nav_left             = ORYX_TLM_NAV_UNKNOWN;
    tlm->nav_right            = ORYX_TLM_NAV_UNKNOWN;
    tlm->charge_state         = ORYX_TLM_CHG_UNKNOWN;
    /* The three currents stay 0, which is already their "we do not know". */
}

/* Weak hook. The oryx module owns the wire format but knows none of the
 * answers; a board that has them (Voyager 2: the wireless module) overrides
 * this. The default leaves everything unknown, which is the truth for a board
 * with nothing to report. */
void oryx_telemetry_fill(oryx_telemetry_t *tlm);
