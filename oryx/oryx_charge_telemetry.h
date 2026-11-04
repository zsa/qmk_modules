// Copyright 2026 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once
/*
What each cell is actually doing, per half.

===========================================================================
WHY THIS EXISTS
===========================================================================

ORYX_CMD_GET_TELEMETRY answers with one percentage per half and what the
charging policy commanded. That is not enough to tell a slow charge from a
stalled one: a board sitting at 40% for an hour looks the same whether it is
taking 500 mA, taking nothing because the charger latched a fault, or taking
nothing because the cell is cold. This is the answer to
ORYX_CMD_GET_CHARGE_TELEMETRY -- cell voltage, current, NTC temperature, the
fuel gauge's time estimates and the charger's own status and fault registers.

The block is a copy of struct zsa_battery_detail from the wireless module's
zsa_uart_proto.h, byte for byte. It is re-declared here rather than included
because the oryx module does not depend on the wireless module: a board with
no nRF still answers this command, with everything unknown. wireless.c
static-asserts the two are the same size before it memcpy()s between them.

===========================================================================
ADDING A FIELD
===========================================================================

Append it where reserved[] starts, shrink reserved[] by the same size, and
raise ORYX_CTLM_LEN. Leave ORYX_CTLM_VERSION alone: `len` is what a host
tests, so an old host reading a new firmware stops at the fields it knows, and
a new host reading an old firmware sees a short `len` and calls the fields it
did not get unknown instead of reading whatever happened to be there. Raise
the version only when a field that has shipped changes meaning -- the one
thing `len` cannot express. Never renumber an offset and never reuse one.

Then mirror the field in zapp at zapp-hid-proto/src/lib.rs, which decodes by
the same rule.

===========================================================================
UNKNOWN IS NOT ZERO
===========================================================================

Every field carries a value for "we could not find out". 0 mA is a real
reading on a terminated charger, 0.00 degC is a real temperature and 0 minutes
to full is a real estimate, so none of them can double as unknown. The one
field whose unknown is 0 is the voltage: a cell at 0 mV is not a cell.
*/

#include <stdint.h>
#include "oryx_telemetry.h" // RAW_EPSIZE, ORYX_TLM_SRC_*

/* Bumped only when a shipped field changes meaning. See ADDING A FIELD. */
#define ORYX_CTLM_VERSION 1

/* Defined bytes after the `len` byte itself. Grows as fields are appended. */
#define ORYX_CTLM_LEN 28

/* Per-field unknowns. Same values as ZSA_BAT_DETAIL_*_UNKNOWN in
 * zsa_uart_proto.h, which is where the blocks come from. */
#define ORYX_CTLM_MV_UNKNOWN  0        /* voltage_mv */
#define ORYX_CTLM_I16_UNKNOWN (-32768) /* current_ma, temp_cdeg */
#define ORYX_CTLM_MIN_UNKNOWN 0xFFFF   /* tte_min, ttf_min */
#define ORYX_CTLM_REG_UNKNOWN 0xFF     /* chg_status, chg_error, chg_sensor */

/* One half's cell and charger. Layout-identical to struct zsa_battery_detail. */
typedef struct __attribute__((packed)) {
    uint16_t voltage_mv; /* cell voltage, or ORYX_CTLM_MV_UNKNOWN */
    int16_t  current_ma; /* positive = into the cell (charging), negative = discharge */
    int16_t  temp_cdeg;  /* NTC battery temperature, 0.01 degC */
    uint16_t tte_min;    /* fuel-gauge time to empty, minutes */
    uint16_t ttf_min;    /* fuel-gauge time to full, minutes */
    uint8_t  chg_status; /* nPM1300 CHARGER.BCHGCHARGESTATUS, raw */
    uint8_t  chg_error;  /* nPM1300 CHARGER.BCHGERRREASON, raw (latched fault) */
    uint8_t  chg_sensor; /* nPM1300 CHARGER.BCHGERRSENSOR, raw */
} oryx_battery_detail_t;

_Static_assert(sizeof(oryx_battery_detail_t) == 13, "battery detail is a wire layout");

/* Fills bytes 1..31 of the 32-byte response report, byte 0 being
 * ORYX_EVT_CHARGE_TELEMETRY. */
typedef struct __attribute__((packed)) {
    uint8_t               version;      /* ORYX_CTLM_VERSION */
    uint8_t               len;          /* defined bytes after this one; ORYX_CTLM_LEN */
    uint8_t               left_source;  /* ORYX_TLM_SRC_*, where the left block came from */
    oryx_battery_detail_t left;
    uint8_t               right_source; /* ORYX_TLM_SRC_*, where the right block came from */
    oryx_battery_detail_t right;
    uint8_t               reserved[1];  /* zero; room for one more one-byte field */
} oryx_charge_telemetry_t;

_Static_assert(sizeof(oryx_charge_telemetry_t) == RAW_EPSIZE - 1, "charge telemetry payload must fill the report");

/* Everything unknown, ready for whoever can actually answer. Not memset(0):
 * zero is a real answer for most of these fields. */
static inline void oryx_battery_detail_reset(oryx_battery_detail_t *d) {
    d->voltage_mv = ORYX_CTLM_MV_UNKNOWN;
    d->current_ma = ORYX_CTLM_I16_UNKNOWN;
    d->temp_cdeg  = ORYX_CTLM_I16_UNKNOWN;
    d->tte_min    = ORYX_CTLM_MIN_UNKNOWN;
    d->ttf_min    = ORYX_CTLM_MIN_UNKNOWN;
    d->chg_status = ORYX_CTLM_REG_UNKNOWN;
    d->chg_error  = ORYX_CTLM_REG_UNKNOWN;
    d->chg_sensor = ORYX_CTLM_REG_UNKNOWN;
}

static inline void oryx_charge_telemetry_reset(oryx_charge_telemetry_t *ctlm) {
    for (uint8_t i = 0; i < sizeof(*ctlm); i++) {
        ((uint8_t *)ctlm)[i] = 0;
    }
    ctlm->version      = ORYX_CTLM_VERSION;
    ctlm->len          = ORYX_CTLM_LEN;
    ctlm->left_source  = ORYX_TLM_SRC_UNKNOWN;
    ctlm->right_source = ORYX_TLM_SRC_UNKNOWN;
    oryx_battery_detail_reset(&ctlm->left);
    oryx_battery_detail_reset(&ctlm->right);
}

/* Weak hook. The oryx module owns the wire format but knows none of the
 * answers; a board that has them (Wireless Voyager: the wireless module) overrides
 * this. The default leaves everything unknown, which is the truth for a board
 * with nothing to report. */
void oryx_charge_telemetry_fill(oryx_charge_telemetry_t *ctlm);
