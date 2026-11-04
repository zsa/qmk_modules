// Copyright 2026 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once
/*
What each half's charger is being fed, per half.

===========================================================================
WHY THIS EXISTS
===========================================================================

ORYX_CMD_GET_CHARGE_TELEMETRY says what the cell is doing. It does not say
what the charger has to work with: a half taking 50 mA looks the same whether
its input is at 5 V and unrestricted or at 4.4 V with the current limit
active. This is the answer to ORYX_CMD_GET_PMIC_RAILS -- VBUS and VSYS in mV,
the nPM1300's input-status bits and whether battery charging is enabled.

The same block travels over Bluetooth in the detail characteristic, so the two
paths show the same numbers. Over the cable it is the only path: a USB host
attached turns the left half's Bluetooth stack off entirely.

The block is a copy of struct zsa_pmic_rails from the wireless module's
zsa_uart_proto.h, byte for byte. It is re-declared here rather than included
because the oryx module does not depend on the wireless module: a board with
no nRF still answers this command, with both blocks absent. wireless.c
static-asserts the two are the same size before it memcpy()s between them.

===========================================================================
ADDING A FIELD
===========================================================================

Append it where reserved[] starts, shrink reserved[] by the same size, and
raise ORYX_RAILS_LEN. Leave ORYX_RAILS_VERSION alone: `len` is what a host
tests, so an old host reading a new firmware stops at the fields it knows, and
a new host reading an old firmware sees a short `len` and calls the fields it
did not get unknown instead of reading whatever happened to be there. Raise
the version only when a field that has shipped changes meaning -- the one
thing `len` cannot express. Never renumber an offset and never reuse one.

Then mirror the field in zapp at zapp-hid-proto/src/lib.rs, which decodes by
the same rule.

===========================================================================
PRESENT IS THE VALIDITY MARKER
===========================================================================

A half with nothing to report sends a zeroed block, and 0 V is not a rail, so
`flags` decides per half whether the reading exists. The rest of the block
means nothing without ORYX_RAILS_PRESENT set -- including the two register
bytes, which carry their own unknown for the case where the block exists but
the register read failed.
*/

#include <stdint.h>
#include "oryx_telemetry.h" // RAW_EPSIZE

/* Bumped only when a shipped field changes meaning. See ADDING A FIELD. */
#define ORYX_RAILS_VERSION 1

/* Defined bytes after the `len` byte itself. Grows as fields are appended. */
#define ORYX_RAILS_LEN 14

/* Same values as ZSA_PMIC_RAILS_* in zsa_uart_proto.h, which is where the
 * blocks come from. */
#define ORYX_RAILS_PRESENT 0x01 /* flags bit 0: the block carries a reading */

/* `vbusin_status` bits, nPM1300 VBUSIN.VBUSINSTATUS raw. */
#define ORYX_RAILS_VBUS_PRESENT   0x01
#define ORYX_RAILS_VBUS_CUR_LIM   0x02
#define ORYX_RAILS_VBUS_OVP       0x04
#define ORYX_RAILS_VBUS_UNDERVOLT 0x08
#define ORYX_RAILS_VBUS_SUSPENDED 0x10
#define ORYX_RAILS_VBUS_OUT       0x20

#define ORYX_RAILS_MV_UNKNOWN  0    /* vbus_mv, vsys_mv */
#define ORYX_RAILS_REG_UNKNOWN 0xFF /* vbusin_status, chg_enable */

/* One half's PMIC input rails. Layout-identical to struct zsa_pmic_rails. */
typedef struct __attribute__((packed)) {
    uint8_t  flags;         /* ORYX_RAILS_PRESENT, else the rest means nothing */
    uint16_t vbus_mv;       /* nPM1300 ADC VBUS */
    uint16_t vsys_mv;       /* nPM1300 ADC VSYS */
    uint8_t  vbusin_status; /* nPM1300 VBUSIN.VBUSINSTATUS, raw */
    uint8_t  chg_enable;    /* nPM1300 CHARGER.BCHGENABLESET readback, raw */
} oryx_pmic_rails_block_t;

_Static_assert(sizeof(oryx_pmic_rails_block_t) == 7, "pmic rails block is a wire layout");

/* Fills bytes 1..31 of the 32-byte response report, byte 0 being
 * ORYX_EVT_PMIC_RAILS. */
typedef struct __attribute__((packed)) {
    uint8_t                 version; /* ORYX_RAILS_VERSION */
    uint8_t                 len;     /* defined bytes after this one; ORYX_RAILS_LEN */
    oryx_pmic_rails_block_t left;
    oryx_pmic_rails_block_t right;
    uint8_t                 reserved[15]; /* zero; room for more fields */
} oryx_pmic_rails_t;

_Static_assert(sizeof(oryx_pmic_rails_t) == RAW_EPSIZE - 1, "pmic rails payload must fill the report");

/* A block that claims nothing. Not memset(0) alone: the two register bytes
 * have an unknown of their own. */
static inline void oryx_pmic_rails_block_reset(oryx_pmic_rails_block_t *r) {
    r->flags         = 0;
    r->vbus_mv       = ORYX_RAILS_MV_UNKNOWN;
    r->vsys_mv       = ORYX_RAILS_MV_UNKNOWN;
    r->vbusin_status = ORYX_RAILS_REG_UNKNOWN;
    r->chg_enable    = ORYX_RAILS_REG_UNKNOWN;
}

static inline void oryx_pmic_rails_reset(oryx_pmic_rails_t *rails) {
    for (uint8_t i = 0; i < sizeof(*rails); i++) {
        ((uint8_t *)rails)[i] = 0;
    }
    rails->version = ORYX_RAILS_VERSION;
    rails->len     = ORYX_RAILS_LEN;
    oryx_pmic_rails_block_reset(&rails->left);
    oryx_pmic_rails_block_reset(&rails->right);
}

/* Weak hook. The oryx module owns the wire format but knows none of the
 * answers; a board that has them (Wireless Voyager: the wireless module) overrides
 * this. The default leaves both blocks absent, which is the truth for a board
 * with no PMIC to read. */
void oryx_pmic_rails_fill(oryx_pmic_rails_t *rails);
