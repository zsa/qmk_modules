// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>

#define NAV_TRACKPAD_PACKET_LEN 17

// Mirrors the wired module's `cgen6_report_t` — same memory layout, same
// semantics. Defined here so both wired (i2c) and wireless (UART-fed) drivers
// can produce one.
typedef struct {
    struct {
        uint8_t  tip;
        uint8_t  confidence;
        uint8_t  id;
        uint16_t x;
        uint16_t y;
    } fingers[2];
    uint8_t  buttons;
    uint8_t  contact_count;
    uint16_t scan_time;
    int8_t   xDelta;
    int8_t   yDelta;
    int8_t   scrollDelta;
    int8_t   panDelta;
} cgen6_report_t;

// Decode a raw 17-byte packet (cirque PTP wire format) into the struct.
// Returns true on a recognized report_id (PTP or mouse), false otherwise.
bool navigator_trackpad_core_decode(const uint8_t bytes[NAV_TRACKPAD_PACKET_LEN],
                                    cgen6_report_t *out);

// Run the PTP HID assembly + mouse-fallback state machine against a decoded
// report. Returns true if the dispatch produced a HID event (used for QMK's
// task return value, which gates further mainloop work).
bool navigator_trackpad_core_dispatch(const cgen6_report_t *report);

// CPI step controls (delegated from wired/wireless keycode handlers).
void     navigator_trackpad_core_set_cpi(uint16_t increment_or_zero);
uint16_t navigator_trackpad_core_get_cpi(void);
