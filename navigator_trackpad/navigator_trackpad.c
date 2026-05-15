// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#include "quantum.h"
#include "navigator_trackpad.h"
#include "navigator_trackpad_i2c.h"
#include "navigator_trackpad_core.h"
#include "digitizer.h"
#include "timer.h"

#ifndef NAVIGATOR_TRACKPAD_POLL_INTERVAL_MS
#define NAVIGATOR_TRACKPAD_POLL_INTERVAL_MS 5
#endif
#ifndef NAVIGATOR_TRACKPAD_PROBE_INTERVAL_MS
#define NAVIGATOR_TRACKPAD_PROBE_INTERVAL_MS 1000
#endif

// Strong override: called once during keyboard_post_init_quantum().
void digitizer_touchpad_init(void) {
    navigator_trackpad_device_init();
}

// Strong override: called from keyboard_task() each iteration.
// Paces i2c poll/probe, reads + decodes a packet, then hands off to the
// core dispatch (PTP HID + mouse fallback).
bool digitizer_touchpad_task(void) {
    if (!trackpad_init) {
        static uint32_t last_probe;
        if (timer_elapsed32(last_probe) >= NAVIGATOR_TRACKPAD_PROBE_INTERVAL_MS) {
            last_probe = timer_read32();
            navigator_trackpad_device_init();
        }
        return false;
    }

    static uint32_t last_poll;
    if (timer_elapsed32(last_poll) < NAVIGATOR_TRACKPAD_POLL_INTERVAL_MS) return false;
    last_poll = timer_read32();

    uint8_t bytes[NAV_TRACKPAD_PACKET_LEN];
    if (!navigator_trackpad_i2c_read_packet(bytes)) return false;
    cgen6_report_t report;
    if (!navigator_trackpad_core_decode(bytes, &report)) return false;
    return navigator_trackpad_core_dispatch(&report);
}

// Keycode handler for module-declared keycodes (TRACKPAD_INC_CPI / _DEC_CPI).
// Declared by the navigator_trackpad_core module; delegated through here so
// QMK's keymap module hooks still resolve when only the wired module is named.
extern bool process_record_navigator_trackpad_core(uint16_t keycode, keyrecord_t *record);
bool process_record_navigator_trackpad(uint16_t keycode, keyrecord_t *record) {
    return process_record_navigator_trackpad_core(keycode, record);
}
