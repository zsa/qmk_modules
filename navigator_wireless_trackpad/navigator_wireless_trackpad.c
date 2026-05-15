// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Wireless QMK driver for the Cirque Gen6 trackpad. The nRF reads the chip and
// forwards raw 17-byte PTP packets over UART as ZSA_MSG_EVT_NAV_REPORT_RAW;
// this module decodes them via navigator_trackpad_core and dispatches PTP or
// mouse-fallback HID reports based on the host's input_mode.

#include "quantum.h"
#include "digitizer.h"
#include "navigator_trackpad_core.h"
#include "wireless.h"
#include "zsa_uart_proto.h"

void digitizer_touchpad_init(void) {
    /* nothing — chip lives on the nRF, our state is per-task in core */
}

bool digitizer_touchpad_task(void) {
    /* Send NAV_ENABLE once the link is up, like navigator_wireless does. */
    static uint32_t last_enable_attempt_ms;
    if (zsa_wireless_link_ready() && zsa_wireless_nav_device_type() == 0) {
        uint32_t now = timer_read32();
        if (last_enable_attempt_ms == 0 ||
            TIMER_DIFF_32(now, last_enable_attempt_ms) >= 1000) {
            struct zsa_nav_enable en = { .enabled = 1 };
            zsa_wireless_send_frame(ZSA_MSG_CMD_NAV_ENABLE, (const uint8_t *)&en, sizeof(en));
            last_enable_attempt_ms = now;
        }
    }

    uint8_t bytes[NAV_TRACKPAD_PACKET_LEN];
    if (!zsa_wireless_nav_consume_raw(bytes)) return false;
    cgen6_report_t report;
    if (!navigator_trackpad_core_decode(bytes, &report)) return false;
    return navigator_trackpad_core_dispatch(&report);
}

extern bool process_record_navigator_trackpad_core(uint16_t, keyrecord_t *);
bool process_record_navigator_wireless_trackpad(uint16_t keycode, keyrecord_t *record) {
    return process_record_navigator_trackpad_core(keycode, record);
}
