// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

// QMK pointing-device driver for Voyager 2 (and future ZSA wireless keyboards).
// The trackball / touchpad is read by the nRF (locally on left-mounted, or
// remote-via-ESB on right-mounted) and motion is delivered over UART as
// generic ZSA_MSG_EVT_NAV_MOTION events. This driver consumes those events
// from the wireless module's accumulator — no i2c master/slave on the QMK
// side at all.
//
// For old wired hardware (Moonlander, OG Voyager) keep using the
// navigator_trackball module instead — it talks to the SCI18IS606 chip
// directly over i2c.

#include "navigator_wireless.h"
#include "wireless.h"
#include "zsa_uart_proto.h"
#include "quantum.h"

uint8_t current_cpi = NAVIGATOR_TRACKBALL_CPI;

/* QMK calls pointing_device_init BEFORE bluetooth_init (see keyboard.c init
 * order). We can't send UART frames during pointing_device_driver_init because
 * the wireless module hasn't called uart_init() yet — uart_transmit on an
 * uninitialized peripheral hangs the whole boot. So defer the enable+CPI send
 * to the first get_report call.
 *
 * We also retry the enable+CPI on every call until the nRF acks via
 * EVT_NAV_PRESENT (visible as a non-zero device type from
 * zsa_wireless_nav_device_type()). The first send can be lost if the UART rx
 * queue on the nRF isn't fully drained, or if our log line is among the early
 * messages dropped by the BLE log ring buffer. Retrying is cheap (one frame
 * every ~1 s), idempotent on the nRF side, and gives users a recovery path
 * after a transient link drop without rebooting. */
#ifndef NAVIGATOR_WIRELESS_RETRY_INTERVAL_MS
#    define NAVIGATOR_WIRELESS_RETRY_INTERVAL_MS 1000
#endif
static uint32_t last_enable_attempt_ms;

static void send_enable(uint8_t enabled) {
    struct zsa_nav_enable msg = { .enabled = enabled };
    zsa_wireless_send_frame(ZSA_MSG_CMD_NAV_ENABLE, (const uint8_t *)&msg, sizeof(msg));
}

static void send_cpi(uint8_t cpi) {
    struct zsa_nav_cpi msg = { .cpi = cpi };
    zsa_wireless_send_frame(ZSA_MSG_CMD_NAV_CPI, (const uint8_t *)&msg, sizeof(msg));
}

void pointing_device_driver_init(void) {
    /* No UART work here — see comment near last_enable_attempt_ms above. */
}

report_mouse_t pointing_device_driver_get_report(report_mouse_t mouse_report) {
    /* Don't send anything until the LINK_UP handshake has completed at least
     * once — same gate send_full_state uses. Sending earlier means the bytes
     * hit a not-yet-listening UART rx on the nRF and get dropped silently. */
    if (zsa_wireless_link_ready() && zsa_wireless_nav_device_type() == 0) {
        uint32_t now = timer_read32();
        if (last_enable_attempt_ms == 0 ||
            TIMER_DIFF_32(now, last_enable_attempt_ms) >= NAVIGATOR_WIRELESS_RETRY_INTERVAL_MS) {
            send_enable(1);
            send_cpi(current_cpi);
            last_enable_attempt_ms = now;
        }
    }

    int16_t dx = 0, dy = 0;
    zsa_wireless_nav_consume_motion(&dx, &dy);
    if (dx == 0 && dy == 0) {
        return mouse_report;
    }
    mouse_report.x = dx;
    mouse_report.y = dy;
    return mouse_report;
}

uint16_t pointing_device_driver_get_cpi(void) {
    return current_cpi;
}

/* CPI keycodes route through pointing_device_set_cpi → this function. The
 * argument is a magic 0/1 sentinel from the navigator.c keycode handlers
 * meaning "decrement by one tick" / "increment by one tick", matching the
 * existing navigator_trackball semantics. Keep behaviour identical so the
 * keymap-side keycodes work the same. */
void pointing_device_driver_set_cpi(uint16_t cpi) {
    if (cpi == 0) {
        if (current_cpi > NAVIGATOR_TRACKBALL_CPI_TICK) {
            current_cpi -= NAVIGATOR_TRACKBALL_CPI_TICK;
            send_cpi(current_cpi);
        }
    } else {
        if (current_cpi <= NAVIGATOR_TRACKBALL_CPI_MAX - NAVIGATOR_TRACKBALL_CPI_TICK) {
            current_cpi += NAVIGATOR_TRACKBALL_CPI_TICK;
            send_cpi(current_cpi);
        }
    }
}

/* Mirror the trackball driver's restore_cpi for keymap-level callers that
 * want to push a specific CPI value (e.g. eeprom restore). Only emits once
 * the link is up (device_type acked); before that we just cache the value,
 * the retry loop in get_report will pick it up. */
void restore_cpi(uint8_t cpi) {
    current_cpi = cpi;
    if (zsa_wireless_nav_device_type() != 0) {
        send_cpi(current_cpi);
    }
}
