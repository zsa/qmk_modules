// Copyright 2025 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

// PTP (Precision Touchpad) mode implementation for Navigator trackpad
// Converts Cirque Gen 6 sensor data to Windows Precision Touchpad HID reports
// Also provides a fallback mouse collection for systems that don't support PTP

#include <math.h>
#include "navigator_trackpad_ptp.h"
#include "navigator_trackpad_common.h"
#include "navigator_trackpad_contacts.h"
#include "navigator_trackpad_rotation.h"
#include "quantum.h"
#include "report.h"
#include "timer.h"

#if COMMUNITY_MODULE_AUTOMOUSE_ENABLE == TRUE
#    include <automouse.h>
#endif

// Normalize the configured angle to [0, 360) so negative or >360 values still
// hit the integer right-angle fast-paths.
#define _NAVIGATOR_TRACKPAD_ROT (((NAVIGATOR_TRACKPAD_ROTATION % 360) + 360) % 360)

#if _NAVIGATOR_TRACKPAD_ROT != 0 && _NAVIGATOR_TRACKPAD_ROT != 90 && \
    _NAVIGATOR_TRACKPAD_ROT != 180 && _NAVIGATOR_TRACKPAD_ROT != 270
#    define _NT_PAD_ROT_RAD (_NAVIGATOR_TRACKPAD_ROT * 3.14159265358979f / 180.0f)
// Evaluate cos/sin at compile time to avoid runtime trig / math-library calls.
static const float pad_rotation_cos = __builtin_cosf(_NT_PAD_ROT_RAD);
static const float pad_rotation_sin = __builtin_sinf(_NT_PAD_ROT_RAD);
#endif

// Apply the configured rotation to an absolute point (lvalues px, py) and to a
// relative delta (lvalues dx, dy). Selected at compile time: 0 -> no-op,
// right angles -> integer fast-path, else -> float path. Passing the same
// lvalue as input and output is safe (the helpers read by value first).
#if _NAVIGATOR_TRACKPAD_ROT == 90
#    define NT_ROTATE_POINT(px, py) nt_rotate_point_ortho((px), (py), NAVIGATOR_TRACKPAD_CENTER_X, NAVIGATOR_TRACKPAD_CENTER_Y, 1, TRACKPAD_LOGICAL_MAX, &(px), &(py))
#    define NT_ROTATE_DELTA(dx, dy) nt_rotate_delta_ortho((dx), (dy), 1, &(dx), &(dy))
#elif _NAVIGATOR_TRACKPAD_ROT == 180
#    define NT_ROTATE_POINT(px, py) nt_rotate_point_ortho((px), (py), NAVIGATOR_TRACKPAD_CENTER_X, NAVIGATOR_TRACKPAD_CENTER_Y, 2, TRACKPAD_LOGICAL_MAX, &(px), &(py))
#    define NT_ROTATE_DELTA(dx, dy) nt_rotate_delta_ortho((dx), (dy), 2, &(dx), &(dy))
#elif _NAVIGATOR_TRACKPAD_ROT == 270
#    define NT_ROTATE_POINT(px, py) nt_rotate_point_ortho((px), (py), NAVIGATOR_TRACKPAD_CENTER_X, NAVIGATOR_TRACKPAD_CENTER_Y, 3, TRACKPAD_LOGICAL_MAX, &(px), &(py))
#    define NT_ROTATE_DELTA(dx, dy) nt_rotate_delta_ortho((dx), (dy), 3, &(dx), &(dy))
#elif _NAVIGATOR_TRACKPAD_ROT != 0
#    define NT_ROTATE_POINT(px, py) nt_rotate_point((px), (py), NAVIGATOR_TRACKPAD_CENTER_X, NAVIGATOR_TRACKPAD_CENTER_Y, pad_rotation_cos, pad_rotation_sin, TRACKPAD_LOGICAL_MAX, &(px), &(py))
#    define NT_ROTATE_DELTA(dx, dy) nt_rotate_delta((dx), (dy), pad_rotation_cos, pad_rotation_sin, &(dx), &(dy))
#else
#    define NT_ROTATE_POINT(px, py) ((void)0)
#    define NT_ROTATE_DELTA(dx, dy) ((void)0)
#endif

// External declarations for report sending (defined in usb_main.c)
extern void send_digitizer_touchpad(report_digitizer_touchpad_t *report);
extern void send_digitizer_touchpad_mouse(report_digitizer_touchpad_mouse_t *report);

// Input mode: 0 = Mouse, 3 = PTP
// Defined in usb_main.c
extern uint8_t digitizer_touchpad_get_input_mode(void);

// Input mode values (set by host via HID feature report)
#define TRACKPAD_INPUT_MODE_MOUSE 0
#define TRACKPAD_INPUT_MODE_PTP   3

// PTP report structure constants
#define PTP_REPORT_ID           0x01
#define PTP_REPORT_SIZE         16
#define PTP_FINGER0_OFFSET      1
#define PTP_FINGER1_OFFSET      7
#define PTP_SCAN_TIME_OFFSET    13
#define PTP_COUNT_BUTTONS_OFFSET 15

// Button masks
#define BUTTON_PRIMARY          0x01

// Tap-to-click configuration
#ifndef TRACKPAD_TAP_TERM_MS
#    define TRACKPAD_TAP_TERM_MS 200  // Maximum duration for a tap (ms)
#endif

#ifndef TRACKPAD_TAP_MOVE_THRESHOLD_SQ
#    define TRACKPAD_TAP_MOVE_THRESHOLD_SQ 100  // Max movement squared before tap becomes drag
#endif

#ifndef TRACKPAD_TAP_SETTLE_TIME_MS
#    define TRACKPAD_TAP_SETTLE_TIME_MS 30  // Ignore movement during initial contact (ms)
#endif

#ifndef TRACKPAD_MAX_DELTA
#    define TRACKPAD_MAX_DELTA 250  // Max allowed delta per frame to prevent jumps
#endif

// Build a finger's 6 bytes into the report buffer
// Format: [conf:1 + tip:1 + pad:6] [contact_id:3 + pad:5] [X_lo] [X_hi] [Y_lo] [Y_hi]
static void build_finger_bytes(uint8_t *buf, uint8_t contact_id, uint16_t x, uint16_t y, bool tip, bool confidence) {
    buf[0] = (confidence ? 0x01 : 0x00) | (tip ? 0x02 : 0x00);
    buf[1] = contact_id & 0x07;  // contact_id in bits 0-2
    buf[2] = x & 0xFF;           // X low byte
    buf[3] = (x >> 8) & 0xFF;    // X high byte
    buf[4] = y & 0xFF;           // Y low byte
    buf[5] = (y >> 8) & 0xFF;    // Y high byte
}

// Fallback mouse state
static struct {
    // Position tracking for relative movement
    bool     tracking;
    uint16_t last_x;
    uint16_t last_y;
    // Subpixel accumulation for smooth low-sensitivity movement
    float    dx_accum;
    float    dy_accum;
    // Tap detection - uses settled position like mouse mode
    uint32_t touch_start_time;
    uint16_t settled_x;
    uint16_t settled_y;
    bool     settled;
    bool     is_drag;  // Set when movement exceeds tap threshold - enables cursor movement
    // Click state - pending_release triggers button release on next cycle
    bool     pending_release;
    // Previous state for change detection
    uint8_t  prev_buttons;
} mouse_state = {0};

// Track input mode to detect changes
static uint8_t prev_input_mode = TRACKPAD_INPUT_MODE_PTP;

// Send fallback mouse report
static void send_mouse_report(int8_t dx, int8_t dy, uint8_t buttons) {
    report_digitizer_touchpad_mouse_t report = {
        .report_id = DIGITIZER_TOUCHPAD_MOUSE_REPORT_ID,
        .buttons   = buttons,
        .x         = dx,
        .y         = dy
    };
    send_digitizer_touchpad_mouse(&report);
}

// Reset mouse state when mode changes to avoid stale timers/state
static void reset_mouse_state(void) {
    // Send release if button was pressed
    if (mouse_state.prev_buttons != 0 || mouse_state.pending_release) {
        send_mouse_report(0, 0, 0);
    }
    mouse_state.tracking = false;
    mouse_state.dx_accum = 0.0f;
    mouse_state.dy_accum = 0.0f;
    mouse_state.touch_start_time = 0;
    mouse_state.settled = false;
    mouse_state.is_drag = false;
    mouse_state.pending_release = false;
    mouse_state.prev_buttons = 0;
}

// Clamp value to int8_t range
static inline int8_t clamp_to_int8(int32_t value) {
    if (value > 127) return 127;
    if (value < -127) return -127;
    return (int8_t)value;
}

// Process fallback mouse movement and tap-to-click
// Uses settle time and movement suppression like navigator_trackpad_mouse for smooth operation
static void process_fallback_mouse(cgen6_report_t *sensor_report, bool finger_down, bool prev_finger_down) {
    int8_t dx = 0;
    int8_t dy = 0;
    uint8_t buttons = 0;

    // Handle pending click release from previous cycle (like mouse mode)
    if (mouse_state.pending_release) {
        mouse_state.pending_release = false;
        send_mouse_report(0, 0, 0);
        mouse_state.prev_buttons = 0;
        // Continue processing - don't return early so we handle any new movement
    }

    // Handle physical button (from sensor)
    if (sensor_report->buttons & BUTTON_PRIMARY) {
        buttons |= BUTTON_PRIMARY;
    }

    // Handle finger down transition (start tracking)
    if (finger_down && !prev_finger_down) {
        mouse_state.tracking = true;
        mouse_state.last_x = sensor_report->fingers[0].x;
        mouse_state.last_y = sensor_report->fingers[0].y;
        // Reset subpixel accumulators for new touch
        mouse_state.dx_accum = 0.0f;
        mouse_state.dy_accum = 0.0f;
        // Start tap detection with settle time approach
        mouse_state.touch_start_time = timer_read32();
        mouse_state.settled = false;
        mouse_state.settled_x = 0;
        mouse_state.settled_y = 0;
        mouse_state.is_drag = false;
    }

    // Handle finger movement
    if (finger_down && mouse_state.tracking) {
        uint32_t duration = timer_elapsed32(mouse_state.touch_start_time);

        // Record settled position once settle time elapses (for tap detection)
        if (!mouse_state.settled && duration >= TRACKPAD_TAP_SETTLE_TIME_MS) {
            mouse_state.settled = true;
            mouse_state.settled_x = sensor_report->fingers[0].x;
            mouse_state.settled_y = sensor_report->fingers[0].y;
        }

        // Check if movement from settled position exceeds tap threshold → becomes a drag
        if (mouse_state.settled && !mouse_state.is_drag) {
            int16_t move_x = (int16_t)sensor_report->fingers[0].x - (int16_t)mouse_state.settled_x;
            int16_t move_y = (int16_t)sensor_report->fingers[0].y - (int16_t)mouse_state.settled_y;
            int32_t dist_sq = (int32_t)move_x * move_x + (int32_t)move_y * move_y;
            if (dist_sq > TRACKPAD_TAP_MOVE_THRESHOLD_SQ) {
                mouse_state.is_drag = true;
            }
        }

        // Only report movement once we've determined this is a drag (not a tap)
        if (mouse_state.is_drag) {
            int16_t raw_dx = (int16_t)sensor_report->fingers[0].x - (int16_t)mouse_state.last_x;
            int16_t raw_dy = (int16_t)sensor_report->fingers[0].y - (int16_t)mouse_state.last_y;

            // Rotate the relative motion vector (same convention as the trackball).
            // Tap/drag detection uses squared distance, which rotation leaves
            // invariant, so only the reported deltas need rotating.
            NT_ROTATE_DELTA(raw_dx, raw_dy);

            // Clamp deltas to prevent jumps from bad sensor data
            if (raw_dx > TRACKPAD_MAX_DELTA) raw_dx = TRACKPAD_MAX_DELTA;
            if (raw_dx < -TRACKPAD_MAX_DELTA) raw_dx = -TRACKPAD_MAX_DELTA;
            if (raw_dy > TRACKPAD_MAX_DELTA) raw_dy = TRACKPAD_MAX_DELTA;
            if (raw_dy < -TRACKPAD_MAX_DELTA) raw_dy = -TRACKPAD_MAX_DELTA;

            if (raw_dx != 0 || raw_dy != 0) {
                // Apply configurable acceleration for cursor feel
                float acc_dx = (raw_dx < 0) ? -powf(-raw_dx, TRACKPAD_MOUSE_ACCELERATION) : powf(raw_dx, TRACKPAD_MOUSE_ACCELERATION);
                float acc_dy = (raw_dy < 0) ? -powf(-raw_dy, TRACKPAD_MOUSE_ACCELERATION) : powf(raw_dy, TRACKPAD_MOUSE_ACCELERATION);

                // Apply sensitivity scaling and accumulate for subpixel precision
                mouse_state.dx_accum += acc_dx * TRACKPAD_MOUSE_SENSITIVITY;
                mouse_state.dy_accum += acc_dy * TRACKPAD_MOUSE_SENSITIVITY;

                // Extract integer portion for reporting, keep fractional for next frame
                dx = clamp_to_int8((int32_t)mouse_state.dx_accum);
                dy = clamp_to_int8((int32_t)mouse_state.dy_accum);
                mouse_state.dx_accum -= dx;
                mouse_state.dy_accum -= dy;
            }
        }

        // Always update last position for delta calculation
        mouse_state.last_x = sensor_report->fingers[0].x;
        mouse_state.last_y = sensor_report->fingers[0].y;
    }

    // Handle finger up transition (end tracking, check for tap)
    if (!finger_down && prev_finger_down) {
        mouse_state.tracking = false;

        uint32_t touch_duration = timer_elapsed32(mouse_state.touch_start_time);

        // Tap conditions: not a drag AND short duration
        bool is_tap = !mouse_state.is_drag && (touch_duration <= TRACKPAD_TAP_TERM_MS);

        if (is_tap) {
            // Valid tap - send click press, release will happen next cycle
            send_mouse_report(0, 0, BUTTON_PRIMARY);
            mouse_state.prev_buttons = BUTTON_PRIMARY;
            mouse_state.pending_release = true;
        }

        mouse_state.settled = false;
    }

    // Only send report if there's actual movement or button state changed
    bool buttons_changed = (buttons != mouse_state.prev_buttons);
    bool has_movement = (dx != 0 || dy != 0);

    if (has_movement || buttons_changed) {
        send_mouse_report(dx, dy, buttons);
        mouse_state.prev_buttons = buttons;
    }
}

// Scale sensor X coordinate to logical range using fixed-point multiplication
static inline uint16_t scale_x(uint16_t raw) {
    if (raw < SENSOR_X_MIN) raw = SENSOR_X_MIN;
    if (raw > SENSOR_X_MAX) raw = SENSOR_X_MAX;
    return ((uint32_t)(raw - SENSOR_X_MIN) * SENSOR_SCALE_X_MULT) >> 16;
}

// Scale sensor Y coordinate to logical range using fixed-point multiplication
static inline uint16_t scale_y(uint16_t raw) {
    if (raw < SENSOR_Y_MIN) raw = SENSOR_Y_MIN;
    if (raw > SENSOR_Y_MAX) raw = SENSOR_Y_MAX;
    return ((uint32_t)(raw - SENSOR_Y_MIN) * SENSOR_SCALE_Y_MULT) >> 16;
}

// PTP task function - synchronous polling with timer-based throttling
bool navigator_trackpad_ptp_task(void) {
    static uint32_t last_poll_time  = 0;
    static uint32_t last_probe_time = 0;
    static uint8_t  prev_buttons = 0;
    // Slot-0 presence from last frame, for the mouse-mode fallback tap detector.
    static bool     prev_finger0_tip = false;
    // Contacts the host currently believes are down, keyed to the sensor's
    // stable per-finger id. Reconciled against each frame so every lifted
    // contact gets a clean tip=0 and none is ever stranded (see
    // navigator_trackpad_contacts.h).
    static nt_contact_state_t host_contacts = {0};

    uint32_t now = timer_read32();

    // Throttle polling to NAVIGATOR_TRACKPAD_POLL_INTERVAL_MS
    if (timer_elapsed32(last_poll_time) < NAVIGATOR_TRACKPAD_POLL_INTERVAL_MS) {
        return false;
    }
    last_poll_time = now;

    // Handle disconnected/uninitialized state with slower probe interval
    if (!trackpad_init) {
        if (timer_elapsed32(last_probe_time) < NAVIGATOR_TRACKPAD_PROBE_INTERVAL_MS) {
            return false;
        }
        last_probe_time = now;
        navigator_trackpad_device_init();
        return false;
    }

    // Read the report data into local struct
    cgen6_report_t sensor_report = {0};
    if (!cirque_gen_6_read_report(&sensor_report)) {
        return false;
    }

    // Slot-0 presence (raw tip) drives the mouse-mode fallback tap detector below.
    bool finger0_present = sensor_report.fingers[0].tip;

    // --- Contact assembly ---
    // Gather currently-down contacts with the sensor's stable per-finger id. The
    // Cirque keeps a finger's id constant even when it moves the contact to a
    // different packet slot, so we key on id (not slot) to avoid teleporting/
    // dropped contacts. Confidence is reported as-is (stays 1 for a real contact).
    nt_sensor_contact_t cur[NT_MAX_CONTACTS];
    uint8_t             cur_n = 0;
    for (uint8_t ss = 0; ss < 2 && cur_n < NT_MAX_CONTACTS; ss++) {
        if (sensor_report.fingers[ss].tip) {
            uint16_t px = scale_x(sensor_report.fingers[ss].x);
            uint16_t py = scale_y(sensor_report.fingers[ss].y);
            // Rotate the absolute contact about the configured center. Both
            // contacts share the center, so the transform is rigid: their
            // separation (used for two-finger gestures) is preserved.
            NT_ROTATE_POINT(px, py);
            cur[cur_n].id   = sensor_report.fingers[ss].id;
            cur[cur_n].x    = px;
            cur[cur_n].y    = py;
            cur[cur_n].conf = sensor_report.fingers[ss].confidence;
            cur_n++;
        }
    }

    uint8_t buttons = sensor_report.buttons & BUTTON_PRIMARY;
    bool button_changed = (buttons != prev_buttons);

#if COMMUNITY_MODULE_AUTOMOUSE_ENABLE == TRUE
    // Feed primary-contact motion to automouse so a finger moving on the pad can
    // activate the mouse layer. This runs regardless of input mode: in PTP mode
    // the host derives cursor motion from the absolute contacts we emit, so there
    // is no relative-delta report (send_mouse_report) for automouse to observe.
    // Deltas are in logical units (0..TRACKPAD_LOGICAL_MAX); keyed on the sensor's
    // stable contact id so a fresh touch doesn't produce a jump from a stale slot.
    {
        static int16_t  am_prev_id = -1;
        static uint16_t am_prev_x  = 0;
        static uint16_t am_prev_y  = 0;
        if (cur_n > 0) {
            if (cur_id[0] == am_prev_id) {
                automouse_report_motion((int16_t)cur_x[0] - (int16_t)am_prev_x,
                                        (int16_t)cur_y[0] - (int16_t)am_prev_y,
                                        buttons);
            }
            am_prev_id = cur_id[0];
            am_prev_x  = cur_x[0];
            am_prev_y  = cur_y[0];
        } else {
            am_prev_id = -1;  // all fingers lifted; next touch starts fresh
        }
    }
#endif

    // Assemble the emitted contacts, packed into the first report slots so
    // contact_count always matches the slots the host should read:
    //   1. currently-down contacts (tip=1), then
    //   2. a single clean tip=0 lift for any id reported last frame but gone now,
    //      as space allows (2 slots). If both slots are filled by current
    //      contacts, a simultaneously-lifted id simply stops being reported and
    //      the host infers the lift from its absence + the lower contact_count.
    int16_t  em_id[2];
    uint16_t em_x[2], em_y[2];
    bool     em_conf[2], em_tip[2];
    uint8_t  em_n = 0;
    for (uint8_t i = 0; i < cur_n && em_n < 2; i++) {
        em_id[em_n]   = cur_id[i];
        em_x[em_n]    = cur_x[i];
        em_y[em_n]    = cur_y[i];
        em_conf[em_n] = cur_conf[i];
        em_tip[em_n]  = true;
        em_n++;
    }
    for (uint8_t p = 0; p < prev_cn && em_n < 2; p++) {
        bool still_down = false;
        for (uint8_t i = 0; i < cur_n; i++) {
            if (cur_id[i] == prev_cid[p]) { still_down = true; break; }
        }
        if (!still_down) {
            em_id[em_n]   = prev_cid[p];
            em_x[em_n]    = prev_cx[p];
            em_y[em_n]    = prev_cy[p];
            em_conf[em_n] = prev_cconf[p];
            em_tip[em_n]  = false;  // clean lift
            em_n++;
        }
    }
    // Reconcile against what the host believes is down: continue still-present
    // contacts, release (tip=0) any that vanished, and pick up new contacts as
    // slots allow. Guarantees no contact is ever stranded on the host.
    nt_emit_list_t emit;
    nt_reconcile_contacts(&host_contacts, cur, cur_n, &emit);

    uint8_t contact_count = emit.count;

    // Build report from the emit list (one finger per HID slot).
    static const uint8_t finger_offset[NT_MAX_CONTACTS] = {PTP_FINGER0_OFFSET, PTP_FINGER1_OFFSET};
    uint8_t report[PTP_REPORT_SIZE] = {0};
    report[0] = PTP_REPORT_ID;
    for (uint8_t i = 0; i < emit.count; i++) {
        build_finger_bytes(&report[finger_offset[i]], emit.items[i].host_id,
                           emit.items[i].x, emit.items[i].y, emit.items[i].tip, emit.items[i].conf);
    }

    // Scan time (2 bytes, little-endian)
    report[PTP_SCAN_TIME_OFFSET]     = sensor_report.scan_time & 0xFF;
    report[PTP_SCAN_TIME_OFFSET + 1] = (sensor_report.scan_time >> 8) & 0xFF;

    // Contact count (bits 0-3) + buttons (bits 4-6)
    report[PTP_COUNT_BUTTONS_OFFSET] = (contact_count & 0x0F) | ((buttons & BUTTON_PRIMARY) << 4);

    // Get current input mode and handle mode changes
    uint8_t input_mode = digitizer_touchpad_get_input_mode();
    if (input_mode != prev_input_mode) {
        // Mode changed - reset mouse state to avoid stale timers/state
        reset_mouse_state();
        prev_input_mode = input_mode;
    }

    // Send PTP report only in PTP mode (mode 3)
    if (input_mode == TRACKPAD_INPUT_MODE_PTP) {
        if (contact_count > 0 || button_changed) {
            send_digitizer_touchpad((report_digitizer_touchpad_t *)report);
        }
    }

    // Process fallback mouse only in mouse mode (mode 0)
    if (input_mode == TRACKPAD_INPUT_MODE_MOUSE) {
        process_fallback_mouse(&sensor_report, finger0_present, prev_finger0_tip);
    }

    // Update previous state
    prev_finger0_tip = finger0_present;
    prev_buttons     = buttons;

    return contact_count > 0 || button_changed;
}
