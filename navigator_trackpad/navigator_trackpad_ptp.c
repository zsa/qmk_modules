// Copyright 2025 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

// PTP (Precision Touchpad) mode implementation for Navigator trackpad
// Converts Cirque Gen 6 sensor data to Windows Precision Touchpad HID reports
// Also provides a fallback mouse collection for systems that don't support PTP

#include <math.h>
#include "navigator_trackpad_ptp.h"
#include "navigator_trackpad_common.h"
#include "navigator_trackpad_contacts.h"
#include "navigator_trackpad_filter.h"
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

// --- PTP coordinate smoothing (One Euro filter) ---------------------------
// Velocity-adaptive low-pass on the absolute contacts we emit in PTP mode:
// smooths hard when the finger is nearly still (kills sensor jitter so straight
// lines and slow circles stop wobbling) and backs off to near-passthrough when
// moving fast (so quick strokes stay crisp instead of feeling laggy/floaty —
// the failure mode of a plain fixed IIR). Set NAVIGATOR_TRACKPAD_PTP_SMOOTHING
// to FALSE to A/B against raw passthrough.
//
// Tuning (coordinates are in logical units 0..TRACKPAD_LOGICAL_MAX, speed in
// units/second): MINCUTOFF sets how hard we smooth when still (lower =
// smoother, more low-speed lag); BETA sets how quickly smoothing relaxes as
// speed rises (higher = passthrough sooner = less lag while moving).
#ifndef NAVIGATOR_TRACKPAD_PTP_SMOOTHING
#    define NAVIGATOR_TRACKPAD_PTP_SMOOTHING TRUE
#endif
// Defaults tuned on the captured Linux/libinput jitter trace (~125 Hz, +/-8 unit
// noise floor) and refined on-pad toward less lag: ~74% jitter reduction at rest
// with ~0.2 frame (~0.75 mm) lag on a fast stroke. DCUTOFF is deliberately well
// above the classic 1 Hz so the speed estimate tracks within a frame — at 1 Hz a
// quick stroke lags ~3 frames and feels floaty, the exact failure of the earlier
// fixed IIR. To trade smoothness for even less lag, raise BETA (and DCUTOFF a
// little); e.g. 0.010 / 18 gives ~69% / ~0.5 mm.
#ifndef NAVIGATOR_TRACKPAD_SMOOTHING_MINCUTOFF
#    define NAVIGATOR_TRACKPAD_SMOOTHING_MINCUTOFF 1.5f
#endif
#ifndef NAVIGATOR_TRACKPAD_SMOOTHING_BETA
#    define NAVIGATOR_TRACKPAD_SMOOTHING_BETA 0.0070f
#endif
#ifndef NAVIGATOR_TRACKPAD_SMOOTHING_DCUTOFF
#    define NAVIGATOR_TRACKPAD_SMOOTHING_DCUTOFF 15.0f
#endif

// Consecutive empty sensor reads (at NAVIGATOR_TRACKPAD_POLL_INTERVAL_MS each)
// required before we declare lift-off and release a still-tracked contact. The
// Cirque streams reports continuously while a finger is on the pad and goes
// quiet on lift-off, but because we poll faster than the sensor samples, a lone
// empty read also happens *between* samples while a finger is down. Waiting for
// a short run avoids releasing a still-present contact, while still flushing a
// stranded one within a few ms if the single tip=0 lift packet is ever missed.
#ifndef TRACKPAD_LIFTOFF_CONFIRM_FRAMES
#    define TRACKPAD_LIFTOFF_CONFIRM_FRAMES 3
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
    // Consecutive empty reads while a contact is still tracked. Used to confirm
    // lift-off before flushing a stranded contact (see the read path below).
    static uint8_t  no_data_frames = 0;
#if NAVIGATOR_TRACKPAD_PTP_SMOOTHING == TRUE
    // Per-emitted-slot One Euro filter state, the slot's down-flag from last
    // frame (a rising edge means a fresh contact -> reset the filter), and the
    // last frame timestamp used to derive the filter's dt.
    static nt_euro_point_t contact_filter[NT_MAX_CONTACTS] = {0};
    static bool            prev_emit_down[NT_MAX_CONTACTS]  = {0};
    static uint32_t        last_filter_time                 = 0;
#endif
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

    // Read the report data into local struct.
    //
    // A failed read is one of two things: a genuine I2C/bus error (the read
    // helper clears trackpad_init and the probe path re-inits next cycle), or a
    // successful transaction that returned no touch packet. The Cirque streams
    // reports continuously while any contact is present and goes quiet on
    // lift-off, so a *run* of empty reads means every finger has left the pad.
    //
    // We must still reconcile in that case — with zero current contacts — so any
    // contact the host believes is down gets a clean tip=0. Otherwise the
    // contact is stranded, and the next touch (with a reused sensor id) is taken
    // as a continuation, teleporting the cursor by the lift-to-retouch vector
    // (the "jump back to where the last stroke started" bug). The zeroed
    // sensor_report below carries no fingers, so falling through drives cur_n==0
    // and nt_reconcile_contacts emits the release.
    cgen6_report_t sensor_report = {0};
    if (!cirque_gen_6_read_report(&sensor_report)) {
        if (!trackpad_init) {
            // Dead bus: let the probe/re-init path recover; don't synthesize
            // lift-offs off a failed transaction.
            no_data_frames = 0;
            return false;
        }
        if (host_contacts.count == 0) {
            // Pad already idle — nothing to release.
            no_data_frames = 0;
            return false;
        }
        // A contact is still tracked but this poll had no packet. Empty reads
        // also occur between samples while a finger is down (poll interval <
        // sensor sample period), so wait for a short run before declaring
        // lift-off. Until then, leave the host's contacts untouched (as before).
        if (++no_data_frames < TRACKPAD_LIFTOFF_CONFIRM_FRAMES) {
            return false;
        }
        // Confirmed lift-off: fall through with the zeroed report so the
        // reconciler releases the stranded contact(s) with tip=0.
        no_data_frames = 0;
    } else {
        no_data_frames = 0;
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
            if (cur[0].id == am_prev_id) {
                automouse_report_motion((int16_t)cur[0].x - (int16_t)am_prev_x,
                                        (int16_t)cur[0].y - (int16_t)am_prev_y,
                                        buttons);
            }
            am_prev_id = cur[0].id;
            am_prev_x  = cur[0].x;
            am_prev_y  = cur[0].y;
        } else {
            am_prev_id = -1;  // all fingers lifted; next touch starts fresh
        }
    }
#endif

    // Reconcile against what the host believes is down: continue still-present
    // contacts, release (tip=0) any that vanished, and pick up new contacts as
    // slots allow. Guarantees no contact is ever stranded on the host.
    nt_emit_list_t emit;
    nt_reconcile_contacts(&host_contacts, cur, cur_n, &emit);

    uint8_t contact_count = emit.count;

#if NAVIGATOR_TRACKPAD_PTP_SMOOTHING == TRUE
    // Velocity-adaptive smoothing of the emitted absolute contacts, keyed by
    // host_id (stable for a contact's lifetime). A host_id appearing with tip=1
    // that was not down last frame is a fresh contact, so its filter is reset to
    // seed at the true touch position (no startup ramp, no smear from a previous
    // contact that reused the id). Released (tip=0) contacts pass through
    // unfiltered and clear their slot.
    {
        // dt between emitted frames, clamped so the first frame after init or an
        // idle gap can't produce a degenerate derivative / alpha.
        float dt = (float)(now - last_filter_time) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.050f) dt = 0.050f;

        bool seen[NT_MAX_CONTACTS] = {0};
        for (uint8_t i = 0; i < emit.count; i++) {
            uint8_t id = emit.items[i].host_id;
            if (id >= NT_MAX_CONTACTS) continue;  // defensive; host_ids are 0..1
            if (!emit.items[i].tip) {
                prev_emit_down[id] = false;       // release: drop history
                continue;
            }
            if (!prev_emit_down[id]) {
                nt_euro_point_reset(&contact_filter[id]);
            }
            float fx = (float)emit.items[i].x;
            float fy = (float)emit.items[i].y;
            nt_euro_point_filter(&contact_filter[id], &fx, &fy, dt,
                                 NAVIGATOR_TRACKPAD_SMOOTHING_MINCUTOFF,
                                 NAVIGATOR_TRACKPAD_SMOOTHING_BETA,
                                 NAVIGATOR_TRACKPAD_SMOOTHING_DCUTOFF);
            int32_t ix = (int32_t)(fx + 0.5f);
            int32_t iy = (int32_t)(fy + 0.5f);
            if (ix < 0) ix = 0;
            if (ix > TRACKPAD_LOGICAL_MAX) ix = TRACKPAD_LOGICAL_MAX;
            if (iy < 0) iy = 0;
            if (iy > TRACKPAD_LOGICAL_MAX) iy = TRACKPAD_LOGICAL_MAX;
            emit.items[i].x    = (uint16_t)ix;
            emit.items[i].y    = (uint16_t)iy;
            prev_emit_down[id] = true;
            seen[id]           = true;
        }
        // Any slot not emitted this frame is no longer down.
        for (uint8_t id = 0; id < NT_MAX_CONTACTS; id++) {
            if (!seen[id]) prev_emit_down[id] = false;
        }
        last_filter_time = now;
    }
#endif

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
