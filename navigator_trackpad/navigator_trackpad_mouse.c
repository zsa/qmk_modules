// Copyright 2025 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

// Mouse mode implementation for Navigator trackpad
// Handles gesture detection, tap-to-click, two-finger scrolling, and scroll inertia

#include <math.h>
#include "navigator_trackpad_mouse.h"
#include "navigator_trackpad_common.h"
#include "navigator.h"  // For scroll utilities
#include "quantum.h"
#include "timer.h"

// Pointing device driver definition
const pointing_device_driver_t navigator_trackpad_pointing_device_driver = {
    .init       = navigator_trackpad_device_init,
    .get_report = navigator_trackpad_get_report,
    .get_cpi    = navigator_trackpad_get_cpi,
    .set_cpi    = navigator_trackpad_set_cpi
};

// Mode-specific globals
trackpad_gesture_t  gesture = {0};
extern bool         set_scrolling;  // Declared in navigator.c

// Local sensor report for mouse mode
static cgen6_report_t ptp_report;

#ifdef NAVIGATOR_TRACKPAD_SCROLL_INERTIA_ENABLE
scroll_inertia_t    scroll_inertia = {0};
#endif

#ifdef NAVIGATOR_TRACKPAD_MACOS_SCROLLING
float macos_scroll_accumulated_h = 0;
float macos_scroll_accumulated_v = 0;
#endif

report_mouse_t navigator_trackpad_get_report(report_mouse_t mouse_report) {
    // Handle pending click release from previous cycle
    if (gesture.pending_click) {
        gesture.pending_click = false;
        mouse_report.buttons  = 0;
        return mouse_report;
    }

#ifdef NAVIGATOR_TRACKPAD_SCROLL_INERTIA_ENABLE
    // Process scroll inertia when active
    if (scroll_inertia.active && timer_elapsed(scroll_inertia.timer) >= NAVIGATOR_TRACKPAD_SCROLL_INERTIA_INTERVAL) {
        scroll_inertia.timer = timer_read();

        // Apply friction to velocity (Q8 fixed point math)
        // Friction reduces velocity towards zero
        int16_t friction_x = (scroll_inertia.vx * NAVIGATOR_TRACKPAD_SCROLL_INERTIA_FRICTION) / 256;
        int16_t friction_y = (scroll_inertia.vy * NAVIGATOR_TRACKPAD_SCROLL_INERTIA_FRICTION) / 256;

        // Ensure we always reduce by at least 1 if not zero
        if (scroll_inertia.vx > 0 && friction_x < 1) friction_x = 1;
        if (scroll_inertia.vx < 0 && friction_x > -1) friction_x = -1;
        if (scroll_inertia.vy > 0 && friction_y < 1) friction_y = 1;
        if (scroll_inertia.vy < 0 && friction_y > -1) friction_y = -1;

        scroll_inertia.vx -= friction_x;
        scroll_inertia.vy -= friction_y;

        // Convert Q8 velocity to scroll value
#    ifdef NAVIGATOR_TRACKPAD_MACOS_SCROLLING
        // macOS mode: send raw velocity deltas (descriptor tells macOS the resolution)
        int16_t scroll_x = scroll_inertia.vx / 256;
        int16_t scroll_y = scroll_inertia.vy / 256;
#    else
        // Hi-res mode: apply multiplier for Windows/Linux
        int16_t scroll_x = (scroll_inertia.vx * NAVIGATOR_TRACKPAD_SCROLL_MULTIPLIER) / 256;
        int16_t scroll_y = (scroll_inertia.vy * NAVIGATOR_TRACKPAD_SCROLL_MULTIPLIER) / 256;
#    endif

        // Clamp to int8_t range
        scroll_x = (scroll_x > 127) ? 127 : ((scroll_x < -127) ? -127 : scroll_x);
        scroll_y = (scroll_y > 127) ? 127 : ((scroll_y < -127) ? -127 : scroll_y);

        // Check if velocity is too low to continue
        int16_t abs_vx = scroll_inertia.vx < 0 ? -scroll_inertia.vx : scroll_inertia.vx;
        int16_t abs_vy = scroll_inertia.vy < 0 ? -scroll_inertia.vy : scroll_inertia.vy;

#    ifdef NAVIGATOR_TRACKPAD_MACOS_SCROLLING
        // In macOS mode, track consecutive frames with no output to detect janky tail
        if (scroll_x == 0 && scroll_y == 0) {
            scroll_inertia.no_output_count++;
        } else {
            scroll_inertia.no_output_count = 0;
        }

        // Stop if: velocity very low OR too many consecutive frames without output
        bool velocity_too_low = (abs_vx < 64 && abs_vy < 64);  // Same as hi-res mode
        bool stalled = (scroll_inertia.no_output_count > 5);    // 5 frames (~35ms) with no output
        if (velocity_too_low || stalled) {
            scroll_inertia.active = false;
        } else
#    else
        if (abs_vx < 64 && abs_vy < 64) {  // Threshold in Q8 (0.25 in real units)
            scroll_inertia.active = false;
        } else
#    endif
        {
            // Apply scroll inversion if configured
#    ifdef NAVIGATOR_SCROLL_INVERT_X
            mouse_report.h = -scroll_x;
#    else
            mouse_report.h = scroll_x;
#    endif
#    ifdef NAVIGATOR_SCROLL_INVERT_Y
            mouse_report.v = scroll_y;
#    else
            mouse_report.v = -scroll_y;
#    endif
            return mouse_report;
        }
    }
#endif

    if (!trackpad_init) {
        return mouse_report;
    }
    // Create local snapshot to avoid race condition with callback updating ptp_report
    cgen6_report_t local_report = ptp_report;

    uint8_t raw_fingers = cirque_gen6_finger_count(&local_report);
    bool    is_touching = local_report.fingers[0].tip;
    bool    was_idle    = (gesture.state == TP_IDLE);
    uint8_t fingers     = raw_fingers;

    // Handle finger down - record start position (regardless of current state)
    if (is_touching && was_idle) {
        gesture.touch_start_time = timer_read();
        gesture.prev_x           = local_report.fingers[0].x;
        gesture.prev_y           = local_report.fingers[0].y;
        gesture.settled_x        = 0;  // Will be set after settle time
        gesture.settled_y        = 0;
        gesture.settled          = false;
        gesture.max_finger_count = fingers;
        gesture.state            = TP_MOVING;
#    ifdef NAVIGATOR_TRACKPAD_SCROLL_INERTIA_ENABLE
        // Stop any ongoing scroll inertia when finger touches
        scroll_inertia.active    = false;
        scroll_inertia.smooth_vx = 0;
        scroll_inertia.smooth_vy = 0;
#        ifdef NAVIGATOR_TRACKPAD_MACOS_SCROLLING
        scroll_inertia.no_output_count = 0;
#        endif
#    endif
#    ifdef NAVIGATOR_TRACKPAD_MACOS_SCROLLING
        // Reset macOS scroll accumulator when starting new gesture
        macos_scroll_accumulated_h = 0;
        macos_scroll_accumulated_v = 0;
#    endif
    }

    // Handle finger up - evaluate tap at lift time (libinput style)
    if (!is_touching && !was_idle) {
        uint16_t duration = timer_elapsed(gesture.touch_start_time);

        // Calculate distance from settled position (or treat as no movement if never settled)
        int32_t dist_sq = 0;
        if (gesture.settled) {
            int16_t dx = gesture.prev_x - gesture.settled_x;
            int16_t dy = gesture.prev_y - gesture.settled_y;
            dist_sq    = (int32_t)dx * dx + (int32_t)dy * dy;
        }

        // Check tap conditions: short duration AND small movement from settled position
        bool is_tap = (duration <= NAVIGATOR_TRACKPAD_TAP_TIMEOUT) &&
                      (dist_sq <= NAVIGATOR_TRACKPAD_TAP_MOVE_THRESHOLD);

        // Don't trigger single-finger taps that happen shortly after a scroll ends (within 100ms)
        // But allow two-finger taps (right-click) even after scrolling
#    ifdef NAVIGATOR_TRACKPAD_SCROLL_WITH_TWO_FINGERS
        if (is_tap && gesture.max_finger_count == 1 &&
            timer_elapsed(gesture.last_scroll_end) < 100) {
            is_tap = false;  // Suppress single-finger tap after scrolling
        }
#    endif

        if (is_tap) {
#    ifdef NAVIGATOR_TRACKPAD_ENABLE_DOUBLE_TAP
            if (gesture.max_finger_count >= 2) {
                mouse_report.x        = 0;
                mouse_report.y        = 0;
                mouse_report.buttons  = pointing_device_handle_buttons(mouse_report.buttons, true, POINTING_DEVICE_BUTTON2);
                gesture.pending_click = true;
            } else
#    endif
#    ifdef NAVIGATOR_TRACKPAD_ENABLE_TAP
            if (gesture.max_finger_count == 1) {
                mouse_report.x        = 0;
                mouse_report.y        = 0;
                mouse_report.buttons  = pointing_device_handle_buttons(mouse_report.buttons, true, POINTING_DEVICE_BUTTON1);
                gesture.pending_click = true;
            }
#    endif
        }

#    if defined(NAVIGATOR_TRACKPAD_SCROLL_INERTIA_ENABLE) && defined(NAVIGATOR_TRACKPAD_SCROLL_WITH_TWO_FINGERS)
        // Start scroll inertia if we were scrolling and have enough velocity
        if (gesture.state == TP_SCROLLING) {
            // Use smoothed velocity (already in Q8 format)
            int16_t abs_vx = scroll_inertia.smooth_vx < 0 ? -scroll_inertia.smooth_vx : scroll_inertia.smooth_vx;
            int16_t abs_vy = scroll_inertia.smooth_vy < 0 ? -scroll_inertia.smooth_vy : scroll_inertia.smooth_vy;
            // Trigger threshold is now in Q8 (multiply by 256)
            if (abs_vx >= (NAVIGATOR_TRACKPAD_SCROLL_INERTIA_TRIGGER * 256) ||
                abs_vy >= (NAVIGATOR_TRACKPAD_SCROLL_INERTIA_TRIGGER * 256)) {
                scroll_inertia.vx     = scroll_inertia.smooth_vx;
                scroll_inertia.vy     = scroll_inertia.smooth_vy;
                scroll_inertia.timer  = timer_read();
                scroll_inertia.active = true;
            }
        }
#    endif

        // Record if this was a scroll gesture ending
#    ifdef NAVIGATOR_TRACKPAD_SCROLL_WITH_TWO_FINGERS
        if (gesture.state == TP_SCROLLING || gesture.max_finger_count >= 2) {
            gesture.last_scroll_end = timer_read();
        }
#    endif

        gesture.state = TP_IDLE;
        set_scrolling = false;
    }

    // Handle ongoing touch - movement and scrolling
    if (is_touching && !was_idle) {
        // Track max fingers during gesture
        if (fingers > gesture.max_finger_count) {
            gesture.max_finger_count = fingers;
        }

#    ifdef NAVIGATOR_TRACKPAD_SCROLL_WITH_TWO_FINGERS
        // Determine mode based on finger count
        // Once scrolling starts, keep scrolling until all fingers lift (no mid-gesture transitions)
        if (fingers >= 2 && gesture.state != TP_SCROLLING) {
            gesture.state = TP_SCROLLING;
            // Reset position tracking - use finger[0] initially
            gesture.prev_x = local_report.fingers[0].x;
            gesture.prev_y = local_report.fingers[0].y;
        }
        // Note: We don't transition from SCROLLING back to MOVING mid-gesture anymore
        // Once scroll starts, it continues until all fingers lift (gesture ends)
#    endif

        uint16_t duration = timer_elapsed(gesture.touch_start_time);

        // Record settled position once settle time elapses
        if (!gesture.settled && duration >= NAVIGATOR_TRACKPAD_TAP_SETTLE_TIME) {
            gesture.settled   = true;
            gesture.settled_x = local_report.fingers[0].x;
            gesture.settled_y = local_report.fingers[0].y;
        }

        // Check if we should suppress movement (might still be a tap)
        int32_t dist_sq = 0;
        if (gesture.settled) {
            int16_t dx = local_report.fingers[0].x - gesture.settled_x;
            int16_t dy = local_report.fingers[0].y - gesture.settled_y;
            dist_sq    = (int32_t)dx * dx + (int32_t)dy * dy;
        }

        // Only report movement if: timeout exceeded OR moved beyond tap threshold (after settling)
        bool should_move = (duration > NAVIGATOR_TRACKPAD_TAP_TIMEOUT) ||
                           (gesture.settled && dist_sq > NAVIGATOR_TRACKPAD_TAP_MOVE_THRESHOLD);

        if (should_move) {
            int16_t delta_x = local_report.fingers[0].x - gesture.prev_x;
            int16_t delta_y = local_report.fingers[0].y - gesture.prev_y;

            // Clamp deltas to prevent jumps from bad data
            if (delta_x > NAVIGATOR_TRACKPAD_MAX_DELTA) delta_x = NAVIGATOR_TRACKPAD_MAX_DELTA;
            if (delta_x < -NAVIGATOR_TRACKPAD_MAX_DELTA) delta_x = -NAVIGATOR_TRACKPAD_MAX_DELTA;
            if (delta_y > NAVIGATOR_TRACKPAD_MAX_DELTA) delta_y = NAVIGATOR_TRACKPAD_MAX_DELTA;
            if (delta_y < -NAVIGATOR_TRACKPAD_MAX_DELTA) delta_y = -NAVIGATOR_TRACKPAD_MAX_DELTA;

            if (delta_x != 0 || delta_y != 0) {
#    ifdef NAVIGATOR_TRACKPAD_SCROLL_WITH_TWO_FINGERS
                if (gesture.state == TP_SCROLLING) {
#    ifdef NAVIGATOR_TRACKPAD_MACOS_SCROLLING
                    // macOS mode: send raw deltas, macOS handles scaling via HIDScrollResolution
                    // Apple trackpads report raw sensor deltas and let macOS apply acceleration
                    int16_t scroll_x = delta_x;
                    int16_t scroll_y = delta_y;
#    else
                    // Hi-res mode: apply multiplier for Windows/Linux
                    // These OSes divide by the Resolution Multiplier (120)
                    int16_t scroll_x = delta_x * NAVIGATOR_TRACKPAD_SCROLL_MULTIPLIER;
                    int16_t scroll_y = delta_y * NAVIGATOR_TRACKPAD_SCROLL_MULTIPLIER;
#    endif

#    ifdef NAVIGATOR_TRACKPAD_SCROLL_INERTIA_ENABLE
                    // Track velocity for inertia using exponential smoothing (Q8 fixed point)
                    // Alpha = 0.3 (77/256) - balances responsiveness with smoothness
                    // When direction changes, reset smoothing to avoid fighting old momentum
                    int16_t new_vx = delta_x * 256;
                    int16_t new_vy = delta_y * 256;

                    // Detect direction change (signs differ and both non-zero)
                    bool dir_change_x = (scroll_inertia.smooth_vx > 0 && new_vx < 0) ||
                                        (scroll_inertia.smooth_vx < 0 && new_vx > 0);
                    bool dir_change_y = (scroll_inertia.smooth_vy > 0 && new_vy < 0) ||
                                        (scroll_inertia.smooth_vy < 0 && new_vy > 0);

                    if (dir_change_x) {
                        // Direction changed - blend toward zero first, then pick up new direction
                        scroll_inertia.smooth_vx = (scroll_inertia.smooth_vx * 128 + new_vx * 128) / 256;
                    } else {
                        // Same direction - normal EMA smoothing
                        scroll_inertia.smooth_vx = (scroll_inertia.smooth_vx * 179 + new_vx * 77) / 256;
                    }

                    if (dir_change_y) {
                        scroll_inertia.smooth_vy = (scroll_inertia.smooth_vy * 128 + new_vy * 128) / 256;
                    } else {
                        scroll_inertia.smooth_vy = (scroll_inertia.smooth_vy * 179 + new_vy * 77) / 256;
                    }
#    endif

                    // Clamp to int8_t range for the report
                    scroll_x = (scroll_x > 127) ? 127 : ((scroll_x < -127) ? -127 : scroll_x);
                    scroll_y = (scroll_y > 127) ? 127 : ((scroll_y < -127) ? -127 : scroll_y);

                    // Apply scroll inversion if configured
#    ifdef NAVIGATOR_SCROLL_INVERT_X
                    mouse_report.h = -scroll_x;
#    else
                    mouse_report.h = scroll_x;
#    endif
#    ifdef NAVIGATOR_SCROLL_INVERT_Y
                    mouse_report.v = scroll_y;
#    else
                    mouse_report.v = -scroll_y;
#    endif
                    mouse_report.x = 0;
                    mouse_report.y = 0;
                } else
#    endif
                {
                    // One-finger movement: mouse cursor
                    mouse_report.x = (delta_x < 0) ? -powf(-delta_x, 1.2) : powf(delta_x, 1.2);
                    mouse_report.y = (delta_y < 0) ? -powf(-delta_y, 1.2) : powf(delta_y, 1.2);
                }
            }
        }

        // Update prev position for next frame
        gesture.prev_x = local_report.fingers[0].x;
        gesture.prev_y = local_report.fingers[0].y;
    }

    return mouse_report;
}
