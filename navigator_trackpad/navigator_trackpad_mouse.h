// Copyright 2025 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include "navigator_trackpad_common.h"
#include "pointing_device.h"
#include "report.h"

// Tap detection configuration
#ifndef NAVIGATOR_TRACKPAD_TAP_MOVE_THRESHOLD
#    define NAVIGATOR_TRACKPAD_TAP_MOVE_THRESHOLD 100  // Max movement (squared) before tap becomes a drag
#endif

#ifndef NAVIGATOR_TRACKPAD_TAP_TIMEOUT
#    define NAVIGATOR_TRACKPAD_TAP_TIMEOUT 200  // Max duration (ms) for a tap
#endif

#ifndef NAVIGATOR_TRACKPAD_TAP_SETTLE_TIME
#    define NAVIGATOR_TRACKPAD_TAP_SETTLE_TIME 30  // Ignore movement during initial contact (ms)
#endif

#ifndef NAVIGATOR_TRACKPAD_MAX_DELTA
#    define NAVIGATOR_TRACKPAD_MAX_DELTA 250  // Max allowed delta per frame to prevent jumps
#endif

// Scroll configuration
#ifndef NAVIGATOR_TRACKPAD_SCROLL_DIVIDER
#    define NAVIGATOR_TRACKPAD_SCROLL_DIVIDER 10
#endif

#ifndef NAVIGATOR_TRACKPAD_SCROLL_MULTIPLIER
#    define NAVIGATOR_TRACKPAD_SCROLL_MULTIPLIER 3
#endif

// Two-finger scrolling (define to enable)
// #define NAVIGATOR_TRACKPAD_SCROLL_WITH_TWO_FINGERS

// macOS scrolling mode (define to enable)
// macOS doesn't respect the HID Resolution Multiplier descriptor.
// When enabled, sends raw scroll deltas without the multiplier (like Apple trackpads).
// macOS applies its own HIDScrollResolution (400 DPI) and acceleration curves.
// When disabled (default), applies multiplier for Windows/Linux hi-res scrolling.
// #define NAVIGATOR_TRACKPAD_MACOS_SCROLLING

// Scroll inversion configuration
// Define these to invert scroll direction on respective axes
// #define NAVIGATOR_SCROLL_INVERT_X
// #define NAVIGATOR_SCROLL_INVERT_Y

// Scroll inertia configuration
/*
To enable, add to your config.h:
#define NAVIGATOR_TRACKPAD_SCROLL_INERTIA_ENABLE

Configurable values (all optional):
- NAVIGATOR_TRACKPAD_SCROLL_INERTIA_FRICTION - Higher = stops faster (default: 50, range 1-255)
- NAVIGATOR_TRACKPAD_SCROLL_INERTIA_INTERVAL - Time between glide reports in ms (default: 15)
- NAVIGATOR_TRACKPAD_SCROLL_INERTIA_TRIGGER - Minimum velocity to trigger glide (default: 3)
*/
#ifdef NAVIGATOR_TRACKPAD_SCROLL_INERTIA_ENABLE
#ifndef NAVIGATOR_TRACKPAD_SCROLL_INERTIA_FRICTION
#define NAVIGATOR_TRACKPAD_SCROLL_INERTIA_FRICTION 5  // Higher = stops faster (1-255)
#endif
#ifndef NAVIGATOR_TRACKPAD_SCROLL_INERTIA_INTERVAL
#define NAVIGATOR_TRACKPAD_SCROLL_INERTIA_INTERVAL 7  // Glide report interval in ms
#endif
#ifndef NAVIGATOR_TRACKPAD_SCROLL_INERTIA_TRIGGER
#define NAVIGATOR_TRACKPAD_SCROLL_INERTIA_TRIGGER 1    // Min velocity to trigger glide
#endif

typedef struct {
    int16_t  vx;           // Current X velocity (Q8 fixed point)
    int16_t  vy;           // Current Y velocity (Q8 fixed point)
    int16_t  smooth_vx;    // Smoothed X velocity (Q8 fixed point)
    int16_t  smooth_vy;    // Smoothed Y velocity (Q8 fixed point)
    uint16_t timer;        // Timer for interval tracking
    bool     active;       // Is glide currently active
#ifdef NAVIGATOR_TRACKPAD_MACOS_SCROLLING
    uint8_t  no_output_count; // Counter for consecutive frames with no output
#endif
} scroll_inertia_t;
#endif

// Trackpad gesture state machine
typedef enum {
    TP_IDLE,        // No fingers touching
    TP_MOVING,      // One finger movement = mouse cursor
    TP_SCROLLING,   // Two finger movement = scroll
} trackpad_state_t;

typedef struct {
    trackpad_state_t state;
    uint16_t         touch_start_time;  // When finger first touched
    uint16_t         settled_x;         // Position after settle time (for tap threshold)
    uint16_t         settled_y;
    uint16_t         prev_x;            // Previous position (for delta calculation)
    uint16_t         prev_y;
    uint8_t          max_finger_count;  // Max fingers seen during this gesture
    bool             settled;           // Has the settle time elapsed?
    bool             pending_click;     // Need to send a click release next cycle
    uint16_t         last_scroll_end;   // Time when last scroll gesture ended
} trackpad_gesture_t;

// Mouse mode functions
report_mouse_t navigator_trackpad_get_report(report_mouse_t mouse_report);

// Pointing device driver struct
extern const pointing_device_driver_t navigator_trackpad_pointing_device_driver;
