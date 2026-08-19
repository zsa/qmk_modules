// Copyright 2025 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifndef AUTOMOUSE_LAYER
#    error "AUTOMOUSE_LAYER must be defined when using the automouse module"
#endif

// Each device can raise its own mouse layer. Both default to AUTOMOUSE_LAYER, so
// existing keymaps are unaffected; define these to give the trackball and trackpad
// distinct layers.
#ifndef AUTOMOUSE_LAYER_TRACKBALL
#    define AUTOMOUSE_LAYER_TRACKBALL AUTOMOUSE_LAYER
#endif
#ifndef AUTOMOUSE_LAYER_TRACKPAD
#    define AUTOMOUSE_LAYER_TRACKPAD AUTOMOUSE_LAYER
#endif

// Timeout, motion threshold and oneshot mode can each be set per device. Every
// per-device macro defaults to the matching global, so a keymap that only sets
// the globals behaves exactly as before. Define the _TRACKBALL / _TRACKPAD
// variants to diverge.

#ifndef AUTOMOUSE_TIMEOUT
#    define AUTOMOUSE_TIMEOUT 650
#endif
#ifndef AUTOMOUSE_TIMEOUT_TRACKBALL
#    define AUTOMOUSE_TIMEOUT_TRACKBALL AUTOMOUSE_TIMEOUT
#endif
#ifndef AUTOMOUSE_TIMEOUT_TRACKPAD
#    define AUTOMOUSE_TIMEOUT_TRACKPAD AUTOMOUSE_TIMEOUT
#endif

#ifndef AUTOMOUSE_THRESHOLD
#    define AUTOMOUSE_THRESHOLD 10
#endif
#ifndef AUTOMOUSE_THRESHOLD_TRACKBALL
#    define AUTOMOUSE_THRESHOLD_TRACKBALL AUTOMOUSE_THRESHOLD
#endif
#ifndef AUTOMOUSE_THRESHOLD_TRACKPAD
#    define AUTOMOUSE_THRESHOLD_TRACKPAD AUTOMOUSE_THRESHOLD
#endif

#ifndef AUTOMOUSE_SCROLL_THRESHOLD
#    define AUTOMOUSE_SCROLL_THRESHOLD AUTOMOUSE_THRESHOLD
#endif
#ifndef AUTOMOUSE_SCROLL_THRESHOLD_TRACKBALL
#    define AUTOMOUSE_SCROLL_THRESHOLD_TRACKBALL AUTOMOUSE_SCROLL_THRESHOLD
#endif
#ifndef AUTOMOUSE_SCROLL_THRESHOLD_TRACKPAD
#    define AUTOMOUSE_SCROLL_THRESHOLD_TRACKPAD AUTOMOUSE_SCROLL_THRESHOLD
#endif

// Oneshot ("tap to exit") mode. The legacy global AUTOMOUSE_ONESHOT (presence =
// on) sets the default for both devices; the per-device variants take a boolean
// (true/false) so one device can use oneshot while the other uses plain timeout.
#ifdef AUTOMOUSE_ONESHOT
#    define AUTOMOUSE_ONESHOT_DEFAULT true
#else
#    define AUTOMOUSE_ONESHOT_DEFAULT false
#endif
#ifndef AUTOMOUSE_ONESHOT_TRACKBALL
#    define AUTOMOUSE_ONESHOT_TRACKBALL AUTOMOUSE_ONESHOT_DEFAULT
#endif
#ifndef AUTOMOUSE_ONESHOT_TRACKPAD
#    define AUTOMOUSE_ONESHOT_TRACKPAD AUTOMOUSE_ONESHOT_DEFAULT
#endif

// Idle safety timeout (ms) for ONESHOT "tap to exit" mode. Before the first tap,
// the layer drops after this much inactivity. 0 = disabled (layer stays until tapped).
#ifndef AUTOMOUSE_ONESHOT_TIMEOUT
#    define AUTOMOUSE_ONESHOT_TIMEOUT 0
#endif
#ifndef AUTOMOUSE_ONESHOT_TIMEOUT_TRACKBALL
#    define AUTOMOUSE_ONESHOT_TIMEOUT_TRACKBALL AUTOMOUSE_ONESHOT_TIMEOUT
#endif
#ifndef AUTOMOUSE_ONESHOT_TIMEOUT_TRACKPAD
#    define AUTOMOUSE_ONESHOT_TIMEOUT_TRACKPAD AUTOMOUSE_ONESHOT_TIMEOUT
#endif

// Compile-time default enable state per device. Set to false to keep a device
// from triggering automouse at all (e.g. when only the other device has an
// automouse layer configured). The runtime per-device API and keycodes can
// still re-enable it; the global automouse_enable() restores these defaults.
#ifndef AUTOMOUSE_ENABLED_TRACKBALL
#    define AUTOMOUSE_ENABLED_TRACKBALL true
#endif
#ifndef AUTOMOUSE_ENABLED_TRACKPAD
#    define AUTOMOUSE_ENABLED_TRACKPAD true
#endif

#ifndef AUTOMOUSE_DEBOUNCE
#    define AUTOMOUSE_DEBOUNCE 25
#endif

#ifndef AUTOMOUSE_DELAY
#    define AUTOMOUSE_DELAY GET_TAPPING_TERM(QK_MOUSE_BUTTON_1, &(keyrecord_t){})
#endif

// Automouse can be enabled independently for each motion source so a build with
// both a trackball and a trackpad can, e.g., keep automouse on for one and off
// for the other. The two sources map to the module's two motion entry points.
typedef enum {
    AUTOMOUSE_DEVICE_TRACKBALL = 0,  // QMK pointing-device pipeline (report_mouse_t)
    AUTOMOUSE_DEVICE_TRACKPAD  = 1,  // external digitizer via automouse_report_motion()
    AUTOMOUSE_DEVICE_COUNT
} automouse_device_t;

// Global controls act on every device at once (backward compatible).
void automouse_enable(void);
void automouse_disable(void);
void automouse_toggle(void);
bool automouse_is_enabled(void);
bool automouse_is_active(void);

// Per-device controls.
void automouse_set_device_enabled(automouse_device_t dev, bool enabled);
bool automouse_get_device_enabled(automouse_device_t dev);
void automouse_toggle_device(automouse_device_t dev);

// Feed motion from a sensor that bypasses the QMK pointing-device pipeline
// (e.g. a digitizer/trackpad) so it can activate the mouse layer.
void automouse_report_motion(int16_t dx, int16_t dy, uint8_t buttons);
