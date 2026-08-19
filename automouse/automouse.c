// Copyright 2025 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H
#include "automouse.h"
#include <stdlib.h>

ASSERT_COMMUNITY_MODULES_MIN_API_VERSION(1, 1, 0);

static struct {
    uint16_t last_activity;
    uint16_t last_keypress;
    uint16_t last_activation;
    int16_t  accumulated_x[AUTOMOUSE_DEVICE_COUNT];
    int16_t  accumulated_y[AUTOMOUSE_DEVICE_COUNT];
    int16_t  accumulated_h[AUTOMOUSE_DEVICE_COUNT];
    int16_t  accumulated_v[AUTOMOUSE_DEVICE_COUNT];
    int8_t             held_keys;
    bool               is_active;
    automouse_device_t active_device;  // device that owns the session while is_active
    uint8_t            active_layer;    // mouse layer it raised
    bool               enabled[AUTOMOUSE_DEVICE_COUNT];
    bool               oneshot_triggered;
} state = {
    .enabled = {[AUTOMOUSE_DEVICE_TRACKBALL] = AUTOMOUSE_ENABLED_TRACKBALL, [AUTOMOUSE_DEVICE_TRACKPAD] = AUTOMOUSE_ENABLED_TRACKPAD},
};

static bool automouse_default_enabled_for(automouse_device_t dev) {
    return dev == AUTOMOUSE_DEVICE_TRACKPAD ? AUTOMOUSE_ENABLED_TRACKPAD : AUTOMOUSE_ENABLED_TRACKBALL;
}

// Per-device settings. Each lookup returns the compile-time value for the device,
// every one defaulting to the global in automouse.h.
static uint8_t automouse_layer_for(automouse_device_t dev) {
    return dev == AUTOMOUSE_DEVICE_TRACKPAD ? AUTOMOUSE_LAYER_TRACKPAD : AUTOMOUSE_LAYER_TRACKBALL;
}

static uint16_t automouse_timeout_for(automouse_device_t dev) {
    return dev == AUTOMOUSE_DEVICE_TRACKPAD ? AUTOMOUSE_TIMEOUT_TRACKPAD : AUTOMOUSE_TIMEOUT_TRACKBALL;
}

static int16_t automouse_threshold_for(automouse_device_t dev) {
    return dev == AUTOMOUSE_DEVICE_TRACKPAD ? AUTOMOUSE_THRESHOLD_TRACKPAD : AUTOMOUSE_THRESHOLD_TRACKBALL;
}

static int16_t automouse_scroll_threshold_for(automouse_device_t dev) {
    return dev == AUTOMOUSE_DEVICE_TRACKPAD ? AUTOMOUSE_SCROLL_THRESHOLD_TRACKPAD : AUTOMOUSE_SCROLL_THRESHOLD_TRACKBALL;
}

static bool automouse_oneshot_for(automouse_device_t dev) {
    return dev == AUTOMOUSE_DEVICE_TRACKPAD ? AUTOMOUSE_ONESHOT_TRACKPAD : AUTOMOUSE_ONESHOT_TRACKBALL;
}

static uint16_t automouse_oneshot_timeout_for(automouse_device_t dev) {
    return dev == AUTOMOUSE_DEVICE_TRACKPAD ? AUTOMOUSE_ONESHOT_TIMEOUT_TRACKPAD : AUTOMOUSE_ONESHOT_TIMEOUT_TRACKBALL;
}

static bool automouse_any_enabled(void) {
    for (uint8_t i = 0; i < AUTOMOUSE_DEVICE_COUNT; i++) {
        if (state.enabled[i]) {
            return true;
        }
    }
    return false;
}

static void automouse_reset_accumulators(void) {
    for (uint8_t i = 0; i < AUTOMOUSE_DEVICE_COUNT; i++) {
        state.accumulated_x[i] = 0;
        state.accumulated_y[i] = 0;
        state.accumulated_h[i] = 0;
        state.accumulated_v[i] = 0;
    }
}

static bool layer_held_externally(uint8_t layer) {
#ifdef LAYER_LOCK_ENABLE
    if (is_layer_locked(layer)) return true;
#endif
    if (is_oneshot_layer_active() && get_oneshot_layer() == layer) return true;
    return false;
}

static void automouse_activate(automouse_device_t dev) {
    uint8_t layer = automouse_layer_for(dev);
    if (state.is_active) {
        // A different device crossed the threshold and is taking over the session.
        if (state.active_device != dev) {
            // Don't yank a layer that has keys held on it; keep the current one alive.
            if (state.held_keys > 0) {
                state.last_activity = timer_read();
                return;
            }
            if (state.active_layer != layer && !layer_held_externally(state.active_layer)) {
                layer_off(state.active_layer);
            }
            state.active_device = dev;
            state.active_layer  = layer;
            state.held_keys     = 0;
        }
    } else {
        // Don't take ownership if the layer is already on by other means
        if (layer_state_is(layer)) {
            return;
        }
        state.is_active     = true;
        state.active_device = dev;
        state.active_layer  = layer;
        state.held_keys     = 0;
    }
    // Ensure layer is on — it may have been turned off externally (e.g. TO(), TG())
    if (!layer_state_is(layer)) {
        layer_on(layer);
    }
    state.last_activity     = timer_read();
    state.oneshot_triggered = false;
}

static void automouse_deactivate(void) {
    if (state.is_active) {
        uint8_t layer   = state.active_layer;
        state.is_active = false;
        state.held_keys = 0;
        automouse_reset_accumulators();
        if (!layer_held_externally(layer)) {
            layer_off(layer);
        }
    }
}

// Accumulate motion deltas and activate the device's mouse layer once movement
// crosses the threshold. Each motion source keeps its own accumulators so a
// device only raises its layer from its own movement. Called by the
// pointing-device task (trackball) and by external sensors that bypass that
// pipeline — e.g. the Navigator trackpad, which emits digitizer reports directly.
static void automouse_accumulate(automouse_device_t dev, int16_t x, int16_t y, int16_t h, int16_t v, uint8_t buttons) {
    // Skip activation checks during debounce or post-keypress delay
    if (timer_elapsed(state.last_activation) <= AUTOMOUSE_DEBOUNCE ||
        timer_elapsed(state.last_keypress) <= AUTOMOUSE_DELAY) {
        return;
    }

    state.accumulated_x[dev] += x;
    state.accumulated_y[dev] += y;
    state.accumulated_h[dev] += h;
    state.accumulated_v[dev] += v;

    int16_t threshold        = automouse_threshold_for(dev);
    int16_t scroll_threshold = automouse_scroll_threshold_for(dev);

    bool threshold_exceeded = abs(state.accumulated_x[dev]) > threshold ||
                              abs(state.accumulated_y[dev]) > threshold ||
                              abs(state.accumulated_h[dev]) > scroll_threshold ||
                              abs(state.accumulated_v[dev]) > scroll_threshold ||
                              (buttons && state.is_active);

    if (threshold_exceeded) {
        state.accumulated_x[dev] = 0;
        state.accumulated_y[dev] = 0;
        state.accumulated_h[dev] = 0;
        state.accumulated_v[dev] = 0;
        state.last_activation    = timer_read();
        automouse_activate(dev);
    }
}

// --- Public API ---

void automouse_enable(void) {
    // Restore the compile-time defaults rather than forcing every device on,
    // so a device excluded from automouse in the build stays excluded when
    // automouse is globally toggled back on.
    for (uint8_t i = 0; i < AUTOMOUSE_DEVICE_COUNT; i++) {
        state.enabled[i] = automouse_default_enabled_for((automouse_device_t)i);
    }
}

void automouse_disable(void) {
    automouse_deactivate();
    for (uint8_t i = 0; i < AUTOMOUSE_DEVICE_COUNT; i++) {
        state.enabled[i] = false;
    }
}

void automouse_toggle(void) {
    if (automouse_any_enabled()) {
        automouse_disable();
    } else {
        automouse_enable();
    }
}

bool automouse_is_enabled(void) {
    return automouse_any_enabled();
}

void automouse_set_device_enabled(automouse_device_t dev, bool enabled) {
    if (dev >= AUTOMOUSE_DEVICE_COUNT) {
        return;
    }
    state.enabled[dev] = enabled;
    // Only drop the layer once no device is left to drive it.
    if (!automouse_any_enabled()) {
        automouse_deactivate();
    }
}

bool automouse_get_device_enabled(automouse_device_t dev) {
    if (dev >= AUTOMOUSE_DEVICE_COUNT) {
        return false;
    }
    return state.enabled[dev];
}

void automouse_toggle_device(automouse_device_t dev) {
    if (dev >= AUTOMOUSE_DEVICE_COUNT) {
        return;
    }
    automouse_set_device_enabled(dev, !state.enabled[dev]);
}

bool automouse_is_active(void) {
    return state.is_active;
}

// --- Module hooks ---

report_mouse_t pointing_device_task_automouse(report_mouse_t mouse_report) {
    // Housekeeping (timeout/deactivation) must keep running while either device
    // is enabled — the active layer may be owned by the trackpad.
    if (!automouse_any_enabled()) {
        return mouse_report;
    }

    // Keep layer alive while keys are held on it
    if (state.is_active && state.held_keys > 0) {
        state.last_activity = timer_read();
    }

    // Timeout/oneshot behaviour follows the device that owns the active session.
    if (state.is_active) {
        uint16_t elapsed   = timer_elapsed(state.last_activity);
        bool     timed_out;
        if (automouse_oneshot_for(state.active_device)) {
            uint16_t idle = automouse_oneshot_timeout_for(state.active_device);
            // After the exit tap: short timeout. Before it: optional idle safety drop.
            bool short_timeout = state.oneshot_triggered &&
                                 elapsed > automouse_timeout_for(state.active_device);
            bool idle_timeout  = !state.oneshot_triggered && idle > 0 && elapsed > idle;
            timed_out = short_timeout || idle_timeout;
        } else {
            timed_out = elapsed > automouse_timeout_for(state.active_device);
        }
        if (timed_out) {
            automouse_deactivate();
        }
    }

    // Trackball motion arrives through the pointing-device pipeline.
    if (state.enabled[AUTOMOUSE_DEVICE_TRACKBALL]) {
        automouse_accumulate(AUTOMOUSE_DEVICE_TRACKBALL, mouse_report.x, mouse_report.y, mouse_report.h, mouse_report.v, mouse_report.buttons);
    }

    return mouse_report;
}

// Activation entry point for pointing sensors outside the QMK pointing-device
// pipeline (the trackpad sends digitizer reports, never a report_mouse_t).
// The module's pointing_device_task hook still runs every cycle — automouse
// forces POINTING_DEVICE_ENABLE — so timeout/deactivation housekeeping is
// unaffected; this only injects motion as an activation signal.
void automouse_report_motion(int16_t dx, int16_t dy, uint8_t buttons) {
    if (!state.enabled[AUTOMOUSE_DEVICE_TRACKPAD]) {
        return;
    }
    automouse_accumulate(AUTOMOUSE_DEVICE_TRACKPAD, dx, dy, 0, 0, buttons);
}

bool process_record_automouse(uint16_t keycode, keyrecord_t *record) {
    if (keycode == KC_AUTOMOUSE_TOGGLE) {
        if (record->event.pressed) {
            automouse_toggle();
        }
        return false;
    }
    if (keycode == KC_AUTOMOUSE_TOGGLE_TRACKBALL) {
        if (record->event.pressed) {
            automouse_toggle_device(AUTOMOUSE_DEVICE_TRACKBALL);
        }
        return false;
    }
    if (keycode == KC_AUTOMOUSE_TOGGLE_TRACKPAD) {
        if (record->event.pressed) {
            automouse_toggle_device(AUTOMOUSE_DEVICE_TRACKPAD);
        }
        return false;
    }

    if (!automouse_any_enabled() || !state.is_active) {
        // Track keypresses for the post-keypress delay only when layer is inactive
        if (record->event.pressed) {
            state.last_keypress = timer_read();
        }
        return true;
    }

    // Track held keys while automouse layer is active
    if (record->event.pressed) {
        state.held_keys++;
    } else if (state.held_keys > 0) {
        state.held_keys--;
    }

    if (automouse_oneshot_for(state.active_device)) {
        // Start timeout on key up so the full tap completes on the mouse layer
        if (!record->event.pressed && !state.oneshot_triggered) {
            state.oneshot_triggered = true;
            state.last_activity     = timer_read();
        }
    } else {
        if (record->event.pressed) {
            state.last_activity = timer_read();
        }
    }

    return true;
}
