// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#include "quantum.h"

#include "navigator_trackpad.h"
#include "navigator_trackpad_common.h"
#include "navigator_trackpad_ptp.h"
#include "digitizer.h"

// Strong override: called once during keyboard_post_init_quantum().
void digitizer_touchpad_init(void) {
    navigator_trackpad_device_init();
}

// Strong override: called from keyboard_task() each iteration.
// navigator_trackpad_ptp_task internally checks digitizer_touchpad_get_input_mode()
// and dispatches between PTP and mouse-fallback paths as needed.
bool digitizer_touchpad_task(void) {
    return navigator_trackpad_ptp_task();
}

// Keycode handler for module-declared keycodes.
// navigator_trackpad_set_cpi(0) decrements one CPI tick; any non-zero value increments.
// CPI controls only matter in mouse-fallback mode; harmless in PTP mode.
//
// All NAVIGATOR_* / TRACKPAD_* / TOGGLE_* / DRAG_SCROLL keycodes are declared
// once in the zsa/defaults module (always loaded) so trackball and trackpad can
// be compiled together without enum redefinitions, and so layouts authored for a
// trackball (e.g. via Oryx) compile unchanged on a trackpad. Here the CPI
// variants map to the trackpad's real CPI control; the trackball-only
// speed/scroll keycodes have no trackpad equivalent and are accepted as no-ops
// (swallowed so they don't type anything).
bool process_record_navigator_trackpad(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case TRACKPAD_INC_CPI:
        case NAVIGATOR_INC_CPI:
            if (record->event.pressed) navigator_trackpad_set_cpi(1);
            return false;
        case TRACKPAD_DEC_CPI:
        case NAVIGATOR_DEC_CPI:
            if (record->event.pressed) navigator_trackpad_set_cpi(0);
            return false;

        // Trackball-only keycodes: no trackpad behavior, swallowed for compatibility.
        case NAVIGATOR_TURBO:
        case NAVIGATOR_AIM:
        case TOGGLE_TURBO:
        case TOGGLE_AIM:
        case DRAG_SCROLL:
        case TOGGLE_SCROLL:
        case NAVIGATOR_CLEAR_SPEED:
            return false;
    }
    return true;
}
