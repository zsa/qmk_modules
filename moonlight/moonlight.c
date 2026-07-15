// Copyright 2026 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Moonlight: light control for upcycled keyboards. Purely a lamp module —
// see README.md. All behavior is driven by rgb_matrix.

#include QMK_KEYBOARD_H

ASSERT_COMMUNITY_MODULES_MIN_API_VERSION(1, 0, 0);

#ifndef RGB_MATRIX_ENABLE
#    error "The moonlight module requires an rgb_matrix-enabled keyboard (RGB_MATRIX_ENABLE = yes)."
#endif

bool process_record_moonlight(uint16_t keycode, keyrecord_t *record) {
    if (!process_record_moonlight_kb(keycode, record)) {
        return false;
    }

    // Moonlight keycodes act on press only; consume the release too.
    if (!record->event.pressed) {
        switch (keycode) {
            case MOONLIGHT_ON ... MOONLIGHT_PRESET_8:
                return false;
            default:
                return true;
        }
    }

    switch (keycode) {
        case MOONLIGHT_ON:
            rgb_matrix_enable();
            return false;
        case MOONLIGHT_OFF:
            rgb_matrix_disable();
            return false;
        case MOONLIGHT_BRIGHTER:
            rgb_matrix_increase_val(); // clamps at RGB_MATRIX_MAXIMUM_BRIGHTNESS
            return false;
        case MOONLIGHT_DIMMER: {
            HSV     hsv = rgb_matrix_get_hsv();
            uint8_t v   = hsv.v > MOONLIGHT_MIN_BRIGHTNESS + RGB_MATRIX_VAL_STEP
                              ? hsv.v - RGB_MATRIX_VAL_STEP
                              : MOONLIGHT_MIN_BRIGHTNESS;
            rgb_matrix_sethsv(hsv.h, hsv.s, v);
            return false;
        }
        case MOONLIGHT_HUE_UP:
            rgb_matrix_increase_hue();
            return false;
        case MOONLIGHT_HUE_DOWN:
            rgb_matrix_decrease_hue();
            return false;
        default:
            return true;
    }
}
