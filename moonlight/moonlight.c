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

// Modes that must never run on a lamp: reactive/keypress-driven effects,
// plus NONE and out-of-range. SOLID_COLOR is excluded from the carousel
// too — it is the "steady" state reached via MOONLIGHT_ANIM_STOP.
static bool moonlight_mode_is_lamp_safe(uint8_t mode) {
    switch (mode) {
        case RGB_MATRIX_NONE:
        case RGB_MATRIX_SOLID_COLOR:
#ifdef ENABLE_RGB_MATRIX_SOLID_REACTIVE_SIMPLE
        case RGB_MATRIX_SOLID_REACTIVE_SIMPLE:
#endif
#ifdef ENABLE_RGB_MATRIX_SOLID_REACTIVE
        case RGB_MATRIX_SOLID_REACTIVE:
#endif
#ifdef ENABLE_RGB_MATRIX_SOLID_REACTIVE_WIDE
        case RGB_MATRIX_SOLID_REACTIVE_WIDE:
#endif
#ifdef ENABLE_RGB_MATRIX_SOLID_REACTIVE_MULTIWIDE
        case RGB_MATRIX_SOLID_REACTIVE_MULTIWIDE:
#endif
#ifdef ENABLE_RGB_MATRIX_SOLID_REACTIVE_CROSS
        case RGB_MATRIX_SOLID_REACTIVE_CROSS:
#endif
#ifdef ENABLE_RGB_MATRIX_SOLID_REACTIVE_MULTICROSS
        case RGB_MATRIX_SOLID_REACTIVE_MULTICROSS:
#endif
#ifdef ENABLE_RGB_MATRIX_SOLID_REACTIVE_NEXUS
        case RGB_MATRIX_SOLID_REACTIVE_NEXUS:
#endif
#ifdef ENABLE_RGB_MATRIX_SOLID_REACTIVE_MULTINEXUS
        case RGB_MATRIX_SOLID_REACTIVE_MULTINEXUS:
#endif
#ifdef ENABLE_RGB_MATRIX_SPLASH
        case RGB_MATRIX_SPLASH:
#endif
#ifdef ENABLE_RGB_MATRIX_MULTISPLASH
        case RGB_MATRIX_MULTISPLASH:
#endif
#ifdef ENABLE_RGB_MATRIX_SOLID_SPLASH
        case RGB_MATRIX_SOLID_SPLASH:
#endif
#ifdef ENABLE_RGB_MATRIX_SOLID_MULTISPLASH
        case RGB_MATRIX_SOLID_MULTISPLASH:
#endif
#ifdef ENABLE_RGB_MATRIX_TYPING_HEATMAP
        case RGB_MATRIX_TYPING_HEATMAP: // framebuffer effect, keypress-driven
#endif
            return false;
        default:
            return mode < RGB_MATRIX_EFFECT_MAX;
    }
}

// Next lamp-safe animation after `from`, wrapping. Falls back to
// SOLID_COLOR if the board somehow has no lamp-safe animations.
static uint8_t moonlight_next_anim(uint8_t from) {
    uint8_t mode = from;
    for (uint8_t i = 0; i < RGB_MATRIX_EFFECT_MAX; i++) {
        mode = (mode + 1 < RGB_MATRIX_EFFECT_MAX) ? mode + 1 : 1;
        if (moonlight_mode_is_lamp_safe(mode)) {
            return mode;
        }
    }
    return RGB_MATRIX_SOLID_COLOR;
}

#if !defined(MOONLIGHT_DEFAULT_ANIMATION) && defined(ENABLE_RGB_MATRIX_BREATHING)
#    define MOONLIGHT_DEFAULT_ANIMATION RGB_MATRIX_BREATHING
#endif

static uint8_t moonlight_default_anim(void) {
#ifdef MOONLIGHT_DEFAULT_ANIMATION
    if (moonlight_mode_is_lamp_safe(MOONLIGHT_DEFAULT_ANIMATION)) {
        return MOONLIGHT_DEFAULT_ANIMATION;
    }
#endif
    return moonlight_next_anim(RGB_MATRIX_SOLID_COLOR);
}

// Last animation used this power session (0 = none yet), so
// stop-then-start resumes the same animation.
static uint8_t moonlight_last_anim = 0;

// Preset palette. Keymaps override any slot in their config.h, e.g.
//     #define MOONLIGHT_PRESET_1_HSV {HSV_TEAL}
// Only hue and saturation are applied; current brightness is preserved
// (the v component is accepted for HSV_* macro convenience but ignored).
#ifndef MOONLIGHT_PRESET_1_HSV
#    define MOONLIGHT_PRESET_1_HSV {HSV_RED}
#endif
#ifndef MOONLIGHT_PRESET_2_HSV
#    define MOONLIGHT_PRESET_2_HSV {HSV_CORAL}
#endif
#ifndef MOONLIGHT_PRESET_3_HSV
#    define MOONLIGHT_PRESET_3_HSV {HSV_GOLD}
#endif
#ifndef MOONLIGHT_PRESET_4_HSV
#    define MOONLIGHT_PRESET_4_HSV {HSV_GREEN}
#endif
#ifndef MOONLIGHT_PRESET_5_HSV
#    define MOONLIGHT_PRESET_5_HSV {HSV_AZURE}
#endif
#ifndef MOONLIGHT_PRESET_6_HSV
#    define MOONLIGHT_PRESET_6_HSV {HSV_BLUE}
#endif
#ifndef MOONLIGHT_PRESET_7_HSV
#    define MOONLIGHT_PRESET_7_HSV {HSV_PURPLE}
#endif
#ifndef MOONLIGHT_PRESET_8_HSV
#    define MOONLIGHT_PRESET_8_HSV {HSV_WHITE}
#endif

static const HSV moonlight_presets[] = {
    MOONLIGHT_PRESET_1_HSV, MOONLIGHT_PRESET_2_HSV, MOONLIGHT_PRESET_3_HSV, MOONLIGHT_PRESET_4_HSV,
    MOONLIGHT_PRESET_5_HSV, MOONLIGHT_PRESET_6_HSV, MOONLIGHT_PRESET_7_HSV, MOONLIGHT_PRESET_8_HSV,
};

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
        case MOONLIGHT_ANIM_START: {
            uint8_t target = moonlight_last_anim ? moonlight_last_anim : moonlight_default_anim();
            rgb_matrix_enable();
            rgb_matrix_mode(target);
            moonlight_last_anim = target;
            return false;
        }
        case MOONLIGHT_ANIM_STOP: {
            uint8_t cur = rgb_matrix_get_mode();
            if (moonlight_mode_is_lamp_safe(cur)) {
                moonlight_last_anim = cur; // resume point for the next START
            }
            rgb_matrix_mode(RGB_MATRIX_SOLID_COLOR);
            return false;
        }
        case MOONLIGHT_ANIM_NEXT: {
            uint8_t cur  = rgb_matrix_get_mode();
            uint8_t base = moonlight_mode_is_lamp_safe(cur)
                               ? cur
                               : (moonlight_last_anim ? moonlight_last_anim : RGB_MATRIX_SOLID_COLOR);
            uint8_t next = moonlight_next_anim(base);
            rgb_matrix_enable();
            rgb_matrix_mode(next);
            moonlight_last_anim = next;
            return false;
        }
        case MOONLIGHT_ANIM_FASTER:
            rgb_matrix_increase_speed();
            return false;
        case MOONLIGHT_ANIM_SLOWER:
            rgb_matrix_decrease_speed();
            return false;
        case MOONLIGHT_PRESET_1 ... MOONLIGHT_PRESET_8: {
            HSV preset = moonlight_presets[keycode - MOONLIGHT_PRESET_1];
            rgb_matrix_mode(RGB_MATRIX_SOLID_COLOR); // presets always land steady
            rgb_matrix_sethsv(preset.h, preset.s, rgb_matrix_get_hsv().v);
            return false;
        }
        default:
            return true;
    }
}

void keyboard_post_init_moonlight(void) {
    keyboard_post_init_moonlight_kb();

    // A lamp on a wall switch always comes on.
    rgb_matrix_enable();

    // Never boot dark.
    HSV hsv = rgb_matrix_get_hsv();
    if (hsv.v < MOONLIGHT_MIN_BOOT_BRIGHTNESS) {
        rgb_matrix_sethsv(hsv.h, hsv.s, MOONLIGHT_MIN_BOOT_BRIGHTNESS);
    }

    // EEPROM may hold a reactive or out-of-range mode (previous firmware,
    // fewer animations, etc.). Snap those to steady; remember valid
    // animations as the START resume point.
    uint8_t mode = rgb_matrix_get_mode();
    if (mode == RGB_MATRIX_SOLID_COLOR) {
        // steady — nothing to do
    } else if (moonlight_mode_is_lamp_safe(mode)) {
        moonlight_last_anim = mode; // restored mid-animation; keep animating
    } else {
        rgb_matrix_mode(RGB_MATRIX_SOLID_COLOR);
    }
}
