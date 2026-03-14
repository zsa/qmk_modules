// Copyright 2024 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H

#if COMMUNITY_MODULE_ORYX_ENABLE == TRUE
#    include <oryx.h>
#endif

ASSERT_COMMUNITY_MODULES_MIN_API_VERSION(1, 1, 0);

extern const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3];

static RGB keycolors_hsv_to_rgb_with_value(HSV hsv) {
    RGB   rgb = hsv_to_rgb(hsv);
    float f   = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
    return (RGB){f * rgb.r, f * rgb.g, f * rgb.b};
}

static void keycolors_set_layer_color(int layer) {
    for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
        HSV hsv = {
            .h = pgm_read_byte(&ledmap[layer][i][0]),
            .s = pgm_read_byte(&ledmap[layer][i][1]),
            .v = pgm_read_byte(&ledmap[layer][i][2]),
        };
        if (!hsv.h && !hsv.s && !hsv.v) {
            rgb_matrix_set_color(i, 0, 0, 0);
        } else {
            RGB rgb = keycolors_hsv_to_rgb_with_value(hsv);
            rgb_matrix_set_color(i, rgb.r, rgb.g, rgb.b);
        }
    }
}

extern const uint32_t startup_fade_in_duration_ms;
extern const uint32_t inactivity_dim_duration_ms;
extern const uint32_t active_again_duration_ms;

extern const uint8_t inactivity_brightness;
extern const uint8_t normal_brightness;

extern const uint32_t inactivity_threshold_ms;

typedef enum {
    // Keyboard is starting up. Play fade-in animation.
    STARTUP_FADE_IN,
    // Normal keyboard operation, brightness is set to normal.
    NORMAL,
    // Keyboard has been left untouched for a while, slowly reduce brightness to save power.
    INACTIVE_DIMMING,
    // Keyboard is still inactive, brightness has been reduced to its target value.
    INACTIVE_DIMMED,
    // Keyboard was inactive but was just touched, increase the brightness again to normal.
    ACTIVE_AGAIN,
} animation_state_t;

static uint8_t           target_brightness;
static uint32_t          current_animation_end_time;
int8_t                   current_animation_slope; // Unit: ms / brightness
static animation_state_t current_state;

static bool keycolors_is_animation_playing(void) {
    return current_state == STARTUP_FADE_IN || current_state == INACTIVE_DIMMING || current_state == ACTIVE_AGAIN;
}

static uint32_t last_keypress_time;
static bool     key_currently_pressed;

static bool keycolors_keyboard_is_inactive(const uint32_t now) {
    if (key_currently_pressed) {
        return false;
    }

    return (now - last_keypress_time > inactivity_threshold_ms);
}

static void keycolors_trigger_animation(const uint32_t now, const uint32_t animation_duration_ms, const uint8_t initial_brightness, const uint8_t final_brightness, animation_state_t new_state) {
    const int32_t expected_brightness_delta = (int32_t)final_brightness - (int32_t)initial_brightness;
    const int32_t actual_brightness_delta   = (int32_t)final_brightness - (int32_t)rgb_matrix_config.hsv.v;

    current_animation_slope    = (int32_t)animation_duration_ms / expected_brightness_delta;
    current_animation_end_time = now + actual_brightness_delta * current_animation_slope;
    current_state              = new_state;
    target_brightness          = final_brightness;
}

static void keycolors_trigger_startup_fade_in(const uint32_t now) {
    keycolors_trigger_animation(now, startup_fade_in_duration_ms, 0, normal_brightness, STARTUP_FADE_IN);
}

static void keycolors_trigger_inactivity_dimming(const uint32_t now) {
    keycolors_trigger_animation(now, inactivity_dim_duration_ms, normal_brightness, inactivity_brightness, INACTIVE_DIMMING);
}

static void keycolors_trigger_active_again(const uint32_t now) {
    keycolors_trigger_animation(now, active_again_duration_ms, inactivity_brightness, normal_brightness, ACTIVE_AGAIN);
}

static void keycolors_mark_animation_finished(const uint32_t now) {
    switch (current_state) {
        case STARTUP_FADE_IN:
            last_keypress_time = now;
            current_state      = NORMAL;
            break;
        case NORMAL:
            break;
        case INACTIVE_DIMMING:
            current_state = INACTIVE_DIMMED;
            break;
        case INACTIVE_DIMMED:
            break;
        case ACTIVE_AGAIN:
            current_state = NORMAL;
            break;
    }
}

static void keycolors_adjust_led_brightness(void) {
    const uint32_t now = timer_read32();

    if (current_state == NORMAL && keycolors_keyboard_is_inactive(now)) {
        keycolors_trigger_inactivity_dimming(now);
    }

    bool currently_playing_animation = keycolors_is_animation_playing();

    if (now > current_animation_end_time) {
        if (!currently_playing_animation) {
            return;
        }

        keycolors_mark_animation_finished(now);
    }

    int32_t computed_brightness = currently_playing_animation ? target_brightness - ((int32_t)(current_animation_end_time - now) / (int32_t)current_animation_slope) : target_brightness;
    uint8_t brightness          = (computed_brightness > RGB_MATRIX_MAXIMUM_BRIGHTNESS) ? RGB_MATRIX_MAXIMUM_BRIGHTNESS : (computed_brightness < 0) ? 0 : computed_brightness;

    hsv_t color = rgb_matrix_get_hsv();
    color.v     = brightness;
    rgb_matrix_sethsv_noeeprom(color.h, color.s, color.v);
}

bool rgb_matrix_indicators_keycolors_kb(void) {
#if COMMUNITY_MODULE_ORYX_ENABLE == TRUE
    if (rawhid_state.rgb_control) {
        return false;
    }
#endif

    keycolors_adjust_led_brightness();

    if (!keyboard_config.disable_layer_led) {
        switch (biton32(layer_state)) {
            case 1:
                keycolors_set_layer_color(1);
                break;
            case 2:
                keycolors_set_layer_color(2);
                break;
            default:
                if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
                    rgb_matrix_set_color_all(0, 0, 0);
                }
        }
    } else {
        if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
            rgb_matrix_set_color_all(0, 0, 0);
        }
    }

    return rgb_matrix_indicators_keycolors_user();
}

void keyboard_post_init_keycolors(void) {
    // Technically not exactly 0 but close enough since this function is called during
    // initialization.
    uint32_t now = 0;
    keycolors_trigger_startup_fade_in(now);

    hsv_t color = rgb_matrix_get_hsv();
    color.v     = 0;
    rgb_matrix_sethsv_noeeprom(color.h, color.s, color.v);
}

bool pre_process_record_keycolors_kb(uint16_t keycode, keyrecord_t *record) {
    const uint32_t now = timer_read32();

    if (keycolors_keyboard_is_inactive(now)) {
        keycolors_trigger_active_again(now);
    }

    last_keypress_time    = now;
    key_currently_pressed = record->event.pressed;

    return pre_process_record_keycolors_user(keycode, record);
}
