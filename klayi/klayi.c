// Copyright 2025 Nomisolutions
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H
#include "klayi.h"
#include "raw_hid.h"

ASSERT_COMMUNITY_MODULES_MIN_API_VERSION(1, 1, 1);

#ifndef RAW_EPSIZE
#    define RAW_EPSIZE 32
#endif

static uint8_t current_layer = 0;

static void send_layer_event(uint8_t layer) {
    uint8_t buffer[RAW_EPSIZE] = {0};
    buffer[0]                  = ACTIVE_LAYER_INDICATOR_MESSAGE;
    buffer[1]                  = ACTIVE_LAYER_PUSH;
    buffer[2]                  = layer;
    raw_hid_send(buffer, RAW_EPSIZE);
}

layer_state_t layer_state_set_klayi(layer_state_t state) {
    state = layer_state_set_klayi_kb(state);

    uint8_t layer = get_highest_layer(state | default_layer_state);
    if (current_layer != layer) {
        current_layer = layer;
        send_layer_event(layer);
    }

    return state;
}

layer_state_t default_layer_state_set_klayi(layer_state_t state) {
    layer_state_set_klayi(state | layer_state);
    return default_layer_state_set_klayi_kb(state);
}
