// Copyright 2026 David Scarpetti
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Layer Dismiss: an assignable key that turns off the layer that delivered
// it, on release, and produces no output of its own. Where TG() both raises
// and lowers one specific layer, KC_LAYER_DISMISS is just the lowering half,
// aimed at whichever layer resolved the keypress, so the same binding works
// on any layer it is placed on. It pairs conceptually with Layer Lock, which
// holds a layer on. This lets one go. On the base layer the key is a
// deliberate no-op: the root cannot be dismissed.
//
// Bound across the unused positions of a transiently-raised layer (for
// example an automouse layer), it turns every foreign tap into a consumed
// "wake" gesture: the layer drops, nothing types, and the next keystroke
// lands on the layout below. Dropping on release rather than press means a
// rolled-over second keystroke still resolves through the layer's
// transparency to the layout below, so fast typing after a wake tap loses
// nothing.
//
// Composed with automouse, dropping on release also yields a hold behavior
// for free: while the key is held, automouse counts it as layer activity
// and keeps refreshing its timer, so the layer stays open through any
// pause, and release then drops it instantly. Tap dismisses, hold
// sustains, release exits. The sustain depends on module order: automouse
// must be listed before layer_dismiss in the keymap's modules list, so
// that automouse sees the held key's press before this module consumes it.
// Listed the other way, tap-to-dismiss still works but a hold no longer
// counts as activity.

#include QMK_KEYBOARD_H

ASSERT_COMMUNITY_MODULES_MIN_API_VERSION(1, 0, 0);

bool process_record_layer_dismiss(uint16_t keycode, keyrecord_t *record) {
    if (keycode == KC_LAYER_DISMISS) {
        if (!record->event.pressed) {
            uint8_t layer = layer_switch_get_layer(record->event.key);
            if (layer > 0) {
                layer_off(layer);
            }
        }
        return false;
    }
    return true;
}
