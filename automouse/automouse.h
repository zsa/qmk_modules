// Copyright 2025 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifndef AUTOMOUSE_LAYER
#    error "AUTOMOUSE_LAYER must be defined when using the automouse module"
#endif

#ifndef AUTOMOUSE_TIMEOUT
#    define AUTOMOUSE_TIMEOUT 650
#endif

// Idle safety timeout (ms) for ONESHOT "tap to exit" mode. Before the first tap,
// the layer drops after this much inactivity. 0 = disabled (layer stays until tapped).
#ifndef AUTOMOUSE_ONESHOT_TIMEOUT
#    define AUTOMOUSE_ONESHOT_TIMEOUT 0
#endif

#ifndef AUTOMOUSE_THRESHOLD
#    define AUTOMOUSE_THRESHOLD 10
#endif

#ifndef AUTOMOUSE_SCROLL_THRESHOLD
#    define AUTOMOUSE_SCROLL_THRESHOLD AUTOMOUSE_THRESHOLD
#endif

#ifndef AUTOMOUSE_DEBOUNCE
#    define AUTOMOUSE_DEBOUNCE 25
#endif

#ifndef AUTOMOUSE_DELAY
#    define AUTOMOUSE_DELAY GET_TAPPING_TERM(QK_MOUSE_BUTTON_1, &(keyrecord_t){})
#endif

void automouse_enable(void);
void automouse_disable(void);
void automouse_toggle(void);
bool automouse_is_enabled(void);
bool automouse_is_active(void);

// Feed motion from a sensor that bypasses the QMK pointing-device pipeline
// (e.g. a digitizer/trackpad) so it can activate the mouse layer.
void automouse_report_motion(int16_t dx, int16_t dy, uint8_t buttons);
