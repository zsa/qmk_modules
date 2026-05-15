// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#include "quantum.h"
#include "navigator_trackpad_core.h"

// CPI ticks (lifted from navigator_trackpad_common.c).
#define CPI_TICKS 7
#define DEFAULT_CPI_TICK 4
#define CPI_1 200
#define CPI_2 400
#define CPI_3 800
#define CPI_4 1024
#define CPI_5 1400
#define CPI_6 1800
#define CPI_7 2048

static uint16_t current_cpi = DEFAULT_CPI_TICK;

static void step_cpi(void) {
    switch (current_cpi) {
        case CPI_1: current_cpi = CPI_2; break;
        case CPI_2: current_cpi = CPI_3; break;
        case CPI_3: current_cpi = CPI_4; break;
        case CPI_4: current_cpi = CPI_5; break;
        case CPI_5: current_cpi = CPI_6; break;
        case CPI_6: current_cpi = CPI_7; break;
        case CPI_7: current_cpi = CPI_1; break;
        default:    current_cpi = CPI_4; break;
    }
}

void navigator_trackpad_core_set_cpi(uint16_t increment_or_zero) {
    if (increment_or_zero == 0) {
        if (current_cpi > 1) current_cpi--;
    } else {
        if (current_cpi < CPI_TICKS) current_cpi++;
    }
    step_cpi();
}

uint16_t navigator_trackpad_core_get_cpi(void) { return current_cpi; }

bool process_record_navigator_trackpad_core(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case TRACKPAD_INC_CPI:
            if (record->event.pressed) navigator_trackpad_core_set_cpi(1);
            return false;
        case TRACKPAD_DEC_CPI:
            if (record->event.pressed) navigator_trackpad_core_set_cpi(0);
            return false;
    }
    return true;
}
