// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once
/* Right-half VBUS stability while wired, and the charge budget step-down it
 * drives. No QMK dependencies, so wireless/tests/charge_stability_test.c
 * builds it on the host. Times are timer_read32() milliseconds.
 *
 * A loss is the right PMIC's VBUSIN PRESENT bit going from set to clear
 * between two consecutive usable samples. Losses are counted in tumbling
 * windows of ZSA_CHARGE_UNSTABLE_WINDOW_MS that open at the first sample of a
 * chain. ZSA_CHARGE_UNSTABLE_LOSSES in one window is a trip: the budget steps
 * down by ZSA_CHARGE_UNSTABLE_STEP_MA (never below ZSA_PWR_BUDGET_DEFAULT_MA)
 * and a fresh window opens. A trip with no step left reports AT_FLOOR. The
 * reported state only rises until zsa_charge_stability_reset() or a new
 * source budget. */

#include <stdbool.h>
#include <stdint.h>
#include "zsa_uart_proto.h"

#ifndef ZSA_CHARGE_UNSTABLE_LOSSES
#    define ZSA_CHARGE_UNSTABLE_LOSSES 4
#endif
#ifndef ZSA_CHARGE_UNSTABLE_WINDOW_MS
#    define ZSA_CHARGE_UNSTABLE_WINDOW_MS 60000
#endif
#ifndef ZSA_CHARGE_UNSTABLE_STEP_MA
#    define ZSA_CHARGE_UNSTABLE_STEP_MA 500
#endif

enum zsa_charge_stability_state {
    ZSA_CHARGE_STABLE   = 0, /* nothing detected */
    ZSA_CHARGE_REDUCED  = 1, /* unstable, budget stepped down */
    ZSA_CHARGE_AT_FLOOR = 2, /* unstable with no step left */
};

struct zsa_charge_stability {
    uint16_t budget_ma;       /* source budget the steps apply to */
    uint8_t  steps;           /* step-downs taken */
    uint8_t  state;           /* enum zsa_charge_stability_state */
    uint8_t  losses;          /* losses in the current window */
    bool     window_open;     /* window_start_ms is valid */
    bool     have_prev;       /* prev_present holds the previous sample */
    bool     prev_present;    /* previous sample's PRESENT bit */
    uint32_t window_start_ms;
};

/* Effective budget for `steps` step-downs from `budget_ma`. */
static inline uint16_t zsa_charge_stability_budget_for(uint16_t budget_ma, uint8_t steps) {
    uint32_t cut = (uint32_t)steps * ZSA_CHARGE_UNSTABLE_STEP_MA;
    if (budget_ma <= ZSA_PWR_BUDGET_DEFAULT_MA) {
        return budget_ma;
    }
    if ((uint32_t)budget_ma - ZSA_PWR_BUDGET_DEFAULT_MA <= cut) {
        return ZSA_PWR_BUDGET_DEFAULT_MA;
    }
    return (uint16_t)(budget_ma - cut);
}

/* Budget to derive the charge limits from. */
static inline uint16_t zsa_charge_stability_budget(const struct zsa_charge_stability *s) {
    return zsa_charge_stability_budget_for(s->budget_ma, s->steps);
}

/* Drops the sample chain and the window; steps and state are kept. */
static inline void zsa_charge_stability_break(struct zsa_charge_stability *s) {
    s->have_prev   = false;
    s->window_open = false;
    s->losses      = 0;
}

/* Clears steps, state, window and sample chain; keeps the source budget. */
static inline void zsa_charge_stability_reset(struct zsa_charge_stability *s) {
    zsa_charge_stability_break(s);
    s->steps = 0;
    s->state = ZSA_CHARGE_STABLE;
}

static inline void zsa_charge_stability_init(struct zsa_charge_stability *s, uint16_t budget_ma) {
    zsa_charge_stability_reset(s);
    s->budget_ma       = budget_ma;
    s->window_start_ms = 0;
}

/* The source reported `budget_ma`. A different budget resets everything.
 * Returns true when it reset. */
static inline bool zsa_charge_stability_source(struct zsa_charge_stability *s, uint16_t budget_ma) {
    if (budget_ma == s->budget_ma) {
        return false;
    }
    zsa_charge_stability_reset(s);
    s->budget_ma = budget_ma;
    return true;
}

/* One usable sample of the right PMIC's VBUSIN PRESENT bit. Returns true when
 * the effective budget changed. */
static inline bool zsa_charge_stability_sample(struct zsa_charge_stability *s, uint32_t now_ms, bool present) {
    bool loss = s->have_prev && s->prev_present && !present;

    s->have_prev    = true;
    s->prev_present = present;

    if (!s->window_open || (uint32_t)(now_ms - s->window_start_ms) >= ZSA_CHARGE_UNSTABLE_WINDOW_MS) {
        s->window_open     = true;
        s->window_start_ms = now_ms;
        s->losses          = 0;
    }
    if (!loss) {
        return false;
    }
    if (++s->losses < ZSA_CHARGE_UNSTABLE_LOSSES) {
        return false;
    }

    s->window_start_ms = now_ms;
    s->losses          = 0;

    uint16_t before = zsa_charge_stability_budget(s);
    uint16_t after  = zsa_charge_stability_budget_for(s->budget_ma, (uint8_t)(s->steps + 1u));
    if (after < before) {
        s->steps++;
        if (s->state < ZSA_CHARGE_REDUCED) {
            s->state = ZSA_CHARGE_REDUCED;
        }
        return true;
    }
    s->state = ZSA_CHARGE_AT_FLOOR;
    return false;
}
