// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Standalone host test for zsa_charge_stability.h.
// Build & run from the module root:
//   gcc -Wall -Wextra -o /tmp/charge_stability_test wireless/tests/charge_stability_test.c
//   /tmp/charge_stability_test
//
// Drives the detector with 1 s samples of the right PMIC's VBUSIN PRESENT bit
// and checks the trip points, the step-down sequence and the resets.

#include <stdio.h>
#include "../zsa_charge_stability.h"

static int checks;
static int failures;

#define CHECK(cond, ...)                                       \
    do {                                                       \
        checks++;                                              \
        if (!(cond)) {                                         \
            failures++;                                        \
            printf("FAIL %s:%d: %s: ", __FILE__, __LINE__, #cond); \
            printf(__VA_ARGS__);                               \
            printf("\n");                                      \
        }                                                      \
    } while (0)

#define SAMPLE_MS 1000u

// Present except for one absent sample every `period_s` seconds. Feeds
// `seconds` samples from *t and returns how many reported a budget change.
static int run_pattern(struct zsa_charge_stability *s, uint32_t *t, uint32_t seconds, uint32_t period_s) {
    int changes = 0;
    for (uint32_t i = 0; i < seconds; i++) {
        uint32_t sec     = *t / SAMPLE_MS;
        bool     present = period_s == 0 || (sec % period_s) != period_s - 1;
        if (zsa_charge_stability_sample(s, *t, present)) {
            changes++;
        }
        *t += SAMPLE_MS;
    }
    return changes;
}

// Seconds of the field pattern until the effective budget changes, or -1.
static int seconds_to_change(struct zsa_charge_stability *s, uint32_t *t, uint32_t limit_s) {
    for (uint32_t i = 0; i < limit_s; i++) {
        uint32_t sec = *t / SAMPLE_MS;
        bool     chg = zsa_charge_stability_sample(s, *t, (sec % 5u) != 4u);
        *t += SAMPLE_MS;
        if (chg) {
            return (int)i + 1;
        }
    }
    return -1;
}

static void test_limits_table(void) {
    struct zsa_charge_limits l = zsa_charge_limits_for_budget(1500);
    CHECK(l.vbus_limit_ma == 700 && l.charge_ma == 550, "1500 -> %u/%u", l.vbus_limit_ma, l.charge_ma);
    l = zsa_charge_limits_for_budget(1000);
    CHECK(l.vbus_limit_ma == 500 && l.charge_ma == 350, "1000 -> %u/%u", l.vbus_limit_ma, l.charge_ma);
    l = zsa_charge_limits_for_budget(500);
    CHECK(l.vbus_limit_ma == 300 && l.charge_ma == 150, "500 -> %u/%u", l.vbus_limit_ma, l.charge_ma);

    CHECK(zsa_charge_stability_budget_for(1500, 0) == 1500, "0 steps");
    CHECK(zsa_charge_stability_budget_for(1500, 1) == 1000, "1 step");
    CHECK(zsa_charge_stability_budget_for(1500, 2) == 500, "2 steps");
    CHECK(zsa_charge_stability_budget_for(1500, 9) == 500, "floor");
    CHECK(zsa_charge_stability_budget_for(700, 1) == 500, "partial step clamps to floor");
    CHECK(zsa_charge_stability_budget_for(300, 3) == 300, "below floor untouched");
}

static void test_healthy(void) {
    struct zsa_charge_stability s;
    uint32_t                    t = 0;
    zsa_charge_stability_init(&s, 1500);
    int changes = run_pattern(&s, &t, 3600, 0);
    CHECK(changes == 0, "healthy changed %d times", changes);
    CHECK(s.state == ZSA_CHARGE_STABLE && s.steps == 0, "healthy state %u steps %u", s.state, s.steps);
    CHECK(zsa_charge_stability_budget(&s) == 1500, "healthy budget %u", zsa_charge_stability_budget(&s));
}

static void test_field_pattern(void) {
    struct zsa_charge_stability s;
    uint32_t                    t = 0;
    zsa_charge_stability_init(&s, 1500);

    int secs = seconds_to_change(&s, &t, 60);
    CHECK(secs > 0 && secs <= 60, "first trip after %d s", secs);
    CHECK(s.state == ZSA_CHARGE_REDUCED && zsa_charge_stability_budget(&s) == 1000, "after first trip state %u budget %u", s.state, zsa_charge_stability_budget(&s));
    printf("  field pattern: 1500 -> 1000 after %d s\n", secs);

    secs = seconds_to_change(&s, &t, 60);
    CHECK(secs > 0 && secs <= 60, "second trip after %d s", secs);
    CHECK(s.state == ZSA_CHARGE_REDUCED && zsa_charge_stability_budget(&s) == 500, "after second trip state %u budget %u", s.state, zsa_charge_stability_budget(&s));
    printf("  field pattern: 1000 -> 500 after %d more s\n", secs);

    secs = seconds_to_change(&s, &t, 60);
    CHECK(secs == -1, "no further budget change, got %d", secs);
    CHECK(s.state == ZSA_CHARGE_AT_FLOOR && zsa_charge_stability_budget(&s) == 500, "at floor state %u budget %u", s.state, zsa_charge_stability_budget(&s));

    // Sticky: a healthy hour afterwards changes nothing.
    run_pattern(&s, &t, 3600, 0);
    CHECK(s.state == ZSA_CHARGE_AT_FLOOR && s.steps == 2, "sticky state %u steps %u", s.state, s.steps);
}

static void test_three_per_window(void) {
    struct zsa_charge_stability s;
    uint32_t                    t = 0;
    zsa_charge_stability_init(&s, 1500);
    // One loss every 20 s = 3 per 60 s.
    int changes = run_pattern(&s, &t, 3600, 20);
    CHECK(changes == 0 && s.state == ZSA_CHARGE_STABLE, "3/window changed %d state %u", changes, s.state);

    // Exactly 3 losses back to back in one window.
    zsa_charge_stability_init(&s, 1500);
    t = 0;
    for (int i = 0; i < 3; i++) {
        zsa_charge_stability_sample(&s, t, true);
        t += SAMPLE_MS;
        zsa_charge_stability_sample(&s, t, false);
        t += SAMPLE_MS;
    }
    CHECK(s.losses == 3 && s.state == ZSA_CHARGE_STABLE, "3 burst losses %u state %u", s.losses, s.state);
    zsa_charge_stability_sample(&s, t, true);
    t += SAMPLE_MS;
    CHECK(zsa_charge_stability_sample(&s, t, false), "4th loss trips");
}

// Present, gap, absent: the transition across the gap is not a loss.
static void test_chain_breaks(void) {
    struct zsa_charge_stability s;
    uint32_t                    t = 0;
    zsa_charge_stability_init(&s, 1500);

    // Three losses, then a TRRS unplug / failed read clears the window.
    for (int i = 0; i < 3; i++) {
        zsa_charge_stability_sample(&s, t, true);
        t += SAMPLE_MS;
        zsa_charge_stability_sample(&s, t, false);
        t += SAMPLE_MS;
    }
    zsa_charge_stability_sample(&s, t, true);
    t += SAMPLE_MS;
    CHECK(s.losses == 3, "losses before break %u", s.losses);
    zsa_charge_stability_break(&s);
    CHECK(s.losses == 0 && !s.window_open && !s.have_prev, "break cleared window");

    // The previous sample was present; this absent one follows the break.
    t += SAMPLE_MS;
    CHECK(!zsa_charge_stability_sample(&s, t, false), "absent after break: no change");
    CHECK(s.losses == 0, "absent after break counted %u", s.losses);

    // Three more losses: still only 3 in the new window.
    for (int i = 0; i < 3; i++) {
        t += SAMPLE_MS;
        zsa_charge_stability_sample(&s, t, true);
        t += SAMPLE_MS;
        zsa_charge_stability_sample(&s, t, false);
    }
    CHECK(s.losses == 3 && s.state == ZSA_CHARGE_STABLE, "post-break losses %u state %u", s.losses, s.state);

    // Break keeps steps and state.
    zsa_charge_stability_init(&s, 1500);
    t = 0;
    CHECK(seconds_to_change(&s, &t, 60) > 0, "trip before break");
    zsa_charge_stability_break(&s);
    CHECK(s.steps == 1 && s.state == ZSA_CHARGE_REDUCED && zsa_charge_stability_budget(&s) == 1000, "break kept steps %u state %u", s.steps, s.state);
}

static void test_vbus_off_reset(void) {
    struct zsa_charge_stability s;
    uint32_t                    t = 0;
    zsa_charge_stability_init(&s, 1500);
    seconds_to_change(&s, &t, 60);
    seconds_to_change(&s, &t, 60);
    seconds_to_change(&s, &t, 60);
    CHECK(s.state == ZSA_CHARGE_AT_FLOOR, "setup at floor %u", s.state);

    zsa_charge_stability_reset(&s);
    CHECK(s.steps == 0 && s.state == ZSA_CHARGE_STABLE && s.losses == 0 && !s.have_prev && !s.window_open, "reset cleared");
    CHECK(zsa_charge_stability_budget(&s) == 1500, "reset budget %u", zsa_charge_stability_budget(&s));
}

static void test_budget_change_reset(void) {
    struct zsa_charge_stability s;
    uint32_t                    t = 0;
    zsa_charge_stability_init(&s, 1500);
    seconds_to_change(&s, &t, 60);
    CHECK(s.steps == 1, "setup steps %u", s.steps);

    CHECK(!zsa_charge_stability_source(&s, 1500), "same budget does not reset");
    CHECK(s.steps == 1 && s.state == ZSA_CHARGE_REDUCED, "same budget kept steps %u", s.steps);

    CHECK(zsa_charge_stability_source(&s, 500), "new budget resets");
    CHECK(s.steps == 0 && s.state == ZSA_CHARGE_STABLE && zsa_charge_stability_budget(&s) == 500, "new budget state %u budget %u", s.state, zsa_charge_stability_budget(&s));

    CHECK(zsa_charge_stability_source(&s, 1500), "back to high resets");
    CHECK(zsa_charge_stability_budget(&s) == 1500, "back to high budget %u", zsa_charge_stability_budget(&s));
}

static void test_default_source(void) {
    struct zsa_charge_stability s;
    uint32_t                    t = 0;
    zsa_charge_stability_init(&s, ZSA_PWR_BUDGET_DEFAULT_MA);
    int secs = seconds_to_change(&s, &t, 120);
    CHECK(secs == -1, "default source budget changed after %d s", secs);
    CHECK(s.state == ZSA_CHARGE_AT_FLOOR && s.steps == 0, "default source state %u steps %u", s.state, s.steps);
    CHECK(zsa_charge_stability_budget(&s) == ZSA_PWR_BUDGET_DEFAULT_MA, "default source budget %u", zsa_charge_stability_budget(&s));
}

int main(void) {
    test_limits_table();
    test_healthy();
    test_field_pattern();
    test_three_per_window();
    test_chain_breaks();
    test_vbus_off_reset();
    test_budget_change_reset();
    test_default_source();

    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
