// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Standalone host test for the One Euro smoothing filter.
// Build & run from the module root:
//   gcc -Wall -lm -o /tmp/nt_filter_test navigator_trackpad/tests/filter_test.c
//   /tmp/nt_filter_test
//
// Verifies the two properties the PTP path relies on:
//   1. Jitter on a near-still signal is strongly attenuated.
//   2. A fast ramp is followed with little lag (no floaty trailing).
// plus the basics (seed-on-first-sample, reset).

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../navigator_trackpad_filter.h"

// Matches the defaults in navigator_trackpad_ptp.c.
#define MINCUTOFF 1.5f
#define BETA 0.0070f
#define DCUTOFF 15.0f
#define DT 0.008f  // ~125 Hz, the observed sensor cadence

// Deterministic pseudo-noise in [-amp, amp], no libc rand() dependency.
static float pseudo_noise(int i, float amp) {
    float s = sinf((float)i * 12.9898f) * 43758.5453f;
    s -= floorf(s);  // fractional part in [0,1)
    return (s * 2.0f - 1.0f) * amp;
}

// 1. A stationary finger with +/-8 unit sensor jitter must come out far calmer.
static void test_jitter_attenuation(void) {
    nt_euro_point_t f = {0};
    const float center = 1000.0f;
    float in_dev = 0.0f, out_dev = 0.0f;
    int   n = 0;

    for (int i = 0; i < 400; i++) {
        float x = center + pseudo_noise(i, 8.0f);
        float y = center + pseudo_noise(i + 777, 8.0f);
        float fx = x, fy = y;
        nt_euro_point_filter(&f, &fx, &fy, DT, MINCUTOFF, BETA, DCUTOFF);
        if (i > 50) {  // let it settle
            in_dev += fabsf(x - center);
            out_dev += fabsf(fx - center);
            n++;
        }
    }
    in_dev /= n;
    out_dev /= n;
    printf("  jitter: in mean|dev|=%.2f  out mean|dev|=%.2f  (%.0f%% reduction)\n",
           in_dev, out_dev, 100.0f * (1.0f - out_dev / in_dev));
    assert(out_dev < in_dev * 0.4f && "filter should cut still-finger jitter by >60%");
}

// 2. A fast linear ramp (150 units/frame ~= 18750 units/s) must be tracked with
//    little lag, i.e. the filtered output stays close to the live input.
static void test_fast_ramp_low_lag(void) {
    nt_euro_axis_t f = {0};
    float          x = 0.0f, fx = 0.0f;
    float          max_lag = 0.0f;

    for (int i = 0; i < 12; i++) {
        x += 150.0f;
        fx = nt_euro_axis_filter(&f, x, DT, MINCUTOFF, BETA, DCUTOFF);
        float lag = x - fx;  // how far behind the live position we are (units)
        if (i > 2 && lag > max_lag) max_lag = lag;
    }
    // One frame of motion is 150 units; keep steady-state lag well under that so
    // fast strokes never feel floaty. (< ~0.6 frame.)
    printf("  fast ramp: max lag=%.1f units (%.2f frame)\n", max_lag, max_lag / 150.0f);
    assert(max_lag < 90.0f && "fast motion must not lag more than ~0.6 frame");
}

// 3. First sample passes through unchanged; reset forgets history.
static void test_seed_and_reset(void) {
    nt_euro_point_t f = {0};
    float           x = 500.0f, y = 500.0f;
    nt_euro_point_filter(&f, &x, &y, DT, MINCUTOFF, BETA, DCUTOFF);
    assert(x == 500.0f && y == 500.0f && "first sample must seed exactly");

    nt_euro_point_reset(&f);
    x = 1500.0f;
    y = 200.0f;
    nt_euro_point_filter(&f, &x, &y, DT, MINCUTOFF, BETA, DCUTOFF);
    assert(x == 1500.0f && y == 200.0f && "reset then first sample must seed exactly");
}

int main(void) {
    test_seed_and_reset();
    test_jitter_attenuation();
    test_fast_ramp_low_lag();
    printf("All filter tests passed\n");
    return 0;
}
