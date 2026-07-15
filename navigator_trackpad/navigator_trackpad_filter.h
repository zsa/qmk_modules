// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Velocity-adaptive smoothing for the Navigator trackpad PTP path.
//
// The PTP path forwards absolute sensor coordinates to the host, which derives
// cursor motion from them. The Cirque's raw position carries a few logical
// units of jitter per frame; at low speed that noise is a large fraction of the
// real signal, so straight lines wobble and slow circles drift.
//
// A plain fixed low-pass (IIR) kills that jitter but adds the same lag to *all*
// motion, so fast intentional strokes trail the finger and feel floaty. The One
// Euro Filter (Casiez, Roussel & Vogel, CHI 2012) fixes exactly that: it raises
// its cutoff with speed, so it smooths hard when nearly still and backs off to
// near-passthrough when moving fast (near-zero added lag where lag is felt).
//
// Pure and host-testable — no hardware or QMK dependencies (see tests/).

#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef NT_PI
#    define NT_PI 3.14159265358979f
#endif

// First-order exponential low-pass with externally supplied smoothing factor.
typedef struct {
    float hatx;  // last filtered value
    bool  init;  // false until the first sample seeds hatx
} nt_lowpass_t;

// Seed-on-first-sample so a fresh contact starts at its true position (no
// startup ramp). Subsequent samples blend by alpha in [0, 1] (1 = passthrough).
static inline float nt_lowpass(nt_lowpass_t *lp, float x, float alpha) {
    if (!lp->init) {
        lp->hatx = x;
        lp->init = true;
        return x;
    }
    lp->hatx = alpha * x + (1.0f - alpha) * lp->hatx;
    return lp->hatx;
}

// One Euro state for a single axis. Group two (x, y) per contact.
typedef struct {
    nt_lowpass_t x;      // position low-pass
    nt_lowpass_t dx;     // derivative (speed) low-pass
    float        xprev;  // previous raw input, for the derivative
    bool         init;   // false until the first sample
} nt_euro_axis_t;

typedef struct {
    nt_euro_axis_t x;
    nt_euro_axis_t y;
} nt_euro_point_t;

// Smoothing factor for a given cutoff frequency (Hz) and timestep dt (s).
static inline float nt_euro_alpha(float cutoff, float dt) {
    float tau = 1.0f / (2.0f * NT_PI * cutoff);
    return 1.0f / (1.0f + tau / dt);
}

// Filter one axis sample.
//   mincutoff (Hz): cutoff when the contact is still — lower = smoother/laggier.
//   beta (Hz per unit/s): how fast the cutoff rises with speed — higher = less
//                         lag when moving (passthrough kicks in sooner).
//   dcutoff (Hz): cutoff for the speed estimate itself (1.0 is the usual value).
static inline float nt_euro_axis_filter(nt_euro_axis_t *f, float x, float dt,
                                        float mincutoff, float beta, float dcutoff) {
    float dx;
    if (!f->init) {
        dx      = 0.0f;
        f->init = true;
    } else {
        dx = (x - f->xprev) / dt;
    }
    f->xprev = x;

    float edx    = nt_lowpass(&f->dx, dx, nt_euro_alpha(dcutoff, dt));
    float cutoff = mincutoff + beta * fabsf(edx);
    return nt_lowpass(&f->x, x, nt_euro_alpha(cutoff, dt));
}

// Filter an (x, y) point in place.
static inline void nt_euro_point_filter(nt_euro_point_t *p, float *x, float *y, float dt,
                                        float mincutoff, float beta, float dcutoff) {
    *x = nt_euro_axis_filter(&p->x, *x, dt, mincutoff, beta, dcutoff);
    *y = nt_euro_axis_filter(&p->y, *y, dt, mincutoff, beta, dcutoff);
}

// Forget all history so the next sample seeds fresh (call on a new contact).
static inline void nt_euro_point_reset(nt_euro_point_t *p) {
    p->x = (nt_euro_axis_t){0};
    p->y = (nt_euro_axis_t){0};
}
