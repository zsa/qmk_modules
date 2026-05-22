// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Pure, self-contained coordinate-rotation helpers for the Navigator trackpad.
// No QMK dependencies (only <stdint.h>/<math.h>) so they are unit-testable on
// the host. Angles are clockwise; the sign convention matches the trackball.
//
// cos/sin are passed in (not read from config) so production code can supply
// compile-time constants while the functions stay pure and testable.

#pragma once

#include <math.h>
#include <stdint.h>

// Rotate absolute point (x,y) about center (cx,cy) by an arbitrary angle whose
// cosine/sine are supplied, then clamp the result to [0, max].
static inline void nt_rotate_point(uint16_t x, uint16_t y, uint16_t cx, uint16_t cy,
                                   float cos_t, float sin_t, uint16_t max,
                                   uint16_t *ox, uint16_t *oy) {
    float   ddx = (float)x - (float)cx;
    float   ddy = (float)y - (float)cy;
    int32_t rx  = (int32_t)lroundf((float)cx + ddx * cos_t - ddy * sin_t);
    int32_t ry  = (int32_t)lroundf((float)cy + ddx * sin_t + ddy * cos_t);
    if (rx < 0) rx = 0;
    if (rx > (int32_t)max) rx = max;
    if (ry < 0) ry = 0;
    if (ry > (int32_t)max) ry = max;
    *ox = (uint16_t)rx;
    *oy = (uint16_t)ry;
}

// Rotate relative delta (dx,dy) about the origin by an arbitrary angle.
static inline void nt_rotate_delta(int16_t dx, int16_t dy, float cos_t, float sin_t,
                                   int16_t *odx, int16_t *ody) {
    *odx = (int16_t)lroundf((float)dx * cos_t - (float)dy * sin_t);
    *ody = (int16_t)lroundf((float)dx * sin_t + (float)dy * cos_t);
}

// Integer fast-path (no float) for right-angle point rotation.
// quadrant: 1 = 90 deg, 2 = 180 deg, 3 = 270 deg (clockwise).
static inline void nt_rotate_point_ortho(uint16_t x, uint16_t y, uint16_t cx, uint16_t cy,
                                         uint8_t quadrant, uint16_t max,
                                         uint16_t *ox, uint16_t *oy) {
    int32_t ddx = (int32_t)x - (int32_t)cx;
    int32_t ddy = (int32_t)y - (int32_t)cy;
    int32_t rx, ry;
    switch (quadrant) {
        case 1:  rx = (int32_t)cx - ddy; ry = (int32_t)cy + ddx; break; // 90
        case 2:  rx = (int32_t)cx - ddx; ry = (int32_t)cy - ddy; break; // 180
        default: rx = (int32_t)cx + ddy; ry = (int32_t)cy - ddx; break; // 270
    }
    if (rx < 0) rx = 0;
    if (rx > (int32_t)max) rx = max;
    if (ry < 0) ry = 0;
    if (ry > (int32_t)max) ry = max;
    *ox = (uint16_t)rx;
    *oy = (uint16_t)ry;
}

// Integer fast-path (no float) for right-angle delta rotation.
static inline void nt_rotate_delta_ortho(int16_t dx, int16_t dy, uint8_t quadrant,
                                         int16_t *odx, int16_t *ody) {
    switch (quadrant) {
        case 1:  *odx = -dy; *ody =  dx; break; // 90
        case 2:  *odx = -dx; *ody = -dy; break; // 180
        default: *odx =  dy; *ody = -dx; break; // 270
    }
}
