// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Geometric linearization LUT for the Navigator trackpad.
//
// The Cirque Gen6 reports a position that carries a fixed, position-locked
// spatial distortion: axis-aligned moves stay straight, but diagonals bow in a
// ~1 mm peak ripple (confirmed speed-invariant and phase-locked to absolute pad
// position, so it is a static warp of the coordinate field — not jitter, lag,
// or sampling cadence). This is a coarse 2D displacement field E(x,y); we
// subtract it from each absolute contact to straighten the reported path.
//
// The field was calibrated by recording many straightedge / freehand strokes
// (libinput record, raw ABS_MT positions with smoothing disabled), fitting a
// bilinear correction grid that minimizes per-stroke curvature, and bounding
// every node to +/-40 logical units (~1 mm) so sparsely-sampled edge cells can
// never produce an unphysical warp. Values are in logical units
// (0..TRACKPAD_LOGICAL_MAX). Index order is [x_node][y_node].
//
// NOTE: calibrated against one physical unit. If the distortion proves to vary
// between sensors, this table is unit-specific and must not ship as-is.

#pragma once

#include <math.h>
#include <stdint.h>
#include "navigator_trackpad_common.h"  // TRACKPAD_LOGICAL_MAX

#ifndef NAVIGATOR_TRACKPAD_LUT_CORRECTION
#    define NAVIGATOR_TRACKPAD_LUT_CORRECTION TRUE
#endif

#define NT_LUT_G 9

static const int8_t NT_LUT_EX[NT_LUT_G][NT_LUT_G] = {
    {   1,  -10,  -25,  -40,  -40,  -32,  -13,    9,   20},
    {   0,   -8,  -18,  -26,  -37,  -40,  -38,   11,   20},
    {   1,   -3,  -10,  -17,  -31,  -40,  -38,   -9,   -3},
    {  12,    7,    0,   -9,  -15,  -15,  -16,  -15,  -36},
    {  17,   28,   15,    5,   10,   14,    7,   -8,  -40},
    {  -2,   15,   32,   31,   31,   27,   19,   11,    1},
    { -29,    0,   40,   40,   30,   18,   20,   16,   10},
    { -32,    0,   40,   38,   19,    4,   15,   11,    7},
    { -12,    7,   23,   12,  -18,  -40,  -16,   -1,    5},
};

static const int8_t NT_LUT_EY[NT_LUT_G][NT_LUT_G] = {
    {   0,   16,   36,   40,   27,   -2,  -40,  -20,    5},
    {  -6,   -4,   -4,   -1,    8,    5,   -1,   20,   10},
    { -17,  -25,  -30,  -31,   -2,   25,   31,   31,    2},
    { -32,  -39,  -40,  -40,   -8,   40,   40,   36,    1},
    { -27,  -39,  -40,  -40,  -13,   39,   40,   40,   25},
    { -17,  -34,  -37,  -40,  -15,   36,   40,   40,   33},
    {  -4,  -19,  -29,  -40,  -17,   34,   40,   40,   24},
    {  -1,   -9,  -10,  -34,  -20,   19,    6,   11,    8},
    {   3,    3,   -4,  -31,  -26,   40,  -29,  -26,   -8},
};

// Subtract the bilinearly-interpolated distortion field from an absolute
// logical point (x, y), straightening the reported coordinate in place.
static inline void nt_lut_correct(uint16_t *x, uint16_t *y) {
    const float scale = (float)(NT_LUT_G - 1) / (float)TRACKPAD_LOGICAL_MAX;
    float fx = (float)(*x) * scale;
    float fy = (float)(*y) * scale;
    if (fx < 0.0f) fx = 0.0f;
    if (fx > NT_LUT_G - 1.001f) fx = NT_LUT_G - 1.001f;
    if (fy < 0.0f) fy = 0.0f;
    if (fy > NT_LUT_G - 1.001f) fy = NT_LUT_G - 1.001f;
    int   ix = (int)fx, iy = (int)fy;
    float tx = fx - ix, ty = fy - iy;

#define NT_LUT_BILINEAR(M)                                            \
    ((float)(M)[ix][iy] * (1.0f - tx) * (1.0f - ty) +                 \
     (float)(M)[ix + 1][iy] * tx * (1.0f - ty) +                      \
     (float)(M)[ix][iy + 1] * (1.0f - tx) * ty +                      \
     (float)(M)[ix + 1][iy + 1] * tx * ty)

    float ex = NT_LUT_BILINEAR(NT_LUT_EX);
    float ey = NT_LUT_BILINEAR(NT_LUT_EY);
#undef NT_LUT_BILINEAR

    int32_t nx = (int32_t)lroundf((float)(*x) - ex);
    int32_t ny = (int32_t)lroundf((float)(*y) - ey);
    if (nx < 0) nx = 0;
    if (nx > TRACKPAD_LOGICAL_MAX) nx = TRACKPAD_LOGICAL_MAX;
    if (ny < 0) ny = 0;
    if (ny > TRACKPAD_LOGICAL_MAX) ny = TRACKPAD_LOGICAL_MAX;
    *x = (uint16_t)nx;
    *y = (uint16_t)ny;
}
