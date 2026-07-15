// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Standalone host test for the pure trackpad rotation helpers.
// Build & run from the module root:
//   gcc -Wall -o /tmp/nt_rotation_test navigator_trackpad/tests/rotation_test.c -lm
//   /tmp/nt_rotation_test

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "../navigator_trackpad_rotation.h"

#define DEG2RAD(d) ((d) * 3.14159265358979f / 180.0f)

// 0 degrees is the identity for both point and delta rotation.
static void test_identity(void) {
    uint16_t ox, oy;
    nt_rotate_point(1500, 600, 1024, 1024, cosf(0), sinf(0), 2048, &ox, &oy);
    assert(ox == 1500 && oy == 600);

    int16_t odx, ody;
    nt_rotate_delta(40, -25, cosf(0), sinf(0), &odx, &ody);
    assert(odx == 40 && ody == -25);
}

// Integer right-angle path must equal the float path (after rounding).
static void test_point_ortho_matches_float(void) {
    const int     angles[3] = {90, 180, 270};
    const uint8_t quad[3]   = {1, 2, 3};
    uint16_t pts[][2] = {{1024, 1024}, {1500, 1024}, {1024, 400}, {1700, 1700}, {300, 1500}};
    for (int a = 0; a < 3; a++) {
        for (int p = 0; p < 5; p++) {
            uint16_t fx, fy, ix, iy;
            nt_rotate_point(pts[p][0], pts[p][1], 1024, 1024,
                            cosf(DEG2RAD(angles[a])), sinf(DEG2RAD(angles[a])), 2048, &fx, &fy);
            nt_rotate_point_ortho(pts[p][0], pts[p][1], 1024, 1024, quad[a], 2048, &ix, &iy);
            assert(fx == ix && fy == iy);
        }
    }
}

// Right-angle delta values match the trackball sign convention (clockwise).
static void test_delta_ortho_values(void) {
    int16_t odx, ody;
    nt_rotate_delta_ortho(40, -25, 1, &odx, &ody); assert(odx == 25 && ody == 40);    // 90:  (-dy, dx)
    nt_rotate_delta_ortho(40, -25, 2, &odx, &ody); assert(odx == -40 && ody == 25);   // 180: (-dx,-dy)
    nt_rotate_delta_ortho(40, -25, 3, &odx, &ody); assert(odx == -25 && ody == -40);  // 270: ( dy,-dx)

    int16_t fdx, fdy;
    nt_rotate_delta(40, -25, cosf(DEG2RAD(90)), sinf(DEG2RAD(90)), &fdx, &fdy);
    assert(fdx == 25 && fdy == 40);
}

// Arbitrary angle is a rigid transform: distance/magnitude is preserved (within rounding).
static void test_arbitrary_rigidity(void) {
    uint16_t ox, oy;
    nt_rotate_point(1124, 1024, 1024, 1024, cosf(DEG2RAD(45)), sinf(DEG2RAD(45)), 2048, &ox, &oy);
    float dx = (float)ox - 1024.0f, dy = (float)oy - 1024.0f;
    float dist = sqrtf(dx * dx + dy * dy);
    assert(dist > 99.0f && dist < 101.0f);

    int16_t odx, ody;
    nt_rotate_delta(100, 0, cosf(DEG2RAD(45)), sinf(DEG2RAD(45)), &odx, &ody);
    float mag = sqrtf((float)odx * odx + (float)ody * ody);
    assert(mag > 99.0f && mag < 101.0f);
}

// Out-of-range results clamp to [0, max]; in-range results are untouched.
static void test_clamp(void) {
    uint16_t ox, oy;
    nt_rotate_point(0, 0, 1024, 1024, cosf(DEG2RAD(180)), sinf(DEG2RAD(180)), 2048, &ox, &oy);
    assert(ox == 2048 && oy == 2048);                 // (0,0) -> (2048,2048), at bound

    nt_rotate_point(0, 0, 100, 100, cosf(DEG2RAD(180)), sinf(DEG2RAD(180)), 2048, &ox, &oy);
    assert(ox == 200 && oy == 200);                   // 180 about (100,100): (0,0)->(200,200)

    nt_rotate_point(2000, 2000, 100, 100, cosf(DEG2RAD(180)), sinf(DEG2RAD(180)), 2048, &ox, &oy);
    assert(ox == 0 && oy == 0);                        // (2000,2000)->(-1800,-1800) clamped to 0
}

int main(void) {
    test_identity();
    test_point_ortho_matches_float();
    test_delta_ortho_values();
    test_arbitrary_rigidity();
    test_clamp();
    printf("All rotation tests passed\n");
    return 0;
}
