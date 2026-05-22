# Navigator Trackpad Rotation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `NAVIGATOR_TRACKPAD_ROTATION` config parameter that rotates the trackpad's reported orientation by an arbitrary clockwise angle, working in both PTP (absolute) and mouse-fallback (relative) modes.

**Architecture:** The rotation math lives in a pure, dependency-free header (`navigator_trackpad_rotation.h`) so it is unit-testable on the host with plain `gcc`. `navigator_trackpad_ptp.c` reads the config-defined angle, derives compile-time `cos`/`sin` constants, and applies rotation at two existing points: scaled PTP contacts and raw mouse-fallback deltas. Right angles use integer fast-paths; `0` compiles out entirely. The circular touch surface makes arbitrary-angle rotation about the center clip-free.

**Tech Stack:** C (QMK module, STM32F303 target), `__builtin_cosf`/`__builtin_sinf` for compile-time trig, standalone `gcc` host test.

**Reference spec:** `docs/superpowers/specs/2026-05-22-navigator-trackpad-rotation-design.md`

---

## File Structure

- **Create:** `navigator_trackpad/navigator_trackpad_rotation.h` — pure rotation helpers (point + delta, float + integer-orthogonal). No QMK dependencies.
- **Create:** `navigator_trackpad/tests/rotation_test.c` — standalone host test for the helpers.
- **Modify:** `navigator_trackpad/config.h` — add `NAVIGATOR_TRACKPAD_ROTATION` and `NAVIGATOR_TRACKPAD_CENTER_X/Y` defaults.
- **Modify:** `navigator_trackpad/navigator_trackpad_ptp.c` — include the helper header, derive normalized angle + cos/sin constants + dispatch macros, apply rotation in the PTP contact loop and in `process_fallback_mouse`.

Both rotation call sites already live in `navigator_trackpad_ptp.c`, so all wiring is in one file (mirroring how the trackball keeps its rotation locals in `navigator.c`).

---

## Task 1: Pure rotation helpers + host test (TDD)

**Files:**
- Create: `navigator_trackpad/tests/rotation_test.c`
- Create: `navigator_trackpad/navigator_trackpad_rotation.h`

- [ ] **Step 1: Write the failing test**

Create `navigator_trackpad/tests/rotation_test.c`:

```c
// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Standalone host test for the pure trackpad rotation helpers.
// Build & run: gcc -Wall -o /tmp/nt_rotation_test \
//                  navigator_trackpad/tests/rotation_test.c -lm && /tmp/nt_rotation_test

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
```

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
cd /home/florian/dev/zsa/qmk/modules/zsa
gcc -Wall -o /tmp/nt_rotation_test navigator_trackpad/tests/rotation_test.c -lm
```
Expected: FAIL — `fatal error: ../navigator_trackpad_rotation.h: No such file or directory` (the header does not exist yet).

- [ ] **Step 3: Write the helper header**

Create `navigator_trackpad/navigator_trackpad_rotation.h`:

```c
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
```

- [ ] **Step 4: Run the test to verify it passes**

Run:
```bash
cd /home/florian/dev/zsa/qmk/modules/zsa
gcc -Wall -o /tmp/nt_rotation_test navigator_trackpad/tests/rotation_test.c -lm && /tmp/nt_rotation_test
```
Expected: PASS — prints `All rotation tests passed`, exit code 0, no warnings.

- [ ] **Step 5: Commit**

```bash
cd /home/florian/dev/zsa/qmk/modules/zsa
git add navigator_trackpad/navigator_trackpad_rotation.h navigator_trackpad/tests/rotation_test.c
git commit -m "feat(navigator_trackpad): add pure coordinate-rotation helpers + host test"
```

---

## Task 2: Add rotation config defaults

**Files:**
- Modify: `navigator_trackpad/config.h`

- [ ] **Step 1: Add the config block**

Append to `navigator_trackpad/config.h`, after the mouse-fallback shaping block (before nothing else follows it):

```c
// Rotation of the reported orientation, in degrees clockwise. Applies to both
// PTP (absolute) and mouse-fallback (relative) modes. The touch surface is
// circular, so any angle rotates cleanly about the center without clipping.
#ifndef NAVIGATOR_TRACKPAD_ROTATION
#    define NAVIGATOR_TRACKPAD_ROTATION 0
#endif

// Pivot for absolute (PTP) rotation, in logical coordinates. Default is the
// center of the logical box (TRACKPAD_LOGICAL_MAX / 2 = 1024). Override if the
// circle's center is found to map to a different point during hardware testing.
#ifndef NAVIGATOR_TRACKPAD_CENTER_X
#    define NAVIGATOR_TRACKPAD_CENTER_X 1024
#endif
#ifndef NAVIGATOR_TRACKPAD_CENTER_Y
#    define NAVIGATOR_TRACKPAD_CENTER_Y 1024
#endif
```

- [ ] **Step 2: Verify it parses (no build break)**

Run:
```bash
cd /home/florian/dev/zsa/qmk/modules/zsa
gcc -fsyntax-only -xc -include navigator_trackpad/config.h -e /dev/null 2>&1 | head; echo "exit: ${PIPESTATUS[0]}"
```
Expected: no errors from the config block (an empty translation unit is fine; `#pragma once` + `#define`s parse cleanly).

- [ ] **Step 3: Commit**

```bash
cd /home/florian/dev/zsa/qmk/modules/zsa
git add navigator_trackpad/config.h
git commit -m "feat(navigator_trackpad): add NAVIGATOR_TRACKPAD_ROTATION and center config"
```

---

## Task 3: Wire rotation constants and dispatch macros into PTP source

**Files:**
- Modify: `navigator_trackpad/navigator_trackpad_ptp.c` (top-of-file region, after the existing `#include`s ~lines 8-13)

- [ ] **Step 1: Add the include, normalized angle, cos/sin constants, and dispatch macros**

In `navigator_trackpad/navigator_trackpad_ptp.c`, immediately after the existing include block (after `#include "timer.h"`), add:

```c
#include "navigator_trackpad_rotation.h"

// Normalize the configured angle to [0, 360) so negative or >360 values still
// hit the integer right-angle fast-paths.
#define _NAVIGATOR_TRACKPAD_ROT (((NAVIGATOR_TRACKPAD_ROTATION % 360) + 360) % 360)

#if _NAVIGATOR_TRACKPAD_ROT != 0 && _NAVIGATOR_TRACKPAD_ROT != 90 && \
    _NAVIGATOR_TRACKPAD_ROT != 180 && _NAVIGATOR_TRACKPAD_ROT != 270
#    define _NT_PAD_ROT_RAD (_NAVIGATOR_TRACKPAD_ROT * 3.14159265358979f / 180.0f)
// Evaluate cos/sin at compile time to avoid runtime trig / math-library calls.
static const float pad_rotation_cos = __builtin_cosf(_NT_PAD_ROT_RAD);
static const float pad_rotation_sin = __builtin_sinf(_NT_PAD_ROT_RAD);
#endif

// Apply the configured rotation to an absolute point (lvalues px, py) and to a
// relative delta (lvalues dx, dy). Selected at compile time: 0 -> no-op,
// right angles -> integer fast-path, else -> float path. Passing the same
// lvalue as input and output is safe (the helpers read by value first).
#if _NAVIGATOR_TRACKPAD_ROT == 90
#    define NT_ROTATE_POINT(px, py) nt_rotate_point_ortho((px), (py), NAVIGATOR_TRACKPAD_CENTER_X, NAVIGATOR_TRACKPAD_CENTER_Y, 1, TRACKPAD_LOGICAL_MAX, &(px), &(py))
#    define NT_ROTATE_DELTA(dx, dy) nt_rotate_delta_ortho((dx), (dy), 1, &(dx), &(dy))
#elif _NAVIGATOR_TRACKPAD_ROT == 180
#    define NT_ROTATE_POINT(px, py) nt_rotate_point_ortho((px), (py), NAVIGATOR_TRACKPAD_CENTER_X, NAVIGATOR_TRACKPAD_CENTER_Y, 2, TRACKPAD_LOGICAL_MAX, &(px), &(py))
#    define NT_ROTATE_DELTA(dx, dy) nt_rotate_delta_ortho((dx), (dy), 2, &(dx), &(dy))
#elif _NAVIGATOR_TRACKPAD_ROT == 270
#    define NT_ROTATE_POINT(px, py) nt_rotate_point_ortho((px), (py), NAVIGATOR_TRACKPAD_CENTER_X, NAVIGATOR_TRACKPAD_CENTER_Y, 3, TRACKPAD_LOGICAL_MAX, &(px), &(py))
#    define NT_ROTATE_DELTA(dx, dy) nt_rotate_delta_ortho((dx), (dy), 3, &(dx), &(dy))
#elif _NAVIGATOR_TRACKPAD_ROT != 0
#    define NT_ROTATE_POINT(px, py) nt_rotate_point((px), (py), NAVIGATOR_TRACKPAD_CENTER_X, NAVIGATOR_TRACKPAD_CENTER_Y, pad_rotation_cos, pad_rotation_sin, TRACKPAD_LOGICAL_MAX, &(px), &(py))
#    define NT_ROTATE_DELTA(dx, dy) nt_rotate_delta((dx), (dy), pad_rotation_cos, pad_rotation_sin, &(dx), &(dy))
#else
#    define NT_ROTATE_POINT(px, py) ((void)0)
#    define NT_ROTATE_DELTA(dx, dy) ((void)0)
#endif
```

- [ ] **Step 2: Verify the module still compiles (rotation = 0, no behavior change yet)**

Run (use any keyboard/keymap that includes the `navigator_trackpad` module; substitute the correct `-kb`/`-km` if different):
```bash
cd /home/florian/dev/zsa/qmk
qmk compile -kb zsa/voyager2 -km wireless 2>&1 | tail -20
```
Expected: build succeeds. The `static inline` helpers and unused macros produce no warnings (inline functions are exempt from unused warnings). If `zsa/voyager2 -km wireless` does not include the trackpad module, compile whichever keyboard/keymap does.

- [ ] **Step 3: Commit**

```bash
cd /home/florian/dev/zsa/qmk/modules/zsa
git add navigator_trackpad/navigator_trackpad_ptp.c
git commit -m "feat(navigator_trackpad): add rotation constants and dispatch macros"
```

---

## Task 4: Apply rotation in the PTP contact loop

**Files:**
- Modify: `navigator_trackpad/navigator_trackpad_ptp.c` — the contact-assembly loop inside `navigator_trackpad_ptp_task` (currently ~lines 309-317)

- [ ] **Step 1: Rotate each scaled contact**

Locate this loop in `navigator_trackpad_ptp_task` (find it by the `scale_x(`/`scale_y(` calls, not the line number, which may drift):

```c
    for (uint8_t ss = 0; ss < 2; ss++) {
        if (sensor_report.fingers[ss].tip) {
            cur_id[cur_n]   = sensor_report.fingers[ss].id;
            cur_x[cur_n]    = scale_x(sensor_report.fingers[ss].x);
            cur_y[cur_n]    = scale_y(sensor_report.fingers[ss].y);
            cur_conf[cur_n] = sensor_report.fingers[ss].confidence;
            cur_n++;
        }
    }
```

Replace it with (adds the `NT_ROTATE_POINT` call on the scaled coordinates):

```c
    for (uint8_t ss = 0; ss < 2; ss++) {
        if (sensor_report.fingers[ss].tip) {
            cur_id[cur_n]   = sensor_report.fingers[ss].id;
            cur_x[cur_n]    = scale_x(sensor_report.fingers[ss].x);
            cur_y[cur_n]    = scale_y(sensor_report.fingers[ss].y);
            // Rotate the absolute contact about the configured center. Both
            // contacts share the center, so the transform is rigid: their
            // separation (used for two-finger gestures) is preserved.
            NT_ROTATE_POINT(cur_x[cur_n], cur_y[cur_n]);
            cur_conf[cur_n] = sensor_report.fingers[ss].confidence;
            cur_n++;
        }
    }
```

- [ ] **Step 2: Verify it compiles at rotation = 0 (no-op) and at an arbitrary angle**

Run (rotation = 0, default):
```bash
cd /home/florian/dev/zsa/qmk
qmk compile -kb zsa/voyager2 -km wireless 2>&1 | tail -5
```
Expected: build succeeds.

Run (force an arbitrary angle to exercise the float path):
```bash
cd /home/florian/dev/zsa/qmk
qmk compile -kb zsa/voyager2 -km wireless -e EXTRAFLAGS="-DNAVIGATOR_TRACKPAD_ROTATION=30" 2>&1 | tail -5
```
Expected: build succeeds. (If `EXTRAFLAGS` is not honored by the build, instead temporarily set `#define NAVIGATOR_TRACKPAD_ROTATION 30` in a keymap `config.h`, compile, then revert.)

- [ ] **Step 3: Commit**

```bash
cd /home/florian/dev/zsa/qmk/modules/zsa
git add navigator_trackpad/navigator_trackpad_ptp.c
git commit -m "feat(navigator_trackpad): rotate PTP absolute contacts"
```

---

## Task 5: Apply rotation in the mouse-fallback path

**Files:**
- Modify: `navigator_trackpad/navigator_trackpad_ptp.c` — `process_fallback_mouse` (raw delta computation, currently ~lines 183-184)

- [ ] **Step 1: Rotate the raw deltas before clamping**

Locate this block inside `process_fallback_mouse` (find it by the `raw_dx`/`raw_dy` assignment):

```c
            int16_t raw_dx = (int16_t)sensor_report->fingers[0].x - (int16_t)mouse_state.last_x;
            int16_t raw_dy = (int16_t)sensor_report->fingers[0].y - (int16_t)mouse_state.last_y;

            // Clamp deltas to prevent jumps from bad sensor data
            if (raw_dx > TRACKPAD_MAX_DELTA) raw_dx = TRACKPAD_MAX_DELTA;
```

Insert the rotation call between the `raw_dy` assignment and the clamp comment:

```c
            int16_t raw_dx = (int16_t)sensor_report->fingers[0].x - (int16_t)mouse_state.last_x;
            int16_t raw_dy = (int16_t)sensor_report->fingers[0].y - (int16_t)mouse_state.last_y;

            // Rotate the relative motion vector (same convention as the trackball).
            // Tap/drag detection uses squared distance, which rotation leaves
            // invariant, so only the reported deltas need rotating.
            NT_ROTATE_DELTA(raw_dx, raw_dy);

            // Clamp deltas to prevent jumps from bad sensor data
            if (raw_dx > TRACKPAD_MAX_DELTA) raw_dx = TRACKPAD_MAX_DELTA;
```

- [ ] **Step 2: Verify it compiles at rotation = 0 and at a right angle**

Run (default):
```bash
cd /home/florian/dev/zsa/qmk
qmk compile -kb zsa/voyager2 -km wireless 2>&1 | tail -5
```
Expected: build succeeds.

Run (right angle → integer fast-path):
```bash
cd /home/florian/dev/zsa/qmk
qmk compile -kb zsa/voyager2 -km wireless -e EXTRAFLAGS="-DNAVIGATOR_TRACKPAD_ROTATION=90" 2>&1 | tail -5
```
Expected: build succeeds.

- [ ] **Step 3: Re-run the host test to confirm helpers are unchanged**

Run:
```bash
cd /home/florian/dev/zsa/qmk/modules/zsa
gcc -Wall -o /tmp/nt_rotation_test navigator_trackpad/tests/rotation_test.c -lm && /tmp/nt_rotation_test
```
Expected: PASS — `All rotation tests passed`.

- [ ] **Step 4: Commit**

```bash
cd /home/florian/dev/zsa/qmk/modules/zsa
git add navigator_trackpad/navigator_trackpad_ptp.c
git commit -m "feat(navigator_trackpad): rotate mouse-fallback deltas"
```

---

## Task 6: Manual hardware verification (no code change)

The per-frame integration into `navigator_trackpad_ptp_task` cannot be unit-tested without hardware. Verify on a device that includes the trackpad module.

- [ ] **Step 1: Build with rotation = 90 and flash**

Set `#define NAVIGATOR_TRACKPAD_ROTATION 90` in the test keymap's `config.h`, then:
```bash
cd /home/florian/dev/zsa/qmk
qmk compile -kb zsa/voyager2 -km wireless
```
Flash the resulting firmware.

- [ ] **Step 2: Verify behavior**

On the host (PTP mode, e.g. Windows/macOS): drag a finger straight up the pad and confirm the cursor moves in the direction expected for a 90 deg clockwise rotation, consistent with the trackball at the same setting. Confirm two-finger tap (right-click) and two-finger scroll still work (contact separation must be preserved).

- [ ] **Step 3: Confirm rotation = 0 is unchanged**

Rebuild with `NAVIGATOR_TRACKPAD_ROTATION` unset (default 0), flash, and confirm cursor/gesture behavior is identical to before this change.

- [ ] **Step 4: Revert the temporary keymap config**

Remove the temporary `#define NAVIGATOR_TRACKPAD_ROTATION` from the keymap `config.h`.

---

## Self-Review Notes

- **Spec coverage:** config knob + center (Task 2), arbitrary-angle PTP rotation about center (Tasks 3-4), mouse-fallback delta rotation (Task 5), integer fast-paths + 0-compiles-out (Task 3 macros), unit tests of pure helpers (Task 1), manual hardware check for the un-unit-testable integration (Task 6).
- **Sign convention:** delta fast-paths (`nt_rotate_delta_ortho`) match the trackball: 90 → `(-dy, dx)`, 180 → `(-dx, -dy)`, 270 → `(dy, -dx)`. The point fast-paths apply the same transform to `(x-cx, y-cy)` then re-center, and the host test asserts they equal the float path.
- **Naming consistency:** helpers `nt_rotate_point` / `nt_rotate_delta` / `nt_rotate_point_ortho` / `nt_rotate_delta_ortho`; macros `NT_ROTATE_POINT` / `NT_ROTATE_DELTA`; constants `pad_rotation_cos` / `pad_rotation_sin`; normalized angle `_NAVIGATOR_TRACKPAD_ROT` — used identically across Tasks 1, 3, 4, 5.
- **Aliasing:** all helpers take inputs by value before writing through output pointers, so `NT_ROTATE_*(v, w)` (which aliases input and output) is safe.
