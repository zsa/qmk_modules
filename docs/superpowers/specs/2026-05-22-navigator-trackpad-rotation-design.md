# Navigator Trackpad Rotation — Design

Date: 2026-05-22
Status: Approved (Approach A)

## Goal

Add a `NAVIGATOR_TRACKPAD_ROTATION` configuration parameter to the Navigator
trackpad module, analogous to the trackball's `NAVIGATOR_TRACKBALL_ROTATION`.
It lets a keymap rotate the trackpad's reported orientation by an arbitrary
angle (degrees, clockwise).

## Background

The trackpad differs fundamentally from the trackball:

- **Trackball** reports *relative deltas*. Its rotation is a cos/sin transform
  of the `(x, y)` motion vector, with integer fast-paths for 90/180/270 and a
  general path for arbitrary angles (`navigator_trackball/navigator.c`).
- **Trackpad** has two modes, both implemented in
  `navigator_trackpad/navigator_trackpad_ptp.c`:
  - **PTP mode** (primary): reports *absolute coordinates* of the touch surface
    to the OS as a Windows Precision Touchpad. Coordinates are scaled to a
    logical `[0, TRACKPAD_LOGICAL_MAX]` (2048) square via `scale_x`/`scale_y`.
  - **Mouse-fallback mode**: computes *relative deltas* from sensor coordinate
    changes, for hosts without PTP support.

The touch surface is **physically circular** (with a small flat chord at the
top). This is the key enabler for arbitrary-angle rotation in PTP mode: the
physically-touchable area is the circle inscribed in the square coordinate
system, so the corners of the logical box are unreachable dead space. Rotating
the coordinate mapping about the circle's center by *any* angle keeps every real
touch inside the circle — and therefore inside the logical box. There is **no
corner clipping** and no shrinking of usable area, unlike a rectangular pad.

## Approach

Pure software rotation driven by a single angle value, applied in both modes
from shared math. Chosen over a hardware (Cirque `swap_xy`/`invert`) approach
because hardware registers only express right angles and would create a second
source of orientation truth; the circular surface makes software rotation both
correct for arbitrary angles and cheap (STM32F303 has a single-precision FPU,
and only up to two contacts are transformed per frame).

## Configuration

Added to `navigator_trackpad/config.h` (each `#ifndef`-guarded so keymaps can
override):

```c
#ifndef NAVIGATOR_TRACKPAD_ROTATION    // degrees, clockwise
#    define NAVIGATOR_TRACKPAD_ROTATION 0
#endif
#ifndef NAVIGATOR_TRACKPAD_CENTER_X     // logical-space pivot; = TRACKPAD_LOGICAL_MAX / 2
#    define NAVIGATOR_TRACKPAD_CENTER_X 1024
#endif
#ifndef NAVIGATOR_TRACKPAD_CENTER_Y
#    define NAVIGATOR_TRACKPAD_CENTER_Y 1024
#endif
```

The center constants exist because the circle's center may not map exactly to
`(1024, 1024)`; they let the pivot be tuned without code changes if hardware
testing reveals an offset. Default is the geometric center of the logical box.

The raw angle is normalized to `[0, 360)` so that negative or >360 values still
trigger the orthogonal fast-paths:

```c
#define _NAVIGATOR_TRACKPAD_ROT (((NAVIGATOR_TRACKPAD_ROTATION % 360) + 360) % 360)
```

When `_NAVIGATOR_TRACKPAD_ROT == 0`, all rotation code compiles out — behavior
is byte-identical to today.

## Rotation math

For arbitrary angles, cosine/sine are evaluated at compile time via
`__builtin_cosf`/`__builtin_sinf` (same technique as the trackball) so no
runtime trig or math-library calls are emitted:

```c
#define _NT_PAD_ROT_RAD (_NAVIGATOR_TRACKPAD_ROT * 3.14159265358979f / 180.0f)
static const float pad_rotation_cos = __builtin_cosf(_NT_PAD_ROT_RAD);
static const float pad_rotation_sin = __builtin_sinf(_NT_PAD_ROT_RAD);
```

**Point rotation (PTP, absolute)** — rotate about `(cx, cy)`:

```
dx = x - cx;  dy = y - cy;
x' = cx + dx*cos - dy*sin
y' = cy + dx*sin + dy*cos
clamp x', y' to [0, TRACKPAD_LOGICAL_MAX]
```

**Delta rotation (mouse-fallback, relative)** — origin-centered, same cos/sin:

```
dx' = dx*cos - dy*sin
dy' = dx*sin + dy*cos
```

**Orthogonal fast-paths** (integer, no float), matching the trackball's sign
convention (clockwise). For deltas they are about the origin; for points the
same transform is applied to `(x - cx, y - cy)` then re-centered:

| Angle | delta (dx', dy') | point (x', y') |
|-------|------------------|----------------|
| 90    | (-dy,  dx)       | (cx - (y-cy),  cy + (x-cx)) |
| 180   | (-dx, -dy)       | (cx - (x-cx),  cy - (y-cy)) |
| 270   | ( dy, -dx)       | (cx + (y-cy),  cy - (x-cx)) |

## Integration points

Both call sites are in `navigator_trackpad_ptp.c`. The rotation constants and
two small static helpers (`rotate_point`, `rotate_delta`) live in that file,
mirroring how the trackball keeps its rotation locals in `navigator.c`.

1. **PTP path** — in `navigator_trackpad_ptp_task`, the contact assembly loop
   (currently ~lines 309–317) scales each finger with `scale_x`/`scale_y`.
   After scaling, rotate the `(cur_x, cur_y)` pair about the center. Both
   contacts use the same center, so the transform is rigid: inter-contact
   distance is preserved and two-finger-tap separation / gestures are
   unaffected.

2. **Mouse-fallback path** — in `process_fallback_mouse`, rotate
   `(raw_dx, raw_dy)` (currently computed ~lines 183–184) immediately after they
   are derived, before the `TRACKPAD_MAX_DELTA` clamp and acceleration. Tap/drag
   detection uses squared distance from the settled position, which is invariant
   under rotation, so it needs no change.

## Non-goals / unaffected

- Tap-to-click, drag detection, CPI handling, and contact-id tracking are
  unchanged (rotation preserves distances and is applied only to reported
  coordinates/deltas).
- No change to the HID descriptor or physical-size reporting.
- No hardware-register (`swap_xy`/`invert_*`) changes; the existing baseline
  orientation in `navigator_trackpad_device_init` stays as-is and represents
  rotation 0.

## Testing

The rotation helpers are pure functions, host-testable without hardware:

- Identity at 0° (input == output) for both point and delta rotation.
- The three orthogonal cases (90/180/270) for both, checked against the table
  above with exact integer expectations.
- One arbitrary angle (45°): verify distance from center is preserved for point
  rotation (rigidity) and vector magnitude is preserved for delta rotation, and
  that clamping keeps PTP output within `[0, TRACKPAD_LOGICAL_MAX]`.

## Risks

- **Center offset**: if the circle is not centered at `(1024, 1024)`, off-axis
  rotation will look slightly skewed. Mitigated by the configurable
  `NAVIGATOR_TRACKPAD_CENTER_X/Y`.
- **Rounding for arbitrary angles**: float-to-int truncation may introduce
  sub-pixel error per frame; negligible at 2048 logical resolution and
  acceptable for a pointing device.
