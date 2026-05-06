// Copyright 2025 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

// Define the macro that navigator.h checks to set trackball-style scroll/turbo defaults.
// Same flag as the navigator_trackball module so the shared navigator.c keymap
// helpers (smooth-scroll dividers, turbo multiplier, aim divider) keep their
// existing tuning regardless of which transport delivered the motion event.
#define POINTING_DEVICE_DRIVER_navigator_trackball

// Motion deltas come over the wire as int16. Match the trackball module's
// extended-report config so QMK keeps the 16-bit mouse path enabled.
#ifndef MOUSE_EXTENDED_REPORT
#    define MOUSE_EXTENDED_REPORT
#endif

#ifndef WHEEL_EXTENDED_REPORT
#    define WHEEL_EXTENDED_REPORT
#endif

// Enable high-resolution scrolling
#ifndef POINTING_DEVICE_HIRES_SCROLL_ENABLE
#    define POINTING_DEVICE_HIRES_SCROLL_ENABLE
#endif

#ifndef POINTING_DEVICE_HIRES_SCROLL_EXPONENT
#    define POINTING_DEVICE_HIRES_SCROLL_EXPONENT 0
#endif
