// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

// Physical sensor dimensions in 0.01-inch units (HID convention).
// The actual ZSA navigator pad is ~40 mm (1.57"). We report ~3.5x (5.5") so the OS
// treats cursor movement as faster than the tiny surface alone would suggest.
// CAP: do NOT inflate beyond ~3.5x. Above it, the two contacts of a two-finger tap
// report far enough apart that Windows' tap-separation threshold rejects the
// gesture, silently breaking two-finger-tap right-click. 550 is the empirical max
// that keeps it reliable. Override in a keymap if you need different sensitivity.
#ifndef DIGITIZER_TOUCHPAD_PHYSICAL_WIDTH
#    define DIGITIZER_TOUCHPAD_PHYSICAL_WIDTH 550
#endif
#ifndef DIGITIZER_TOUCHPAD_PHYSICAL_HEIGHT
#    define DIGITIZER_TOUCHPAD_PHYSICAL_HEIGHT 550
#endif

// Logical max stays at the core default of 2048.
// Contact count stays at the core default of 2.
