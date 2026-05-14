// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

// Physical sensor dimensions in 0.01-inch units (HID convention).
// The actual ZSA navigator pad is ~40 mm (1.57"). The values below
// deliberately inflate the reported area 5x so the OS interprets cursor
// movement as faster than the tiny physical surface alone would suggest.
// Override in a keymap if you need different sensitivity.
#ifndef DIGITIZER_TOUCHPAD_PHYSICAL_WIDTH
#    define DIGITIZER_TOUCHPAD_PHYSICAL_WIDTH 785
#endif
#ifndef DIGITIZER_TOUCHPAD_PHYSICAL_HEIGHT
#    define DIGITIZER_TOUCHPAD_PHYSICAL_HEIGHT 785
#endif

// Logical max stays at the core default of 2048.
// Contact count stays at the core default of 2.
