// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

// Cirque Gen6 I2C address
#ifndef NAVIGATOR_TRACKPAD_ADDRESS
#    define NAVIGATOR_TRACKPAD_ADDRESS 0x58
#endif

// Per-transaction I2C timeout (ms)
#ifndef NAVIGATOR_TRACKPAD_TIMEOUT
#    define NAVIGATOR_TRACKPAD_TIMEOUT 100
#endif

// Mouse-fallback shaping (only used when host hasn't switched to PTP mode)
#ifndef TRACKPAD_MOUSE_SENSITIVITY
#    define TRACKPAD_MOUSE_SENSITIVITY 0.3f
#endif
#ifndef TRACKPAD_MOUSE_ACCELERATION
#    define TRACKPAD_MOUSE_ACCELERATION 1.1f
#endif
