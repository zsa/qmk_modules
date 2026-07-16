// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

// Cirque Gen6 I2C address
#ifndef NAVIGATOR_TRACKPAD_ADDRESS
#    define NAVIGATOR_TRACKPAD_ADDRESS 0x58
#endif

// Real trackpad = 0x2C<<1 = 0x58 (left-plugged, direct Cirque Gen6 on the
// STM32's bus, same as NAVIGATOR_TRACKPAD_ADDRESS above). Left-nRF wireless
// mirror = 0x2E<<1 = 0x5C (right-plugged, wireless; the left nRF emulates a
// register-compatible mirror of the real trackpad at this address).
#ifndef NAVIGATOR_TRACKPAD_MIRROR_ADDRESS
#    define NAVIGATOR_TRACKPAD_MIRROR_ADDRESS 0x5C
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
