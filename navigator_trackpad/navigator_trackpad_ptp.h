// Copyright 2025 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "navigator_trackpad_common.h"
#include "report.h"

// Mouse fallback mode configuration (when host doesn't support PTP)
// TRACKPAD_MOUSE_SENSITIVITY: Movement multiplier (0.5 = slower, 1.0 = normal, 2.0 = faster)
// TRACKPAD_MOUSE_ACCELERATION: Acceleration curve exponent (1.0 = linear, 1.2 = moderate accel)
// Add to your config.h to customize:
#ifndef TRACKPAD_MOUSE_SENSITIVITY
#    define TRACKPAD_MOUSE_SENSITIVITY 0.3f
#endif
#ifndef TRACKPAD_MOUSE_ACCELERATION
#    define TRACKPAD_MOUSE_ACCELERATION 1.1f
#endif

// PTP task function - called by navigator_trackpad.c each cycle
bool navigator_trackpad_ptp_task(void);
