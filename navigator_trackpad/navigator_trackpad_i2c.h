// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "navigator_trackpad_core.h"  // for NAV_TRACKPAD_PACKET_LEN

// Cirque chip i2c init: ping, clear pipe, set PTP mode + swap/invert/disable
// logical scaling. Sets the file-scope `trackpad_init` flag on success.
void navigator_trackpad_device_init(void);

// Read one 17-byte raw cirque packet from i2c. Returns true on success;
// false on i2c failure (also clears `trackpad_init` so the next task tick
// triggers re-init).
bool navigator_trackpad_i2c_read_packet(uint8_t out[NAV_TRACKPAD_PACKET_LEN]);

// File-scope state shared with navigator_trackpad.c.
extern bool trackpad_init;
