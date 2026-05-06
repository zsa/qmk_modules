// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "pointing_device.h"

/* The navigator_wireless transport carries a 0-255 normalized CPI value over
 * the wire (ZSA_MSG_CMD_NAV_CPI). The nRF-side sensor driver maps that to its
 * native units (e.g. paw3805ek register 0x8D for the trackball, an internal
 * multiplier for a touchpad). The constants below match navigator_trackball's
 * defaults so the same keymap-side keycode logic (NAVIGATOR_INC_CPI etc.) and
 * scroll/turbo/aim helpers in navigator.c work unchanged. */

#ifndef NAVIGATOR_TRACKBALL_CPI
#    define NAVIGATOR_TRACKBALL_CPI 40
#endif

#ifndef NAVIGATOR_TRACKBALL_CPI_TICK
#    define NAVIGATOR_TRACKBALL_CPI_TICK 5
#endif

#define NAVIGATOR_TRACKBALL_CPI_MAX 125

void restore_cpi(uint8_t cpi);
