// Copyright 2026 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

// Keep running when USB never enumerates (dumb charger / power bank).
// Compiles out the suspend trap in tmk_core/protocol/chibios/chibios.c that
// would otherwise park the board in suspend_power_down() forever.
#define NO_USB_STARTUP_CHECK

// Swallow every non-moonlight keycode so a broken matrix can never type
// into a host. Keymaps for working keyboards may set this to 0.
#ifndef MOONLIGHT_LAMP_ONLY
#    define MOONLIGHT_LAMP_ONLY 1
#endif

// A lamp never boots dark: brightness floor applied at power-up.
#ifndef MOONLIGHT_MIN_BOOT_BRIGHTNESS
#    define MOONLIGHT_MIN_BOOT_BRIGHTNESS 40
#endif

// MOONLIGHT_DIMMER floor (dim ≠ off; MOONLIGHT_OFF turns the light off).
#ifndef MOONLIGHT_MIN_BRIGHTNESS
#    define MOONLIGHT_MIN_BRIGHTNESS 16
#endif
