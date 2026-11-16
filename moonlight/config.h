// Copyright 2026 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

// Keep running when USB never enumerates (dumb charger / power bank).
// Compiles out the suspend trap in tmk_core/protocol/chibios/chibios.c that
// would otherwise park the board in suspend_power_down() forever.
#define NO_USB_STARTUP_CHECK
