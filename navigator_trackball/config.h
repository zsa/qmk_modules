// Copyright 2025 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

// Define the macro that navigator.h checks to set trackball-specific values
#define POINTING_DEVICE_DRIVER_navigator_trackball

// Enable 16-bit motion data from the PAW3805EK sensor
#ifndef MOUSE_EXTENDED_REPORT
#    define MOUSE_EXTENDED_REPORT
#endif

#ifndef WHEEL_EXTENDED_REPORT
#    define WHEEL_EXTENDED_REPORT
#endif
