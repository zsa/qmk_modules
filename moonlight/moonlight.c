// Copyright 2026 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Moonlight: light control for upcycled keyboards. Purely a lamp module —
// see README.md. All behavior is driven by rgb_matrix.

#include QMK_KEYBOARD_H

ASSERT_COMMUNITY_MODULES_MIN_API_VERSION(1, 0, 0);

#ifndef RGB_MATRIX_ENABLE
#    error "The moonlight module requires an rgb_matrix-enabled keyboard (RGB_MATRIX_ENABLE = yes)."
#endif
