#pragma once

#include <stdint.h>
#include <stdbool.h>

/* BLE status overlay rendering.
 *
 * The keymap owns which layer is the status layer; this module owns what that
 * layer looks like. Call zsa_wireless_status_render() from
 * rgb_matrix_indicators_user() while that layer is active:
 *
 *     if (biton32(layer_state) == BT_LAYER) {
 *         zsa_wireless_status_render();
 *         return true;
 *     }
 *
 * It paints the whole matrix — background everywhere, then a six-LED battery
 * fuel gauge on the top row of each half — so the caller does not need to
 * blank anything first. Geometry, colors and animation timing are tunable via
 * the ZSA_STATUS_* defines in this module's config.h.
 *
 * Compiles to a no-op without RGB_MATRIX_ENABLE. */
void zsa_wireless_status_render(void);

/* Draw a single six-LED gauge starting at first_led. Exposed for keymaps that
 * want to place a gauge somewhere other than the two default rows. A soc of 0
 * means "unknown" and leaves the row on the background color. */
void zsa_wireless_status_draw_gauge(uint8_t first_led, uint8_t soc);
