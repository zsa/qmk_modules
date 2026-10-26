/*
 * ZSA Wireless — BLE status overlay rendering.
 *
 * Draws the status layer's LED readouts: currently a battery fuel gauge for
 * each half on the top rows. Also draws the Bluetooth bank menu. Layer selection lives in the keymap; everything
 * about how the layer looks lives here.
 */

#include "status_layer.h"

#ifdef RGB_MATRIX_ENABLE

#    include "wireless.h"
#    include "rgb_matrix.h"
#    include "timer.h"
#    include "progmem.h"

/* Phase lag per LED (256 = one full cycle) — this is what makes the crest
 * travel along the row instead of every LED pulsing together. */
#    define STATUS_BREATHE_LED_STEP (256 / ZSA_STATUS_GAUGE_LEDS)

/* Fixed position colors, low charge first: an even hue ramp from red (0) to
 * green (85) — red, orange, amber, yellow, yellow-green, green. Evenly spaced
 * so adjacent LEDs blend instead of stepping in hard pairs. */
static const uint8_t PROGMEM status_gauge_hue[ZSA_STATUS_GAUGE_LEDS] = {0, 17, 34, 51, 68, 85};

/* Scale an HSV to RGB by the matrix's current brightness, so status readouts
 * track the user's brightness setting like every other effect. Integer math —
 * the keymap-side equivalent uses float, which is not worth it here. */
static RGB status_hsv_to_rgb(HSV hsv) {
    RGB     rgb = hsv_to_rgb(hsv);
    uint8_t v   = zsa_wireless_led_val();

    return (RGB){
        .r = (uint8_t)(((uint16_t)rgb.r * v) / UINT8_MAX),
        .g = (uint8_t)(((uint16_t)rgb.g * v) / UINT8_MAX),
        .b = (uint8_t)(((uint16_t)rgb.b * v) / UINT8_MAX),
    };
}

/* Sweep period from the detected source budget, linear between the two ends:
 * a slow charger breathes lazily, a high-budget one visibly hurries. */
static uint16_t status_breathe_period_ms(void) {
    uint16_t ma = zsa_wireless_power_budget_ma();

    if (ma <= ZSA_PWR_BUDGET_DEFAULT_MA) return ZSA_STATUS_BREATHE_SLOW_MS;
    if (ma >= ZSA_PWR_BUDGET_HIGH_MA) return ZSA_STATUS_BREATHE_FAST_MS;

    uint32_t span = ZSA_PWR_BUDGET_HIGH_MA - ZSA_PWR_BUDGET_DEFAULT_MA;
    uint32_t drop = (uint32_t)(ZSA_STATUS_BREATHE_SLOW_MS - ZSA_STATUS_BREATHE_FAST_MS) * (ma - ZSA_PWR_BUDGET_DEFAULT_MA);

    return (uint16_t)(ZSA_STATUS_BREATHE_SLOW_MS - (drop / span));
}

/* Fuel-gauge fill from the low end, each LED keeping its position color.
 * How full the bar is, in 8.8 fixed point: the high byte is the count of
 * fully-lit LEDs, the low byte is the brightness of the one being filled, so
 * the leading edge fades in instead of jumping a whole LED at a time.
 *
 * While charging, a brightness wave travels left to right across the lit part.
 * The nRF reports charging only while current actually flows into the cell, so
 * the animation stops on its own once the battery is full. */
void zsa_wireless_status_draw_gauge(uint8_t first_led, uint8_t soc) {
    if (soc > 100) soc = 100;

    uint16_t level = (uint16_t)(((uint32_t)soc * ZSA_STATUS_GAUGE_LEDS * 256) / 100);
    uint8_t  full  = level >> 8;
    uint8_t  frac  = level & 0xFF;

    bool    charging = (zsa_wireless_battery_flags() & ZSA_BAT_FLAG_CHARGING) != 0;
    uint8_t phase    = 0;
    if (charging) {
        uint16_t period = status_breathe_period_ms();
        phase           = (uint8_t)(((uint32_t)(timer_read32() % period) * 256) / period);
    }

    for (uint8_t i = 0; i < ZSA_STATUS_GAUGE_LEDS; i++) {
        uint8_t v = (i < full) ? 255 : (i == full ? frac : 0);

        if (v && charging) {
            /* Subtracting the per-LED lag walks the crest from left to right. */
            uint8_t wave = zsa_wireless_status_tri((uint8_t)(phase - (uint8_t)(i * STATUS_BREATHE_LED_STEP)));
            uint8_t amp  = (uint8_t)(ZSA_STATUS_BREATHE_MIN + ((uint16_t)(255 - ZSA_STATUS_BREATHE_MIN) * wave) / 255);
            v            = (uint8_t)(((uint16_t)v * amp) / 255);
        }

        if (v) {
            RGB rgb = status_hsv_to_rgb((HSV){pgm_read_byte(&status_gauge_hue[i]), 255, v});
            rgb_matrix_set_color(first_led + i, rgb.r, rgb.g, rgb.b);
        } else {
            rgb_matrix_set_color(first_led + i, ZSA_STATUS_BG_R, ZSA_STATUS_BG_G, ZSA_STATUS_BG_B);
        }
    }
}

/* The gauge has no unknown appearance, so an unknown half gets no gauge at
 * all: its row stays at the background color. A level from a link that stopped
 * reporting is unknown, not a reading. */
void zsa_wireless_status_render(void) {
    rgb_matrix_set_color_all(ZSA_STATUS_BG_R, ZSA_STATUS_BG_G, ZSA_STATUS_BG_B);

    if (!zsa_wireless_battery_fresh()) return;

    uint8_t left  = zsa_wireless_battery_soc();
    uint8_t right = zsa_wireless_battery_right_soc();

    if (left <= 100) {
        zsa_wireless_status_draw_gauge(ZSA_STATUS_GAUGE_LEFT_FIRST, left);
    }
    /* 0 is unknown here too — see telemetry_battery(). */
    if (right > 0 && right <= 100) {
        zsa_wireless_status_draw_gauge(ZSA_STATUS_GAUGE_RIGHT_FIRST, right);
    }
}

/* Full-scale colors, unscaled by the user's brightness and the VBUS cap: the
 * menu lights five LEDs at most. */
static RGB ble_menu_color(uint8_t view, uint8_t blink) {
    switch (view) {
        case ZSA_BLE_BANK_VIEW_EMPTY:
            return (RGB){ZSA_BLE_MENU_EMPTY_R, ZSA_BLE_MENU_EMPTY_G, ZSA_BLE_MENU_EMPTY_B};
        case ZSA_BLE_BANK_VIEW_STORED:
            return (RGB){0, 0, 255};
        case ZSA_BLE_BANK_VIEW_CONNECTING:
            return (RGB){0, blink, 0};
        case ZSA_BLE_BANK_VIEW_PAIRING:
            return (RGB){0, 0, blink};
        case ZSA_BLE_BANK_VIEW_CONNECTED:
            return (RGB){0, 255, 0};
        default:
            return (RGB){ZSA_BLE_MENU_UNKNOWN_VAL, ZSA_BLE_MENU_UNKNOWN_VAL, ZSA_BLE_MENU_UNKNOWN_VAL};
    }
}

bool zsa_wireless_ble_menu_render(uint8_t led_min, uint8_t led_max) {
    if (!zsa_wireless_ble_menu_open()) return false;

    for (uint8_t i = led_min; i < led_max; i++) {
        rgb_matrix_set_color(i, 0, 0, 0);
    }

    uint8_t blink = (timer_read32() % ZSA_BLE_MENU_BLINK_MS) < ZSA_BLE_MENU_BLINK_MS / 2 ? 255 : 0;

    for (uint8_t bank = 0; bank < ZSA_BLE_BANK_COUNT; bank++) {
        uint8_t led = g_led_config.matrix_co[ZSA_BLE_MENU_ROW][ZSA_BLE_MENU_COL + bank];
        if (led == NO_LED || led < led_min || led >= led_max) continue;

        RGB rgb = ble_menu_color(zsa_wireless_ble_bank_view(bank), blink);
        rgb_matrix_set_color(led, rgb.r, rgb.g, rgb.b);
    }
    return true;
}

#else /* !RGB_MATRIX_ENABLE */

bool zsa_wireless_ble_menu_render(uint8_t led_min, uint8_t led_max) {
    (void)led_min;
    (void)led_max;
    return false;
}

void zsa_wireless_status_render(void) {}
void zsa_wireless_status_draw_gauge(uint8_t first_led, uint8_t soc) {
    (void)first_led;
    (void)soc;
}

#endif
