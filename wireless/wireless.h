#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "zsa_uart_proto.h"

enum zsa_idle_step {
    ZSA_IDLE_ACTIVE    = 0,
    ZSA_IDLE_DIM       = 1,
    ZSA_IDLE_LEDS_OFF  = 2,  /* LEDs off + slow matrix scan */
    ZSA_IDLE_DEEP      = 3,
};

/* Cached nRF state, updated via UART events */
bool    zsa_wireless_ble_connected(void);
/* True while the nRF is in BLE discovery mode (advertising / awaiting pairing),
 * i.e. discoverable so a host can find and pair with it. Driven by
 * ZSA_MSG_EVT_PAIR_STATE (ACTIVE). */
bool    zsa_wireless_discovery_active(void);
/* True while a pairing request is waiting for the user to confirm it
 * (ZSA_MSG_EVT_PAIR_STATE / ZSA_PAIR_CONFIRM). Mutually exclusive with
 * discovery: the nRF reports CONFIRM in preference to ACTIVE. */
bool    zsa_wireless_pair_confirm_pending(void);
uint8_t zsa_wireless_battery_soc(void);
/* Right half SOC, relayed by the left nRF over the inter-half link. 0 when the
 * right half has never reported (e.g. powered down or not yet linked). */
uint8_t zsa_wireless_battery_right_soc(void);
uint8_t zsa_wireless_battery_flags(void);
/* Detected USB source current budget in mA (EVT_PWR_BUDGET), between
 * ZSA_PWR_BUDGET_DEFAULT_MA and the source's advertised capability. Reads as
 * the default until a CC event reports otherwise. */
uint16_t zsa_wireless_power_budget_ma(void);
/* True while the nRF reports VBUS present (EVT_VBUS_STATE), false at boot
 * and once VBUS goes away. */
bool zsa_wireless_vbus_present(void);

/* Weak hook: keyboards override this to react to VBUS present/absent
 * (EVT_VBUS_STATE), e.g. to drive their own USB D+ pullup. */
void zsa_wireless_vbus_changed(bool present);

/* State of the pairing status LED. RELEASE hands the pin back to whatever
 * else drives it. */
enum zsa_pair_led_state {
    ZSA_PAIR_LED_RELEASE = 0,
    ZSA_PAIR_LED_OFF     = 1,
    ZSA_PAIR_LED_ON      = 2,
};

/* Weak hook: keyboards override this to drive their own status LED pin. The
 * module owns the blink phase and only calls on a change. */
void zsa_wireless_pair_led(uint8_t state);

/* True while the module is driving the pairing status LED. Keyboards check
 * this before writing that pin for any other purpose. */
bool zsa_wireless_pair_led_active(void);

/* Send a query/command to the nRF */
void zsa_wireless_query_ble_status(void);
void zsa_wireless_query_battery(void);
void zsa_wireless_cmd_hibernate(void);
void zsa_wireless_cmd_pair(void);
/* Accept the pending pairing request. Sent automatically, once per request,
 * when any key on a row below ZSA_PAIR_ACCEPT_ROWS (the left half) is pressed
 * while zsa_wireless_pair_confirm_pending(). */
void zsa_wireless_cmd_pair_accept(void);
/* Drop the active BLE link, keeping the bond. The nRF re-advertises after. */
void zsa_wireless_cmd_ble_disconnect(void);
/* Clear every BLE bond and refresh the identity. Sent by BT_UNPR after a
 * ZSA_UNPAIR_HOLD_MS hold. */
void zsa_wireless_cmd_ble_unpair(void);

/* Idle power management */
enum zsa_idle_step zsa_wireless_idle_step(void);

/* Low-level UART frame sender. Used by optional sub-modules like log_tap. */
void zsa_wireless_send_frame(uint8_t type, const uint8_t *payload, uint16_t len);

/* True once the LINK_UP handshake with the nRF has completed at least once.
 * Sub-modules (e.g. navigator_wireless) should gate any control sends on this
 * — frames sent before the nRF's UART rx is ready get dropped at the hardware
 * level with no diagnostic. Same gate that send_full_state already uses. */
bool zsa_wireless_link_ready(void);

/* Navigator event hooks. The wireless module tracks the currently-detected
 * device type from EVT_NAV_PRESENT events. Both trackball and trackpad
 * motion/touch data are delivered exclusively via the nRF's emulated-I2C nav
 * register mirrors — trackball: 0x28 real bridge (left-plugged) / 0x2D
 * left-nRF mirror (right-plugged); trackpad: 0x2C real pad (left-plugged) /
 * 0x2E left-nRF mirror (right-plugged) — never over UART. (Digitizer PTP HID
 * *reports*, once decoded from that mirror, do ride UART — see
 * ZSA_MSG_HID_DIGITIZER in zsa_uart_proto.h and navigator_trackpad_ptp.c's
 * navigator_trackpad_output_touchpad() hook.) */
uint8_t zsa_wireless_nav_device_type(void);  /* enum zsa_nav_device_type, 0 if none */
