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
uint8_t zsa_wireless_battery_soc(void);
uint8_t zsa_wireless_battery_flags(void);

/* Send a query/command to the nRF */
void zsa_wireless_query_ble_status(void);
void zsa_wireless_query_battery(void);
void zsa_wireless_cmd_hibernate(void);
void zsa_wireless_cmd_pair(void);

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
