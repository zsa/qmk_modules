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

/* Navigator event hooks. The wireless module accumulates motion deltas and
 * tracks the currently-detected device type from EVT_NAV_PRESENT events.
 *
 * navigator_wireless (the pointing_device driver) consumes these — call
 * consume_motion every pointing_device_task to drain the accumulator. */
uint8_t zsa_wireless_nav_device_type(void);  /* enum zsa_nav_device_type, 0 if none */
void    zsa_wireless_nav_consume_motion(int16_t *dx, int16_t *dy);

/* True if a fresh raw nav report (17 B) has arrived from the nRF since the
 * previous call. On true, *out is filled and the freshness flag is cleared. */
bool zsa_wireless_nav_consume_raw(uint8_t out[17]);
