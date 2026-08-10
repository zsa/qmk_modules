#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "zsa_uart_proto.h"
#include "zsa_right_telemetry.h"

enum zsa_idle_step {
    ZSA_IDLE_ACTIVE    = 0,
    ZSA_IDLE_DIM       = 1,
    ZSA_IDLE_LEDS_OFF  = 2,  /* LEDs off + slow matrix scan */
    ZSA_IDLE_DEEP      = 3,
};

/* Cached nRF state, updated via UART events */
bool    zsa_wireless_ble_connected(void);
/* True while the nRF is discoverable, i.e. advertising as general-discoverable
 * so a new host can find and pair with it. Driven by ZSA_MSG_EVT_PAIR_STATE
 * (ACTIVE). */
bool    zsa_wireless_discovery_active(void);
/* True while an SMP pairing is actually running (ZSA_MSG_EVT_PAIR_STATE /
 * ZSA_PAIR_CONFIRM). Mutually exclusive with discovery: the nRF reports CONFIRM
 * in preference to ACTIVE. */
bool    zsa_wireless_pairing_active(void);
uint8_t zsa_wireless_battery_soc(void);
/* Right half SOC, relayed by the left nRF over the inter-half link. 0 when the
 * right half has never reported (e.g. powered down or not yet linked). */
uint8_t zsa_wireless_battery_right_soc(void);
uint8_t zsa_wireless_battery_flags(void);
/* True while the nRF's battery report is recent enough to act on. The nRF
 * re-sends the level every second, so a lapse means the link is gone and the
 * cached soc/flags are last week's news. Without this a boot-time 0 and a flat
 * battery are the same byte. */
bool zsa_wireless_battery_fresh(void);
/* The right half's own reading, taken over the TRRS I2C bus rather than
 * relayed. Fills @p out and returns true only while the cable is plugged and
 * the last read succeeded; false means fall back to the relayed value, or to
 * nothing. See zsa_right_telemetry.h for why this path exists at all. */
bool zsa_wireless_right_telemetry(struct zsa_right_telemetry *out);
/* Detected USB source current budget in mA (EVT_PWR_BUDGET): what the attached
 * source advertised over CC. ZSA_PWR_BUDGET_UNKNOWN_MA until one says — no
 * source attached, or the link has not reported yet — which is deliberately
 * not the same as the 500 mA default class. */
uint16_t zsa_wireless_power_budget_ma(void);
/* True while the nRF reports VBUS present (EVT_VBUS_STATE), false at boot
 * and once VBUS goes away. */
bool zsa_wireless_vbus_present(void);

/* True while a firmware update is running through the nRF (EVT_DFU_STATE),
 * on either half. The module makes the keyboard inert for the duration: key
 * events dropped in pre_process_record, pointer reports dropped, whole matrix
 * painted blue. The nRF re-sends the level every second and the module ages
 * it out after ZSA_DFU_STATE_STALE_MS, so a link lost mid-update recovers. */
bool zsa_wireless_dfu_active(void);

/* Weak hook: keyboards override this to react to VBUS present/absent
 * (EVT_VBUS_STATE), e.g. to drive their own USB D+ pullup. */
void zsa_wireless_vbus_changed(bool present);

/* Weak hook: reboot into the keyboard's bootloader, invoked on
 * ZSA_MSG_DFU_ENTER so a host can start an STM32 update over BLE. Does not
 * return on a keyboard that implements it; the default is a no-op. */
void zsa_wireless_enter_bootloader(void);

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
/* Enter pairing mode: the nRF advertises as general-discoverable for its
 * pairing window. Sent by BLE_PAIR_MODE. */
void zsa_wireless_cmd_pair(void);
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
/* True once an EVT_NAV_PRESENT has arrived, so 0 can be read as "nothing
 * plugged in" rather than "never asked". */
bool zsa_wireless_nav_device_seen(void);

/* What the compiled-in nav driver found the last time it probed. The driver
 * tries the real device address first — a chip on the STM32's own I2C bus —
 * and only then the left nRF's mirror address, which exists solely to relay a
 * right-plugged device while wireless. Which one answered is the only
 * side-of-the-board signal QMK has.
 *
 * Note the ambiguity it does not resolve: the TRRS segments do not isolate, so
 * a right-plugged device is on the same net and answers at the same real
 * address whenever the cable is plugged. LOCAL means "on our bus", not "on the
 * left half". */
enum zsa_nav_probe {
    ZSA_NAV_PROBE_UNSUPPORTED = 0, /* no nav driver in this build: nothing looked */
    ZSA_NAV_PROBE_NONE,            /* looked, neither address answered */
    ZSA_NAV_PROBE_LOCAL,           /* the real device address answered */
    ZSA_NAV_PROBE_MIRROR,          /* only the left-nRF mirror answered */
};

/* Weak hook: the nav module compiled into this build overrides it. Returns
 * enum zsa_nav_probe. */
uint8_t zsa_nav_probe_state(void);
/* Which device that driver drives (enum zsa_nav_device_type). Constant for a
 * build — only one nav module ever links at a time. */
uint8_t zsa_nav_probe_device(void);
