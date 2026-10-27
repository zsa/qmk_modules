#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "zsa_uart_proto.h"
#include "zsa_right_telemetry.h"
#include "nav_presence.h"

/* ZSA_MSG_CMD_IDLE_STATE values. Only ACTIVE and SLEEP are sent; DIM and DEEP
 * are kept for the wire and are never sent by this module. */
enum zsa_idle_step {
    ZSA_IDLE_ACTIVE    = 0,
    ZSA_IDLE_DIM       = 1,
    ZSA_IDLE_SLEEP     = 2,  /* LEDs off, STM32 in STOP when wireless */
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
 * source advertised over CC. ZSA_PWR_BUDGET_DEFAULT_MA until the nRF says
 * otherwise, which is also what it reports with no source attached. */
uint16_t zsa_wireless_power_budget_ma(void);
/* True while the nRF reports VBUS present (EVT_VBUS_STATE), false at boot
 * and once VBUS goes away. */
bool zsa_wireless_vbus_present(void);

#ifdef RGB_MATRIX_ENABLE
/* Effective RGB value to scale LED output by: the stored setting
 * (rgb_matrix_config.hsv.v), clamped to ZSA_WIRED_LED_VAL_CAP while VBUS is
 * present. Full brightness on the cable starves the right half's charger.
 * The stored setting itself is never written. */
uint8_t zsa_wireless_led_val(void);
#endif

/* True while a firmware update is running through the nRF (EVT_DFU_STATE),
 * on either half. The module makes the keyboard inert for the duration: key
 * events dropped in pre_process_record, pointer reports dropped, idle steps
 * held off. The nRF re-sends the level every second and the module ages it
 * out after ZSA_DFU_STATE_STALE_MS, so a link lost mid-update recovers. */
bool zsa_wireless_dfu_active(void);

/* True while the two nRF halves report mismatched keyboard-protocol versions
 * (EVT_LINK_STATE / ZSA_LINK_STATE_MISMATCH). Freshness-gated like the pair
 * state: the nRF re-sends the level every second. */
bool zsa_wireless_link_mismatch(void);

/* Weak hook: keyboards override this to react to VBUS present/absent
 * (EVT_VBUS_STATE), e.g. to drive their own USB D+ pullup. */
void zsa_wireless_vbus_changed(bool present);

/* Weak hook: reboot into the keyboard's bootloader, invoked on
 * ZSA_MSG_DFU_ENTER so a host can start an STM32 update over BLE. Does not
 * return on a keyboard that implements it; the default is a no-op. */
void zsa_wireless_enter_bootloader(void);

/* State of a driven status LED. RELEASE hands the pin back to whatever else
 * drives it. */
enum zsa_status_led_state {
    ZSA_STATUS_LED_RELEASE = 0,
    ZSA_STATUS_LED_OFF     = 1,
    ZSA_STATUS_LED_ON      = 2,
};

/* Weak hook: the firmware-update indication on the half's two status LEDs,
 * green and red (enum zsa_status_led_state each). Green fast-blinks while
 * this half's image is being written and stays solid while it settles; red is
 * solid on a failed update and slow-blinks on a protocol mismatch. The module
 * owns the blink phase and only calls on a change. Outranks the layer
 * indicators — see the *_owned() accessors. */
void zsa_wireless_dfu_led(uint8_t green, uint8_t red);

/* True while the update indication drives that pin. Keyboards check these
 * before writing either pin for any other purpose. */
bool zsa_wireless_dfu_led_green_owned(void);
bool zsa_wireless_dfu_led_red_owned(void);

/* Send a query/command to the nRF */
void zsa_wireless_query_ble_status(void);
void zsa_wireless_query_battery(void);
void zsa_wireless_cmd_hibernate(void);
/* Drop the active BLE link or stop advertising, keeping the bond
 * (CMD_BLE_BANK DISCONNECT). Sent by BLE_DISCONNECT. */
void zsa_wireless_cmd_ble_disconnect(void);

/* ---- Bluetooth bank menu (BLE_MENU) ----
 *
 * Open: the matrix shows banks 0-4 on the key columns of ZSA_BLE_MENU_KEYS
 * and every other LED is dark. An unknown bank lights its number key in gray,
 * any other bank its whole column. A tap
 * on any key of a bank pairs an empty bank, connects a stored one and
 * disconnects an active one; a ZSA_BLE_FORGET_HOLD_MS hold forgets it. Further
 * keys of a bank pressed during its hold are ignored. Any other key closes the
 * menu. Every
 * key event while open is swallowed, and so is the release of a key pressed
 * while open. BLE_MENU does nothing with a USB host, and the menu closes when
 * one appears. */

/* What a bank shows. UNKNOWN: no fresh EVT_BLE_BANKS. */
enum zsa_ble_bank_view {
    ZSA_BLE_BANK_VIEW_UNKNOWN = 0,
    ZSA_BLE_BANK_VIEW_EMPTY,      /* #28a5cf (ZSA_BLE_MENU_EMPTY_R/G/B) */
    ZSA_BLE_BANK_VIEW_STORED,     /* blue */
    ZSA_BLE_BANK_VIEW_CONNECTING, /* blinking green */
    ZSA_BLE_BANK_VIEW_PAIRING,    /* blinking blue */
    ZSA_BLE_BANK_VIEW_CONNECTED,  /* green */
};

bool    zsa_wireless_ble_menu_open(void);
/* {row, col} of each bank's keys, from ZSA_BLE_MENU_KEYS. */
extern const uint8_t zsa_ble_menu_keys[ZSA_BLE_BANK_COUNT][ZSA_BLE_MENU_BANK_KEYS][2];
/* enum zsa_ble_bank_view of bank (< ZSA_BLE_BANK_COUNT). With the BLE stack
 * off (ZSA_BLE_BANK_STATE_OFF) only EMPTY and STORED are reported. */
uint8_t zsa_wireless_ble_bank_view(uint8_t bank);

/* ---- Diagnostic view (hold BLE_MENU) ----
 *
 * Holding BLE_MENU for ZSA_DIAG_HOLD_MS closes the bank menu and lights the
 * ZSA_DIAG_KEYS indicators, every other LED dark. It works with or without a
 * USB host. Any key press closes it and is swallowed, as is that key's
 * release; it also closes after ZSA_DIAG_VIEW_TIMEOUT_MS. */

/* Indicators, in ZSA_DIAG_KEYS order. */
enum zsa_diag_key {
    ZSA_DIAG_KEY_LINK = 0,     /* LINK_UP handshake with the left nRF done */
    ZSA_DIAG_KEY_RX_FRESH,     /* a frame from the left nRF within ZSA_DIAG_RX_FRESH_MS */
    ZSA_DIAG_KEY_NRF_ANSWERS,  /* the left nRF answered a query since the view opened */
    ZSA_DIAG_KEY_SERIAL_OK,    /* no STM32 UART errors this boot */
    ZSA_DIAG_KEY_FRAMES_OK,    /* no dropped or cut-short frames this boot */
    ZSA_DIAG_KEY_VBUS,         /* the left nRF reports USB power */
    ZSA_DIAG_KEY_USB_HOST,     /* a USB host enumerated the keyboard */
    ZSA_DIAG_KEY_TRANSPORT,    /* reports go to USB, Bluetooth, or not decided */
    ZSA_DIAG_KEY_POWER_GATE,   /* boot power verdict */
    ZSA_DIAG_KEY_NO_WATCHDOG,  /* this boot was not a watchdog reset */
    ZSA_DIAG_KEY_BLE,          /* the left nRF reports a Bluetooth host */
    ZSA_DIAG_KEY_BATTERY,      /* battery levels are fresh */
    ZSA_DIAG_KEY_RIGHT_LINK,   /* the right half's radio link */
    ZSA_DIAG_KEY_RIGHT_LED,    /* the right half's LED driver */
    ZSA_DIAG_KEY_AGE_1S,       /* last left nRF frame: < 1 s */
    ZSA_DIAG_KEY_AGE_5S,       /* < 5 s */
    ZSA_DIAG_KEY_AGE_30S,      /* < 30 s */
    ZSA_DIAG_KEY_AGE_5MIN,     /* < 5 min */
    ZSA_DIAG_KEY_AGE_OLDER,    /* 5 min or more */
    ZSA_DIAG_KEY_COUNT,
};

enum zsa_diag_color {
    ZSA_DIAG_OFF = 0,
    ZSA_DIAG_GREEN,
    ZSA_DIAG_RED,
    ZSA_DIAG_UNKNOWN, /* dim gray */
    ZSA_DIAG_BLUE,
    ZSA_DIAG_YELLOW,
    ZSA_DIAG_ORANGE,
    ZSA_DIAG_NA, /* does not apply in this mode; shown as orange */
};

/* {row, col} of each indicator, from ZSA_DIAG_KEYS. */
extern const uint8_t zsa_diag_keys[ZSA_DIAG_KEY_COUNT][2];

bool zsa_wireless_diag_view_open(void);
/* enum zsa_diag_color of each indicator. */
void zsa_wireless_diag_colors(uint8_t colors[ZSA_DIAG_KEY_COUNT]);

/* Power stage: ZSA_IDLE_ACTIVE or ZSA_IDLE_SLEEP. */
enum zsa_idle_step zsa_wireless_idle_step(void);

/* Keyboards override this to blank their status LEDs on entering Sleep and
 * repaint them on leaving it. */
void zsa_wireless_sleep_changed(bool sleeping);

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

/* enum zsa_nav_probe: nav_presence.h. */

/* Weak hooks, one per nav module; each module compiled into the build
 * overrides its own. Return enum zsa_nav_probe. */
uint8_t zsa_nav_trackball_probe_state(void);
uint8_t zsa_nav_trackpad_probe_state(void);

/* Left nav probe window, weak no-ops by default, one pair per nav module.
 * probe_local: if the device is absent, probe its real address next; if it is
 * up on the mirror, move it to the real address when that answers.
 * local_pending: the real address answered and the device is not up yet. */
void zsa_nav_trackball_probe_local(void);
void zsa_nav_trackpad_probe_local(void);
bool zsa_nav_trackball_local_pending(void);
bool zsa_nav_trackpad_local_pending(void);

/* ---- Left nav device in Sleep ----
 *
 * In Sleep the STM32 stops (STOP mode) between wakes. A nav device answering
 * at its real address on the STM32's own bus (ZSA_NAV_PROBE_LOCAL) cannot wake
 * it, so STOP is allowed with one only if its module provides these hooks;
 * without them (weak default: NULL) the STM32 stays running, as before.
 *
 * enter: Sleep with the device local, before the first STOP. Normal context,
 *   regular i2c_* calls. Prepares the device to be polled (e.g. keeps its own
 *   sampling fast). false: cannot, the STM32 stays running for this Sleep.
 * idle: before every STOP. false while the normal read path still has data to
 *   pick up; the STM32 then stays running for that pass.
 * poll: on every RTC wake inside STOP (every 16 ms), interrupts locked, core on
 *   the 8 MHz HSI. Only zsa_stop_i2c_write/read and zsa_stop_delay_us may be
 *   used. Must not drop input: data it reads is kept for the normal read path.
 *   Returns enum zsa_nav_sleep_poll.
 * exit: leaving Sleep, or the device no longer local. Normal context. Undoes
 *   enter. Must cope with the device being gone. */
enum zsa_nav_sleep_poll {
    ZSA_NAV_SLEEP_IDLE = 0, /* nothing happened */
    ZSA_NAV_SLEEP_ACTIVITY, /* motion or touch: the board wakes */
    ZSA_NAV_SLEEP_ERROR,    /* bus error: the STM32 wakes, the normal path checks */
};

typedef struct {
    bool (*enter)(void);
    bool (*idle)(void);
    uint8_t (*poll)(void);
    void (*exit)(void);
} zsa_nav_sleep_ops_t;

/* Weak hooks, one per nav module, returning NULL by default. */
const zsa_nav_sleep_ops_t *zsa_nav_trackball_sleep_ops(void);
const zsa_nav_sleep_ops_t *zsa_nav_trackpad_sleep_ops(void);

/* I2C1 and delay for zsa_nav_sleep_ops_t.poll only: polled, no DMA or
 * interrupts, 8-bit (shifted) addresses, one transfer ending in STOP, len >= 1.
 * Fail outside a poll. */
bool zsa_stop_i2c_write(uint8_t addr, const uint8_t *data, uint8_t len);
bool zsa_stop_i2c_read(uint8_t addr, uint8_t *data, uint8_t len);
void zsa_stop_delay_us(uint16_t us);

/* Wake filter (ZSA_NAV_SLEEP_WAKE_COUNTS), for poll only: called with the
 * deltas of a poll that saw motion. ZSA_NAV_SLEEP_ACTIVITY: wake, *wake_dx and
 * *wake_dy hold the counts of this poll and the held ones, to deliver.
 * ZSA_NAV_SLEEP_IDLE: the counts are held, and dropped at the next poll that
 * does not call this or at the next STOP. */
uint8_t zsa_nav_sleep_filter(int16_t dx, int16_t dy, int32_t *wake_dx, int32_t *wake_dy);

/* Sleep diagnostics (ZSA_SLEEP_DIAG): the last poll's raw bytes, up to 6, and
 * a motion read by the normal path. Safe from a poll. */
void zsa_nav_sleep_note_raw(const uint8_t *raw, uint8_t len);
void zsa_nav_sleep_note_motion(int16_t x, int16_t y);
