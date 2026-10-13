#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "zsa_uart_proto.h"
#include "zsa_right_telemetry.h"

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

/* Weak hook: keyboards override this to drive their own status LED pin. The
 * module owns the blink phase and only calls on a change. */
void zsa_wireless_pair_led(uint8_t state);

/* True while the module is driving the pairing status LED. Keyboards check
 * this before writing that pin for any other purpose. */
bool zsa_wireless_pair_led_active(void);

/* Weak hook: the firmware-update indication on the half's two status LEDs,
 * green and red (enum zsa_status_led_state each). Green fast-blinks while
 * this half's image is being written and stays solid while it settles; red is
 * solid on a failed update and slow-blinks on a protocol mismatch. The module
 * owns the blink phase and only calls on a change. Outranks the pairing blink
 * and the layer indicators — see the *_owned() accessors. */
void zsa_wireless_dfu_led(uint8_t green, uint8_t red);

/* True while the update indication drives that pin. Keyboards check these
 * before writing either pin for any other purpose. */
bool zsa_wireless_dfu_led_green_owned(void);
bool zsa_wireless_dfu_led_red_owned(void);

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
/* Hand over to the next remembered host. Sent by BLE_SWITCH_HOST. */
void zsa_wireless_cmd_ble_switch_host(void);

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
