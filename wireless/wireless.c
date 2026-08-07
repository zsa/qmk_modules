/*
 * ZSA Wireless module — implements bluetooth.h weak functions
 * to route HID reports over UART to the nRF54 wireless chip.
 *
 * Runtime switching between USB and wireless is handled by QMK's
 * CONNECTION_ENABLE subsystem (connection.c / host.c).
 */

#include <stddef.h>
#include "quantum.h"
#include "wireless.h"
#include "status_layer.h"
#include "bluetooth.h"
#include "report.h"
#include "uart.h"
#include "usb_util.h"
#include "keyboard.h"
#include "timer.h"
#include "wait.h"
#include "connection.h"
#include "matrix.h"
/* log_tap.h resolves zsa_log()/zsa_log_tap_init() to real symbols under
 * ZSA_BLE_LOG and to no-op static inlines otherwise, so include it always. */
#include "log_tap.h"
/* rtt_debug.h resolves to no-op static inlines unless ZSA_RTT_DEBUG is set. */
#include "rtt_debug.h"
#ifdef RGB_MATRIX_ENABLE
#    include "rgb_matrix.h"
#    include "is31fl3731.h"
#endif

/* keyboard.c — not in keyboard.h but we need it to reset the idle timer */
void last_matrix_activity_trigger(void);

/* ---- Link state ---- */

static bool     link_ready = false;
static uint32_t last_link_up_ms = 0;
#define LINK_UP_INTERVAL_MS 100

/* ---- Cached nRF state ---- */

static bool     nrf_ble_connected = true;
static uint8_t  nrf_ble_device_idx;
static bool     nrf_discovery_active; /* nRF discoverable (pairing mode) */
static uint8_t  nrf_pair_state;       /* enum zsa_pair_state_val, raw */
static uint32_t last_pair_state_ms;
static bool     pair_state_seen;      /* any EVT_PAIR_STATE received yet */
static uint8_t  nrf_battery_soc;
static uint8_t  nrf_battery_right_soc;
static uint8_t  nrf_battery_flags;
static uint8_t  nrf_led_state;
static bool     nrf_vbus_present;

/* ---- Adaptive charging ---- *
 *
 * The left nRF reports the USB source's CC-detected current budget
 * (EVT_PWR_BUDGET); this module owns the charging policy: on a high-budget
 * source it commands both halves to raise their VBUS input current limit
 * to their devicetree value — the left over UART (CMD_VBUS_ILIM), the
 * right over the TRRS I2C bus (voyager_right_nrf_vbus_ilim_high()).
 * The raise is re-sent every PWR_BUDGET_MAINTAIN_INTERVAL_MS while the
 * budget stays high: both commands are idempotent, and the resend covers
 * right-half TRRS hot-plug, right VBUS arriving late, and nRF reboots.
 * There is no explicit lower command — the nPM1300s auto-revert to their
 * startup limit on VBUS removal. */
static uint16_t nrf_power_budget_ma = ZSA_PWR_BUDGET_DEFAULT_MA;
static uint32_t last_pwr_maintain_ms = 0;
#define PWR_BUDGET_MAINTAIN_INTERVAL_MS 5000

/* Board-provided (voyager_tng.c) right-half raise command; extern rather
 * than including the board header to keep the module decoupled. */
extern bool voyager_right_nrf_vbus_ilim_high(void);
extern bool is_transport_connected(void);
/* Bus ownership decision, made in the keyboard's matrix.c by probing the real
 * expander. LED addressing and the grant must follow the same signal. */
extern bool zsa_stm32_owns_right_bus(void);

#ifdef RGB_MATRIX_ENABLE
/* Tracks the last-reported left LED rail state so EVT_LED_RAIL only resyncs
 * the driver on the OFF->ON edge (see the handler below). Starts "on" since
 * the STM32 only ever boots with the rail already up. */
static bool left_led_rail_on = true;

/* Confirmed via drivers/led/issi/is31fl3731.c (i2c_addresses[] mirrors the
 * IS31FL3731_I2C_ADDRESS_N chain in order) and voyager_tng.c's
 * g_is31fl3731_leds[] table: driver 0 is the left half's real chip
 * (IS31FL3731_I2C_ADDRESS_1 / 0x74), driver 1 is the right half
 * (IS31FL3731_I2C_ADDRESS_2 / 0x77) - the left nRF's TWIS emulation when
 * wireless, the real chip on the TRRS bus when wired. */
#    define ZSA_LED_DRIVER_LEFT 0
#    define ZSA_LED_DRIVER_RIGHT 1

static uint32_t last_right_led_flush_ms = 0;

/* Point driver 1 at the real right-half chip while we own its bus, at the left
 * nRF's mirror otherwise. Must track the same signal the matrix uses, or we
 * would scan one device and light the other. */
static void update_right_led_address(void) {
    is31fl3731_set_i2c_address(ZSA_LED_DRIVER_RIGHT, zsa_stm32_owns_right_bus() ? IS31FL3731_I2C_ADDRESS_2 : ZSA_RIGHT_LED_MIRROR_ADDRESS);
}
#else
static inline void update_right_led_address(void) {}
#endif

static uint8_t nav_device_type;  /* enum zsa_nav_device_type */

#ifdef DIGITIZER_MODE_TOUCHPAD
/* ---- Digitizer/PTP report + input-mode routing ---- *
 *
 * navigator_trackpad_ptp.c defines navigator_trackpad_output_touchpad(),
 * navigator_trackpad_output_touchpad_mouse() and
 * navigator_trackpad_wireless_input_mode() as weak (default: straight to USB,
 * no mode override), so wired-only builds without this module (e.g.
 * test_trackpad) are unaffected. The strong definitions below override them
 * at link time whenever this module is also linked in (e.g. fdprod_trackpad),
 * routing PTP reports over UART and shadowing the BLE host's input-mode
 * writes whenever we're not on USB.
 */
extern void send_digitizer_touchpad(report_digitizer_touchpad_t *report);
extern void send_digitizer_touchpad_mouse(report_digitizer_touchpad_mouse_t *report);

/* Shadow input-mode state, set from ZSA_MSG_EVT_DIGITIZER_MODE (the BLE
 * host's HID feature-report write, relayed by the nRF). Only trusted while
 * actually wireless (not on USB) — see navigator_trackpad_wireless_input_mode()
 * below. */
static uint8_t digitizer_wireless_mode     = 0; /* mirrors TRACKPAD_INPUT_MODE_MOUSE */
static bool    digitizer_wireless_mode_set = false;
#endif

/* ---- Nav capability announce ---- *
 *
 * QMK's nav support is compile-time (which modules a keymap pulls in).
 * We announce the compiled-in capability bitmask to the nRF at link-up so
 * it only runs nav-device detection — and only exposes the emulated-I2C
 * mirrors (trackball: 0x28 real bridge / 0x2D left-nRF mirror; trackpad:
 * 0x2C real pad / 0x2E left-nRF mirror, all left-plugged real / right-plugged
 * wireless) — when this build actually supports a nav device. Retried every
 * 1 s until the nRF acks with any EVT_NAV_PRESENT. */
static const uint8_t nav_cap_mask =
    0
#ifdef POINTING_DEVICE_ENABLE
    | ZSA_NAV_CAP_TRACKBALL
#endif
#ifdef DIGITIZER_ENABLE
    | ZSA_NAV_CAP_TRACKPAD
#endif
    ;
static bool     nav_announce_acked  = false;
static uint32_t last_nav_announce_ms = 0;
#define NAV_ANNOUNCE_INTERVAL_MS 1000

/* ---- Parser instance ---- */

static struct zsa_parser rx_parser;

/* ---- Idle state (declared early for use in handle_rx_frame) ---- */

static enum zsa_idle_step idle_step       = ZSA_IDLE_ACTIVE;
static uint32_t           nrf_wake_time  = 0;
static bool               nrf_ship_mode  = false;

/* ---- Helpers ---- */

void zsa_wireless_send_frame(uint8_t type, const uint8_t *payload, uint16_t len) {
    /* Sized for the worst case: (ZSA_LOG_MAX_PAYLOAD + ZSA_FRAME_OVERHEAD) * 2 + 1
     * (every payload byte escapes). static is safe because QMK runs one cooperative
     * task for the keymap/uart path — no reentrancy. */
    static uint8_t buf[(ZSA_LOG_MAX_PAYLOAD + ZSA_FRAME_OVERHEAD) * 2 + 1];
    uint16_t frame_len = zsa_frame_encode(buf, type, payload, len);
    uint32_t rtt_t0    = zsa_rtt_cycles(); // TEMP DEBUG
    uart_transmit(buf, frame_len);
    zsa_rtt_tx(type, frame_len, rtt_t0);   // TEMP DEBUG
}

static void send_command(uint8_t type) {
    zsa_wireless_send_frame(type, NULL, 0);
}

static void idle_power_wake(void);
static void send_usb_state(bool connected);
static void send_idle_state(enum zsa_idle_step step);

/* Send all current STM32 state to nRF — called on link establishment */
static void send_full_state(void) {
    send_usb_state(usb_connected_state());
    send_idle_state(idle_step);
}

/* Announce compiled-in nav capabilities to the nRF once the link is up,
 * retrying every 1 s until acked with an EVT_NAV_PRESENT. Sent even when the
 * bitmask is 0 (tells the nRF to disable nav detection for this build). */
static void nav_announce_task(void) {
    if (!link_ready || nav_announce_acked) {
        return;
    }
    if (last_nav_announce_ms != 0 &&
        timer_elapsed32(last_nav_announce_ms) < NAV_ANNOUNCE_INTERVAL_MS) {
        return;
    }
    struct zsa_nav_enable en = { .enabled = nav_cap_mask };
    zsa_wireless_send_frame(ZSA_MSG_CMD_NAV_ENABLE, (const uint8_t *)&en, sizeof(en));
    last_nav_announce_ms = timer_read32();
}

/* Fail-safe gate for the indicators. The nRF refreshes the pair state every
 * second; a lapse means the link is gone, and a stale state must not leave the
 * LED lit. pair_state_seen covers boot, where last_pair_state_ms and
 * timer_read32() are both near 0. */
static bool pair_state_fresh(void) {
    return link_ready && pair_state_seen && timer_elapsed32(last_pair_state_ms) < ZSA_PAIR_STATE_STALE_MS;
}

/* Discoverable, freshness-gated. Distinct from zsa_wireless_discovery_active(),
 * which reports the raw last-known value. */
static bool pair_advertising(void) {
    return nrf_discovery_active && pair_state_fresh();
}

/* A pairing starting is worth showing: leave the idle steps and power the matrix
 * back up. Discoverability deliberately does not do this — it can run for two
 * minutes and would pin the LEDs on. */
static void pair_confirm_wake(void) {
    if (idle_step != ZSA_IDLE_ACTIVE) {
        idle_power_wake();
    }
#ifdef RGB_MATRIX_ENABLE
    rgb_matrix_enable_noeeprom();
#endif
}

static void handle_rx_frame(const struct zsa_frame *frame) {
    zsa_rtt_rx(frame->type); // TEMP DEBUG
    switch (frame->type) {
        case ZSA_MSG_LINK_UP:
            if (!link_ready) zsa_rtt_event("LINK_UP latched"); // TEMP DEBUG
            link_ready = true;
            /* Reply (rate-limited) so the nRF's link-probe loop latches even
             * when we were already up: after an nRF-only reboot it probes
             * LINK_UP every 100 ms and stops only on receiving ours — without
             * this reply the half-open link probes forever. */
            if (timer_elapsed32(last_link_up_ms) >= LINK_UP_INTERVAL_MS) {
                send_command(ZSA_MSG_LINK_UP);
                last_link_up_ms = timer_read32();
            }
            /* Link (re)established — the nRF may have rebooted, so re-announce
             * our nav capabilities immediately and wait for a fresh ack. */
            nav_announce_acked   = false;
            last_nav_announce_ms = 0;
            /* Same reasoning for the power budget: drop any stale "high"
             * until the nRF re-reports its real budget (sent right after
             * link-up). */
            nrf_power_budget_ma = ZSA_PWR_BUDGET_DEFAULT_MA;
            /* nrf_vbus_present is left alone: the nRF re-reports it right
             * after link-up via send_nrf_state(). */
            /* nRF rebooted (e.g. after ship mode) — clear ship flag */
            if (nrf_ship_mode) {
                nrf_ship_mode = false;
                nrf_wake_time = timer_read32();
                idle_power_wake();
            }
            send_full_state();
            break;

        case ZSA_MSG_EVT_BLE_STATUS:
            if (frame->len >= sizeof(struct zsa_ble_status)) {
                const struct zsa_ble_status *s = (const struct zsa_ble_status *)frame->payload;
                zsa_rtt_event(s->connected ? "BLE_STATUS connected" : "BLE_STATUS disconnected"); // TEMP DEBUG
                nrf_ble_connected  = s->connected != 0;
                nrf_ble_device_idx = s->device_idx;
                /* nRF is back from ship mode */
                nrf_ship_mode = false;
                nrf_wake_time = timer_read32();
                if (idle_step != ZSA_IDLE_ACTIVE) {
                    idle_power_wake();
                }
            }
            break;

        case ZSA_MSG_EVT_BATTERY:
            if (frame->len >= sizeof(struct zsa_battery_status)) {
                const struct zsa_battery_status *b = (const struct zsa_battery_status *)frame->payload;
                nrf_battery_soc       = b->soc;
                nrf_battery_right_soc = b->right_soc;
                nrf_battery_flags     = b->flags;
            }
            break;

        case ZSA_MSG_EVT_PAIR_STATE:
            if (frame->len >= sizeof(struct zsa_pair_state)) {
                const struct zsa_pair_state *p = (const struct zsa_pair_state *)frame->payload;
                /* ACTIVE = discoverable (pairing mode advertising). */
                nrf_discovery_active = (p->state == ZSA_PAIR_ACTIVE);
                /* CONFIRM = an SMP pairing is running. */
                if (p->state == ZSA_PAIR_CONFIRM && nrf_pair_state != ZSA_PAIR_CONFIRM) {
                    pair_confirm_wake();
                }
                nrf_pair_state     = p->state;
                last_pair_state_ms = timer_read32();
                pair_state_seen    = true;
            }
            break;

        case ZSA_MSG_EVT_LED_STATE:
            if (frame->len >= sizeof(struct zsa_led_state)) {
                const struct zsa_led_state *l = (const struct zsa_led_state *)frame->payload;
                nrf_led_state = l->leds;
            }
            break;

        case ZSA_MSG_EVT_LED_RAIL:
            if (frame->len >= sizeof(struct zsa_led_rail_state)) {
                const struct zsa_led_rail_state *r = (const struct zsa_led_rail_state *)frame->payload;
#ifdef RGB_MATRIX_ENABLE
                bool rail_on = (r->rails & ZSA_LED_RAIL_LEFT) != 0;
                // The nRF also reports this on unrelated transitions (usb_connected,
                // idle_step), so only resync on the OFF->ON edge - rail power actually
                // returning after LEDS_OFF idle. A repeated "on" report is a no-op.
                if (rail_on && !left_led_rail_on && idle_step < ZSA_IDLE_LEDS_OFF) {
                    zsa_rtt_event("LED_RAIL -> rgb_matrix_init"); // TEMP DEBUG
                    rgb_matrix_init();
                    // rgb_matrix_init() re-zeroes hardware PWM but not the driver's
                    // software cache, so set_color() early-returns until a color
                    // actually changes. Zero the cache too so it matches hardware and
                    // the next rendered frame repaints every LED.
                    rgb_matrix_set_color_all(0, 0, 0);
                    rgb_matrix_driver.flush();
                }
                left_led_rail_on = rail_on;
#endif
            }
            break;

        case ZSA_MSG_QUERY_USB_STATE:
            send_usb_state(usb_connected_state());
            break;

        case ZSA_MSG_EVT_NAV_PRESENT:
            /* The nRF acks a capability announce with the currently detected
             * device type (possibly 0=none) and re-sends on any presence change. */
            nav_announce_acked = true;
            if (frame->len >= sizeof(struct zsa_nav_present)) {
                const struct zsa_nav_present *p = (const struct zsa_nav_present *)frame->payload;
                nav_device_type = p->device_type;
            }
            break;

#ifdef DIGITIZER_MODE_TOUCHPAD
        case ZSA_MSG_EVT_DIGITIZER_MODE:
            if (frame->len >= sizeof(struct zsa_digitizer_mode)) {
                const struct zsa_digitizer_mode *m = (const struct zsa_digitizer_mode *)frame->payload;
                digitizer_wireless_mode     = m->mode;
                digitizer_wireless_mode_set = true;
            }
            break;
#endif

        case ZSA_MSG_EVT_PWR_BUDGET:
            if (frame->len >= sizeof(struct zsa_pwr_budget)) {
                const struct zsa_pwr_budget *p = (const struct zsa_pwr_budget *)frame->payload;
                nrf_power_budget_ma  = p->budget_ma;
                /* Act on the next loop iteration, not the next 5 s tick. */
                last_pwr_maintain_ms = 0;
            }
            break;

        case ZSA_MSG_EVT_VBUS_STATE:
            if (frame->len >= sizeof(struct zsa_vbus_state)) {
                const struct zsa_vbus_state *v = (const struct zsa_vbus_state *)frame->payload;
                bool present = v->present != 0;
                if (present != nrf_vbus_present) {
                    nrf_vbus_present = present;
                    zsa_wireless_vbus_changed(present);
                }
            }
            break;

        default:
            break;
    }
}

/* ---- Public API (wireless.h) ---- */

bool zsa_wireless_ble_connected(void) {
    return nrf_ble_connected;
}

bool zsa_wireless_discovery_active(void) {
    return nrf_discovery_active;
}

bool zsa_wireless_pairing_active(void) {
    return nrf_pair_state == ZSA_PAIR_CONFIRM && pair_state_fresh();
}

uint8_t zsa_wireless_battery_soc(void) {
    return nrf_battery_soc;
}

uint8_t zsa_wireless_battery_right_soc(void) {
    return nrf_battery_right_soc;
}

uint8_t zsa_wireless_battery_flags(void) {
    return nrf_battery_flags;
}

uint16_t zsa_wireless_power_budget_ma(void) {
    return nrf_power_budget_ma;
}

bool zsa_wireless_vbus_present(void) {
    return nrf_vbus_present;
}

void zsa_wireless_query_ble_status(void) {
    send_command(ZSA_MSG_QUERY_BLE_STATUS);
}

void zsa_wireless_query_battery(void) {
    send_command(ZSA_MSG_QUERY_BATTERY);
}

void zsa_wireless_cmd_hibernate(void) {
    send_command(ZSA_MSG_CMD_HIBERNATE);
}

void zsa_wireless_cmd_pair(void) {
    send_command(ZSA_MSG_CMD_PAIR);
}

void zsa_wireless_cmd_ble_disconnect(void) {
    send_command(ZSA_MSG_CMD_BLE_DISCONNECT);
}

void zsa_wireless_cmd_ble_unpair(void) {
    send_command(ZSA_MSG_CMD_BLE_UNPAIR);
}

bool zsa_wireless_link_ready(void) {
    return link_ready;
}

uint8_t zsa_wireless_nav_device_type(void) {
    return nav_device_type;
}

/* ---- Idle power management ---- */

enum zsa_idle_step zsa_wireless_idle_step(void) {
    return idle_step;
}

static void send_idle_state(enum zsa_idle_step step) {
    struct zsa_idle_state payload = { .step = (uint8_t)step };
    zsa_wireless_send_frame(ZSA_MSG_CMD_IDLE_STATE, (const uint8_t *)&payload, sizeof(payload));
}

static void send_usb_state(bool connected) {
    struct zsa_usb_state payload = { .connected = connected ? 1 : 0 };
    zsa_wireless_send_frame(ZSA_MSG_CMD_USB_STATE, (const uint8_t *)&payload, sizeof(payload));
}

/* Bus lease renewal. Resent unconditionally rather than on change: the right
 * half releases the bus when grants stop arriving, so a dropped frame costs a
 * brief stall, never two masters. Also re-syncs a left half that rebooted. */
static void bus_grant_task(void) {
    static uint32_t last_sent_ms;
    static bool     last_granted;
    static bool     primed;

    bool granted = !zsa_stm32_owns_right_bus();

    if (primed && granted == last_granted && timer_elapsed32(last_sent_ms) < ZSA_BUS_GRANT_INTERVAL_MS) {
        return;
    }
    if (granted != last_granted || !primed) {
        update_right_led_address();
    }

    struct zsa_bus_grant payload = { .granted = granted ? 1 : 0 };
    zsa_wireless_send_frame(ZSA_MSG_CMD_BUS_GRANT, (const uint8_t *)&payload, sizeof(payload));
    last_sent_ms = timer_read32();
    last_granted = granted;
    primed       = true;
}

static void idle_power_enter_step(enum zsa_idle_step step) {
#ifdef RGB_MATRIX_ENABLE
    if (step >= ZSA_IDLE_LEDS_OFF && idle_step < ZSA_IDLE_LEDS_OFF) {
        rgb_matrix_disable_noeeprom();
        /* Force LEDs off immediately — rgb_matrix_task() won't run
         * inside the slow-scan loop to pick up the disable flag. */
        rgb_matrix_set_color_all(0, 0, 0);
        rgb_matrix_driver.flush();
    }
#endif
    zsa_log("idle: step %u -> %u", (unsigned)idle_step, (unsigned)step);
    idle_step = step;
    if (step == ZSA_IDLE_DEEP) {
        nrf_ship_mode = true;
    }
    send_idle_state(step);
}

static void idle_power_wake(void) {
    /* Note: when waking from LEDS_OFF, the LED driver (IS31FL3731)
     * is reprogrammed via rgb_matrix_init() when the nRF sends
     * EVT_LED_RAIL after re-enabling BUCK1. */
    zsa_log("idle: wake from step %u", (unsigned)idle_step);
    idle_step = ZSA_IDLE_ACTIVE;
    send_idle_state(ZSA_IDLE_ACTIVE);

    /* Reset the activity timer so idle_power_task() doesn't
     * immediately re-enter the idle state.  The slow-scan loop
     * calls matrix_scan() directly, which consumes the key-change
     * before matrix_task() can see it. */
    last_matrix_activity_trigger();
}

/* Called on both USB<->wireless transitions. Nav modules override this to
 * re-init their sensor: the transport switch resets the shared i2c bus, which
 * leaves the sensor unconfigured without raising an i2c error. */
__attribute__((weak)) void zsa_nav_transport_changed(void) {}

/* Keyboards override this to drive their own USB D+ pullup from VBUS state. */
__attribute__((weak)) void zsa_wireless_vbus_changed(bool present) { (void)present; }

/* ---- Pairing status LED ---- */

/* Keyboards override this to drive their own status LED pin. */
__attribute__((weak)) void zsa_wireless_pair_led(uint8_t state) { (void)state; }

static bool    pair_led_owned;
static uint8_t pair_led_last = ZSA_PAIR_LED_RELEASE;

bool zsa_wireless_pair_led_active(void) {
    return pair_led_owned;
}

/* Blinks the status LED: fast while a pairing is in progress, slow while
 * discoverable, released otherwise. The phase comes from the absolute clock
 * rather than elapsed-time toggling, so the blink rate is independent of how
 * often this runs — it is called both from bluetooth_task() and from inside
 * the idle slow-scan loop. */
static void pair_led_task(void) {
    bool    pairing = zsa_wireless_pairing_active();
    bool    advert  = pair_advertising();
    uint8_t want;

    if (pairing || advert) {
        uint32_t half  = pairing ? ZSA_PAIR_LED_FAST_MS : ZSA_PAIR_LED_SLOW_MS;
        want           = ((timer_read32() / half) & 1u) ? ZSA_PAIR_LED_ON : ZSA_PAIR_LED_OFF;
        pair_led_owned = true;
    } else {
        want           = ZSA_PAIR_LED_RELEASE;
        pair_led_owned = false;
    }

    if (want != pair_led_last) {
        pair_led_last = want;
        zsa_wireless_pair_led(want);
    }
}

static void idle_power_task(void) {
    static bool     was_bluetooth  = false;
    static uint32_t bt_switch_time = 0;

    /* Only manage idle power when running wirelessly */
    if (usb_connected_state()) {
        if (was_bluetooth) {
            was_bluetooth = false;
            send_usb_state(true);
            zsa_nav_transport_changed();
        }
        nrf_ship_mode = false;
        if (idle_step != ZSA_IDLE_ACTIVE) {
            idle_power_wake();
        }
        return;
    }

    /* Just switched to bluetooth — reset idle baseline */
    if (!was_bluetooth) {
        was_bluetooth  = true;
        bt_switch_time = timer_read32();
        send_usb_state(false);
        zsa_nav_transport_changed();
        if (idle_step != ZSA_IDLE_ACTIVE) {
            idle_power_wake();
        }
        return;
    }

#ifdef ZSA_DEV_NO_SLEEP
    /* Dev toggle: keep the USB/BT state sync above (the nRF relies on those
     * edges) but never step into DIM/LEDS_OFF/DEEP — the board stays fully
     * awake so debug sessions (SWD, log captures) aren't interrupted. */
    return;
#endif

    /* nRF is in ship mode — stay in DEEP, don't wake on keypresses.
     * Wake only happens via EVT_BLE_STATUS from nRF (handle_rx_frame). */
    if (nrf_ship_mode) {
        return;
    }

    /* Use the most recent of: last input, bluetooth switch, or nRF wake */
    uint32_t since_input  = last_input_activity_elapsed();
    uint32_t since_switch = timer_elapsed32(bt_switch_time);
    uint32_t since_nrf    = timer_elapsed32(nrf_wake_time);
    uint32_t idle_ms      = since_input;
    if (since_switch < idle_ms) idle_ms = since_switch;
    if (since_nrf    < idle_ms) idle_ms = since_nrf;

    enum zsa_idle_step target = ZSA_IDLE_ACTIVE;
    if      (idle_ms > ZSA_IDLE_DEEP_MS)     target = ZSA_IDLE_DEEP;
    else if (idle_ms > ZSA_IDLE_LEDS_OFF_MS) target = ZSA_IDLE_LEDS_OFF;
    else if (idle_ms > ZSA_IDLE_DIM_MS)      target = ZSA_IDLE_DIM;

    if (target != idle_step) {
        if (target > idle_step) {
            idle_power_enter_step(target);
        } else {
            idle_power_wake();
        }
    }

    /* Slow-scan loop: skip the full QMK task pipeline and only
     * scan the matrix + service UART until activity is detected.
     * matrix_task() keeps its own matrix_previous[], so the
     * keystroke that wakes us will be processed on the next
     * keyboard_task() iteration after we return. */
    while (idle_step >= ZSA_IDLE_LEDS_OFF) {
        wait_ms(ZSA_IDLE_SCAN_DELAY_MS);

        /* Service UART so nRF events (e.g. BLE wake) can reach us */
        while (uart_available()) {
            uint8_t byte = uart_read();
            if (zsa_parser_feed(&rx_parser, byte)) {
                handle_rx_frame(&rx_parser.frame);
            }
        }

        /* This loop never returns to the QMK task pipeline, so drive the
         * status LED from here too. */
        pair_led_task();

        /* handle_rx_frame may have woken us (EVT_BLE_STATUS, or a
         * EVT_PAIR_STATE carrying CONFIRM) */
        if (idle_step < ZSA_IDLE_LEDS_OFF) {
            break;
        }

        /* Transition to DEEP if enough time has passed */
        if (idle_step < ZSA_IDLE_DEEP &&
            last_input_activity_elapsed() > ZSA_IDLE_DEEP_MS) {
            idle_power_enter_step(ZSA_IDLE_DEEP);
            /* nrf_ship_mode is now true — stop scanning */
            break;
        }

        /* Scan matrix — if a key changed, wake and let the
         * normal QMK loop process the keypress. */
        if (matrix_scan()) {
            idle_power_wake();
            break;
        }
    }
}

#ifdef RGB_MATRIX_ENABLE
/* ---- LED flush rate limiting ---- */

/* Strong override of is31fl3731.c's weak is31fl3731_should_flush() hook.
 * Wired is exempt unconditionally, keeping that path byte-for-byte
 * unchanged - the right driver is a real chip on a real bus there, cheap to
 * flush at full rate like the left one. Wireless keeps the left (real,
 * local) driver at full rate too, but caps the right (emulated 0x77, relayed
 * over ESB) driver to ZSA_RIGHT_LED_INTERVAL_MS: every flush it answers
 * blocks on a clock-stretched I2C write (~9 ms vs ~0.65 ms for the left chip
 * alone), and the right half doesn't need matrix-scan-rate refresh.
 * Skipping a flush here does not clear pwm_buffer_dirty (only
 * is31fl3731_update_pwm_buffers() does, after it actually writes), so the
 * buffer stays dirty and the next permitted flush still sends the latest
 * colors - nothing is lost, just delayed. */
bool is31fl3731_should_flush(uint8_t index) {
    if (index != ZSA_LED_DRIVER_RIGHT || usb_connected_state()) {
        return true;
    }
    if (timer_elapsed32(last_right_led_flush_ms) < ZSA_RIGHT_LED_INTERVAL_MS) {
        return false;
    }
    last_right_led_flush_ms = timer_read32();
    return true;
}
#endif

/* ---- BT_UNPR hold ---- */

static bool     unpair_held;
static bool     unpair_sent;
static uint32_t unpair_press_ms;

/* Sends CMD_BLE_UNPAIR once BT_UNPR has been held ZSA_UNPAIR_HOLD_MS, at most
 * once per press. Driven from bluetooth_task() because process_record only
 * runs on key edges, never while a key stays down. */
static void unpair_hold_task(void) {
    if (!unpair_held || unpair_sent) {
        return;
    }
    if (timer_elapsed32(unpair_press_ms) < ZSA_UNPAIR_HOLD_MS) {
        return;
    }
    zsa_wireless_cmd_ble_unpair();
    unpair_sent = true;
}

/* Auto-generated per-module hook, called from process_record_modules() before
 * process_record_kb/_user — returning false swallows the key before the keymap
 * sees it. Both edges are swallowed so no unmatched release leaks through. */
bool process_record_wireless(uint16_t keycode, keyrecord_t *record) {
    if (!process_record_wireless_kb(keycode, record)) {
        return false;
    }

    switch (keycode) {
        case BLE_DISCONNECT:
            if (record->event.pressed) {
                zsa_wireless_cmd_ble_disconnect();
            }
            return false;
        /* Makes the board discoverable for the nRF's pairing-mode window. */
        case BLE_PAIR_MODE:
            if (record->event.pressed) {
                zsa_wireless_cmd_pair();
            }
            return false;
        /* Core's BT_UNPR, left an unimplemented stub by process_connection().
         * Modules run first, so this arm owns it. Destructive:
         * unpair_hold_task() fires it after the hold. */
        case QK_BLUETOOTH_UNPAIR:
            if (record->event.pressed) {
                unpair_held     = true;
                unpair_sent     = false;
                unpair_press_ms = timer_read32();
            } else {
                unpair_held = false;
            }
            return false;
    }

    return true;
}

/* ---- bluetooth.h weak function implementations ---- */

void bluetooth_init(void) {
    uart_init(ZSA_WIRELESS_BAUD_RATE);
    zsa_parser_init(&rx_parser);
    link_ready = false;
    last_link_up_ms = timer_read32();
    /* Before rgb_matrix_init(), so a wireless cold boot never writes to the
     * real 0x77 across the bus. */
    update_right_led_address();
    zsa_rtt_init(); // TEMP DEBUG
#ifdef ZSA_BLE_LOG
    zsa_log_tap_init();
    zsa_log("zsa_wireless: init (ZSA_BLE_LOG=on)");
#endif
}

/* Charging policy: while a high-budget source is attached (host or charger),
 * (re-)command both halves to raise their VBUS input limit. See the
 * adaptive-charging comment block at the top for the full rationale. */
static void power_budget_task(void) {
    /* Gated on the reported budget only, not usb_connected_state(): a wall
     * charger supplies VBUS without ever enumerating. The nRF reports HIGH
     * only while VBUS is present, so the budget alone implies power. */
    if (!link_ready) {
        return;
    }
    if (nrf_power_budget_ma < ZSA_PWR_BUDGET_HIGH_MA) {
        return;
    }
    if (last_pwr_maintain_ms != 0 &&
        timer_elapsed32(last_pwr_maintain_ms) < PWR_BUDGET_MAINTAIN_INTERVAL_MS) {
        return;
    }
    struct zsa_vbus_ilim payload = { .level = ZSA_VBUS_ILIM_HIGH };
    zsa_wireless_send_frame(ZSA_MSG_CMD_VBUS_ILIM, (const uint8_t *)&payload, sizeof(payload));
    if (is_transport_connected()) {
        (void)voyager_right_nrf_vbus_ilim_high();
    }
    last_pwr_maintain_ms = timer_read32();
}

void bluetooth_task(void) {
    zsa_rtt_loop_tick(); // TEMP DEBUG

    /* Process incoming frames */
    while (uart_available()) {
        uint8_t byte = uart_read();
        if (zsa_parser_feed(&rx_parser, byte)) {
            handle_rx_frame(&rx_parser.frame);
        }
    }

    /* Periodically send LINK_UP until nRF responds */
    if (!link_ready && timer_elapsed32(last_link_up_ms) >= LINK_UP_INTERVAL_MS) {
        send_command(ZSA_MSG_LINK_UP);
        last_link_up_ms = timer_read32();
    }

    /* Announce our nav capabilities once the link is up (retries until acked) */
    nav_announce_task();

    /* Fire BT_UNPR once its hold completes */
    unpair_hold_task();

    /* Maintain the adaptive-charging raise while a high-budget source is up */
    bus_grant_task();
    power_budget_task();

    pair_led_task();

    idle_power_task();
}

bool bluetooth_is_connected(void) {
    return nrf_ble_connected;
}

bool bluetooth_can_send_nkro(void) {
    return false;
}

uint8_t bluetooth_keyboard_leds(void) {
    return nrf_led_state;
}

void bluetooth_send_keyboard(report_keyboard_t *report) {
    uint8_t payload[8];
    payload[0] = report->mods;
    payload[1] = report->reserved;
    for (uint8_t i = 0; i < KEYBOARD_REPORT_KEYS; i++) {
        payload[2 + i] = report->keys[i];
    }
    zsa_wireless_send_frame(ZSA_MSG_HID_KEYBOARD, payload, sizeof(payload));
}

void bluetooth_send_nkro(report_nkro_t *report) {
    /* NKRO not supported over BLE — nRF expects 6KRO */
    (void)report;
}

void bluetooth_send_mouse(report_mouse_t *report) {
    /*
     * Send the report_mouse_t as raw bytes, skipping the report_id
     * if MOUSE_SHARED_EP is defined. The payload layout depends on
     * the keymap's mouse configuration (MOUSE_EXTENDED_REPORT, etc.).
     */
    const uint8_t *start = (const uint8_t *)report;
    uint8_t        len   = sizeof(report_mouse_t);

#ifdef MOUSE_SHARED_EP
    /* Skip the leading report_id byte */
    start += 1;
    len -= 1;
#endif

    zsa_wireless_send_frame(ZSA_MSG_HID_MOUSE, start, len);
}

#ifdef DIGITIZER_MODE_TOUCHPAD
/* Strong override of navigator_trackpad_ptp.c's weak
 * navigator_trackpad_output_touchpad() hook (see the extern declarations and
 * comment near the top of this file). Routes the PTP input report (report ID
 * 1, 16 B total) over UART as ZSA_MSG_HID_DIGITIZER — payload is the report
 * bytes minus the constant leading report_id byte — whenever we're not on
 * USB, mirroring the usb_connected_state() gate used by bluetooth_send_mouse()
 * above. Falls straight through to the real USB endpoint on USB so wired
 * behavior stays byte-identical. */
void navigator_trackpad_output_touchpad(report_digitizer_touchpad_t *report) {
    if (usb_connected_state()) {
        send_digitizer_touchpad(report);
        return;
    }
    _Static_assert(sizeof(report_digitizer_touchpad_t) == 1 + ZSA_HID_DIGITIZER_LEN,
                   "zsa_hid_digitizer payload size must track report_digitizer_touchpad_t");
    const uint8_t *bytes = (const uint8_t *)report;
    zsa_wireless_send_frame(ZSA_MSG_HID_DIGITIZER, bytes + 1, ZSA_HID_DIGITIZER_LEN);
}

/* Strong override of navigator_trackpad_ptp.c's weak
 * navigator_trackpad_output_touchpad_mouse() hook. The mouse-mode fallback
 * report (report ID 6: buttons + relative x/y) has no PTP-specific shape —
 * it's a plain relative pointer report — so rather than invent a second UART
 * message type, convert it to a report_mouse_t and reuse the existing mouse
 * path (ZSA_MSG_HID_MOUSE / bluetooth_send_mouse() above), which already
 * carries trackball motion over BLE. */
void navigator_trackpad_output_touchpad_mouse(report_digitizer_touchpad_mouse_t *report) {
    if (usb_connected_state()) {
        send_digitizer_touchpad_mouse(report);
        return;
    }
    report_mouse_t mouse_report = {0};
    mouse_report.buttons        = report->buttons;
    mouse_report.x              = (mouse_xy_report_t)report->x;
    mouse_report.y              = (mouse_xy_report_t)report->y;
    bluetooth_send_mouse(&mouse_report);
}

/* Strong override of navigator_trackpad_ptp.c's weak
 * navigator_trackpad_wireless_input_mode() hook. Shadows the BLE host's PTP
 * input-mode writes (ZSA_MSG_EVT_DIGITIZER_MODE) whenever we're actually
 * wireless; on USB (or before any mode has ever arrived over UART) returns -1
 * so the USB feature-report state keeps governing. */
int16_t navigator_trackpad_wireless_input_mode(void) {
    if (usb_connected_state() || !digitizer_wireless_mode_set) {
        return -1;
    }
    return (int16_t)digitizer_wireless_mode;
}
#endif // DIGITIZER_MODE_TOUCHPAD

void bluetooth_send_consumer(uint16_t usage) {
    uint8_t payload[3];
    payload[0] = REPORT_ID_CONSUMER;
    payload[1] = (uint8_t)(usage & 0xFF);
    payload[2] = (uint8_t)(usage >> 8);
    zsa_wireless_send_frame(ZSA_MSG_HID_SYSTEM, payload, sizeof(payload));
}

void bluetooth_send_system(uint16_t usage) {
    uint8_t payload[3];
    payload[0] = REPORT_ID_SYSTEM;
    payload[1] = (uint8_t)(usage & 0xFF);
    payload[2] = (uint8_t)(usage >> 8);
    zsa_wireless_send_frame(ZSA_MSG_HID_SYSTEM, payload, sizeof(payload));
}

void bluetooth_send_raw_hid(uint8_t *data, uint8_t length) {
    /* Raw HID not supported over wireless */
    (void)data;
    (void)length;
}
