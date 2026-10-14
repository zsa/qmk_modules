/*
 * ZSA Wireless module — implements bluetooth.h weak functions
 * to route HID reports over UART to the nRF54 wireless chip.
 *
 * Runtime switching between USB and wireless is handled by QMK's
 * CONNECTION_ENABLE subsystem (connection.c / host.c).
 */

#include <stddef.h>
#include <hal.h>
#include "quantum.h"
#include "usb_main.h"
#include "wireless.h"
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
#include "busdev.h"
#ifdef RGB_MATRIX_ENABLE
#    include "rgb_matrix.h"
#    include "is31fl3731.h"
#endif

/* Power stage timeouts, checked after the keymap config.h and -D flags. */
#ifndef ZSA_DEV_NO_SLEEP
#    if (ZSA_SLEEP_TIMEOUT_MS) < 1000
#        error "ZSA_SLEEP_TIMEOUT_MS must be at least 1000 ms"
#    endif
#    if (ZSA_HIBERNATE_TIMEOUT_MS) <= (ZSA_SLEEP_TIMEOUT_MS)
#        error "ZSA_HIBERNATE_TIMEOUT_MS must be greater than ZSA_SLEEP_TIMEOUT_MS"
#    endif
#    if (ZSA_HIBERNATE_TIMEOUT_MS) > 0x7FFFFFFF
#        error "ZSA_HIBERNATE_TIMEOUT_MS is out of range (max 0x7FFFFFFF ms)"
#    endif
#endif

/* keyboard.c — not in keyboard.h: digitizer reports count as input */
void last_pointing_device_activity_trigger(void);

/* ---- Independent watchdog ----
 *
 * A wedged main loop (unbounded UART drain, stuck blocking TX, a hung I2C
 * transaction that escapes its timeout) leaves the keyboard dead until the
 * battery is pulled — unacceptable for a sealed wireless product. The IWDG
 * runs off the independent LSI (~40 kHz, keeps counting in sleep/Stop) and
 * hard-resets the MCU if not kicked within ZSA_WATCHDOG_TIMEOUT_MS.
 *
 * Kick points cover every steady state of the firmware:
 *   - bluetooth_task()        — every keyboard_task() pass, wired or not
 *   - the STOP loop           — every RTC wakeup while the STM32 is stopped
 *
 * USB suspend needs no kick of its own. This module sets BLUETOOTH_ENABLE,
 * and common_features.mk forces NO_USB_STARTUP_CHECK with it, which compiles
 * out the `while (USB_DRIVER.state == USB_SUSPENDED) { suspend_power_down(); }`
 * loop in protocol_pre_task(). protocol_pre_task() then returns immediately
 * and main() keeps running keyboard_task() — and so bluetooth_task() — every
 * pass while suspended. suspend_power_down() has no caller left on ChibiOS,
 * so a suspend_power_down_wireless() hook would be dead code.
 *
 * The IWDG is disabled by any MCU reset, so bootloader_jump's soft reset
 * into Ignition (which arms its own watchdog) is unaffected. On Hibernate the
 * nRF cuts the rail (ship mode) — the watchdog dies with the MCU, and if ship
 * mode never engages the main loop keeps kicking.
 *
 * STM32-only: both boards that pull this module in are STM32F303, but the
 * register access is guarded so a non-STM32 target still compiles.
 */
#if defined(MCU_STM32)
#    include <hal.h>

#    ifndef ZSA_WATCHDOG_TIMEOUT_MS
#        define ZSA_WATCHDOG_TIMEOUT_MS 2000
#    endif

static void watchdog_start(void) {
    /* Freeze the counter while a debugger has the core halted so
     * breakpoints don't reboot the board mid-session. */
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
    /* LSI/64 => 1.6 ms/tick at the nominal 40 kHz (F303 LSI spans
     * 30-50 kHz; at 50 kHz a 2 s setting still gives 1.6 s — fine, the
     * loop kicks every pass). First-ever configuration, so IWDG_SR is 0
     * and PR/RLR are writable immediately (no PVU/RVU wait needed). */
    uint32_t reload = (uint32_t)ZSA_WATCHDOG_TIMEOUT_MS * 40u / 64u;
    if (reload > 0xFFFu) reload = 0xFFFu;
    IWDG->KR  = 0x5555u; /* unlock PR/RLR */
    IWDG->PR  = 4u;      /* prescaler /64 */
    IWDG->RLR = reload;
    IWDG->KR  = 0xAAAAu; /* load the reload value */
    IWDG->KR  = 0xCCCCu; /* start — from here on it can't be stopped */
}

static inline void watchdog_kick(void) {
    IWDG->KR = 0xAAAAu;
}
#else
static inline void watchdog_start(void) {}
static inline void watchdog_kick(void) {}
#endif

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
static uint8_t  nrf_battery_right_flags;
static bool     nrf_battery_right_flags_known; /* the report carried the field */
static uint32_t last_battery_ms;
static bool     battery_seen;         /* any EVT_BATTERY received yet */
/* The nRF re-sends the battery level every second (STATUS_UPDATE_INTERVAL_MS),
 * so anything older than a few of those means the link stopped, not that the
 * value stopped changing. */
#define BATTERY_STALE_MS 5000
/* EVT_BATTERY_DETAIL: the same levels spelled out — cell voltage, current,
 * temperature, time estimates and the charger registers. Sent alongside
 * EVT_BATTERY at the same cadence, so it ages out on the same window, but on
 * its own timestamp: left firmware that predates it sends EVT_BATTERY only. */
static struct zsa_battery_detail nrf_detail_left;
static struct zsa_battery_detail nrf_detail_right;
static uint32_t last_detail_ms;
static bool     detail_seen;          /* any EVT_BATTERY_DETAIL received yet */
/* EVT_BLE_IDENTITY: the local Bluetooth address of the host slot the nRF is
 * currently on. Sent alongside EVT_BATTERY_DETAIL at the same cadence, so it
 * ages out on the same window, on its own timestamp: left firmware that
 * predates it never sends one. */
static struct zsa_ble_identity nrf_ble_identity;
static uint32_t last_ble_identity_ms;
static bool     ble_identity_seen;    /* any EVT_BLE_IDENTITY received yet */
/* EVT_PMIC_RAILS: both halves' PMIC input rails. Sent alongside
 * EVT_BLE_IDENTITY at the same cadence, so it ages out on the same window, on
 * its own timestamp: left firmware that predates it never sends one. */
static struct zsa_pmic_rails_event nrf_pmic_rails;
static uint32_t last_pmic_rails_ms;
static bool     pmic_rails_seen;      /* any EVT_PMIC_RAILS received yet */
static uint8_t  nrf_led_state;
static bool     nrf_vbus_present;

/* ---- Firmware update in progress ---- *
 *
 * EVT_DFU_STATE's `active` mirrors the nRF's DFU lock, so it covers both the
 * mcumgr SMP path and the STM32 relay. While it is set the keyboard is inert —
 * no key output, no pointer, no idle power-down — so a flash cannot be
 * disturbed by typing. The nRF re-sends the level every second and
 * last_dfu_state_ms ages it out, so a link that dies mid-update recovers
 * instead of leaving the keyboard dead.
 *
 * `target`/`phase` describe the left half's own image and drive the status
 * LEDs (dfu_led_task). They age out on their own, much slower: the left nRF
 * reboots into the swap mid-update and re-asserts the phase when it returns. */
static bool     nrf_dfu_active;
static uint32_t last_dfu_state_ms;
static uint8_t  nrf_dfu_target;    /* enum zsa_dfu_target */
static uint8_t  nrf_dfu_phase;     /* enum zsa_dfu_phase */
static uint32_t last_dfu_phase_ms;
static bool     dfu_phase_seen;    /* any 3-byte EVT_DFU_STATE received yet */
/* ZSA_DFU_FLAG_KEEP_AWAKE: an update session or step is under way. Holds the
 * idle state off like `active`, without dropping keys. Aged out with it. */
static bool     nrf_dfu_keep_awake;

/* ---- Inter-half link state ---- *
 *
 * EVT_LINK_STATE: the left nRF's view of the right half over ESB. MISMATCH
 * means both halves are alive but running different keyboard protocols, so
 * neither drives the other — indicated as a red slow blink. Level-refreshed
 * every second like the pair state, hence the same staleness gate. */
static uint8_t  nrf_link_state;    /* enum zsa_link_state_val */
static uint32_t last_link_state_ms;
static bool     link_state_seen;
static struct zsa_nav_link_age nav_link_age; /* the same, aged for nav presence */

/* ---- Adaptive charging ---- *
 *
 * The left nRF reports the USB source's CC-detected current budget
 * (EVT_PWR_BUDGET); this module owns the charging policy and both halves
 * execute it. zsa_charge_limits_for_budget() in the shared wire header turns that
 * budget into the two currents to apply — how much a half may draw from VBUS,
 * and how much of it goes into its cell — and they are commanded to the left
 * over UART (CMD_CHARGE_LIMITS) and to the right over the TRRS I2C bus
 * (voyager_right_nrf_set_charge_limits()).
 *
 * Both halves get the same derived pair. Each is tracked separately, so a
 * change on either side is sent on either side.
 *
 * Sent whenever the computed limits change, so a source that renegotiates
 * downward is followed down and not only up, and re-sent every
 * PWR_BUDGET_MAINTAIN_INTERVAL_MS otherwise: both commands are idempotent, and
 * the resend covers right-half TRRS hot-plug, right VBUS arriving late, and
 * nRF reboots. Nothing here has to lower the input limit on unplug — the
 * nPM1300s revert to their startup limit by themselves when VBUS goes away.
 *
 * There is no user setting and no host command for any of this. The rate
 * follows the source, and only the source.
 *
 * Bench builds only: defining both ZSA_RIGHT_VBUS_LIMIT_OVERRIDE_MA and
 * ZSA_RIGHT_CHARGE_OVERRIDE_MA (milliamps) replaces the right half's computed
 * pair with those two fixed values. Both must be given; neither alone has any
 * effect, and the left half is never overridden.
 *
 * Bench builds only: defining ZSA_BENCH_FORCE_NO_HOST makes every host
 * decision in this file report "no USB host" — CMD_USB_STATE to the nRF, idle
 * stepping, and the nav HID routing gates — so a cabled board behaves as if it
 * were on a wall charger. The USB stack itself is untouched: the board still
 * enumerates and QMK still routes keyboard HID over USB. */
static uint16_t nrf_power_budget_ma  = ZSA_PWR_BUDGET_DEFAULT_MA;
static bool     pwr_budget_seen;      /* any EVT_PWR_BUDGET received yet */
static uint32_t last_pwr_maintain_ms = 0;
/* What we last commanded to each half, and whether we have commanded anything
 * yet. Reported as telemetry, so it has to be what went out rather than what we
 * would send. */
static struct zsa_charge_limits commanded_limits;       /* left */
static struct zsa_charge_limits commanded_right_limits; /* right */
static bool                     limits_commanded;
#define PWR_BUDGET_MAINTAIN_INTERVAL_MS 5000

/* Board-provided (voyager_tng.c) right-half charge command; extern rather
 * than including the board header to keep the module decoupled. */
extern bool voyager_right_nrf_set_charge_limits(uint16_t vbus_limit_ma, uint16_t charge_ma);
extern bool is_transport_connected(void);
/* Bus ownership decision, made in the keyboard's matrix.c by probing the real
 * expander. The grant must follow the same signal. */
extern bool zsa_stm32_owns_right_bus(void);
/* TRRS cable presence, from the same probe: an ACK from the real right-half
 * expander is the only evidence the cable is plugged. */
extern bool zsa_trrs_plugged(void);
/* Failed right-half expander scan reads, cumulative, kept by the same
 * matrix.c. Reported in the scan diagnostics below. */
extern uint16_t zsa_matrix_right_read_errors(void);
/* Right LED driver, supervised by the keyboard (voyager_tng.c). */
extern void voyager_right_led_resync(void);
extern bool voyager_right_led_up(void);

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

/* Sliced: the start of the current (or last) block cycle. Otherwise the last
 * whole-frame flush. */
static uint32_t last_right_led_flush_ms = 0;
#    if ZSA_RIGHT_LED_SLICED
static uint8_t right_led_block;        /* next block to send */
static bool    right_led_cycle_active; /* a frame is part-sent */
static bool    right_frame_ready;      /* right_led_frame holds a whole frame to send */
static uint8_t right_led_frame[IS31FL3731_PWM_REGISTER_COUNT]; /* the frame the cycle sends */
#    endif

#    if ZSA_LEFT_LED_SLICED
static uint8_t left_led_block;        /* next block to send */
static bool    left_led_cycle_active; /* a frame is part-sent */
static bool    left_frame_ready;      /* left_led_frame holds a whole frame to send */
static uint8_t left_led_frame[IS31FL3731_PWM_REGISTER_COUNT]; /* the frame the cycle sends */

/* Drops a part-sent left frame and asks for the whole frame again. */
static void left_led_slice_abandon(void) {
    left_led_cycle_active = false;
    left_frame_ready      = false;
    is31fl3731_set_pwm_buffer_dirty(ZSA_LED_DRIVER_LEFT, true);
}
#    else
#        define left_led_slice_abandon() ((void)0)
#    endif
#endif

static uint8_t nav_device_type;  /* enum zsa_nav_device_type */
static bool    nav_device_seen;  /* any EVT_NAV_PRESENT received yet */

/* ---- Right-half telemetry over the wired bus ---- *
 *
 * On USB the right half suspends ESB, so the SOC the left relays in
 * EVT_BATTERY stops being refreshed and the last value sits there looking
 * fresh. The right's TWIS slave stays live on the TRRS net though, and the
 * STM32 masters that bus whenever the cable is plugged — so read it there.
 * Polled slowly: it is a status field, and the bus is busy with the matrix. */
static struct zsa_right_telemetry right_tlm;
static bool     right_tlm_valid;
static uint32_t last_right_tlm_ms;
#define RIGHT_TLM_INTERVAL_MS 1000

/* Board-provided (voyager_tng.c), like voyager_right_nrf_set_charge_limits(). */
extern bool voyager_right_nrf_read_telemetry(struct zsa_right_telemetry *out);

/* ---- Scan diagnostics ---- *
 *
 * Per-window pass and gap counters plus cumulative right-half read failures
 * and keyboard reports sent to the nRF. Finalized every
 * ZSA_SCAN_DIAG_INTERVAL_MS into scan_diag_last, sent as CMD_SCAN_DIAG and
 * served as the answer to ORYX_CMD_GET_SCAN_DIAG. Zero (version 0) until the
 * first window completes. */
static struct zsa_scan_diag scan_diag_last;
static uint32_t scan_diag_prev_ms;   /* previous pass */
static uint32_t scan_diag_window_ms; /* window start */
static bool     scan_diag_started;   /* a previous pass exists to measure from */
static uint16_t scan_diag_scans;
static uint16_t scan_diag_max_gap_ms;
static uint16_t scan_diag_over_5ms;
static uint16_t scan_diag_over_20ms;
static uint16_t scan_diag_kbd_reports; /* cumulative */
#define SCAN_DIAG_GAP_5MS  5u
#define SCAN_DIAG_GAP_20MS 20u

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

/* ---- Power stage (declared early for use in handle_rx_frame) ---- */

static enum zsa_idle_step idle_step = ZSA_IDLE_ACTIVE;
/* Last byte received from the nRF; a sleeping STM32 stays up a window after
 * it. */
static uint32_t last_rx_ms;
/* CMD_HIBERNATE went out for the current Sleep. Cleared when VBUS changes,
 * since the nRF refuses it while VBUS is present. */
static bool hibernate_sent;

/* ---- Boot power gate ---- *
 *
 * The STM32 shares the PMIC's rail with the left nRF and boots first, so after
 * a brownout it would light the matrix and flatten the cell again before the
 * nRF could read RSTCAUSE. keyboard_post_init_kb() therefore leaves the matrix
 * disabled and this gate decides.
 *
 * The nRF pushes EVT_POWER_VERDICT on every link-up. ZSA_POWER_CRITICAL means
 * the last PMIC reset was a brownout and there is no USB power: keep the matrix
 * dark, pulse the red status LED, then ask the nRF for ship mode. A verdict
 * without the flag — including the one re-sent when USB is plugged in mid
 * sequence — boots normally.
 *
 * Transitions:
 *   PG_WAIT     -> PG_CRITICAL  verdict with ZSA_POWER_CRITICAL
 *   PG_WAIT     -> PG_OK        verdict without it, or ZSA_POWER_GATE_TIMEOUT_MS
 *   PG_CRITICAL -> PG_DONE      pulse sequence finished
 *   PG_CRITICAL -> PG_OK        verdict without ZSA_POWER_CRITICAL
 *   PG_DONE     -> PG_OK        verdict without ZSA_POWER_CRITICAL
 */
enum zsa_power_gate {
    PG_WAIT = 0, /* no verdict yet, matrix held off */
    PG_OK,       /* matrix enabled, terminal until the next boot */
    PG_CRITICAL, /* running the red pulse sequence */
    PG_DONE,     /* dark, asking the nRF to ship */
};

#define POWER_PULSE_PERIOD_MS (ZSA_POWER_LED_ON_MS + ZSA_POWER_LED_OFF_MS)
#define POWER_PULSE_TOTAL_MS  (POWER_PULSE_PERIOD_MS * ZSA_POWER_LED_PULSES)

static enum zsa_power_gate power_gate       = PG_WAIT;
static uint32_t            power_gate_ms;       /* when the current state was entered */
static uint32_t            power_hibernate_ms;  /* last CMD_HIBERNATE sent in PG_DONE */

/* False while the gate holds the matrix off, so the other paths that enable it
 * cannot undo the hold. */
static inline bool power_gate_allows_rgb(void) {
    return power_gate == PG_OK;
}

static void power_gate_enter(enum zsa_power_gate next) {
    if (power_gate == next) {
        return;
    }
    power_gate    = next;
    power_gate_ms = timer_read32();
    zsa_log("power: gate -> %u", (unsigned)next);

#ifdef RGB_MATRIX_ENABLE
    if (next == PG_OK) {
        rgb_matrix_enable_noeeprom();
    }
#endif
}

/* CRITICAL only decides while nothing else has, so a late verdict cannot dark
 * a running keyboard. OK always wins, which is how USB plugged in during the
 * pulses gets back to a normal boot. */
static void power_gate_verdict(uint8_t flags) {
    if (power_gate == PG_OK) {
        return;
    }
    if (!(flags & ZSA_POWER_CRITICAL)) {
        power_gate_enter(PG_OK);
    } else if (power_gate == PG_WAIT) {
        power_gate_enter(PG_CRITICAL);
    }
}

/* Red channel of the pulse sequence, phase taken from the absolute clock since
 * PG_CRITICAL was entered so the cadence does not depend on the tick rate. */
static uint8_t power_gate_red(void) {
    uint32_t t = timer_elapsed32(power_gate_ms);
    if (t >= POWER_PULSE_TOTAL_MS) {
        return ZSA_STATUS_LED_OFF;
    }
    return (t % POWER_PULSE_PERIOD_MS) < ZSA_POWER_LED_ON_MS ? ZSA_STATUS_LED_ON : ZSA_STATUS_LED_OFF;
}

static void power_gate_task(void) {
#ifdef RGB_MATRIX_ENABLE
    /* Re-assert the hold, so a runtime enable cannot lift it before the verdict. */
    if (power_gate != PG_OK && rgb_matrix_is_enabled()) {
        rgb_matrix_disable_noeeprom();
    }
#endif

    switch (power_gate) {
        case PG_WAIT:
            /* No verdict in time: the nRF is dead, in serial recovery, or on a
             * build that does not send one. Light the matrix. */
            if (timer_elapsed32(power_gate_ms) >= ZSA_POWER_GATE_TIMEOUT_MS) {
                power_gate_enter(PG_OK);
            }
            break;

        case PG_CRITICAL:
            if (timer_elapsed32(power_gate_ms) >= POWER_PULSE_TOTAL_MS) {
                power_gate_enter(PG_DONE);
                zsa_wireless_cmd_hibernate();
                power_hibernate_ms = timer_read32();
            }
            break;

        case PG_DONE:
            if (timer_elapsed32(power_hibernate_ms) >= ZSA_POWER_HIBERNATE_RETRY_MS) {
                zsa_wireless_cmd_hibernate();
                power_hibernate_ms = timer_read32();
            }
            break;

        default:
            break;
    }
}

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

/* Why a pass in Sleep did not stop; SB_NONE: it stopped. */
enum sleep_block {
    SB_NONE = 0,
    SB_WINDOW,       /* running window after a wake */
    SB_POWER,        /* STOP unsupported, VBUS or cable */
    SB_BOOST,        /* nav probe window open or boost claim not settled */
    SB_STATUS_LED,   /* a status LED owned */
    SB_KEY_HELD,     /* a key down in the matrix */
    SB_LEDS,         /* black frame not settled */
    SB_STRETCH,      /* last stretch before hibernate or a nav window */
    SB_NAV_NO_HOOK,  /* left nav device without hooks */
    SB_NAV_FAILED,   /* enter refused or poll error limit, see nav_fail */
    SB_NAV_NOT_IDLE, /* normal read path has data pending */
    SB_I2C_BUSY,     /* ChibiOS I2C driver stopped or bus busy */
    SB_COUNT,
};

/* What ended a Sleep. */
enum sleep_end {
    SE_NONE = 0,
    SE_KEY,          /* matrix activity */
    SE_POINTER,      /* pointing activity from the normal read path */
    SE_INPUT_OTHER,  /* other input activity */
    SE_NAV_POLL,     /* the STOP poll returned activity */
    SE_NAV_FOUND,    /* a Sleep nav window found a device */
    SE_USB,          /* USB host */
    SE_TRANSPORT,    /* switch to wireless */
    SE_DFU_PAIRING,  /* update or pairing in progress */
    SE_PAIR_CONFIRM, /* pairing confirmation */
    SE_DFU_START,    /* update started */
    SE_DFU_AWAKE,    /* update session or step under way */
};

static void sleep_exit_for(uint8_t why);
static void send_usb_state(bool connected);
static void send_trrs_state(bool plugged);
static void send_idle_state(enum zsa_idle_step step);
static void right_telemetry_task(void);
static bool usb_hid_on_usb(void);

/* A USB host, as opposed to a wall charger or an unplugged cable. VBUS alone
 * is a charger: it raises the D+ pull-up, no host ever resets the bus, and the
 * USBv1 driver walks USB_READY -> USB_SUSPENDED on the idle bus (SUSP ISR ->
 * _usb_suspend()). USB_SUSPENDED is therefore only a host when the driver was
 * configured before it suspended — selective suspend. _usb_suspend() records
 * that in saved_state; _usb_wakeup() restores state from it.
 *
 * saved_state also survives an unplug, so usb_configured tracks whether the
 * enumeration behind it belongs to the current VBUS session; it is cleared
 * when VBUS goes away. VBUS comes from the nRF as EVT_VBUS_STATE. */
static bool usb_configured;

static bool usb_host_present(void) {
#ifdef ZSA_BENCH_FORCE_NO_HOST
    /* Bench knob: report no USB host regardless of the bus state. */
    return false;
#endif
    usbstate_t state = usbGetDriverStateI(&USB_DRIVER);
    if (state == USB_ACTIVE) {
        usb_configured = true;
    }
    return nrf_vbus_present && usb_configured &&
           (state == USB_ACTIVE || (state == USB_SUSPENDED && USB_DRIVER.saved_state == USB_ACTIVE));
}

/* Send all current STM32 state to nRF — called on link establishment */
static void send_full_state(void) {
    send_usb_state(usb_host_present());
    send_trrs_state(zsa_trrs_plugged());
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

/* ---- Left nav boost ---- *
 *
 * The left boost powers the left nav connector. The left nRF keeps it off on
 * VBUS, on with the TRRS cable plugged, and otherwise on only while
 * CMD_NAV_BOOST claims it (off for a build without nav support, CMD_NAV_ENABLE
 * 0). A left nav device sits on our own bus and cannot answer with the boost
 * off, so the claim is held while one answers at its real address, and
 * otherwise only for a probe window: the claim goes out, the real addresses
 * are probed every NAV_PROBE_KICK_MS from NAV_PROBE_FIRST_KICK_MS on, and the
 * claim is dropped after ZSA_NAV_PROBE_WINDOW_MS unless a device answered.
 * Windows open at link-up, on leaving Sleep, when a present device stops
 * answering, every ZSA_NAV_PROBE_INTERVAL_MS while Active and every
 * ZSA_NAV_PROBE_SLEEP_INTERVAL_MS of Sleep time (STOP included). A device found
 * by a Sleep window wakes the board. None while the connector is powered anyway
 * (VBUS or cable). The supervisor's own probes stop once a device is up on the
 * mirror, so on VBUS without the cable the real addresses are still probed
 * every ZSA_NAV_PROBE_INTERVAL_MS, with no claim: VBUS feeds the rail. */
#define NAV_PROBE_FIRST_KICK_MS 40
#define NAV_PROBE_KICK_MS 30
/* Longest a window stays open past ZSA_NAV_PROBE_WINDOW_MS while a device that
 * answered is being set up. */
#define NAV_PROBE_PENDING_MAX_MS 1000

static bool     nav_boost_sent;            /* last claim sent */
static bool     nav_boost_dirty = true;    /* the nRF may not hold the claim */
static bool     nav_window_open;
static bool     nav_window_request = true; /* open a window at the next chance */
static uint32_t nav_window_start_ms;
static uint32_t nav_window_end_ms;
static uint8_t  nav_window_kicks;
static bool     nav_window_sleep;      /* opened by the Sleep interval */
static uint32_t nav_sleep_due_ms;      /* next Sleep window, in sleep_clock_ms() time */
static bool     nav_left_present;
static uint32_t nav_vbus_probe_ms;     /* last real-address probe on VBUS */

static uint32_t sleep_clock_ms(void);

static bool nav_left_local(void) {
    return zsa_nav_trackball_probe_state() == ZSA_NAV_PROBE_LOCAL || zsa_nav_trackpad_probe_state() == ZSA_NAV_PROBE_LOCAL;
}

static bool nav_left_pending(void) {
    return zsa_nav_trackball_local_pending() || zsa_nav_trackpad_local_pending();
}

static void send_nav_boost(bool needed) {
    struct zsa_nav_boost b = {.needed = needed ? 1 : 0};
    zsa_wireless_send_frame(ZSA_MSG_CMD_NAV_BOOST, (const uint8_t *)&b, sizeof(b));
    nav_boost_sent  = needed;
    nav_boost_dirty = false;
}

static void nav_window_close(void) {
    nav_window_open   = false;
    nav_window_sleep  = false;
    nav_window_end_ms = timer_read32();
    if (idle_step != ZSA_IDLE_ACTIVE) {
        nav_sleep_due_ms = sleep_clock_ms() + ZSA_NAV_PROBE_SLEEP_INTERVAL_MS;
    }
}

/* Sleep windows apply: nav support, a link, and the connector unpowered. */
static bool nav_sleep_windows(void) {
    return nav_cap_mask != 0 && link_ready && !nrf_vbus_present && !zsa_trrs_plugged();
}

/* The nRF holds the final claim: STOP may follow. */
static bool nav_boost_settled(void) {
    return nav_cap_mask == 0 || !link_ready || (!nav_window_open && !nav_boost_dirty);
}

static void nav_boost_task(void) {
    if (nav_cap_mask == 0 || !link_ready) {
        return;
    }

    bool present = nav_left_local();
    if (nav_left_present && !present) {
        nav_window_request = true;
    }
    nav_left_present = present;

    if (nrf_vbus_present || zsa_trrs_plugged()) {
        nav_window_request = false;
        if (nav_window_open) {
            nav_window_close();
        }
        /* VBUS without the cable: a device latched on the mirror is never
         * moved by the supervisor's own probes, so probe the real addresses. */
        if (nrf_vbus_present && !zsa_trrs_plugged() && !present && !zsa_wireless_dfu_active() && timer_elapsed32(nav_vbus_probe_ms) >= ZSA_NAV_PROBE_INTERVAL_MS) {
            nav_vbus_probe_ms = timer_read32();
            zsa_nav_trackball_probe_local();
            zsa_nav_trackpad_probe_local();
        }
        nav_window_end_ms = timer_read32();
    } else if (nav_window_open) {
        uint32_t elapsed = timer_elapsed32(nav_window_start_ms);
        if (present) {
            bool wake = nav_window_sleep && idle_step != ZSA_IDLE_ACTIVE;
            nav_window_close();
            if (wake) {
                /* A device plugged while asleep: input. */
                zsa_log("power: left nav found in sleep");
                last_pointing_device_activity_trigger();
                sleep_exit_for(SE_NAV_FOUND);
            }
        } else if (elapsed >= ZSA_NAV_PROBE_WINDOW_MS && (!nav_left_pending() || elapsed >= ZSA_NAV_PROBE_WINDOW_MS + NAV_PROBE_PENDING_MAX_MS)) {
            nav_window_close();
        } else if (elapsed >= NAV_PROBE_FIRST_KICK_MS + (uint32_t)nav_window_kicks * NAV_PROBE_KICK_MS) {
            nav_window_kicks++;
            zsa_nav_trackball_probe_local();
            zsa_nav_trackpad_probe_local();
        }
    } else if (!present && !zsa_wireless_dfu_active()) {
        bool due_sleep = idle_step != ZSA_IDLE_ACTIVE && sleep_clock_ms() >= nav_sleep_due_ms;
        bool due       = due_sleep || (idle_step == ZSA_IDLE_ACTIVE && timer_elapsed32(nav_window_end_ms) >= ZSA_NAV_PROBE_INTERVAL_MS);
        if (nav_window_request || due) {
            nav_window_request  = false;
            nav_window_open     = true;
            nav_window_sleep    = due_sleep;
            nav_window_start_ms = timer_read32();
            nav_window_kicks    = 0;
        }
    }

    bool needed = present || nav_window_open;
    if (needed != nav_boost_sent || nav_boost_dirty) {
        send_nav_boost(needed);
    }
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

/* A pairing starting is worth showing: leave Sleep, which restores the matrix
 * the way it was. Discoverability deliberately does not do this — it can run
 * for two minutes and would pin the LEDs on. */
static void pair_confirm_wake(void) {
    sleep_exit_for(SE_PAIR_CONFIRM);
}

bool zsa_wireless_dfu_active(void) {
    return nrf_dfu_active;
}

bool zsa_wireless_link_mismatch(void) {
    return link_ready && link_state_seen && nrf_link_state == ZSA_LINK_STATE_MISMATCH &&
           timer_elapsed32(last_link_state_ms) < ZSA_LINK_STATE_STALE_MS;
}

/* Enter/leave the inert update state. Releases everything held on the way in
 * and on the way out, so a key swallowed mid-press cannot leave a stuck
 * report. The matrix is left alone — it keeps doing whatever the user's
 * config was doing, including staying off. */
static void dfu_set_active(bool active) {
    if (active == nrf_dfu_active) {
        return;
    }
    nrf_dfu_active = active;
    zsa_log("dfu: %s", active ? "active" : "idle");

    /* An update produces no key activity, so leave Sleep rather than let the
     * board power down mid-transfer. */
    if (active) {
        sleep_exit_for(SE_DFU_START);
    }

    clear_keyboard();
}

/* Ages out a state that stopped being refreshed. */
static void dfu_state_task(void) {
    if ((nrf_dfu_active || nrf_dfu_keep_awake) && timer_elapsed32(last_dfu_state_ms) >= ZSA_DFU_STATE_STALE_MS) {
        nrf_dfu_keep_awake = false;
        dfu_set_active(false);
    }
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
            nav_device_seen      = false;
            /* The nRF drops the boost claim on LINK_UP and on reboot. */
            nav_boost_dirty = true;
            /* Same reasoning for the power budget: forget what the old link
             * said until the nRF re-reports its real budget (sent right after
             * link-up). Back to DEFAULT, the profile the PMICs are already on.
             * An nRF that rebooted also lost its runtime input limit, so what
             * we commanded is no longer in force. */
            nrf_power_budget_ma = ZSA_PWR_BUDGET_DEFAULT_MA;
            pwr_budget_seen     = false;
            limits_commanded    = false;
            /* An nRF that rebooted lost its DFU lock, so nothing is being
             * flashed any more. send_nrf_state() re-asserts it right after
             * this if the update is in fact still running. The DFU phase is
             * deliberately left alone: the reboot this link-up follows is
             * usually the swap itself, and the new image re-asserts SETTLE. */
            dfu_set_active(false);
            nrf_dfu_keep_awake = false;
            /* nrf_vbus_present is left alone: the nRF re-reports it right
             * after link-up via send_nrf_state(). */
            send_full_state();
            break;

        case ZSA_MSG_EVT_BLE_STATUS:
            if (frame->len >= sizeof(struct zsa_ble_status)) {
                const struct zsa_ble_status *s = (const struct zsa_ble_status *)frame->payload;
                zsa_rtt_event(s->connected ? "BLE_STATUS connected" : "BLE_STATUS disconnected"); // TEMP DEBUG
                nrf_ble_connected  = s->connected != 0;
                nrf_ble_device_idx = s->device_idx;
            }
            break;

        case ZSA_MSG_EVT_BATTERY:
            /* Left firmware that predates right_flags sends only three bytes. */
            if (frame->len >= ZSA_BATTERY_STATUS_LEN_V1) {
                const struct zsa_battery_status *b = (const struct zsa_battery_status *)frame->payload;
                nrf_battery_soc       = b->soc;
                nrf_battery_right_soc = b->right_soc;
                nrf_battery_flags     = b->flags;

                nrf_battery_right_flags_known = frame->len >= sizeof(struct zsa_battery_status);
                nrf_battery_right_flags       = nrf_battery_right_flags_known ? b->right_flags : 0;

                last_battery_ms = timer_read32();
                battery_seen    = true;
            }
            break;

        case ZSA_MSG_EVT_BATTERY_DETAIL:
            if (frame->len >= sizeof(struct zsa_battery_detail_event)) {
                const struct zsa_battery_detail_event *d = (const struct zsa_battery_detail_event *)frame->payload;
                nrf_detail_left  = d->left;
                nrf_detail_right = d->right;
                last_detail_ms   = timer_read32();
                detail_seen      = true;
            }
            break;

        case ZSA_MSG_EVT_BLE_IDENTITY:
            if (frame->len >= sizeof(struct zsa_ble_identity)) {
                memcpy(&nrf_ble_identity, frame->payload, sizeof(nrf_ble_identity));
                last_ble_identity_ms = timer_read32();
                ble_identity_seen    = true;
            }
            break;

        case ZSA_MSG_EVT_PMIC_RAILS:
            if (frame->len >= sizeof(struct zsa_pmic_rails_event)) {
                memcpy(&nrf_pmic_rails, frame->payload, sizeof(nrf_pmic_rails));
                last_pmic_rails_ms = timer_read32();
                pmic_rails_seen    = true;
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
                // returning after Sleep. A repeated "on" report is a no-op.
                if (rail_on && !left_led_rail_on && idle_step == ZSA_IDLE_ACTIVE && power_gate_allows_rgb()) {
                    zsa_rtt_event("LED_RAIL -> resync"); // TEMP DEBUG
                    // The rail powers the left chip, which is not supervised:
                    // resync it here. The right one goes through the supervisor.
                    // The resync clears the chip's PWM registers, so a
                    // part-sent frame has to start over.
                    left_led_slice_abandon();
                    is31fl3731_resync(ZSA_LED_DRIVER_LEFT);
                    voyager_right_led_resync();
                }
                left_led_rail_on = rail_on;
#endif
            }
            break;

        case ZSA_MSG_EVT_POWER_VERDICT:
            if (frame->len >= sizeof(struct zsa_power_verdict)) {
                const struct zsa_power_verdict *v = (const struct zsa_power_verdict *)frame->payload;
                power_gate_verdict(v->flags);
            }
            break;

        case ZSA_MSG_QUERY_USB_STATE:
            send_usb_state(usb_host_present());
            break;

        case ZSA_MSG_EVT_NAV_PRESENT:
            /* The nRF acks a capability announce with the currently detected
             * device type (possibly 0=none) and re-sends on any presence change. */
            nav_announce_acked = true;
            if (frame->len >= sizeof(struct zsa_nav_present)) {
                const struct zsa_nav_present *p = (const struct zsa_nav_present *)frame->payload;
                nav_device_type = p->device_type;
                nav_device_seen = true;
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
                /* Re-derive on the next loop iteration, not the next 5 s
                 * tick: this is the event that makes the rate change. The
                 * budget is a level resent every second, so only a real change
                 * may do it — otherwise every resend re-commands both halves,
                 * and the right answers each command with a PMIC readback. */
                if (!pwr_budget_seen || p->budget_ma != nrf_power_budget_ma) {
                    last_pwr_maintain_ms = 0;
                }
                nrf_power_budget_ma = p->budget_ma;
                pwr_budget_seen     = true;
            }
            break;

        case ZSA_MSG_EVT_VBUS_STATE:
            if (frame->len >= sizeof(struct zsa_vbus_state)) {
                const struct zsa_vbus_state *v = (const struct zsa_vbus_state *)frame->payload;
                bool present = v->present != 0;
                if (present != nrf_vbus_present) {
                    nrf_vbus_present = present;
                    if (!present) {
                        usb_configured = false;
                    }
                    /* The PMICs dropped their input limit back to the startup
                     * value when VBUS went away, so whatever we commanded is
                     * no longer in force. Without this, replugging the same
                     * source computes the same limits, sees no change, and
                     * leaves the board on the default profile until the next
                     * 5 s tick. */
                    limits_commanded = false;
                    hibernate_sent   = false;
                    zsa_wireless_vbus_changed(present);
                }
            }
            break;

        case ZSA_MSG_EVT_DFU_STATE:
            if (frame->len >= 1) {
                const struct zsa_dfu_progress *d = (const struct zsa_dfu_progress *)frame->payload;
                last_dfu_state_ms = timer_read32();
                dfu_set_active(d->active != 0);
                nrf_dfu_keep_awake = frame->len >= 4 && (d->flags & ZSA_DFU_FLAG_KEEP_AWAKE) != 0;
                /* Older senders carry `active` alone: no phase to show. */
                if (frame->len >= 3) {
                    nrf_dfu_target = d->target;
                    nrf_dfu_phase  = d->phase;
                } else {
                    nrf_dfu_target = ZSA_DFU_TARGET_NONE;
                    nrf_dfu_phase  = ZSA_DFU_PHASE_IDLE;
                }
                last_dfu_phase_ms = timer_read32();
                dfu_phase_seen    = true;
            }
            break;

        case ZSA_MSG_EVT_LINK_STATE:
            if (frame->len >= sizeof(struct zsa_link_state)) {
                const struct zsa_link_state *l = (const struct zsa_link_state *)frame->payload;
                nrf_link_state     = l->state;
                last_link_state_ms = timer_read32();
                link_state_seen    = true;
                zsa_nav_link_received(&nav_link_age, last_link_state_ms);
            }
            break;

        case ZSA_MSG_DFU_ENTER:
            /* Frames are dispatched from bluetooth_task(), so this resets
             * inline rather than deferring. Releases any held key first — the
             * nRF keeps reporting the last HID state, and the STM32 is about
             * to stop sending. No ack: ignition announces DFU_READY itself. */
            zsa_log("dfu: entering bootloader");
            clear_keyboard();
            zsa_wireless_enter_bootloader();
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

bool zsa_wireless_battery_fresh(void) {
    return link_ready && battery_seen && timer_elapsed32(last_battery_ms) < BATTERY_STALE_MS;
}

bool zsa_wireless_right_telemetry(struct zsa_right_telemetry *out) {
    if (!right_tlm_valid) {
        return false;
    }
    *out = right_tlm;
    return true;
}

uint16_t zsa_wireless_power_budget_ma(void) {
    return nrf_power_budget_ma;
}

bool zsa_wireless_vbus_present(void) {
    return nrf_vbus_present;
}

#ifdef RGB_MATRIX_ENABLE
uint8_t zsa_wireless_led_val(void) {
    uint8_t v = rgb_matrix_config.hsv.v;

    if (nrf_vbus_present && v > ZSA_WIRED_LED_VAL_CAP) {
        v = ZSA_WIRED_LED_VAL_CAP;
    }
    return v;
}

/* Strong override of the matrix's weak HSV->RGB hook, so the built-in effect
 * engine is capped too: the keymap only paints some layers itself and leaves
 * the rest to the animations. Every effect runner converts through here. */
rgb_t rgb_matrix_hsv_to_rgb(hsv_t hsv);
rgb_t rgb_matrix_hsv_to_rgb(hsv_t hsv) {
    uint8_t cap = zsa_wireless_led_val();

    if (hsv.v > cap) {
        hsv.v = cap;
    }
    return hsv_to_rgb(hsv);
}
#endif

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

void zsa_wireless_cmd_ble_switch_host(void) {
    send_command(ZSA_MSG_CMD_BLE_SWITCH_HOST);
}

bool zsa_wireless_link_ready(void) {
    return link_ready;
}

uint8_t zsa_wireless_nav_device_type(void) {
    return nav_device_type;
}

bool zsa_wireless_nav_device_seen(void) {
    return nav_device_seen;
}

/* Weak defaults: that nav driver is not linked into this build, so it has not
 * looked. Distinct from "looked and found nothing" — see enum zsa_nav_probe. */
__attribute__((weak)) uint8_t zsa_nav_trackball_probe_state(void) {
    return ZSA_NAV_PROBE_UNSUPPORTED;
}

__attribute__((weak)) uint8_t zsa_nav_trackpad_probe_state(void) {
    return ZSA_NAV_PROBE_UNSUPPORTED;
}

__attribute__((weak)) void zsa_nav_trackball_probe_local(void) {}

__attribute__((weak)) void zsa_nav_trackpad_probe_local(void) {}

__attribute__((weak)) bool zsa_nav_trackball_local_pending(void) {
    return false;
}

__attribute__((weak)) bool zsa_nav_trackpad_local_pending(void) {
    return false;
}

__attribute__((weak)) const zsa_nav_sleep_ops_t *zsa_nav_trackball_sleep_ops(void) {
    return NULL;
}

__attribute__((weak)) const zsa_nav_sleep_ops_t *zsa_nav_trackpad_sleep_ops(void) {
    return NULL;
}

/* ---- Nav presence (telemetry nav_* and CMD_NAV_STATE) ---- *
 *
 * zsa_nav_presence() decides; this feeds it. The right half's report from the
 * left nRF counts only while EVT_LINK_STATE is fresh (zsa_nav_link_fresh(),
 * held through Sleep) and says OK, and
 * ZSA_NAV_RIGHT_SETTLE_MS after that and the announce ack both hold, so the
 * right has had time to probe. The level goes to the nRF on every change and
 * every ZSA_NAV_STATE_INTERVAL_MS, at least NAV_STATE_STAGGER_MS after the
 * last 1 s periodic frame (TRRS_STATE, SCAN_DIAG, RIGHT_TELEMETRY). */
#define NAV_STATE_STAGGER_MS 250
#define NAV_STATE_MIN_GAP_MS 100

static bool     nav_right_linked;    /* fresh OK link state, announce acked */
static uint32_t nav_right_linked_ms; /* when nav_right_linked last became true */
static uint32_t periodic_tx_ms;      /* last 1 s periodic frame to the nRF */

static bool link_state_fresh(void) {
    return link_ready && zsa_nav_link_fresh(&nav_link_age, timer_read32(), ZSA_LINK_STATE_STALE_MS);
}

static struct zsa_nav_state nav_state_now(void) {
    bool fresh  = link_state_fresh();
    bool linked = fresh && nrf_link_state == ZSA_LINK_STATE_OK && nav_announce_acked && nav_device_seen;
    if (linked && !nav_right_linked) {
        nav_right_linked_ms = timer_read32();
    }
    nav_right_linked = linked;

    struct zsa_nav_inputs in = {
        .build         = nav_cap_mask,
        .ball          = zsa_nav_trackball_probe_state(),
        .pad           = zsa_nav_trackpad_probe_state(),
        .right_on_bus  = zsa_stm32_owns_right_bus(),
        .nrf_seen      = nav_device_seen,
        .link_fresh    = fresh,
        .right_linked  = fresh && nrf_link_state == ZSA_LINK_STATE_OK,
        .right_settled = linked && timer_elapsed32(nav_right_linked_ms) >= ZSA_NAV_RIGHT_SETTLE_MS,
        .nrf_device    = nav_device_type,
    };
    return zsa_nav_presence(&in);
}

static void nav_state_task(void) {
    static struct zsa_nav_state sent;
    static uint32_t             sent_ms;
    static bool                 primed;

    if (!link_ready) {
        return;
    }

    struct zsa_nav_state now     = nav_state_now();
    bool                 changed = memcmp(&now, &sent, sizeof(now)) != 0;
    uint32_t             since   = timer_elapsed32(sent_ms);

    bool quiet = timer_elapsed32(periodic_tx_ms) >= NAV_STATE_STAGGER_MS;

    if (!primed) {
        if (!quiet) {
            return;
        }
    } else if (changed) {
        if (since < NAV_STATE_MIN_GAP_MS) {
            return;
        }
    } else if (since < ZSA_NAV_STATE_INTERVAL_MS || !quiet) {
        return;
    }

    zsa_wireless_send_frame(ZSA_MSG_CMD_NAV_STATE, (const uint8_t *)&now, sizeof(now));
    sent    = now;
    sent_ms = timer_read32();
    primed  = true;
}

_Static_assert(sizeof(struct zsa_nav_state) == ZSA_NAV_STATE_SIZE, "nav state payload size");

/* ---- Telemetry (oryx raw-HID ORYX_CMD_GET_TELEMETRY) ---- */

#ifdef COMMUNITY_MODULE_ORYX_ENABLE
#    include "oryx_telemetry.h"
#    include "oryx_charge_telemetry.h"
#    include "oryx_pmic_rails.h"
#    include "oryx_scan_diag.h"

/* Same rule and window as zsa_wireless_battery_fresh(), on the detail's own
 * timestamp: a left half too old to send EVT_BATTERY_DETAIL keeps the battery
 * levels fresh while never producing a detail block. */
static bool detail_fresh(void) {
    return link_ready && detail_seen && timer_elapsed32(last_detail_ms) < BATTERY_STALE_MS;
}

/* Same rule and window as detail_fresh(), on its own timestamp. */
static bool ble_identity_fresh(void) {
    return link_ready && ble_identity_seen && timer_elapsed32(last_ble_identity_ms) < BATTERY_STALE_MS;
}

/* Same rule and window as detail_fresh(), on its own timestamp. */
static bool pmic_rails_fresh(void) {
    return link_ready && pmic_rails_seen && timer_elapsed32(last_pmic_rails_ms) < BATTERY_STALE_MS;
}

/* zsa_nav_presence() fills the telemetry nav fields as they are. */
_Static_assert(ORYX_TLM_NAV_TRACKBALL == ZSA_NAV_CAP_TRACKBALL && ORYX_TLM_NAV_TRACKPAD == ZSA_NAV_CAP_TRACKPAD, "nav mask bits");
_Static_assert(ORYX_TLM_NAV_UNKNOWN == ZSA_NAV_STATE_UNKNOWN, "nav unknown value");

/* Only ever called where both answers are actually known — the KNOWN bit is
 * what stops a host reading a cleared bit as a confident "no". */
static uint8_t bat_flags(bool charging, bool usb) {
    return ORYX_TLM_BAT_KNOWN | (charging ? ORYX_TLM_BAT_CHARGING : 0) | (usb ? ORYX_TLM_BAT_USB : 0);
}

static void telemetry_battery(oryx_telemetry_t *tlm) {
    /* Left: the nRF's own half, over UART. Only while it is still refreshing —
     * a cached level from a link that died is not a reading. Anything outside
     * 0-100 is the unknown sentinel. */
    if (zsa_wireless_battery_fresh() && nrf_battery_soc <= 100) {
        tlm->battery_left        = nrf_battery_soc;
        tlm->battery_left_flags  = bat_flags(nrf_battery_flags & ZSA_BAT_FLAG_CHARGING, nrf_battery_flags & ZSA_BAT_FLAG_USB);
        tlm->battery_left_source = ORYX_TLM_SRC_NRF;
    }

    /* Right: prefer the wired read, which is this half answering for itself
     * right now. */
    struct zsa_right_telemetry rt;
    uint8_t                    flags;
    if (zsa_wireless_right_telemetry(&rt)) {
        int soc                   = zsa_right_telemetry_soc(&rt);
        tlm->battery_right        = (soc < 0) ? ORYX_TLM_LEVEL_UNKNOWN : (uint8_t)soc;
        tlm->battery_right_source = ORYX_TLM_SRC_WIRED;
        if (zsa_right_telemetry_flags(&rt, &flags)) {
            tlm->battery_right_flags = bat_flags(flags & ZSA_RIGHT_TLM_FLAG_CHARGING, flags & ZSA_RIGHT_TLM_FLAG_USB);
        }
        return;
    }

    /* Otherwise the ESB relay. Anything outside 0-100 is the unknown sentinel;
     * 0 is treated as unknown too, for left firmware that predates it and sent
     * 0 both for a right half that never reported and for a flat one. The
     * charging/USB bits ride the same report, but only newer left firmware
     * carries them. */
    if (zsa_wireless_battery_fresh() && nrf_battery_right_soc > 0 && nrf_battery_right_soc <= 100) {
        tlm->battery_right        = nrf_battery_right_soc;
        tlm->battery_right_source = ORYX_TLM_SRC_ESB;
        if (nrf_battery_right_flags_known) {
            tlm->battery_right_flags = bat_flags(nrf_battery_right_flags & ZSA_BAT_FLAG_CHARGING, nrf_battery_right_flags & ZSA_BAT_FLAG_USB);
        }
    }
}

static void telemetry_nav(oryx_telemetry_t *tlm) {
    struct zsa_nav_state nav = nav_state_now();
    tlm->nav_build = nav.build;
    tlm->nav_left  = nav.left;
    tlm->nav_right = nav.right;
    tlm->nav_bus   = nav.bus;
}

/* What the charging policy decided and what came of it. Reports what was
 * actually commanded, not what would be commanded now: until the link is up
 * and something has gone out, the answer is that we do not know. */
static void telemetry_charging(oryx_telemetry_t *tlm) {
    if (!link_ready) {
        return;
    }

    tlm->power_budget_ma = nrf_power_budget_ma;

    /* The left's pair. The right's is lower and is not reported: the struct has
     * one pair of fields and 13 reserved bytes, and adding a second pair is a
     * wire change for later. */
    if (limits_commanded) {
        tlm->charge_limit_ma   = commanded_limits.vbus_limit_ma;
        tlm->charge_current_ma = commanded_limits.charge_ma;
    }

    if (!nrf_vbus_present) {
        tlm->charge_state = ORYX_TLM_CHG_BATTERY;
    } else if (zsa_wireless_battery_fresh() && !(nrf_battery_flags & ZSA_BAT_FLAG_CHARGING)) {
        /* The nRF reports charging only while current is really flowing, so a
         * source with the flag clear means the charger terminated. */
        tlm->charge_state = ORYX_TLM_CHG_FULL;
    } else if (zsa_wireless_battery_fresh()) {
        tlm->charge_state = ORYX_TLM_CHG_CHARGING;
    }
}

/* The nRF's answer or nothing: an address cached from a link that died names a
 * host slot the board may already have left. */
static void telemetry_ble_identity(oryx_telemetry_t *tlm) {
    if (!ble_identity_fresh() || nrf_ble_identity.addr_type == ZSA_BLE_ADDR_TYPE_UNKNOWN) {
        return;
    }

    tlm->ble_addr_type  = nrf_ble_identity.addr_type;
    tlm->ble_slot       = nrf_ble_identity.slot;
    tlm->ble_slot_count = nrf_ble_identity.slot_count;
    memcpy(tlm->ble_addr, nrf_ble_identity.addr, sizeof(tlm->ble_addr));
}

/* Strong override of oryx.c's weak hook. */
void oryx_telemetry_fill(oryx_telemetry_t *tlm) {
    oryx_telemetry_reset(tlm);
    telemetry_battery(tlm);
    telemetry_nav(tlm);
    telemetry_charging(tlm);
    telemetry_ble_identity(tlm);
}

/* The oryx module re-declares the block rather than including the wire header,
 * so the two layouts are only ever the same by inspection — check it here,
 * where both are visible, before copying one into the other. */
_Static_assert(sizeof(oryx_battery_detail_t) == sizeof(struct zsa_battery_detail), "oryx charge telemetry block must match the wire block");

/* Strong override of oryx.c's weak hook. */
void oryx_charge_telemetry_fill(oryx_charge_telemetry_t *ctlm) {
    oryx_charge_telemetry_reset(ctlm);

    /* Left: the nRF's own half, over UART. Only while it is still refreshing —
     * a cached block from a link that died is not a reading. */
    if (detail_fresh()) {
        memcpy(&ctlm->left, &nrf_detail_left, sizeof(ctlm->left));
        ctlm->left_source = ORYX_TLM_SRC_NRF;
    }

    /* Right: prefer the wired read, which is this half answering for itself
     * right now. A right half too old to send a detail block falls through. */
    struct zsa_right_telemetry rt;
    struct zsa_battery_detail  rd;
    if (zsa_wireless_right_telemetry(&rt) && zsa_right_telemetry_detail(&rt, &rd)) {
        memcpy(&ctlm->right, &rd, sizeof(ctlm->right));
        ctlm->right_source = ORYX_TLM_SRC_WIRED;
        return;
    }

    /* Otherwise the ESB relay, under the same condition telemetry_battery()
     * accepts a relayed level: the left only carries the right's block while
     * the radio link does. */
    if (detail_fresh() && zsa_wireless_battery_fresh() && nrf_battery_right_soc > 0 && nrf_battery_right_soc <= 100) {
        memcpy(&ctlm->right, &nrf_detail_right, sizeof(ctlm->right));
        ctlm->right_source = ORYX_TLM_SRC_ESB;
    }
}

/* Same re-declaration, same reason: check the two layouts here, where both are
 * visible, before copying one into the other. */
_Static_assert(sizeof(oryx_pmic_rails_block_t) == sizeof(struct zsa_pmic_rails), "oryx pmic rails block must match the wire block");

/* Strong override of oryx.c's weak hook. */
void oryx_pmic_rails_fill(oryx_pmic_rails_t *rails) {
    oryx_pmic_rails_reset(rails);

    /* Left: the nRF's own half, over UART, while it is still refreshing. */
    if (pmic_rails_fresh() && (nrf_pmic_rails.left.flags & ZSA_PMIC_RAILS_PRESENT)) {
        memcpy(&rails->left, &nrf_pmic_rails.left, sizeof(rails->left));
    }

    /* Right: prefer the wired read, which is this half answering for itself
     * right now. A block without PRESENT is a right half too old to fill the
     * seat, so it falls through to the relay rather than reporting 0 V. */
    struct zsa_right_telemetry rt;
    struct zsa_pmic_rails      rr;
    if (zsa_wireless_right_telemetry(&rt) && zsa_right_telemetry_rails(&rt, &rr) && (rr.flags & ZSA_PMIC_RAILS_PRESENT)) {
        memcpy(&rails->right, &rr, sizeof(rails->right));
        return;
    }

    /* Otherwise whatever the left relayed, radio or cable — it merges both
     * sources itself and marks the block absent when it has neither. */
    if (pmic_rails_fresh() && (nrf_pmic_rails.right.flags & ZSA_PMIC_RAILS_PRESENT)) {
        memcpy(&rails->right, &nrf_pmic_rails.right, sizeof(rails->right));
    }
}

/* The oryx block re-declares the wire struct; both are visible here. */
_Static_assert(sizeof(oryx_scan_diag_block_t) == sizeof(struct zsa_scan_diag), "oryx scan diag block must match the wire block");

/* Strong override of oryx.c's weak hook: the last finalized window. */
void oryx_scan_diag_fill(oryx_scan_diag_t *diag) {
    oryx_scan_diag_reset(diag);
    memcpy(&diag->diag, &scan_diag_last, sizeof(diag->diag));
}
#endif // COMMUNITY_MODULE_ORYX_ENABLE

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

static void send_trrs_state(bool plugged) {
    struct zsa_trrs_state payload = { .plugged = plugged ? 1 : 0 };
    zsa_wireless_send_frame(ZSA_MSG_CMD_TRRS_STATE, (const uint8_t *)&payload, sizeof(payload));
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

    struct zsa_bus_grant payload = { .granted = granted ? 1 : 0 };
    zsa_wireless_send_frame(ZSA_MSG_CMD_BUS_GRANT, (const uint8_t *)&payload, sizeof(payload));
    last_sent_ms = timer_read32();
    last_granted = granted;
    primed       = true;
}

/* Cable presence as a level: sent on every edge and refreshed on a slow tick,
 * so a left half that rebooted picks it up without waiting for an unplug. */
static void trrs_state_task(void) {
    static uint32_t last_sent_ms;
    static bool     last_plugged;
    static bool     primed;

    bool plugged = zsa_trrs_plugged();

    if (primed && plugged == last_plugged && timer_elapsed32(last_sent_ms) < ZSA_TRRS_STATE_INTERVAL_MS) {
        return;
    }

    send_trrs_state(plugged);
    last_sent_ms   = timer_read32();
    periodic_tx_ms = last_sent_ms;
    last_plugged   = plugged;
    primed         = true;
}

/* ---- Power stages ----
 *
 * Active: everything on. Sleep, after ZSA_SLEEP_TIMEOUT_MS without input: the
 * matrix and status LEDs off, the nRF told (CMD_IDLE_STATE 2), and, wireless on
 * battery, the STM32 in STOP between wakes (sleep_stop_task). Hibernate, after
 * ZSA_HIBERNATE_TIMEOUT_MS without input: CMD_HIBERNATE, and the nRF ships both
 * halves. Any input in Sleep goes back to Active; nothing comes back from
 * Hibernate but the hardware wake.
 *
 * QMK's timer does not run in STOP, so time spent there is counted separately
 * (sleep_stop_ms, from the RTC at the calibrated LSI frequency) and added for
 * the hibernate step. */
static uint32_t sleep_idle_base_ms; /* idle time when Sleep began */
static uint32_t sleep_entered_ms;
static uint32_t sleep_stop_ms;      /* time in STOP since Sleep began */
static uint32_t sleep_awake_ms;     /* start of the current running window */
static uint32_t sleep_window_ms;    /* its length */
static uint32_t sleep_input_mark;   /* last_input_activity_time() when it began */
#ifdef RGB_MATRIX_ENABLE
static bool sleep_rgb_on; /* the matrix was on when Sleep began */
#endif
/* Start of the idle count when it is not the last input: a transport switch,
 * or the end of an update, a pairing or a Sleep left for another reason. */
static uint32_t idle_floor_ms;

/* ---- Sleep diagnostics ----
 *
 * Counters since boot for the STOP path in Sleep. With ZSA_SLEEP_DIAG they
 * replace the CMD_SCAN_DIAG block: one page per window, version
 * SLEEP_DIAG_PAGE_BASE + 1..SLEEP_DIAG_PAGES (tools/sleep_diag.py). */
#define SLEEP_DIAG_PAGE_BASE 0xA0u
#define SLEEP_DIAG_PAGES 6u

static struct {
    uint16_t stop_entries;
    uint16_t stop_nav_entries;
    uint16_t wake_input;
    uint16_t wake_uart;
    uint16_t wake_timeout;
    uint16_t wake_nav_activity;
    uint16_t wake_nav_error;
    uint8_t  wake_none; /* STOP skipped, input pending; saturates */
    uint16_t polls_idle;
    uint16_t polls_activity;
    uint16_t polls_error;
    uint8_t  last_poll;
    uint8_t  raw[6];
    uint8_t  last_block;
    uint8_t  nav_fail; /* 1 enter refused, 2 poll errors */
    uint8_t  ends[4];  /* newest first */
    uint8_t  i2c_err;  /* see stop_i2c_note_error() */
    uint8_t  i2c_snap; /* I2CD1.state | PE << 4 | BUSY << 5, at SB_I2C_BUSY */
    uint8_t  sleeps;
    uint32_t stop_ms;
    uint32_t sleep_ms;
    uint32_t block_ms[SB_COUNT];
    uint32_t block_prev_ms;
    uint8_t  block_prev;
    bool     block_timing;
    int16_t  motion_x; /* last normal-path motion read in Sleep */
    int16_t  motion_y;
    uint16_t motion_reads;
    uint16_t lsi_hz;      /* calibrated LSI in use, 0 = nominal */
    uint16_t lsi_last_hz; /* last measurement, accepted or not */
    uint16_t lsi_ok;
    uint16_t lsi_bad;
    uint16_t stop_by_rtc;   /* STOP time from the RTC calendar */
    uint16_t stop_by_wakes; /* STOP time from the RTC period count */
    uint16_t nav_noise_drops; /* nav motion held by the wake filter, dropped */
    uint8_t  page;
} sdiag;

/* Charges the time since the previous pass to its reason. */
static void sdiag_block(uint8_t reason) {
    uint32_t now = timer_read32();
    if (sdiag.block_timing) {
        uint32_t dt = now - sdiag.block_prev_ms;
        sdiag.block_ms[sdiag.block_prev] += dt;
        sdiag.sleep_ms += dt;
    }
    sdiag.block_prev_ms = now;
    sdiag.block_prev    = reason;
    if (reason != SB_NONE && reason != SB_WINDOW) {
        sdiag.last_block = reason;
    }
}

static void sdiag_end(uint8_t why) {
    sdiag_block(SB_NONE);
    sdiag.block_timing = false;
    for (uint8_t i = sizeof(sdiag.ends) - 1; i > 0; i--) {
        sdiag.ends[i] = sdiag.ends[i - 1];
    }
    sdiag.ends[0] = why;
}

static void sdiag_inc(uint16_t *v) {
    if (*v != UINT16_MAX) {
        (*v)++;
    }
}

void zsa_nav_sleep_note_raw(const uint8_t *raw, uint8_t len) {
    for (uint8_t i = 0; i < sizeof(sdiag.raw); i++) {
        sdiag.raw[i] = i < len ? raw[i] : 0;
    }
}

void zsa_nav_sleep_note_motion(int16_t x, int16_t y) {
    if (idle_step == ZSA_IDLE_ACTIVE) {
        return;
    }
    sdiag.motion_x = x;
    sdiag.motion_y = y;
    sdiag_inc(&sdiag.motion_reads);
}


/* ---- Left nav device in Sleep (see wireless.h) ---- */

/* Wake filter state, reset at every STOP. */
static struct {
    int32_t dx;
    int32_t dy;
    uint8_t polls; /* motion polls in a row */
    bool    fed;   /* the current poll called zsa_nav_sleep_filter() */
} nav_wake_filter;

static inline void nav_wake_filter_drop(void) {
    if (nav_wake_filter.polls != 0) {
        sdiag_inc(&sdiag.nav_noise_drops);
    }
    nav_wake_filter.dx    = 0;
    nav_wake_filter.dy    = 0;
    nav_wake_filter.polls = 0;
}

uint8_t zsa_nav_sleep_filter(int16_t dx, int16_t dy, int32_t *wake_dx, int32_t *wake_dy) {
    nav_wake_filter.fed = true;
    nav_wake_filter.dx += dx;
    nav_wake_filter.dy += dy;
    nav_wake_filter.polls++;
    int32_t moved = (dx < 0 ? -(int32_t)dx : dx) + (dy < 0 ? -(int32_t)dy : dy);
    if (moved < ZSA_NAV_SLEEP_WAKE_COUNTS && nav_wake_filter.polls < ZSA_NAV_SLEEP_WAKE_POLLS) {
        return ZSA_NAV_SLEEP_IDLE;
    }
    *wake_dx              = nav_wake_filter.dx;
    *wake_dy              = nav_wake_filter.dy;
    nav_wake_filter.dx    = 0;
    nav_wake_filter.dy    = 0;
    nav_wake_filter.polls = 0;
    return ZSA_NAV_SLEEP_ACTIVITY;
}

/* Consecutive poll errors after which the device is no longer polled for the
 * rest of the Sleep, and the STM32 stays running. */
#define NAV_SLEEP_MAX_POLL_ERRORS 3

static const zsa_nav_sleep_ops_t *nav_sleep_ops; /* entered, exit pending */
static bool    nav_sleep_failed;                 /* no polling for this Sleep */
static uint8_t nav_sleep_errors;

/* Hooks of the device on the STM32's own bus, NULL without one or without
 * hooks. */
static const zsa_nav_sleep_ops_t *nav_left_sleep_ops(void) {
    bool ball = zsa_nav_trackball_probe_state() == ZSA_NAV_PROBE_LOCAL;
    bool pad  = zsa_nav_trackpad_probe_state() == ZSA_NAV_PROBE_LOCAL;
    if (ball && !pad) {
        return zsa_nav_trackball_sleep_ops();
    }
    if (pad && !ball) {
        return zsa_nav_trackpad_sleep_ops();
    }
    return NULL;
}

static void nav_sleep_end(void) {
    if (nav_sleep_ops != NULL) {
        const zsa_nav_sleep_ops_t *ops = nav_sleep_ops;
        nav_sleep_ops                  = NULL;
        ops->exit();
    }
}

/* The STOP poll for the left nav device: NULL when there is none, and the
 * device is ready otherwise. Not SB_NONE: the STM32 stays running this pass,
 * for that reason. */
static uint8_t nav_sleep_prepare(uint8_t (**poll)(void)) {
    *poll = NULL;
    if (!nav_left_local()) {
        nav_sleep_end();
        return SB_NONE;
    }
    const zsa_nav_sleep_ops_t *ops = nav_left_sleep_ops();
    if (ops == NULL) {
        return SB_NAV_NO_HOOK;
    }
    if (nav_sleep_failed) {
        return SB_NAV_FAILED;
    }
    if (nav_sleep_ops != ops) {
        nav_sleep_end();
        if (!ops->enter()) {
            zsa_log("power: left nav refused sleep polling");
            nav_sleep_failed = true;
            sdiag.nav_fail   = 1;
            return SB_NAV_FAILED;
        }
        nav_sleep_ops = ops;
    }
    if (!ops->idle()) {
        return SB_NAV_NOT_IDLE;
    }
    *poll = ops->poll;
    return SB_NONE;
}

/* Time since Sleep began, STOP included. */
static uint32_t sleep_clock_ms(void) {
    return timer_elapsed32(sleep_entered_ms) + sleep_stop_ms;
}

/* Keyboards override this to drive their own status LEDs. */
__attribute__((weak)) void zsa_wireless_sleep_changed(bool sleeping) { (void)sleeping; }

static void sleep_enter(uint32_t idle_ms) {
    zsa_log("power: sleep");
    idle_step = ZSA_IDLE_SLEEP;
    send_idle_state(idle_step);
    zsa_nav_link_sleep(&nav_link_age, true, timer_read32(), ZSA_LINK_STATE_STALE_MS);

    sleep_idle_base_ms = idle_ms;
    sleep_entered_ms   = timer_read32();
    sleep_awake_ms     = sleep_entered_ms;
    sleep_window_ms    = ZSA_SLEEP_WAKE_WINDOW_MS;
    sleep_stop_ms      = 0;
    sleep_input_mark   = last_input_activity_time();
    hibernate_sent     = false;
    nav_sleep_due_ms   = ZSA_NAV_PROBE_SLEEP_INTERVAL_MS;
    nav_sleep_failed   = false;
    nav_sleep_errors   = 0;

    sdiag.sleeps++;
    sdiag.block_timing = true;
    sdiag.block_prev   = SB_WINDOW;
    sdiag.block_prev_ms = timer_read32();

#ifdef RGB_MATRIX_ENABLE
    /* rgb_matrix_task() renders and flushes black while disabled. */
    sleep_rgb_on = rgb_matrix_is_enabled();
    if (sleep_rgb_on) {
        rgb_matrix_disable_noeeprom();
    }
    rgb_matrix_set_color_all(0, 0, 0);
#endif
    zsa_wireless_sleep_changed(true);
}

/* Back to Active, restoring what sleep_enter() turned off. No-op when Active.
 * The LED drivers are resynced when the nRF reports the LED rail back on. */
static void sleep_exit_for(uint8_t why) {
    if (idle_step == ZSA_IDLE_ACTIVE) {
        return;
    }
    sdiag_end(why);
    zsa_log("power: active");
    idle_step          = ZSA_IDLE_ACTIVE;
    idle_floor_ms      = timer_read32();
    nav_window_request = true;
    send_idle_state(idle_step);
    zsa_nav_link_sleep(&nav_link_age, false, timer_read32(), ZSA_LINK_STATE_STALE_MS);
    nav_sleep_end();

#ifdef RGB_MATRIX_ENABLE
    if (sleep_rgb_on && power_gate_allows_rgb()) {
        rgb_matrix_enable_noeeprom();
    }
#endif
    zsa_wireless_sleep_changed(false);
}

/* Called on both USB<->wireless transitions. Nav modules override this to
 * re-init their sensor: the transport switch resets the shared i2c bus, which
 * leaves the sensor unconfigured without raising an i2c error. */
__attribute__((weak)) void zsa_nav_transport_changed(void) {}

#ifdef ZSA_RTT_DEBUG
/* Strong override of busdev's weak transition hook: every device state change
 * on the shared bus, with its reason and the address in use, onto the RTT
 * trace. */
void busdev_on_transition(busdev_id_t id, busdev_state_t old_state, busdev_state_t new_state, uint8_t reason) {
    zsa_rtt_event3(busdev_name(id), ((int)old_state << 8) | (int)new_state, reason, busdev_addr(id));
}
#endif

/* Keyboards override this to drive their own USB D+ pullup from VBUS state. */
__attribute__((weak)) void zsa_wireless_vbus_changed(bool present) { (void)present; }

/* Keyboards with a bootloader QMK can hand off to override this. */
__attribute__((weak)) void zsa_wireless_enter_bootloader(void) {}

/* ---- Firmware-update status LEDs ---- */

/* Keyboards override this to drive their own two status LED pins. */
__attribute__((weak)) void zsa_wireless_dfu_led(uint8_t green, uint8_t red) {
    (void)green;
    (void)red;
}

static bool    dfu_led_green_owned;
static bool    dfu_led_red_owned;
static uint8_t dfu_led_green_last = ZSA_STATUS_LED_RELEASE;
static uint8_t dfu_led_red_last   = ZSA_STATUS_LED_RELEASE;

bool zsa_wireless_dfu_led_green_owned(void) {
    return dfu_led_green_owned;
}

bool zsa_wireless_dfu_led_red_owned(void) {
    return dfu_led_red_owned;
}

/* The reported phase is only about this half's own image: a relay to the
 * right half, or no target at all, has nothing to show on these LEDs. Held
 * for ZSA_DFU_PHASE_STALE_MS without a refresh, then released. */
static uint8_t dfu_left_phase(void) {
    if (!dfu_phase_seen || nrf_dfu_target != ZSA_DFU_TARGET_LEFT) {
        return ZSA_DFU_PHASE_IDLE;
    }
    if (timer_elapsed32(last_dfu_phase_ms) >= ZSA_DFU_PHASE_STALE_MS) {
        return ZSA_DFU_PHASE_IDLE;
    }
    return nrf_dfu_phase;
}

/* Renders the update vocabulary on the two left LEDs: green fast-blinks while
 * the image is written and holds solid while it settles; red is solid on a
 * failed update and slow-blinks on a protocol mismatch, which is orthogonal to
 * the phase and shows whenever the phase is not FAILED. Blink phases come from
 * the absolute clock like the pairing blink, so the rate does not depend on
 * how often this runs. */
static void dfu_led_task(void) {
    uint32_t now   = timer_read32();
    uint8_t  phase = dfu_left_phase();
    uint8_t  green;
    uint8_t  red;

    switch (phase) {
        case ZSA_DFU_PHASE_UPLOAD:
            green = ((now / ZSA_DFU_LED_FAST_MS) & 1u) ? ZSA_STATUS_LED_ON : ZSA_STATUS_LED_OFF;
            break;
        case ZSA_DFU_PHASE_SETTLE:
            green = ZSA_STATUS_LED_ON;
            break;
        default:
            green = ZSA_STATUS_LED_RELEASE;
            break;
    }

    /* The boot power gate outranks both red indications: it only ever runs in
     * the first seconds after boot, where no update can be in flight. */
    if (power_gate == PG_CRITICAL) {
        red = power_gate_red();
    } else if (phase == ZSA_DFU_PHASE_FAILED) {
        red = ZSA_STATUS_LED_ON;
    } else if (zsa_wireless_link_mismatch()) {
        red = ((now / ZSA_DFU_LED_SLOW_MS) & 1u) ? ZSA_STATUS_LED_ON : ZSA_STATUS_LED_OFF;
    } else {
        red = ZSA_STATUS_LED_RELEASE;
    }

    dfu_led_green_owned = (green != ZSA_STATUS_LED_RELEASE);
    dfu_led_red_owned   = (red != ZSA_STATUS_LED_RELEASE);

    if (green != dfu_led_green_last || red != dfu_led_red_last) {
        dfu_led_green_last = green;
        dfu_led_red_last   = red;
        zsa_wireless_dfu_led(green, red);
    }
}

/* ---- Pairing status LED ---- */

/* Keyboards override this to drive their own status LED pin. */
__attribute__((weak)) void zsa_wireless_pair_led(uint8_t state) { (void)state; }

static bool    pair_led_owned;
static bool    pair_led_deferred; /* the update indication holds the pin */
static uint8_t pair_led_last = ZSA_STATUS_LED_RELEASE;

bool zsa_wireless_pair_led_active(void) {
    return pair_led_owned;
}

/* Blinks the status LED: fast while a pairing is in progress, slow while
 * discoverable, released otherwise. The phase comes from the absolute clock
 * rather than elapsed-time toggling, so the blink rate is independent of how
 * often bluetooth_task() runs.
 *
 * Shares its pin with the update indication, which outranks it: while that
 * owns the green LED nothing is written here, and the current state is
 * re-issued as soon as it hands the pin back. pair_led_owned keeps tracking
 * either way, so the layer indicator stays off the pin. */
static void pair_led_task(void) {
    bool    pairing = zsa_wireless_pairing_active();
    bool    advert  = pair_advertising();
    uint8_t want;

    if (pairing || advert) {
        uint32_t half  = pairing ? ZSA_PAIR_LED_FAST_MS : ZSA_PAIR_LED_SLOW_MS;
        want           = ((timer_read32() / half) & 1u) ? ZSA_STATUS_LED_ON : ZSA_STATUS_LED_OFF;
        pair_led_owned = true;
    } else {
        want           = ZSA_STATUS_LED_RELEASE;
        pair_led_owned = false;
    }

    if (zsa_wireless_dfu_led_green_owned()) {
        pair_led_deferred = true;
        return;
    }

    if (want != pair_led_last || pair_led_deferred) {
        pair_led_deferred = false;
        pair_led_last     = want;
        zsa_wireless_pair_led(want);
    }
}

/* Both LED engines, in arbitration order. */
static void status_led_task(void) {
    dfu_led_task();
    pair_led_task();
}

#if defined(MCU_STM32) && defined(STM32F3XX)
/* ---- STM32 STOP mode ----
 *
 * Left matrix (voyager_tng/matrix.c): strobes B10-B15 drive high, sense inputs
 * A0-A3, A6, A7, B0 are pulled down, so a key press is a rising edge. A0 and B0
 * share EXTI0, which goes to B0 (three keys); the one key on A0 is read at
 * every RTC wakeup instead. USART1 RX (PA10, EXTI10) wakes on the falling edge
 * of the nRF's wake preamble or of any frame. The IWDG keeps counting in STOP,
 * so the RTC wakeup timer (LSI, EXTI20) wakes every STOP_RTC_PERIOD_MS to kick
 * it and to read A0, and goes straight back to STOP.
 *
 * The lines are unmasked in IMR for the duration so EXTI->PR records which one
 * fired; nothing is serviced, since the system is locked and PR and the NVIC
 * pending bits are cleared before unlocking.
 *
 * With a nav device on the STM32's own bus, its poll hook runs at every RTC
 * wakeup too, over I2C1 on the HSI (see "I2C1 inside STOP"), and anything but
 * idle ends the STOP. */
#    define STOP_ROW_LINES_A 0xCEu /* A1-A3, A6, A7 */
#    define STOP_ROW_LINES (STOP_ROW_LINES_A | 0x01u)
#    define STOP_UART_LINE (1u << 10)
#    define STOP_RTC_LINE (1u << 20)
#    define STOP_WAKE_LINES (STOP_ROW_LINES | STOP_UART_LINE)
#    define STOP_ALL_LINES (STOP_WAKE_LINES | STOP_RTC_LINE)
/* RTC wakeup on RTCCLK/16: 2.5 kHz at the nominal 40 kHz LSI. */
#    define STOP_RTC_PERIOD_MS 16u
#    define STOP_RTC_WUTR (STOP_RTC_PERIOD_MS * 40u / 16u - 1u)
/* The same period in LSI cycles. */
#    define STOP_RTC_PERIOD_LSI ((STOP_RTC_WUTR + 1u) * 16u)

/* Both sides in nominal-LSI ms: the IWDG also runs on the LSI, so the ratio
 * holds at any LSI frequency. */
_Static_assert(STOP_RTC_PERIOD_MS * 4u < ZSA_WATCHDOG_TIMEOUT_MS, "RTC wakeup must kick the IWDG well inside its timeout");

/* ---- LSI calibration ----
 *
 * The RTC that times STOP runs on the LSI (30-50 kHz on the F303). Its
 * frequency is measured against the HSE-derived system timer: the RTC calendar
 * (subseconds included) is read at both ends of LSI_CAL_WINDOW_MS of running
 * time with no STOP in between, one read per window, each window starting
 * where the last ended. Out-of-range results are dropped; until the first
 * good one, the nominal 40 kHz is used. */
#    define LSI_NOMINAL_HZ 40000u
#    define LSI_MIN_HZ 25000u
#    define LSI_MAX_HZ 55000u
#    define LSI_CAL_WINDOW_MS 5000u
/* Bound on the RSF wait after STOP (~2 RTCCLK cycles). */
#    define RTC_RSF_SPINS 5000u

static bool stop_supported(void);

static uint32_t lsi_hz = LSI_NOMINAL_HZ;
static bool     lsi_cal_armed;
static uint32_t lsi_cal_ms;
static uint32_t lsi_cal_ticks;
static uint32_t lsi_ms_rem; /* stopped time below 1 ms, in 1/lsi_hz ms */

struct rtc_stamp {
    uint32_t ticks; /* time of day in RTC ck_apre ticks */
    uint32_t day;   /* ticks per calendar day */
    uint32_t a_div; /* LSI cycles per tick */
};

/* False while the shadow registers are not synced or the reads disagree. */
static bool rtc_read_stamp(struct rtc_stamp *st) {
    if ((RTC->CR & RTC_CR_BYPSHAD) == 0 && (RTC->ISR & RTC_ISR_RSF) == 0) {
        return false;
    }
    uint32_t ssr = RTC->SSR, tr = RTC->TR;
    (void)RTC->DR;
    for (uint8_t i = 0;; i++) {
        uint32_t ssr2 = RTC->SSR, tr2 = RTC->TR;
        (void)RTC->DR;
        if (ssr2 == ssr && tr2 == tr) {
            break;
        }
        if (i == 3) {
            return false;
        }
        ssr = ssr2;
        tr  = tr2;
    }
    uint32_t prer  = RTC->PRER;
    uint32_t s_div = (prer & 0x7FFFu) + 1u;
    uint32_t hours = ((tr >> 20) & 0x3u) * 10u + ((tr >> 16) & 0xFu);
    uint32_t mins  = ((tr >> 12) & 0x7u) * 10u + ((tr >> 8) & 0xFu);
    uint32_t secs  = ((tr >> 4) & 0x7u) * 10u + (tr & 0xFu);
    ssr &= 0xFFFFu;
    if (ssr >= s_div) {
        return false;
    }
    st->a_div = ((prer >> 16) & 0x7Fu) + 1u;
    st->day   = 86400u * s_div;
    st->ticks = ((hours * 60u + mins) * 60u + secs) * s_div + (s_div - 1u - ssr);
    return true;
}

static uint32_t rtc_ticks_between(const struct rtc_stamp *a, const struct rtc_stamp *b) {
    return b->ticks >= a->ticks ? b->ticks - a->ticks : b->ticks + a->day - a->ticks;
}

/* One main-loop pass. */
static void lsi_cal_task(void) {
    if (lsi_cal_armed && timer_elapsed32(lsi_cal_ms) < LSI_CAL_WINDOW_MS) {
        return;
    }
    struct rtc_stamp st;
    if (!stop_supported() || !rtc_read_stamp(&st)) {
        return;
    }
    uint32_t now = timer_read32();
    if (lsi_cal_armed) {
        struct rtc_stamp prev = {.ticks = lsi_cal_ticks, .day = st.day, .a_div = st.a_div};
        uint32_t         dt   = now - lsi_cal_ms;
        uint64_t         hz   = ((uint64_t)rtc_ticks_between(&prev, &st) * st.a_div * 1000u + dt / 2u) / dt;
        sdiag.lsi_last_hz     = (uint16_t)(hz > UINT16_MAX ? UINT16_MAX : hz);
        if (hz >= LSI_MIN_HZ && hz <= LSI_MAX_HZ) {
            lsi_hz       = (uint32_t)hz;
            sdiag.lsi_hz = (uint16_t)hz;
            sdiag_inc(&sdiag.lsi_ok);
        } else {
            sdiag_inc(&sdiag.lsi_bad);
        }
    }
    lsi_cal_ms    = now;
    lsi_cal_ticks = st.ticks;
    lsi_cal_armed = true;
}

/* RTC wakeups that fit in ms. */
static uint32_t stop_ms_to_wakes(uint32_t ms) {
    uint64_t w = (uint64_t)ms * lsi_hz / (STOP_RTC_PERIOD_LSI * 1000u);
    return w > UINT32_MAX ? UINT32_MAX : (uint32_t)w;
}

/* LSI cycles to ms, carrying the remainder. */
static uint32_t stop_lsi_to_ms(uint64_t cycles) {
    uint64_t num = cycles * 1000u + lsi_ms_rem;
    lsi_ms_rem   = (uint32_t)(num % lsi_hz);
    return (uint32_t)(num / lsi_hz);
}

static const pin_t stop_strobes_pins[] = {B10, B11, B12, B13, B14, B15};

static void stop_strobes(bool high) {
    for (uint8_t i = 0; i < sizeof(stop_strobes_pins) / sizeof(stop_strobes_pins[0]); i++) {
        gpio_write_pin(stop_strobes_pins[i], high);
    }
}

/* Needs the strobes high. */
static bool stop_left_key_down(void) {
    return (GPIOA->IDR & (STOP_ROW_LINES_A | 0x01u)) != 0 || (GPIOB->IDR & 0x01u) != 0;
}

static void stop_rtc_wakeup(bool on) {
    PWR->CR |= PWR_CR_DBP;
    RTC->WPR = 0xCAu;
    RTC->WPR = 0x53u;
    RTC->CR &= ~(RTC_CR_WUTE | RTC_CR_WUTIE);
    if (on) {
        while ((RTC->ISR & RTC_ISR_WUTWF) == 0) {
        }
        RTC->WUTR = STOP_RTC_WUTR;
        RTC->CR   = (RTC->CR & ~RTC_CR_WUCKSEL) | RTC_CR_WUTIE | RTC_CR_WUTE; /* WUCKSEL 0: RTCCLK/16 */
        RTC->ISR  = ~(RTC_ISR_WUTF | RTC_ISR_INIT);
    } else {
        /* RSF cleared: the shadow registers resync after STOP. */
        RTC->ISR = ~(RTC_ISR_WUTF | RTC_ISR_INIT | RTC_ISR_RSF);
    }
    RTC->WPR = 0xFFu;
}

/* nRST_STOP = 0 would reset on STOP entry; the RTC must run on LSI. */
static bool stop_supported(void) {
    return (FLASH->OBR & FLASH_OBR_nRST_STOP) != 0 && (RCC->BDCR & RCC_BDCR_RTCEN) != 0 && (RCC->BDCR & RCC_BDCR_RTCSEL) == RCC_BDCR_RTCSEL_LSI;
}

/* Lets the last frame leave before the clock stops. */
static void stop_uart_drain(void) {
    uint32_t start = timer_read32();

    while (timer_elapsed32(start) < 5) {
        if (oqIsEmptyI(&UART_DRIVER.oqueue) && (UART_DRIVER.usart->ISR & USART_ISR_TC)) {
            return;
        }
    }
}

static bool stop_rx_pending(void) {
    return !iqIsEmptyI(&UART_DRIVER.iqueue) || (UART_DRIVER.usart->ISR & (USART_ISR_RXNE | USART_ISR_BUSY));
}

#    if HAL_USE_I2C == TRUE && STM32_I2C_USE_I2C1 == TRUE
/* ---- I2C1 inside STOP ----
 *
 * STOP wakes on the 8 MHz HSI, and I2C1 is clocked from SYSCLK, so a left nav
 * poll at an RTC wake runs I2C1 at 8 MHz with its own timing, polled, with the
 * DMA requests and interrupts of the ChibiOS driver masked. The driver must be
 * started and idle; its CR1 and TIMINGR are put back before it runs again. */
#        define STOP_HSI_CYCLES_PER_US 8u
#        define STOP_I2C_TIMEOUT_US 2000u
/* At 8 MHz: tLOW 1.875 us, tHIGH 625 ns, SCLDEL 125 ns, SDADEL 0, each no
 * shorter than the 72 MHz setting of the ChibiOS driver. */
#        define STOP_I2C_TIMINGR (STM32_TIMINGR_PRESC(0U) | STM32_TIMINGR_SCLDEL(0U) | STM32_TIMINGR_SDADEL(0U) | STM32_TIMINGR_SCLH(4U) | STM32_TIMINGR_SCLL(14U))
#        define STOP_I2C_CR1_DRIVER (I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_ADDRIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_TCIE | I2C_CR1_ERRIE | I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN)
#        define STOP_I2C_ICR_ALL (I2C_ICR_ADDRCF | I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF | I2C_ICR_PECCF | I2C_ICR_TIMOUTCF | I2C_ICR_ALERTCF)

static bool     stop_i2c_on;
static uint32_t stop_i2c_cr1;
static uint32_t stop_i2c_timingr;

static void stop_i2c_disable(void) {
    I2C1->CR1 &= ~I2C_CR1_PE;
    while (I2C1->CR1 & I2C_CR1_PE) {
        I2C1->CR1 &= ~I2C_CR1_PE;
    }
}

/* The ChibiOS driver is started and between transfers. */
static bool stop_i2c_available(void) {
    return I2CD1.state == I2C_READY && (I2C1->CR1 & I2C_CR1_PE) && (I2C1->ISR & I2C_ISR_BUSY) == 0;
}

static void sdiag_i2c_snapshot(void) {
    sdiag.i2c_snap = (uint8_t)(I2CD1.state & 0x0Fu) | ((I2C1->CR1 & I2C_CR1_PE) ? 0x10u : 0u) | ((I2C1->ISR & I2C_ISR_BUSY) ? 0x20u : 0u);
}

static void stop_i2c_begin(void) {
    stop_i2c_cr1     = I2C1->CR1;
    stop_i2c_timingr = I2C1->TIMINGR;
    stop_i2c_disable();
    I2C1->TIMINGR = STOP_I2C_TIMINGR;
    I2C1->CR1     = (stop_i2c_cr1 & ~STOP_I2C_CR1_DRIVER) | I2C_CR1_PE;
    stop_i2c_on   = true;
}

static void stop_i2c_end(void) {
    stop_i2c_on = false;
    stop_i2c_disable();
    I2C1->ICR     = STOP_I2C_ICR_ALL;
    I2C1->TIMINGR = stop_i2c_timingr;
    I2C1->CR1     = stop_i2c_cr1;
    NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
    NVIC_ClearPendingIRQ(I2C1_ER_IRQn);
}

static bool stop_i2c_timed_out(rtcnt_t start) {
    return (rtcnt_t)(chSysGetRealtimeCounterX() - start) > STOP_I2C_TIMEOUT_US * STOP_HSI_CYCLES_PER_US;
}

/* Context of the next wait, for sdiag.i2c_err: 0x10 read, 0x20 first byte,
 * 0x40 the closing STOP. */
static uint8_t stop_i2c_ctx;

/* sdiag.i2c_err: stop_i2c_ctx | 0x01 NACK | 0x02 bus error | 0x04 lost
 * arbitration | 0x08 timeout. */
static void stop_i2c_note_error(uint32_t isr, bool timeout) {
    sdiag.i2c_err = stop_i2c_ctx | ((isr & I2C_ISR_NACKF) ? 0x01u : 0u) | ((isr & I2C_ISR_BERR) ? 0x02u : 0u) | ((isr & I2C_ISR_ARLO) ? 0x04u : 0u) | (timeout ? 0x08u : 0u);
}

/* Waits for flag; false on NACK, bus error, lost arbitration or timeout. */
static bool stop_i2c_wait(uint32_t flag) {
    rtcnt_t start = chSysGetRealtimeCounterX();
    for (;;) {
        uint32_t isr = I2C1->ISR;
        if (isr & flag) {
            return true;
        }
        bool timeout = stop_i2c_timed_out(start);
        if ((isr & (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO)) || timeout) {
            stop_i2c_note_error(isr, timeout);
            return false;
        }
    }
}

/* Ends a transfer started with AUTOEND: the STOP, or after a failure the STOP
 * the hardware sends on a NACK, else a peripheral reset. */
static bool stop_i2c_finish(bool ok) {
    if (ok) {
        stop_i2c_ctx |= 0x40u;
        ok = stop_i2c_wait(I2C_ISR_STOPF);
    }
    if (!ok) {
        rtcnt_t start = chSysGetRealtimeCounterX();
        while ((I2C1->ISR & I2C_ISR_STOPF) == 0 && !stop_i2c_timed_out(start)) {
        }
        if ((I2C1->ISR & I2C_ISR_STOPF) == 0 || (I2C1->ISR & (I2C_ISR_BERR | I2C_ISR_ARLO))) {
            stop_i2c_disable();
            I2C1->CR1 |= I2C_CR1_PE;
        }
    }
    I2C1->ICR = STOP_I2C_ICR_ALL;
    return ok;
}

static void stop_i2c_start(uint8_t addr, uint8_t len, bool read) {
    I2C1->ICR = STOP_I2C_ICR_ALL;
    I2C1->CR2 = (addr & 0xFEu) | ((uint32_t)len << I2C_CR2_NBYTES_Pos) | (read ? I2C_CR2_RD_WRN : 0u) | I2C_CR2_AUTOEND | I2C_CR2_START;
}

bool zsa_stop_i2c_write(uint8_t addr, const uint8_t *data, uint8_t len) {
    if (!stop_i2c_on || len == 0) {
        return false;
    }
    stop_i2c_start(addr, len, false);
    bool ok = true;
    for (uint8_t i = 0; i < len && ok; i++) {
        stop_i2c_ctx = (i == 0) ? 0x20u : 0u;
        ok           = stop_i2c_wait(I2C_ISR_TXIS);
        if (ok) {
            I2C1->TXDR = data[i];
        }
    }
    return stop_i2c_finish(ok);
}

bool zsa_stop_i2c_read(uint8_t addr, uint8_t *data, uint8_t len) {
    if (!stop_i2c_on || len == 0) {
        return false;
    }
    stop_i2c_start(addr, len, true);
    bool ok = true;
    for (uint8_t i = 0; i < len && ok; i++) {
        stop_i2c_ctx = (i == 0) ? 0x30u : 0x10u;
        ok           = stop_i2c_wait(I2C_ISR_RXNE);
        if (ok) {
            data[i] = (uint8_t)I2C1->RXDR;
        }
    }
    return stop_i2c_finish(ok);
}

void zsa_stop_delay_us(uint16_t us) {
    if (stop_i2c_on) {
        chSysPolledDelayX((rtcnt_t)us * STOP_HSI_CYCLES_PER_US);
    }
}
#    else
static bool stop_i2c_available(void) {
    return false;
}
static void sdiag_i2c_snapshot(void) {}
static void stop_i2c_begin(void) {}
static void stop_i2c_end(void) {}
bool zsa_stop_i2c_write(uint8_t addr, const uint8_t *data, uint8_t len) {
    (void)addr;
    (void)data;
    (void)len;
    return false;
}
bool zsa_stop_i2c_read(uint8_t addr, uint8_t *data, uint8_t len) {
    (void)addr;
    (void)data;
    (void)len;
    return false;
}
void zsa_stop_delay_us(uint16_t us) {
    (void)us;
}
#    endif

enum stop_wake {
    STOP_WAKE_NONE = 0, /* did not stop: input was already pending */
    STOP_WAKE_INPUT,    /* a left key */
    STOP_WAKE_UART,     /* the nRF */
    STOP_WAKE_TIMEOUT,  /* max_ms of RTC time ran out */
    STOP_WAKE_NAV,      /* the left nav poll did not return idle */
};

/* STOP until a left key, a byte from the nRF, the left nav poll returning
 * other than idle (*nav gets its result), or max_ms of RTC time. poll, when
 * set, runs at every RTC wake. *stopped_ms gets the time spent stopped, from
 * the RTC calendar, or from the count of RTC periods when the calendar reads
 * fail or disagree with it, at the calibrated LSI frequency. */
static enum stop_wake stm32_stop(uint32_t max_ms, uint8_t (*poll)(void), uint8_t *nav, uint32_t *stopped_ms) {
    enum stop_wake why        = STOP_WAKE_NONE;
    uint32_t       rtc_wakes  = 0;
    uint32_t       max_wakes  = stop_ms_to_wakes(max_ms);
    struct rtc_stamp t0, t1;
    bool             t0_ok    = false;
    bool             t1_ok    = false;
    *stopped_ms               = 0;
    *nav                      = ZSA_NAV_SLEEP_IDLE;
    lsi_cal_armed             = false;
    nav_wake_filter_drop();

    stop_uart_drain();

    rccEnableAPB2(RCC_APB2ENR_SYSCFGEN, true);
    uint32_t exticr0 = SYSCFG->EXTICR[0];
    uint32_t exticr1 = SYSCFG->EXTICR[1];
    uint32_t exticr2 = SYSCFG->EXTICR[2];
    uint32_t imr     = EXTI->IMR;
    uint32_t imr2    = EXTI->IMR2;
    uint32_t emr     = EXTI->EMR;
    uint32_t rtsr    = EXTI->RTSR;
    uint32_t ftsr    = EXTI->FTSR;

    stop_strobes(true);
    wait_us(30);

    chSysLock();
    /* EXTI0 = PB0, EXTI1-3 = PA1-PA3, EXTI6-7 = PA6-PA7, EXTI10 = PA10. */
    SYSCFG->EXTICR[0] = 0x0001u;
    SYSCFG->EXTICR[1] = exticr1 & ~0xFF00u;
    SYSCFG->EXTICR[2] = exticr2 & ~0x0F00u;
    EXTI->IMR         = STOP_ALL_LINES;
    EXTI->IMR2        = 0;
    EXTI->RTSR        = STOP_ROW_LINES | STOP_RTC_LINE;
    EXTI->FTSR        = STOP_UART_LINE;
    EXTI->EMR         = STOP_ALL_LINES;
    EXTI->PR          = STOP_ALL_LINES;
    stop_rtc_wakeup(true);
    if (poll != NULL) {
        stop_i2c_begin();
    }

    if (stop_rx_pending() || stop_left_key_down()) {
        goto restore;
    }
    t0_ok = rtc_read_stamp(&t0);

    PWR->CR = (PWR->CR & ~PWR_CR_PDDS) | PWR_CR_LPDS;
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    for (;;) {
        watchdog_kick();
        /* SEV + WFE clears the event latch; an edge after this is in PR. */
        __SEV();
        __WFE();
        uint32_t pr = EXTI->PR;
        if (pr & STOP_UART_LINE) {
            why = STOP_WAKE_UART;
            break;
        }
        if ((pr & STOP_ROW_LINES) || stop_left_key_down()) {
            why = STOP_WAKE_INPUT;
            break;
        }
        if (rtc_wakes >= max_wakes) {
            why = STOP_WAKE_TIMEOUT;
            break;
        }
        __WFE();
        if (RTC->ISR & RTC_ISR_WUTF) {
            rtc_wakes++;
            RTC->ISR = ~(RTC_ISR_WUTF | RTC_ISR_INIT);
            EXTI->PR = STOP_RTC_LINE;
            /* The STOP-context I2C timing assumes the HSI. */
            if (poll != NULL && (RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSI) {
                nav_wake_filter.fed = false;
                *nav                = poll();
                if (!nav_wake_filter.fed || *nav == ZSA_NAV_SLEEP_ERROR) {
                    nav_wake_filter_drop();
                }
                sdiag.last_poll = *nav;
                sdiag_inc(*nav == ZSA_NAV_SLEEP_IDLE ? &sdiag.polls_idle : *nav == ZSA_NAV_SLEEP_ACTIVITY ? &sdiag.polls_activity : &sdiag.polls_error);
                if (*nav != ZSA_NAV_SLEEP_IDLE) {
                    why = STOP_WAKE_NAV;
                    break;
                }
            }
        }
    }
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    watchdog_kick();

    /* STOP wakes on HSI: back to HSE + PLL. */
    stm32_clock_init();

restore:
    if (poll != NULL) {
        stop_i2c_end();
    }
    stop_rtc_wakeup(false);
    for (uint32_t i = 0; t0_ok && why != STOP_WAKE_NONE && !t1_ok && i < RTC_RSF_SPINS; i++) {
        t1_ok = rtc_read_stamp(&t1);
    }
    EXTI->EMR         = emr;
    EXTI->RTSR        = rtsr;
    EXTI->FTSR        = ftsr;
    EXTI->PR          = STOP_ALL_LINES;
    EXTI->IMR         = imr;
    EXTI->IMR2        = imr2;
    SYSCFG->EXTICR[0] = exticr0;
    SYSCFG->EXTICR[1] = exticr1;
    SYSCFG->EXTICR[2] = exticr2;
    NVIC_ClearPendingIRQ(EXTI0_IRQn);
    NVIC_ClearPendingIRQ(EXTI1_IRQn);
    NVIC_ClearPendingIRQ(EXTI2_TSC_IRQn);
    NVIC_ClearPendingIRQ(EXTI3_IRQn);
    NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
    chSysUnlock();

    stop_strobes(false);

    if (why != STOP_WAKE_NONE) {
        /* What arrived while the clocks were restarting is noise. */
        while (uart_available()) {
            (void)uart_read();
        }
        zsa_parser_init(&rx_parser);
    }

    if (why != STOP_WAKE_NONE) {
        uint64_t lsi = (uint64_t)rtc_wakes * STOP_RTC_PERIOD_LSI;
        if (t1_ok) {
            uint64_t cal = (uint64_t)rtc_ticks_between(&t0, &t1) * t0.a_div;
            /* The counted periods, within two calendar ticks, up to one more
             * period plus the clock restart. */
            if (cal + 2u * t0.a_div >= lsi && cal <= lsi + 2u * STOP_RTC_PERIOD_LSI + 2u * t0.a_div) {
                lsi   = cal;
                t1_ok = true;
            } else {
                t1_ok = false;
            }
        }
        sdiag_inc(t1_ok ? &sdiag.stop_by_rtc : &sdiag.stop_by_wakes);
        *stopped_ms = stop_lsi_to_ms(lsi);
    }
    return why;
}
#else
static bool stop_supported(void) {
    return false;
}
static void lsi_cal_task(void) {}
enum stop_wake { STOP_WAKE_NONE = 0, STOP_WAKE_INPUT, STOP_WAKE_UART, STOP_WAKE_TIMEOUT, STOP_WAKE_NAV };
static bool stop_i2c_available(void) {
    return false;
}
static void sdiag_i2c_snapshot(void) {}
static enum stop_wake stm32_stop(uint32_t max_ms, uint8_t (*poll)(void), uint8_t *nav, uint32_t *stopped_ms) {
    (void)max_ms;
    (void)poll;
    *nav        = ZSA_NAV_SLEEP_IDLE;
    *stopped_ms = 0;
    return STOP_WAKE_NONE;
}
bool zsa_stop_i2c_write(uint8_t addr, const uint8_t *data, uint8_t len) {
    (void)addr;
    (void)data;
    (void)len;
    return false;
}
bool zsa_stop_i2c_read(uint8_t addr, uint8_t *data, uint8_t len) {
    (void)addr;
    (void)data;
    (void)len;
    return false;
}
void zsa_stop_delay_us(uint16_t us) {
    (void)us;
}
#endif

/* After a wake by the nRF. Long enough to catch the next of the LINK_UP probes
 * a rebooted nRF sends every 100 ms, which carry no preamble. */
#define SLEEP_UART_WAKE_WINDOW_MS 150u

/* STOP is used only where every input can wake it: wireless on battery,
 * nothing held down, and no status LED blinking (it would freeze lit). Nor
 * during a left nav probe window, or before the nRF has the boost claim that
 * ends it. A nav device on the STM32's own bus is checked by
 * nav_sleep_prepare(). */
static uint8_t sleep_stop_block(void) {
    if (!stop_supported() || nrf_vbus_present || zsa_stm32_owns_right_bus()) {
        return SB_POWER;
    }
    if (!nav_boost_settled()) {
        return SB_BOOST;
    }
    if (pair_led_owned || dfu_led_green_owned || dfu_led_red_owned) {
        return SB_STATUS_LED;
    }
    for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
        if (matrix_get_row(row)) {
            return SB_KEY_HELD;
        }
    }
    return SB_NONE;
}

/* The black frame has reached both LED drivers, or stopped trying. The left
 * chip is unpowered while its rail is off. */
static bool sleep_leds_settled(void) {
#ifdef RGB_MATRIX_ENABLE
    if (timer_elapsed32(sleep_entered_ms) >= ZSA_SLEEP_LED_SETTLE_MS) {
        return true;
    }
    if (left_led_rail_on && is31fl3731_pwm_buffer_dirty(ZSA_LED_DRIVER_LEFT)) {
        return false;
    }
    if (voyager_right_led_up() && is31fl3731_pwm_buffer_dirty(ZSA_LED_DRIVER_RIGHT)) {
        return false;
    }
#    if ZSA_LEFT_LED_SLICED
    if (left_led_rail_on && left_led_cycle_active) {
        return false;
    }
#    endif
#    if ZSA_RIGHT_LED_SLICED
    if (right_led_cycle_active) {
        return false;
    }
#    endif
#endif
    return true;
}

/* Stops once the running window has passed. idle_ms is the idle time so far;
 * a pending hibernate bounds the stop. */
static void sleep_stop_task(uint32_t idle_ms) {
    uint8_t block = sleep_stop_block();
    if (block == SB_NONE && !sleep_leds_settled()) {
        block = SB_LEDS;
    }
    if (block != SB_NONE) {
        sdiag_block(block);
        return;
    }
    if (timer_elapsed32(sleep_awake_ms) < sleep_window_ms || timer_elapsed32(last_rx_ms) < ZSA_SLEEP_WAKE_WINDOW_MS) {
        sdiag_block(SB_WINDOW);
        return;
    }

    uint32_t max_ms = UINT32_MAX;
    if (!hibernate_sent && link_ready) {
        /* The last stretch before the hibernate step is spent running. */
        if (idle_ms + ZSA_SLEEP_WAKE_WINDOW_MS >= ZSA_HIBERNATE_TIMEOUT_MS) {
            sdiag_block(SB_STRETCH);
            return;
        }
        max_ms = ZSA_HIBERNATE_TIMEOUT_MS - idle_ms;
    }
    if (nav_sleep_windows() && !nav_left_local()) {
        /* Likewise the last stretch before the next left nav window. */
        uint32_t clock = sleep_clock_ms();
        if (clock + ZSA_SLEEP_WAKE_WINDOW_MS >= nav_sleep_due_ms) {
            sdiag_block(SB_STRETCH);
            return;
        }
        if (nav_sleep_due_ms - clock < max_ms) {
            max_ms = nav_sleep_due_ms - clock;
        }
    }

    /* A left nav device is polled from STOP, on a bus the driver is not
     * using. */
    uint8_t (*poll)(void);
    block = nav_sleep_prepare(&poll);
    if (block == SB_NONE && poll != NULL && !stop_i2c_available()) {
        block = SB_I2C_BUSY;
        sdiag_i2c_snapshot();
    }
    if (block != SB_NONE) {
        sdiag_block(block);
        return;
    }
    sdiag_block(SB_NONE);

    /* Re-asserted every time, so a lost frame cannot leave the nRF without
     * the preamble that wakes us for the right half. */
    send_idle_state(ZSA_IDLE_SLEEP);
    if (nav_cap_mask != 0 && link_ready) {
        send_nav_boost(nav_boost_sent);
    }

    uint32_t       stopped_ms;
    uint8_t        nav;
    enum stop_wake why = stm32_stop(max_ms, poll, &nav, &stopped_ms);

    sleep_stop_ms += stopped_ms;
    sleep_awake_ms  = timer_read32();
    sdiag.stop_ms += stopped_ms;
    sdiag.block_prev_ms = sleep_awake_ms;
    if (why == STOP_WAKE_NONE) {
        if (sdiag.wake_none != UINT8_MAX) {
            sdiag.wake_none++;
        }
    } else {
        sdiag_inc(&sdiag.stop_entries);
        if (poll != NULL) {
            sdiag_inc(&sdiag.stop_nav_entries);
        }
        sdiag_inc(why == STOP_WAKE_INPUT ? &sdiag.wake_input : why == STOP_WAKE_UART ? &sdiag.wake_uart : why == STOP_WAKE_TIMEOUT ? &sdiag.wake_timeout : nav == ZSA_NAV_SLEEP_ACTIVITY ? &sdiag.wake_nav_activity : &sdiag.wake_nav_error);
    }
    sleep_window_ms = (why == STOP_WAKE_UART) ? SLEEP_UART_WAKE_WINDOW_MS : ZSA_SLEEP_WAKE_WINDOW_MS;

    if (why == STOP_WAKE_NAV && nav == ZSA_NAV_SLEEP_ERROR) {
        if (++nav_sleep_errors >= NAV_SLEEP_MAX_POLL_ERRORS) {
            zsa_log("power: left nav poll failing, staying awake");
            nav_sleep_failed = true;
            sdiag.nav_fail   = 2;
        }
    } else if (poll != NULL && stopped_ms != 0) {
        nav_sleep_errors = 0;
    }
    if (why == STOP_WAKE_NAV && nav == ZSA_NAV_SLEEP_ACTIVITY) {
        /* The device holds the input for its normal read path. */
        last_pointing_device_activity_trigger();
        sleep_exit_for(SE_NAV_POLL);
    }
}

static void idle_power_task(void) {
    static bool was_bluetooth = false;

    /* Only manage power stages when running wirelessly */
    if (usb_host_present()) {
        if (was_bluetooth) {
            was_bluetooth = false;
            send_usb_state(true);
            zsa_nav_transport_changed();
        }
        sleep_exit_for(SE_USB);
        return;
    }

    /* Just switched to bluetooth — reset idle baseline */
    if (!was_bluetooth) {
        was_bluetooth = true;
        idle_floor_ms = timer_read32();
        send_usb_state(false);
        zsa_nav_transport_changed();
        sleep_exit_for(SE_TRANSPORT);
        return;
    }

    /* An update produces no key activity, and a pairing is waiting on the
     * user: neither may be slept through. */
    if (zsa_wireless_dfu_active() || zsa_wireless_pairing_active()) {
        sleep_exit_for(SE_DFU_PAIRING);
        idle_floor_ms = timer_read32();
        return;
    }
    if (nrf_dfu_keep_awake) {
        sleep_exit_for(SE_DFU_AWAKE);
        idle_floor_ms = timer_read32();
        return;
    }

#ifdef ZSA_DEV_NO_SLEEP
    /* Dev toggle: keep the USB/BT state sync above (the nRF relies on those
     * edges) but stay Active. */
    return;
#endif

    /* The boot power gate runs its own hibernate. */
    if (power_gate != PG_OK) {
        return;
    }

    if (idle_step == ZSA_IDLE_ACTIVE) {
        uint32_t idle_ms     = last_input_activity_elapsed();
        uint32_t since_floor = timer_elapsed32(idle_floor_ms);
        if (since_floor < idle_ms) {
            idle_ms = since_floor;
        }
        if (idle_ms >= ZSA_SLEEP_TIMEOUT_MS) {
            sleep_enter(idle_ms);
        }
        return;
    }

    if (last_input_activity_time() != sleep_input_mark) {
        uint32_t last = last_input_activity_time();
        sleep_exit_for(last == last_pointing_device_activity_time() ? SE_POINTER : last == last_matrix_activity_time() ? SE_KEY : SE_INPUT_OTHER);
        return;
    }

    uint32_t idle_ms = sleep_idle_base_ms + sleep_clock_ms();

    /* Sent once per Sleep: the nRF refuses with VBUS present, and a VBUS
     * change re-arms it. Not on the cable: the right half's radio is off, so
     * the ship request cannot reach it and it would be left running. */
    if (idle_ms >= ZSA_HIBERNATE_TIMEOUT_MS && !hibernate_sent && !nrf_vbus_present && link_ready && !zsa_stm32_owns_right_bus()) {
        zsa_log("power: hibernate");
        zsa_wireless_cmd_hibernate();
        hibernate_sent = true;
        return;
    }

    sleep_stop_task(idle_ms);
}

#ifdef RGB_MATRIX_ENABLE
/* ---- LED flush rate limiting ---- */

/* Strong override of is31fl3731.c's weak is31fl3731_should_flush() hook.
 * The cap is skipped only while the STM32 drives the real right chip over the
 * cable - a real chip on a real bus, cheap to flush at full rate like the left
 * one. Otherwise the left (real, local) driver flushes whole frames on USB and
 * is sliced off USB, and the right (emulated 0x77, relayed over ESB) driver is
 * capped to ZSA_RIGHT_LED_INTERVAL_MS: every flush it answers blocks on a
 * clock-stretched I2C write (~9 ms, against ~4.3 ms for a whole left frame,
 * 3.6 ms of it on the wire, measured 2026-09-21), and the right half doesn't
 * need matrix-scan-rate refresh.
 * Skipping a flush here does not clear pwm_buffer_dirty (only
 * is31fl3731_update_pwm_buffers() does, after it actually writes), so the
 * buffer stays dirty and the next permitted flush still sends the latest
 * colors - nothing is lost, just delayed. */
bool is31fl3731_should_flush(uint8_t index) {
    if (index == ZSA_LED_DRIVER_RIGHT && !voyager_right_led_up()) {
        return false;
    }
    if (index != ZSA_LED_DRIVER_RIGHT) {
#    if ZSA_LEFT_LED_SLICED
        if (!usb_hid_on_usb()) {
            /* left_led_slice_task() owns the left frame off USB. This is the
             * only point where rgb_matrix has a whole rendered frame, so it is
             * also the only point where a cycle may start. The frame is copied
             * here and kept until the next copy; none is taken mid-cycle. */
            if (!left_led_cycle_active) {
                is31fl3731_copy_pwm_buffer(ZSA_LED_DRIVER_LEFT, left_led_frame);
                left_frame_ready = true;
            }
            return false;
        }
#    endif
        return true;
    }
    if (zsa_stm32_owns_right_bus()) {
        return true;
    }
#    if ZSA_RIGHT_LED_SLICED
    /* right_led_slice_task() owns the mirror's frames. The frame is copied
     * here and kept until the next copy; none is taken mid-cycle. */
    if (!right_led_cycle_active) {
        is31fl3731_copy_pwm_buffer(ZSA_LED_DRIVER_RIGHT, right_led_frame);
        right_frame_ready = true;
    }
    return false;
#    else
    if (timer_elapsed32(last_right_led_flush_ms) < ZSA_RIGHT_LED_INTERVAL_MS) {
        return false;
    }
    last_right_led_flush_ms = timer_read32();
    return true;
#    endif
}

#    if ZSA_RIGHT_LED_SLICED
/* Sends the mirrored right-half frame one 16-byte PWM block per main-loop
 * pass. A cycle starts when is31fl3731_should_flush() has armed
 * right_frame_ready, the buffer is dirty and ZSA_RIGHT_LED_INTERVAL_MS has
 * passed since the last cycle started; dirty is cleared there, so colors set
 * during a cycle start the next one. The arm matters: rgb_matrix renders
 * RGB_MATRIX_LED_PROCESS_LIMIT LEDs per pass and the buffer goes dirty on the
 * first of them, so a cycle keyed on dirty alone would send half-rendered
 * frames. The blocks are sent from right_led_frame, the copy taken when the
 * frame was complete; if the live buffer differs from that copy at cycle start,
 * dirty stays set. A failed block or a change of bus owner abandons
 * the cycle and marks the buffer dirty again, so the next cycle resends the
 * whole frame - the same recovery the whole-frame flush had.
 * Runs from bluetooth_task(), after the trackpad fetch in the pass. */
static void right_led_slice_task(void) {
    if (!voyager_right_led_up() || zsa_stm32_owns_right_bus()) {
        if (right_led_cycle_active) {
            right_led_cycle_active = false;
            is31fl3731_set_pwm_buffer_dirty(ZSA_LED_DRIVER_RIGHT, true);
        }
        right_frame_ready = false;
        return;
    }

    if (!right_led_cycle_active) {
        /* With the matrix off, rgb_matrix flushes its black frame once, and a
         * cycle running at that flush does not take it. Nothing renders while
         * off, so the live buffer is a whole frame: take it here. */
        if (!right_frame_ready && !rgb_matrix_is_enabled() && is31fl3731_pwm_buffer_dirty(ZSA_LED_DRIVER_RIGHT)) {
            is31fl3731_copy_pwm_buffer(ZSA_LED_DRIVER_RIGHT, right_led_frame);
            right_frame_ready = true;
        }
        if (!right_frame_ready) {
            return;
        }
        if (timer_elapsed32(last_right_led_flush_ms) < ZSA_RIGHT_LED_INTERVAL_MS) {
            return;
        }
        right_frame_ready = false;
        if (!is31fl3731_pwm_buffer_dirty(ZSA_LED_DRIVER_RIGHT)) {
            return;
        }
        last_right_led_flush_ms = timer_read32();
        is31fl3731_set_pwm_buffer_dirty(ZSA_LED_DRIVER_RIGHT, !is31fl3731_pwm_buffer_matches(ZSA_LED_DRIVER_RIGHT, right_led_frame));
        right_led_cycle_active = true;
        right_led_block        = 0;
    }

    if (!is31fl3731_write_pwm_block_from(ZSA_LED_DRIVER_RIGHT, right_led_block, right_led_frame)) {
        right_led_cycle_active = false;
        is31fl3731_set_pwm_buffer_dirty(ZSA_LED_DRIVER_RIGHT, true);
        return;
    }
    if (++right_led_block >= IS31FL3731_PWM_BLOCK_COUNT) {
        right_led_cycle_active = false;
    }
}
#    endif

#    if ZSA_LEFT_LED_SLICED
/* Sends the left half's frame one 16-byte PWM block per main-loop pass while
 * off USB, the same way the right mirror is sliced. A cycle starts only where
 * the whole-frame flush would have run (is31fl3731_should_flush() arms
 * left_frame_ready when it refuses the left driver), which is also what caps
 * the rate at one cycle per RGB_MATRIX_LED_FLUSH_LIMIT. The blocks are sent from
 * left_led_frame, the copy taken there when the frame was complete, with the
 * same dirty check at cycle start as the right mirror. A failed write, a host
 * appearing or a resync abandons the cycle and marks the buffer dirty, so the
 * whole frame is sent again. The PWM page is re-selected before every block:
 * the left chip's busdev verify (led_function_page_ok in voyager_tng.c) walks
 * to the function page and back between passes, and a failed walk back would
 * otherwise send PWM data into the function registers. */
static void left_led_slice_task(void) {
    /* Rail off: the chip is unpowered; the rail-on resync resends. */
    if (usb_hid_on_usb() || !left_led_rail_on) {
        if (left_led_cycle_active) {
            left_led_slice_abandon();
        }
        left_frame_ready = false;
        return;
    }

    if (!left_led_cycle_active) {
        /* As for the right mirror: with the matrix off, the one black flush
         * is not taken while a cycle runs. Take the live buffer here. */
        if (!left_frame_ready && !rgb_matrix_is_enabled() && is31fl3731_pwm_buffer_dirty(ZSA_LED_DRIVER_LEFT)) {
            is31fl3731_copy_pwm_buffer(ZSA_LED_DRIVER_LEFT, left_led_frame);
            left_frame_ready = true;
        }
        if (!left_frame_ready) {
            return;
        }
        left_frame_ready = false;
        if (!is31fl3731_pwm_buffer_dirty(ZSA_LED_DRIVER_LEFT)) {
            return;
        }
        is31fl3731_set_pwm_buffer_dirty(ZSA_LED_DRIVER_LEFT, !is31fl3731_pwm_buffer_matches(ZSA_LED_DRIVER_LEFT, left_led_frame));
        left_led_cycle_active = true;
        left_led_block        = 0;
    }

    if (is31fl3731_select_page(ZSA_LED_DRIVER_LEFT, IS31FL3731_COMMAND_FRAME_1) != I2C_STATUS_SUCCESS) {
        left_led_slice_abandon();
        return;
    }
    if (!is31fl3731_write_pwm_block_from(ZSA_LED_DRIVER_LEFT, left_led_block, left_led_frame)) {
        left_led_slice_abandon();
        return;
    }
    if (++left_led_block >= IS31FL3731_PWM_BLOCK_COUNT) {
        left_led_cycle_active = false;
    }
}
#    endif
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

/* Auto-generated per-module hook, called from pre_process_record_quantum()
 * before the tapping state machine and process_record_quantum() — returning
 * false drops the event before any keycode handling at all. This is the
 * lowest point a module can reach, and the only one that also covers held
 * keys, tap-hold resolution, combos and mouse keys in one place. */
bool pre_process_record_wireless(uint16_t keycode, keyrecord_t *record) {
    if (zsa_wireless_dfu_active()) {
        return false;
    }
    return pre_process_record_wireless_kb(keycode, record);
}

#ifdef POINTING_DEVICE_ENABLE
/* Trackball motion and buttons, dropped at the report the module contributes
 * to pointing_device_task_modules(). */
report_mouse_t pointing_device_task_wireless(report_mouse_t mouse_report) {
    if (zsa_wireless_dfu_active()) {
        return (report_mouse_t){0};
    }
    return pointing_device_task_wireless_kb(mouse_report);
}
#endif

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
        /* Drops the current host and cycles through the remembered ones until
         * one answers. */
        case BLE_SWITCH_HOST:
            if (record->event.pressed) {
                zsa_wireless_cmd_ble_switch_host();
            }
            return false;
        /* Powers both halves off. The nRF refuses while USB is connected. */
        case ZSA_SHIP_MODE:
            if (record->event.pressed) {
                zsa_wireless_cmd_hibernate();
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
    power_gate_ms   = timer_read32();
    zsa_rtt_init(); // TEMP DEBUG
#ifdef ZSA_BLE_LOG
    zsa_log_tap_init();
    zsa_log("zsa_wireless: init (ZSA_BLE_LOG=on)");
#endif
    watchdog_start();
}

/* Charging policy: command both halves the rate the attached source allows.
 * See the adaptive-charging comment block at the top for the full rationale. */
static void power_budget_task(void) {
    /* Gated on the reported budget only, not usb_connected_state(): a wall
     * charger supplies VBUS without ever enumerating. The nRF reports a budget
     * only while VBUS is present, so the budget alone implies power. */
    /* Nothing is commanded before the nRF has said what the source is. The
     * PMICs are already sitting at the startup floor, so a guess made first
     * could only be wrong, and it would stop and restart both chargers to
     * apply a rate we are about to replace. */
    if (!link_ready || !pwr_budget_seen) {
        return;
    }

    /* Same pair for both halves: the right's LEDs no longer draw from the
     * cable's 5 V rail, so the right-only ceilings in the wire header are not
     * applied. */
    struct zsa_charge_limits want_left  = zsa_charge_limits_for_budget(nrf_power_budget_ma);
    struct zsa_charge_limits want_right = want_left;
#if defined(ZSA_RIGHT_VBUS_LIMIT_OVERRIDE_MA) && defined(ZSA_RIGHT_CHARGE_OVERRIDE_MA)
    /* Bench knob: command the right half a fixed pair instead of the policy's. */
    want_right.vbus_limit_ma = ZSA_RIGHT_VBUS_LIMIT_OVERRIDE_MA;
    want_right.charge_ma     = ZSA_RIGHT_CHARGE_OVERRIDE_MA;
#endif

    /* A changed budget is acted on at once; an unchanged one on the maintain
     * cadence. Both pairs are compared: the right's ceilings mean it can hold
     * still while the left moves. Both go out — the nRFs drop repeats, so the
     * resend is free. */
    bool changed = !limits_commanded || want_left.vbus_limit_ma != commanded_limits.vbus_limit_ma || want_left.charge_ma != commanded_limits.charge_ma || want_right.vbus_limit_ma != commanded_right_limits.vbus_limit_ma || want_right.charge_ma != commanded_right_limits.charge_ma;
    if (!changed && timer_elapsed32(last_pwr_maintain_ms) < PWR_BUDGET_MAINTAIN_INTERVAL_MS) {
        return;
    }

    zsa_wireless_send_frame(ZSA_MSG_CMD_CHARGE_LIMITS, (const uint8_t *)&want_left, sizeof(want_left));
    if (is_transport_connected()) {
        (void)voyager_right_nrf_set_charge_limits(want_right.vbus_limit_ma, want_right.charge_ma);
    }
    commanded_limits       = want_left;
    commanded_right_limits = want_right;
    limits_commanded       = true;
    last_pwr_maintain_ms   = timer_read32();
}

/* Refresh the right half's own reading while the STM32 masters its bus. Not
 * attempted otherwise: the right's TWIS slave is only reachable over the TRRS
 * net, and a read while the right owns that net would be two masters. Dropping
 * the cached value on the way out is the point — a wireless board must not
 * keep serving the last wired reading as if it were current. */
static void right_telemetry_task(void) {
    if (!zsa_stm32_owns_right_bus()) {
        right_tlm_valid = false;
        return;
    }
    if (last_right_tlm_ms != 0 && timer_elapsed32(last_right_tlm_ms) < RIGHT_TLM_INTERVAL_MS) {
        return;
    }
    last_right_tlm_ms = timer_read32();

    struct zsa_right_telemetry got;
    right_tlm_valid = voyager_right_nrf_read_telemetry(&got) && zsa_right_telemetry_ok(&got);
    if (right_tlm_valid) {
        right_tlm = got;
        /* Relay it to the left: wired, the right's radio is off, so this is
         * the only way the left learns the right's battery for BAS, its own
         * EVT_BATTERY and the BLE detail characteristic. Nothing is sent on a
         * failed read or when we do not own the bus — the left times the last
         * copy out instead. */
        if (link_ready) {
            zsa_wireless_send_frame(ZSA_MSG_CMD_RIGHT_TELEMETRY, (const uint8_t *)&got, sizeof(got));
            periodic_tx_ms = timer_read32();
        }
    }
}

#ifdef ZSA_SLEEP_DIAG
static uint16_t sdiag_sat16(uint32_t v) {
    return v > UINT16_MAX ? UINT16_MAX : (uint16_t)v;
}

static void put16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

/* Next page as a CMD_SCAN_DIAG payload. */
static void sleep_diag_page(uint8_t *out) {
    uint8_t page = (uint8_t)(sdiag.page % SLEEP_DIAG_PAGES) + 1u;
    sdiag.page++;
    memset(out, 0, ZSA_SCAN_DIAG_SIZE);
    out[0]     = SLEEP_DIAG_PAGE_BASE + page;
    out[1]     = ZSA_SCAN_DIAG_LEN;
    uint8_t *p = out + 2;
    switch (page) {
        case 1:
            put16(p + 0, sdiag.stop_entries);
            put16(p + 2, sdiag.stop_nav_entries);
            put16(p + 4, sdiag.wake_input);
            put16(p + 6, sdiag.wake_uart);
            put16(p + 8, sdiag.wake_timeout);
            put16(p + 10, sdiag.wake_nav_activity);
            put16(p + 12, sdiag.wake_nav_error);
            break;
        case 2:
            put16(p + 0, sdiag.polls_idle);
            put16(p + 2, sdiag.polls_activity);
            put16(p + 4, sdiag.polls_error);
            p[6] = sdiag.last_poll;
            p[7] = (nav_sleep_ops == NULL) ? 0 : (nav_sleep_ops == zsa_nav_trackball_sleep_ops()) ? 1 : 2;
            memcpy(p + 8, sdiag.raw, sizeof(sdiag.raw));
            break;
        case 3:
            p[0] = sdiag.last_block;
            p[1] = sdiag.nav_fail;
            memcpy(p + 2, sdiag.ends, sizeof(sdiag.ends));
            p[6] = sdiag.i2c_err;
            p[7] = sdiag.i2c_snap;
            p[8] = sdiag.sleeps;
            p[9] = sdiag.wake_none;
            put16(p + 10, sdiag_sat16(sdiag.stop_ms / 1000u));
            put16(p + 12, sdiag_sat16((sdiag.sleep_ms + sdiag.stop_ms) / 1000u));
            break;
        case 4:
            for (uint8_t i = 0; i < 7; i++) {
                put16(p + 2 * i, sdiag_sat16(sdiag.block_ms[SB_WINDOW + i] / 100u));
            }
            break;
        case 6:
            put16(p + 0, sdiag.lsi_hz);
            put16(p + 2, sdiag.lsi_last_hz);
            put16(p + 4, sdiag.lsi_ok);
            put16(p + 6, sdiag.lsi_bad);
            put16(p + 8, sdiag.stop_by_rtc);
            put16(p + 10, sdiag.stop_by_wakes);
            put16(p + 12, sdiag.nav_noise_drops);
            break;
        default:
            for (uint8_t i = 0; i < 4; i++) {
                put16(p + 2 * i, sdiag_sat16(sdiag.block_ms[SB_NAV_NO_HOOK + i] / 100u));
            }
            put16(p + 8, (uint16_t)sdiag.motion_x);
            put16(p + 10, (uint16_t)sdiag.motion_y);
            put16(p + 12, sdiag.motion_reads);
            break;
    }
}
#endif

/* One main-loop pass: fold the gap since the previous one into the window
 * and finalize the window every ZSA_SCAN_DIAG_INTERVAL_MS. */
static void scan_diag_task(void) {
    uint32_t now = timer_read32();

    if (!scan_diag_started) {
        scan_diag_started   = true;
        scan_diag_prev_ms   = now;
        scan_diag_window_ms = now;
        return;
    }

    uint32_t gap      = now - scan_diag_prev_ms;
    uint16_t gap_ms   = (gap > UINT16_MAX) ? UINT16_MAX : (uint16_t)gap;
    scan_diag_prev_ms = now;

    scan_diag_scans++;
    if (gap_ms > scan_diag_max_gap_ms) {
        scan_diag_max_gap_ms = gap_ms;
    }
    if (gap_ms > SCAN_DIAG_GAP_5MS) {
        scan_diag_over_5ms++;
    }
    if (gap_ms > SCAN_DIAG_GAP_20MS) {
        scan_diag_over_20ms++;
    }

    if ((now - scan_diag_window_ms) < ZSA_SCAN_DIAG_INTERVAL_MS) {
        return;
    }
    scan_diag_window_ms = now;

    uint16_t flags = 0;
    if (zsa_stm32_owns_right_bus()) {
        flags |= ZSA_SCAN_DIAG_FLAG_WIRED_RIGHT;
    }
    if (usb_hid_on_usb()) {
        flags |= ZSA_SCAN_DIAG_FLAG_USB_HOST;
    }

    scan_diag_last.version           = ZSA_SCAN_DIAG_VERSION;
    scan_diag_last.len               = ZSA_SCAN_DIAG_LEN;
    scan_diag_last.scans             = scan_diag_scans;
    scan_diag_last.max_gap_ms        = scan_diag_max_gap_ms;
    scan_diag_last.gaps_over_5ms     = scan_diag_over_5ms;
    scan_diag_last.gaps_over_20ms    = scan_diag_over_20ms;
    scan_diag_last.right_read_errors = zsa_matrix_right_read_errors();
    scan_diag_last.kbd_reports       = scan_diag_kbd_reports;
    scan_diag_last.flags             = flags;

#ifdef ZSA_SLEEP_DIAG
    uint8_t page[ZSA_SCAN_DIAG_SIZE];
    sleep_diag_page(page);
    zsa_wireless_send_frame(ZSA_MSG_CMD_SCAN_DIAG, page, sizeof(page));
#else
    zsa_wireless_send_frame(ZSA_MSG_CMD_SCAN_DIAG, (const uint8_t *)&scan_diag_last, sizeof(scan_diag_last));
#endif
    periodic_tx_ms = timer_read32();

    scan_diag_scans      = 0;
    scan_diag_max_gap_ms = 0;
    scan_diag_over_5ms   = 0;
    scan_diag_over_20ms  = 0;
}

void bluetooth_task(void) {
    watchdog_kick();

    zsa_rtt_loop_tick(); // TEMP DEBUG

    /* Process incoming frames */
    while (uart_available()) {
        uint8_t byte = uart_read();
        last_rx_ms   = timer_read32();
        if (zsa_parser_feed(&rx_parser, byte)) {
            handle_rx_frame(&rx_parser.frame);
        }
    }

    /* Periodically send LINK_UP until nRF responds */
    if (!link_ready && timer_elapsed32(last_link_up_ms) >= LINK_UP_INTERVAL_MS) {
        send_command(ZSA_MSG_LINK_UP);
        last_link_up_ms = timer_read32();
        nav_boost_dirty = true;
    }

    /* Announce our nav capabilities once the link is up (retries until acked) */
    nav_announce_task();

    /* Claim the left boost for a left nav device or a probe window */
    nav_boost_task();

    /* Nav presence level to the nRF */
    nav_state_task();

    /* Fire BT_UNPR once its hold completes */
    unpair_hold_task();

    /* Unblank if the nRF stopped refreshing the update state */
    dfu_state_task();

    /* Decide whether the matrix may come on after a brownout */
    power_gate_task();

    /* Maintain the adaptive-charging raise while a high-budget source is up */
    bus_grant_task();
    trrs_state_task();
    power_budget_task();

    /* Wired-mode source for the right half's battery */
    right_telemetry_task();

#if defined(RGB_MATRIX_ENABLE) && (ZSA_RIGHT_LED_SLICED || ZSA_LEFT_LED_SLICED)
    /* One PWM block of each sliced frame */
#    if ZSA_LEFT_LED_SLICED
    left_led_slice_task();
#    endif
#    if ZSA_RIGHT_LED_SLICED
    right_led_slice_task();
#    endif
#endif

    /* Main-loop regularity and link counters */
    scan_diag_task();

    status_led_task();

    /* LSI frequency for the time spent in STOP */
    lsi_cal_task();

    idle_power_task();
}

/* USB gate for the HID routing below: QMK's usb_connected_state(), forced
 * false by the ZSA_BENCH_FORCE_NO_HOST bench knob. */
static bool usb_hid_on_usb(void) {
#ifdef ZSA_BENCH_FORCE_NO_HOST
    return false;
#else
    return usb_connected_state();
#endif
}

/* True whenever we are not on USB, so connection_auto_detect_host() picks
 * CONNECTION_HOST_BLUETOOTH and QMK hands every report to
 * bluetooth_send_keyboard() below. Deliberately not the BLE link state: the
 * nRF decides what to do with a report — including acting on key presses with
 * no host connected — so the STM32 no longer withholds reports when the link
 * is down. zsa_wireless_ble_connected() remains the real link state. */
bool bluetooth_is_connected(void) {
    return !usb_hid_on_usb();
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
    scan_diag_kbd_reports++;
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
    if (zsa_wireless_dfu_active()) {
        return;
    }
    /* keyboard.c does not count digitizer reports as input. */
    last_pointing_device_activity_trigger();
    if (usb_hid_on_usb()) {
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
    if (zsa_wireless_dfu_active()) {
        return;
    }
    last_pointing_device_activity_trigger();
    if (usb_hid_on_usb()) {
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
    if (usb_hid_on_usb() || !digitizer_wireless_mode_set) {
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
