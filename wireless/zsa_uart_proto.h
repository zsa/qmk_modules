/*
 * zsa_uart_proto.h -- ZSA keyboard UART protocol definition
 *
 * Shared between STM32 (QMK) and nRF54 (Zephyr).
 * No dependencies on either framework.
 *
 * Wire format (with byte-stuffing):
 *   SYNC(0xAA) | escaped(TYPE | LEN | PAYLOAD | CRC8)
 *
 * Byte-stuffing: after SYNC, any occurrence of 0xAA or 0x55 in the
 * remaining bytes (TYPE, LEN, PAYLOAD, CRC) is escaped:
 *   0xAA -> 0x55 0x01
 *   0x55 -> 0x55 0x02
 * This guarantees 0xAA only ever appears as a frame sync marker.
 *
 * CRC-8/CCITT (polynomial 0x07) computed over unescaped TYPE + LEN + PAYLOAD.
 */

#ifndef ZSA_UART_PROTO_H
#define ZSA_UART_PROTO_H

#include <stdint.h>

/* ---- Wire format constants ---- */

#define ZSA_SYNC_BYTE      0xAA
#define ZSA_ESC_BYTE       0x55
#define ZSA_ESC_SYNC       0x01  /* 0x55 0x01 -> 0xAA */
#define ZSA_ESC_ESC        0x02  /* 0x55 0x02 -> 0x55 */
#define ZSA_MAX_PAYLOAD    2048
#define ZSA_HEADER_SIZE    4   /* SYNC + TYPE + LEN_LO + LEN_HI */
#define ZSA_FRAME_OVERHEAD 5   /* SYNC + TYPE + LEN(2) + CRC (before escaping) */

/* ---- Message types ---- */

enum zsa_msg_type {
    /* Bidirectional: link management (0x00) */
    ZSA_MSG_LINK_UP          = 0x00,  /* 0 B: "I'm alive and ready" */

    /* STM32 -> nRF: HID reports (0x01-0x0F) */
    ZSA_MSG_HID_KEYBOARD     = 0x01,
    ZSA_MSG_HID_MOUSE        = 0x02,
    ZSA_MSG_HID_SYSTEM       = 0x03,
    ZSA_MSG_HID_DIGITIZER    = 0x04,  /* 15 B: PTP touchpad input report (report
                                       * ID 1 implied/constant) minus its
                                       * leading report_id byte. See struct
                                       * zsa_hid_digitizer below. The mouse-mode
                                       * fallback report (report ID 6) has no
                                       * PTP-specific shape and rides
                                       * ZSA_MSG_HID_MOUSE instead. */

    /* STM32 -> nRF: queries (0x10-0x1F) */
    ZSA_MSG_QUERY_BLE_STATUS = 0x10,
    ZSA_MSG_QUERY_BATTERY    = 0x11,
    ZSA_MSG_QUERY_PAIR_STATE = 0x12,

    /* STM32 -> nRF: commands (0x20-0x2F) */
    ZSA_MSG_CMD_HIBERNATE    = 0x20,
    ZSA_MSG_CMD_PAIR         = 0x21,  /* 0 B: make the nRF discoverable for a
                                       * bounded pairing window */
    ZSA_MSG_CMD_IDLE_STATE   = 0x22,
    ZSA_MSG_CMD_USB_STATE    = 0x23,  /* 1 B: 0=disconnected, 1=connected */
    ZSA_MSG_CMD_NAV_ENABLE   = 0x24,  /* 1 B: nav capability bitmask (ZSA_NAV_CAP_*);
                                       * 0 = no nav support in this QMK build */
    /* 0x25 reserved — was ZSA_MSG_CMD_NAV_CPI (removed 2026-07). Wireless
     * trackball CPI rides the emulated-i2c 0x8D register snoop instead.
     * Do NOT reuse this ID. */
    ZSA_MSG_CMD_BUS_GRANT    = 0x27,  /* 1 B: struct zsa_bus_grant — tell the
                                       * right half whether it owns its own
                                       * i2c bus. The STM32 is the arbiter: it
                                       * withholds the grant whenever it can
                                       * reach the real right-half expander
                                       * over TRRS, so exactly one master ever
                                       * drives that bus. Relayed onward by the
                                       * left over ESB as
                                       * WPP_REPLY_FLAG_BUS_GRANT. */
    ZSA_MSG_CMD_PAIR_ACCEPT  = 0x28,  /* 0 B: confirm a held pairing request.
                                       * Legacy: pairing is gated by
                                       * discoverability now, so QMK never
                                       * sends this. Handler kept for bench
                                       * testing. */
    ZSA_MSG_CMD_BLE_DISCONNECT = 0x29,  /* 0 B: drop the active BLE link; the
                                         * nRF re-advertises afterwards */
    ZSA_MSG_CMD_BLE_UNPAIR   = 0x2A,  /* 0 B: clear all bonds, refresh the BLE
                                       * identity and re-advertise */
    ZSA_MSG_CMD_BLE_SWITCH_HOST = 0x2B, /* 0 B: hand over to the next remembered
                                         * host. The nRF drops the current link
                                         * and advertises each occupied host
                                         * slot in turn for a short window. Sent
                                         * again mid-cycle it skips to the next
                                         * slot. */
    ZSA_MSG_CMD_CHARGE_LIMITS = 0x26, /* 4 B: struct zsa_charge_limits — the
                                       * VBUS input current limit and the
                                       * charge current to apply, both in mA.
                                       * The STM32 is the charging policy
                                       * point; the left nRF only detects CC
                                       * capability (EVT_PWR_BUDGET) and
                                       * executes this command. Was
                                       * CMD_VBUS_ILIM (1 B level) — same ID,
                                       * wider payload; see struct below. */

    /* nRF -> STM32: DFU commands (0x30-0x3F) */
    ZSA_MSG_DFU_ERASE        = 0x30,  /* 4 B: addr LE32 */
    ZSA_MSG_DFU_SETADDR      = 0x31,  /* 4 B: addr LE32 */
    ZSA_MSG_DFU_DATA         = 0x32,  /* <=ZSA_MAX_PAYLOAD B: firmware chunk.
                                       * Chunks concatenate into one byte
                                       * stream; an odd-length chunk carries
                                       * its trailing byte into the next one. */
    ZSA_MSG_DFU_MANIFEST     = 0x33,  /* 0 B: finalize + reset.
                                       * 8 B: struct zsa_dfu_manifest — verify
                                       * the image before resetting. */
    ZSA_MSG_DFU_ABORT        = 0x34,  /* 0 B: abort transfer */
    ZSA_MSG_DFU_MASS_ERASE   = 0x35,  /* 4 B: end addr LE32 (0 = full app region) */
    ZSA_MSG_DFU_QUERY        = 0x36,  /* 0 B: nRF asks if STM32 is in DFU mode */
    ZSA_MSG_DFU_ENTER        = 0x37,  /* 0 B: reboot into the bootloader. The
                                       * running QMK app arms the backup
                                       * register and resets; ignition, already
                                       * in DFU, just answers DFU_READY. Either
                                       * way the reply is DFU_READY, so the
                                       * host sends this and waits for one. */

    /* STM32 -> nRF: live-app confirm tunnel (0x38-0x39). Unlike the rest of
     * this block it runs against the running application, not a bootloader:
     * the STM32 asks the left nRF to confirm the image it is executing and the
     * app answers. Nothing here reboots or reflashes anything. */
    ZSA_MSG_CONFIRM_QUERY    = 0x38,  /* STM -> nRF, ZSA_CONFIRM_QUERY_LEN B:
                                       * struct zsa_confirm_query */
    ZSA_MSG_CONFIRM_REPLY    = 0x39,  /* nRF -> STM, ZSA_CONFIRM_REPLY_LEN B:
                                       * struct zsa_confirm_reply */

    /* nRF -> STM32: queries (0x70-0x7F) */
    ZSA_MSG_QUERY_USB_STATE  = 0x70,  /* 0 B: nRF asks STM32 for USB state */

    /* nRF -> STM32: events / query responses (0x80-0x8F) */
    ZSA_MSG_EVT_BLE_STATUS   = 0x80,
    ZSA_MSG_EVT_BATTERY      = 0x81,
    ZSA_MSG_EVT_PAIR_STATE   = 0x82,
    ZSA_MSG_EVT_LED_STATE    = 0x83,
    ZSA_MSG_EVT_LED_RAIL     = 0x84,  /* 1 B: bitmask of active LED rails */
    ZSA_MSG_EVT_NAV_PRESENT  = 0x85,  /* 1 B: detected device type — 0=none,
                                       * 1=trackball, 2=touchpad. Sent as the
                                       * CMD_NAV_ENABLE ack and on every
                                       * presence change */
    /* 0x86 reserved — was ZSA_MSG_EVT_NAV_MOTION (removed 2026-07). Trackball
     * motion is delivered over I2C, never UART: real bridge at 0x28 when
     * left-plugged (or over TRRS in wired mode), left-nRF emulated mirror at
     * 0x2C when right-plugged in wireless mode. Do NOT reuse this ID. */
    /* 0x87 reserved — was ZSA_MSG_EVT_NAV_REPORT_RAW (removed 2026-07).
     * Trackpad data now flows over the emulated-i2c pad mirror, never UART.
     * Do NOT reuse this ID. */
    /* Reserved for touchpad future:
     *   ZSA_MSG_EVT_NAV_GESTURE = 0x88  (TLV: gesture id + params)
     */
    ZSA_MSG_EVT_DIGITIZER_MODE = 0x89, /* 1 B: BLE host's PTP input-mode write
                                        * (0=Mouse, 3=PTP), relayed from the
                                        * nRF's SET_REPORT(feature 0x04)
                                        * emulation. See struct
                                        * zsa_digitizer_mode below. */
    ZSA_MSG_EVT_PWR_BUDGET   = 0x8A,  /* 2 B: struct zsa_pwr_budget — detected
                                       * USB source current budget from CC1/CC2.
                                       * Sent on CC/VBUS change and at link-up. */
    ZSA_MSG_EVT_VBUS_STATE   = 0x8B,  /* 1 B: struct zsa_vbus_state — nPM1300
                                       * VBUS presence. Gates the STM32's USB
                                       * D+ pullup (PA15). Sent on VBUS change
                                       * and at link-up. */
    ZSA_MSG_EVT_DFU_STATE    = 0x8C,  /* 3 B: struct zsa_dfu_progress — a
                                       * firmware update is running through
                                       * this half (nRF SMP or the STM32
                                       * relay), plus which half's image and
                                       * what phase, for the status LEDs. Sent
                                       * on every change and re-sent as a
                                       * level. Older senders send 1 B. */
    ZSA_MSG_EVT_LINK_STATE   = 0x8D,  /* 2 B: struct zsa_link_state — the two
                                       * nRF halves' view of each other over
                                       * ESB. Distinct from EVT_DFU_STATE: a
                                       * protocol mismatch is a half-working
                                       * keyboard, not an update in progress,
                                       * and must not read as one. Sent as a
                                       * level. */

    /* STM32 -> nRF: DFU responses (0x90-0x9F) */
    ZSA_MSG_DFU_STATUS       = 0x90,  /* 2 B: state + error code */

    /* STM32 -> nRF: DFU announcement (0xA0) */
    ZSA_MSG_DFU_READY        = 0xA0,  /* 0 B: bootloader is in DFU mode */

    /* Shared (bidirectional): log line (0xB0) */
    ZSA_MSG_LOG_LINE         = 0xB0,  /* src(1)+lvl(1)+ts_ms(4 LE)+text(<=120) */
};

/* ---- Payload structures ---- */

/* ZSA_MSG_HID_KEYBOARD (8 bytes) */
struct __attribute__((packed)) zsa_hid_keyboard {
    uint8_t modifiers;
    uint8_t reserved;
    uint8_t keys[6];
};

/* ZSA_MSG_HID_SYSTEM / consumer (3 bytes) */
struct __attribute__((packed)) zsa_hid_system {
    uint8_t  report_id;
    uint16_t usage;
};

/* ZSA_MSG_HID_DIGITIZER (15 bytes) — PTP touchpad input report, report ID 1
 * implied/constant (the digitizer interface has its own report-ID space, so
 * the ID never varies for this message). Mirrors report_digitizer_touchpad_t
 * (tmk_core/protocol/report.h) minus its leading report_id byte:
 *   [0]      finger0 conf:1 | tip:1 | pad:6
 *   [1]      finger0 contact_id:3 | pad:5
 *   [2..3]   finger0 X (LE16)
 *   [4..5]   finger0 Y (LE16)
 *   [6]      finger1 conf:1 | tip:1 | pad:6
 *   [7]      finger1 contact_id:3 | pad:5
 *   [8..9]   finger1 X (LE16)
 *   [10..11] finger1 Y (LE16)
 *   [12..13] scan_time (LE16)
 *   [14]     contact_count:4 | button1:1 | button2:1 | button3:1 | pad:1
 * The nRF prepends the constant report_id (0x01) before writing the BLE HID
 * input-report characteristic for the PTP application collection. */
#define ZSA_HID_DIGITIZER_LEN 15
struct __attribute__((packed)) zsa_hid_digitizer {
    uint8_t data[ZSA_HID_DIGITIZER_LEN];
};

/* ZSA_MSG_EVT_BLE_STATUS (2 bytes) */
struct __attribute__((packed)) zsa_ble_status {
    uint8_t connected;   /* 0 = disconnected, 1 = connected */
    uint8_t device_idx;  /* BLE slot (0-based) */
};

/* ZSA_MSG_EVT_BATTERY (3 bytes) */
struct __attribute__((packed)) zsa_battery_status {
    uint8_t soc;       /* state of charge, 0-100 */
    uint8_t flags;     /* bit 0: charging, bit 1: USB present */
    uint8_t right_soc; /* right half SOC, 0-100 */
};

#define ZSA_BAT_FLAG_CHARGING 0x01
#define ZSA_BAT_FLAG_USB      0x02

/* ZSA_MSG_EVT_PAIR_STATE (1 byte) */
struct __attribute__((packed)) zsa_pair_state {
    uint8_t state;
};

enum zsa_pair_state_val {
    ZSA_PAIR_IDLE   = 0,
    /* Discoverable: advertising inside a pairing window. */
    ZSA_PAIR_ACTIVE = 1,
    ZSA_PAIR_DONE   = 2,
    ZSA_PAIR_FAILED = 3,
    /* An SMP pairing is running. Excludes the encryption window a bonded
     * reconnect passes through. */
    ZSA_PAIR_CONFIRM = 4,
};

/* ZSA_MSG_EVT_LED_STATE (1 byte) */
struct __attribute__((packed)) zsa_led_state {
    uint8_t leds; /* USB HID LED bits: bit0=num, bit1=caps, bit2=scroll */
};

/* ZSA_MSG_EVT_LED_RAIL (1 byte) — bitmask of active LED voltage rails */
#define ZSA_LED_RAIL_LEFT   (1u << 0)
#define ZSA_LED_RAIL_RIGHT  (1u << 1)
struct __attribute__((packed)) zsa_led_rail_state {
    uint8_t rails;
};

/* ZSA_MSG_CMD_IDLE_STATE (1 byte) */
struct __attribute__((packed)) zsa_idle_state {
    uint8_t step; /* 0=active, 1=dim, 2=leds_off, 3=deep */
};

/* ZSA_MSG_CMD_USB_STATE (1 byte) */
struct __attribute__((packed)) zsa_usb_state {
    uint8_t connected; /* 0 = disconnected, 1 = connected */
};

/* ZSA_MSG_CMD_NAV_ENABLE (1 byte) — QMK announces its compiled-in nav
 * capabilities at link-up, retried every 1 s until acked with
 * EVT_NAV_PRESENT (sent even when 0). The nRF runs device detection —
 * and exposes the emulated 0x28 trackball mirror — only for advertised
 * capabilities. */
#define ZSA_NAV_CAP_TRACKBALL 0x01
#define ZSA_NAV_CAP_TRACKPAD  0x02
struct __attribute__((packed)) zsa_nav_enable {
    uint8_t enabled; /* ZSA_NAV_CAP_* bitmask; 0 = none */
};

/* struct zsa_nav_cpi removed with ZSA_MSG_CMD_NAV_CPI (0x25, reserved). */

/* ZSA_MSG_EVT_NAV_PRESENT (1 byte) */
enum zsa_nav_device_type {
    ZSA_NAV_DEVICE_NONE      = 0,
    ZSA_NAV_DEVICE_TRACKBALL = 1,
    ZSA_NAV_DEVICE_TOUCHPAD  = 2,
};

struct __attribute__((packed)) zsa_nav_present {
    uint8_t device_type; /* enum zsa_nav_device_type */
};

/* struct zsa_nav_motion removed with ZSA_MSG_EVT_NAV_MOTION (0x86, reserved). */

/* struct zsa_nav_report_raw removed with ZSA_MSG_EVT_NAV_REPORT_RAW (0x87,
 * reserved). Trackpad data flows over the emulated-i2c pad mirror instead. */

/* ZSA_MSG_EVT_DIGITIZER_MODE (1 byte) */
struct __attribute__((packed)) zsa_digitizer_mode {
    uint8_t mode; /* 0 = Mouse (boot default), 3 = Windows Precision Touchpad */
};

/* ZSA_MSG_EVT_PWR_BUDGET (2 bytes, LE) — detected USB source current budget.
 * Carries mA (not a bool) so 1.5 A vs 3 A differentiation or a dynamic
 * per-side split later needs no protocol change. Today the left's CC detect
 * only distinguishes default vs high class, so it sends DEFAULT or HIGH.
 *
 * UNKNOWN is its own value rather than a low number: no source is attached, or
 * one is but CC has not resolved yet. It must not read as DEFAULT — a source
 * that really advertised 500 mA is a source we may draw 500 mA from, and one
 * we have not identified is not. */
#define ZSA_PWR_BUDGET_UNKNOWN_MA    0u
#define ZSA_PWR_BUDGET_DEFAULT_MA  500u
#define ZSA_PWR_BUDGET_HIGH_MA    1500u
struct __attribute__((packed)) zsa_pwr_budget {
    uint16_t budget_ma;
};

/* ZSA_MSG_EVT_VBUS_STATE (1 byte) */
struct __attribute__((packed)) zsa_vbus_state { uint8_t present; };

/* ZSA_MSG_EVT_DFU_STATE (3 bytes) — `active` mirrors the nRF's DFU lock, so
 * it covers both the mcumgr SMP path and the STM32 relay; QMK drops keys and
 * holds the idle state off while it is set. `target`/`phase` describe the
 * left half's own image for the status LEDs (the right half indicates
 * itself). Named _progress because the zsa_dfu_state tag is taken by the
 * DFU_STATUS enum below. Sent as a level: the nRF re-sends it every second,
 * so QMK ages it out if the link dies mid-update. A 1-byte payload (older
 * senders) carries only `active`; receivers treat it as target NONE. */
enum zsa_dfu_target {
    ZSA_DFU_TARGET_NONE  = 0,
    ZSA_DFU_TARGET_LEFT  = 1,
    ZSA_DFU_TARGET_RIGHT = 2,
};
enum zsa_dfu_phase {
    ZSA_DFU_PHASE_IDLE   = 0, /* nothing to show */
    ZSA_DFU_PHASE_UPLOAD = 1, /* image being written: green fast blink */
    ZSA_DFU_PHASE_SETTLE = 2, /* written; swapping or awaiting the host's confirm: green solid */
    ZSA_DFU_PHASE_FAILED = 3, /* update refused or aborted: red solid */
};
struct __attribute__((packed)) zsa_dfu_progress {
    uint8_t active; /* 0 = idle, 1 = update in progress */
    uint8_t target; /* enum zsa_dfu_target */
    uint8_t phase;  /* enum zsa_dfu_phase */
};

/* Status LED timing shared by every renderer (QMK, right nRF, ignition):
 * half-periods of the fast (upload) and slow (protocol mismatch) blinks, and
 * how long a FAILED phase is held before the LEDs return to normal. */
#define ZSA_DFU_LED_FAST_MS     125u
#define ZSA_DFU_LED_SLOW_MS     500u
#define ZSA_DFU_FAILED_HOLD_MS  60000u

/* ZSA_MSG_EVT_LINK_STATE (2 bytes) — the ESB link between the two nRF halves.
 * MISMATCH means a right half is present and reachable but running a different
 * keyboard-protocol version, so neither half drives the other's keyboard state:
 * keys, LEDs and the navigator are dead while the update channel still works.
 * It needs its own indication, NOT the blue "updating" matrix. Sent as a level,
 * re-sent every ZSA_STATUS_REFRESH cadence like the other events. */
enum zsa_link_state_val {
    ZSA_LINK_STATE_ABSENT   = 0, /* no right half heard */
    ZSA_LINK_STATE_OK       = 1, /* present, versions agree */
    ZSA_LINK_STATE_MISMATCH = 2, /* present, different protocol version */
};
struct __attribute__((packed)) zsa_link_state {
    uint8_t state;        /* enum zsa_link_state_val */
    uint8_t peer_version; /* right half's reported version, 0 when unknown */
};

/* ---- Adaptive charging policy ----
 *
 * ZSA_MSG_CMD_CHARGE_LIMITS (4 bytes, LE) — the two currents to apply, in mA,
 * following EVT_PWR_BUDGET's precedent of carrying a number rather than a
 * class. It replaced a 1-byte {DEFAULT, HIGH} level, which could say nothing
 * about a source between the two and nothing at all about charge rate.
 *
 *   vbus_limit_ma  how much this half may draw from VBUS (nPM1300 VBUSINILIM)
 *   charge_ma      how much of that goes into the cell (nPM1300 ICHG)
 *
 * Absolute values, not levels: the STM32 is the policy point and must be able
 * to name any rate the source allows, including ones no enum anticipated. Each
 * half still clamps what it receives to its own devicetree bounds, so a half
 * with a different cell or a different VBUS path is never talked past them. */
struct __attribute__((packed)) zsa_charge_limits {
    uint16_t vbus_limit_ma;
    uint16_t charge_ma;
};

/* Input current limit bounds, in mA.
 *
 * FLOOR is what a USB device may draw before it is configured. It is the
 * startup limit in both halves' devicetree (vbus-limit-microamp), where an
 * unidentified source lands, and where the PMIC returns by itself on VBUS
 * removal. Nothing may lower it further and nothing may skip past it.
 *
 * MAX and STEP are the nPM1300's VBUSINILIM register: 100 mA steps up to
 * 1.5 A, which is also why a 3 A source buys nothing over a 1.5 A one. */
#define ZSA_VBUS_LIMIT_FLOOR_MA  100u
#define ZSA_VBUS_LIMIT_MAX_MA   1500u
#define ZSA_VBUS_LIMIT_STEP_MA   100u

/* One source, two halves. The right half's power is exported from the left
 * over TRRS_Vo, so its budget is not a second one — the two divide the one
 * they have, and the old 700 mA-per-side ceiling was that division in
 * disguise. Split evenly, which is the only division that is safe whichever
 * side of the left's own input gate TRRS_Vo turns out to be taken from: in
 * series the pair draws half the budget, in parallel exactly the budget, and
 * neither exceeds it. */
#define ZSA_CHARGE_HALVES 2u

/* Held back from each half's share for that half's own draw, before any of it
 * is offered to its charger: its nRF, its LED rail at a brightness someone
 * plugged in is likely to run, and on the left the STM32 as well. Counted
 * amp-for-amp against the VBUS limit even though the PMIC's buck means an amp
 * in is worth more than an amp out — the error is in the safe direction.
 *
 * Overcommitting is the expensive mistake: the nPM1300 does not back its
 * charger off to fit the input limit, it lets VSYS droop and supplements from
 * the pack, so a battery that is supposed to be charging discharges instead.
 * Undercommitting only charges slower. An estimate until someone puts a meter
 * on a half at a known brightness. */
#define ZSA_SYSTEM_DRAW_MA 150u

/* Charge current bounds, in mA. MAX is the pack's rated charge current and
 * matches current-microamp in both halves' devicetree today; MIN is the
 * nPM1300's smallest programmable ICHG. Policy's belief about the cell — each
 * nRF clamps to the devicetree it was actually built with. */
#define ZSA_CHARGE_CURRENT_MIN_MA  32u
#define ZSA_CHARGE_CURRENT_MAX_MA 700u

/* The whole policy: what a source advertising budget_ma entitles ONE HALF to.
 * Both halves are sent the same answer, and between them they stay inside what
 * the source offered.
 *
 * Derived rather than tabulated, so a source class nobody has written down yet
 * still gets a sensible answer — the same reason EVT_PWR_BUDGET carries mA.
 * The input limit is the largest step the PMIC can program at or below this
 * half's share, never under the pre-enumeration floor and never over what the
 * register can express. The charge current is whatever is left of that after
 * the half has fed itself, inside the cell's own limits.
 *
 * ZSA_PWR_BUDGET_UNKNOWN_MA lands on the floor, which is the point: an
 * unidentified source and a source that advertised nothing are the same source
 * as far as what we are allowed to take from it. At the floor there is not
 * even enough for the keyboard to run, let alone charge — both halves run
 * partly off their packs until a source identifies itself, exactly as they do
 * today — so the charge current there is simply the smallest the PMIC can be
 * set to rather than a share of anything.
 *
 * Pure and total — every uint16_t is a valid input. Tested exhaustively in
 * shared/tests/zsa_charge_policy_test.c. */
static inline struct zsa_charge_limits zsa_charge_limits_for_budget(uint16_t budget_ma)
{
    struct zsa_charge_limits out;
    uint16_t share = (uint16_t)(budget_ma / ZSA_CHARGE_HALVES);
    uint16_t vbus  = (share > ZSA_VBUS_LIMIT_MAX_MA) ? ZSA_VBUS_LIMIT_MAX_MA : share;
    uint16_t charge;

    vbus = (uint16_t)(vbus - (vbus % ZSA_VBUS_LIMIT_STEP_MA));
    if (vbus < ZSA_VBUS_LIMIT_FLOOR_MA) {
        vbus = ZSA_VBUS_LIMIT_FLOOR_MA;
    }

    charge = (vbus > ZSA_SYSTEM_DRAW_MA) ? (uint16_t)(vbus - ZSA_SYSTEM_DRAW_MA) : 0u;
    if (charge < ZSA_CHARGE_CURRENT_MIN_MA) {
        charge = ZSA_CHARGE_CURRENT_MIN_MA;
    }
    if (charge > ZSA_CHARGE_CURRENT_MAX_MA) {
        charge = ZSA_CHARGE_CURRENT_MAX_MA;
    }

    out.vbus_limit_ma = vbus;
    out.charge_ma     = charge;
    return out;
}

/* ZSA_MSG_CMD_BUS_GRANT (1 byte) — a lease, not a latch. QMK resends it on
 * every ownership re-evaluation; the right half releases the bus on its own if
 * grants stop arriving, so a lost frame degrades to "STM32 owns the bus"
 * rather than to two masters. */
struct __attribute__((packed)) zsa_bus_grant {
    uint8_t granted; /* 0 = STM32 owns the right half's i2c bus, 1 = right half does */
};
/* Resend cadence, and how long the left keeps relaying a grant without hearing
 * another one. The left must expire too, or an STM32 that stops talking would
 * leave the grant latched and the right on the bus forever. */
#define ZSA_BUS_GRANT_INTERVAL_MS 100u
#define ZSA_BUS_GRANT_TIMEOUT_MS  500u

/* ---- DFU state / error codes ---- */

/* DFU state values (ZSA_MSG_DFU_STATUS payload[0]) */
enum zsa_dfu_state {
    ZSA_DFU_STATE_IDLE     = 0,
    ZSA_DFU_STATE_BUSY     = 1,
    ZSA_DFU_STATE_DONE     = 2,
    ZSA_DFU_STATE_ERROR    = 3,
};

/* DFU error codes (ZSA_MSG_DFU_STATUS payload[1]) */
enum zsa_dfu_error {
    ZSA_DFU_ERR_NONE       = 0x00,
    ZSA_DFU_ERR_ADDRESS    = 0x01,
    ZSA_DFU_ERR_ERASE      = 0x02,
    ZSA_DFU_ERR_WRITE      = 0x03,
    ZSA_DFU_ERR_VERIFY     = 0x04,
    ZSA_DFU_ERR_UNKNOWN    = 0xFF,
};

/* ZSA_MSG_DFU_MANIFEST (8 bytes, optional — a 0-byte manifest skips the check)
 *
 * The DFU data characteristic is write-without-response and nothing acks a
 * chunk, so a dropped write is invisible to both ends. This is the only
 * end-to-end integrity check on the image: the STM32 reads back what it wrote
 * and refuses to reset into it unless the CRC matches.
 *
 * crc32 is CRC-32/ISO-HDLC (poly 0xEDB88320 reflected, init 0xFFFFFFFF, final
 * xor 0xFFFFFFFF) over the first `len` bytes at APP_ADDRESS — the same value
 * Python's zlib.crc32 and Rust's crc32fast produce over the image file.
 */
struct __attribute__((packed)) zsa_dfu_manifest {
    uint32_t len;    /* image length in bytes */
    uint32_t crc32;
};

#define ZSA_DFU_MANIFEST_LEN 8u

/* ---- Live-app confirm tunnel ---- */

/* The leading bytes of the running image's TLV hash — the same prefix
 * img_mgmt's `sha` carries and the same length the halves' update session
 * names, so a host compares one value everywhere. */
#define ZSA_CONFIRM_HASH_LEN 32u

/* ZSA_MSG_CONFIRM_QUERY mode. The host reads both halves' running hashes first
 * and only sends COMMIT once both reads matched. */
enum zsa_confirm_mode {
    ZSA_CONFIRM_MODE_READ   = 0,  /* report the running hash + confirmed state; write nothing */
    ZSA_CONFIRM_MODE_COMMIT = 1,  /* write the trailer if the running hash matches expect */
};

/* ZSA_MSG_CONFIRM_QUERY: the mode, then the hash the host expects the target to
 * be running. READ reports the running hash and writes nothing. COMMIT writes
 * MCUboot's trailer only if the running hash matches, which is img_mgmt's
 * STATE_WRITE rule. */
struct __attribute__((packed)) zsa_confirm_query {
    uint8_t mode;                        /* enum zsa_confirm_mode */
    uint8_t hash[ZSA_CONFIRM_HASH_LEN];
};

#define ZSA_CONFIRM_QUERY_LEN 33u

/* ZSA_MSG_CONFIRM_REPLY payload[0]. */
enum zsa_confirm_status {
    ZSA_CONFIRM_OK         = 0x00,  /* COMMIT: matched; the trailer is now confirmed */
    ZSA_CONFIRM_ALREADY    = 0x01,  /* matched; it was already confirmed */
    ZSA_CONFIRM_MISMATCH   = 0x02,  /* COMMIT: a different image is running; not written */
    ZSA_CONFIRM_UNREADABLE = 0x03,  /* the running hash could not be read */
    ZSA_CONFIRM_WRITE_FAIL = 0x04,  /* the trailer write itself failed */
    ZSA_CONFIRM_RUNNING    = 0x05,  /* READ: running the reported image, not confirmed; nothing written */
};

/* ZSA_MSG_CONFIRM_REPLY: what the target did, and what it is actually
 * running. `hash` is all zero when status is ZSA_CONFIRM_UNREADABLE. */
struct __attribute__((packed)) zsa_confirm_reply {
    uint8_t status;                     /* enum zsa_confirm_status */
    uint8_t hash[ZSA_CONFIRM_HASH_LEN];
};

#define ZSA_CONFIRM_REPLY_LEN 33u

/* ZSA_MSG_LOG_LINE source tag */
enum zsa_log_src {
    ZSA_LOG_SRC_NRF  = 0,
    ZSA_LOG_SRC_STM  = 1,
    ZSA_LOG_SRC_BOOT = 2,
};

/* ZSA_MSG_LOG_LINE level (matches Zephyr log levels 1..4; 0 reserved) */
enum zsa_log_level {
    ZSA_LOG_LVL_NONE = 0,
    ZSA_LOG_LVL_ERR  = 1,
    ZSA_LOG_LVL_WRN  = 2,
    ZSA_LOG_LVL_INF  = 3,
    ZSA_LOG_LVL_DBG  = 4,
};

#define ZSA_LOG_MAX_TEXT   120u
#define ZSA_LOG_HDR_LEN    6u   /* src(1)+lvl(1)+ts_ms(4) */
#define ZSA_LOG_MAX_PAYLOAD (ZSA_LOG_HDR_LEN + ZSA_LOG_MAX_TEXT)

/* ZSA_MSG_LOG_LINE payload header (little-endian on the wire) */
struct __attribute__((packed)) zsa_log_hdr {
    uint8_t  src;       /* enum zsa_log_src */
    uint8_t  lvl;       /* enum zsa_log_level */
    uint32_t ts_ms;     /* left-nRF uptime (ms); 0 from STM/boot → nRF stamps */
    /* text[N] follows, N = frame.len - ZSA_LOG_HDR_LEN */
};

/* Fill *hdr_out with a log header (text is written separately by caller). */
static inline void zsa_log_hdr_pack(struct zsa_log_hdr *hdr_out,
                                    uint8_t src, uint8_t lvl, uint32_t ts_ms) {
    hdr_out->src   = src;
    hdr_out->lvl   = lvl;
    hdr_out->ts_ms = ts_ms;
}

/* ---- Parser ---- */

enum zsa_parse_state {
    ZSA_PARSE_HUNT,
    ZSA_PARSE_TYPE,
    ZSA_PARSE_LEN_LO,
    ZSA_PARSE_LEN_HI,
    ZSA_PARSE_PAYLOAD,
    ZSA_PARSE_CRC,
};

struct zsa_frame {
    uint8_t  type;
    uint16_t len;
    uint8_t  payload[ZSA_MAX_PAYLOAD];
};

struct zsa_parser {
    enum zsa_parse_state state;
    struct zsa_frame     frame;
    uint16_t             idx;
    uint8_t              esc;   /* 1 = previous byte was ESC */
};

/* ---- CRC-8/CCITT (polynomial 0x07) ---- */

static inline uint8_t zsa_crc8(const uint8_t *data, uint16_t len) {
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* ---- Frame encoding ---- */

/* Emit one byte with escaping. Returns number of bytes written (1 or 2). */
static inline uint8_t zsa_emit_escaped(uint8_t *buf, uint8_t byte) {
    if (byte == ZSA_SYNC_BYTE) {
        buf[0] = ZSA_ESC_BYTE;
        buf[1] = ZSA_ESC_SYNC;
        return 2;
    }
    if (byte == ZSA_ESC_BYTE) {
        buf[0] = ZSA_ESC_BYTE;
        buf[1] = ZSA_ESC_ESC;
        return 2;
    }
    buf[0] = byte;
    return 1;
}

/*
 * Encode a frame into buf. Returns total frame size (including escaping).
 * buf must be at least (len + ZSA_FRAME_OVERHEAD) * 2 bytes to handle
 * worst-case escaping.
 */
static inline uint16_t zsa_frame_encode(uint8_t *buf, uint8_t type, const uint8_t *payload, uint16_t len) {
    uint16_t pos = 0;

    /* SYNC byte is never escaped */
    buf[pos++] = ZSA_SYNC_BYTE;

    /* Compute CRC over unescaped TYPE + LEN(2) + PAYLOAD */
    uint8_t len_lo = (uint8_t)(len & 0xFF);
    uint8_t len_hi = (uint8_t)(len >> 8);

    uint8_t crc = 0x00;
    /* Feed TYPE + LEN_LO + LEN_HI into CRC */
    uint8_t hdr[3] = { type, len_lo, len_hi };
    crc = zsa_crc8(hdr, 3);
    /* Continue CRC over payload */
    for (uint16_t i = 0; i < len; i++) {
        uint8_t b = payload[i];
        crc ^= b;
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }

    /* Emit TYPE, LEN_LO, LEN_HI, PAYLOAD, CRC — all escaped */
    pos += zsa_emit_escaped(&buf[pos], type);
    pos += zsa_emit_escaped(&buf[pos], len_lo);
    pos += zsa_emit_escaped(&buf[pos], len_hi);
    for (uint16_t i = 0; i < len; i++) {
        pos += zsa_emit_escaped(&buf[pos], payload[i]);
    }
    pos += zsa_emit_escaped(&buf[pos], crc);

    return pos;
}

/* ---- Frame parsing ---- */

static inline void zsa_parser_init(struct zsa_parser *p) {
    p->state = ZSA_PARSE_HUNT;
    p->esc   = 0;
}

/*
 * Feed one byte to the parser.
 * Returns 1 when a complete valid frame is ready in p->frame, 0 otherwise.
 */
static inline int zsa_parser_feed(struct zsa_parser *p, uint8_t byte) {
    /* In HUNT state, only look for SYNC */
    if (p->state == ZSA_PARSE_HUNT) {
        if (byte == ZSA_SYNC_BYTE) {
            p->state = ZSA_PARSE_TYPE;
            p->esc   = 0;
        }
        return 0;
    }

    /* Outside HUNT: a raw SYNC means a new frame is starting (abort current) */
    if (byte == ZSA_SYNC_BYTE) {
        p->state = ZSA_PARSE_TYPE;
        p->esc   = 0;
        return 0;
    }

    /* Handle escape sequences */
    if (p->esc) {
        p->esc = 0;
        if (byte == ZSA_ESC_SYNC) {
            byte = ZSA_SYNC_BYTE;
        } else if (byte == ZSA_ESC_ESC) {
            byte = ZSA_ESC_BYTE;
        } else {
            /* Invalid escape sequence — abort frame */
            p->state = ZSA_PARSE_HUNT;
            return 0;
        }
    } else if (byte == ZSA_ESC_BYTE) {
        p->esc = 1;
        return 0;
    }

    /* Process the (unescaped) byte through the state machine */
    switch (p->state) {
        case ZSA_PARSE_TYPE:
            p->frame.type = byte;
            p->state      = ZSA_PARSE_LEN_LO;
            return 0;

        case ZSA_PARSE_LEN_LO:
            p->frame.len = byte;
            p->state     = ZSA_PARSE_LEN_HI;
            return 0;

        case ZSA_PARSE_LEN_HI:
            p->frame.len |= ((uint16_t)byte << 8);
            if (p->frame.len > ZSA_MAX_PAYLOAD) {
                p->state = ZSA_PARSE_HUNT;
                return 0;
            }
            if (p->frame.len == 0) {
                p->state = ZSA_PARSE_CRC;
            } else {
                p->idx   = 0;
                p->state = ZSA_PARSE_PAYLOAD;
            }
            return 0;

        case ZSA_PARSE_PAYLOAD:
            p->frame.payload[p->idx++] = byte;
            if (p->idx == p->frame.len) {
                p->state = ZSA_PARSE_CRC;
            }
            return 0;

        case ZSA_PARSE_CRC: {
            /* Compute CRC over TYPE + LEN_LO + LEN_HI + PAYLOAD */
            uint8_t hdr[3] = {
                p->frame.type,
                (uint8_t)(p->frame.len & 0xFF),
                (uint8_t)(p->frame.len >> 8)
            };
            uint8_t expected = zsa_crc8(hdr, 3);
            /* Continue CRC over payload */
            for (uint16_t i = 0; i < p->frame.len; i++) {
                expected ^= p->frame.payload[i];
                for (uint8_t bit = 0; bit < 8; bit++) {
                    expected = (expected & 0x80)
                        ? (uint8_t)((expected << 1) ^ 0x07)
                        : (uint8_t)(expected << 1);
                }
            }

            p->state = ZSA_PARSE_HUNT;
            return (byte == expected) ? 1 : 0;
        }

        default:
            p->state = ZSA_PARSE_HUNT;
            return 0;
    }
}

#endif /* ZSA_UART_PROTO_H */
