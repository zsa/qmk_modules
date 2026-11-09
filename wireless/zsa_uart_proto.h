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
    ZSA_MSG_ORYX_TO_HOST     = 0x05,  /* ZSA_ORYX_REPORT_LEN B: one Oryx raw-HID
                                       * IN report, as it would go on the USB
                                       * raw IN endpoint. Not known to
                                       * ignition. */

    /* STM32 -> nRF: queries (0x10-0x1F) */
    ZSA_MSG_QUERY_BLE_STATUS = 0x10,
    ZSA_MSG_QUERY_BATTERY    = 0x11,
    ZSA_MSG_QUERY_PAIR_STATE = 0x12,
    ZSA_MSG_QUERY_DIAG       = 0x13,  /* 2 B: struct zsa_diag_query — one page
                                       * of the left's diagnostic record, this
                                       * boot's or the previous boot's.
                                       * Answered with one EVT_DIAG. Sent on
                                       * demand only, one page at a time. Not
                                       * in ignition's copy of this header. */
    ZSA_MSG_QUERY_RADIO_INFO = 0x14,  /* 0 B: the radio firmware each half
                                       * runs. Answered with one
                                       * EVT_RADIO_INFO. Sent on demand only.
                                       * Not in ignition's copy of this
                                       * header. */
    ZSA_MSG_QUERY_HOST_SLOTS = 0x15,  /* 0 B: the board's own address and
                                       * state for every host slot. Answered
                                       * with one EVT_HOST_SLOTS. Sent on
                                       * demand only. Not in ignition's copy of
                                       * this header. */

    /* STM32 -> nRF: commands (0x20-0x2F) */
    ZSA_MSG_CMD_HIBERNATE    = 0x20,
    ZSA_MSG_CMD_PAIR         = 0x21,  /* 0 B: Legacy, ignored. Pairing is
                                       * ZSA_MSG_CMD_BLE_BANK op PAIR. */
    ZSA_MSG_CMD_IDLE_STATE   = 0x22,
    ZSA_MSG_CMD_USB_STATE    = 0x23,  /* 1 B: 0=no host, 1=USB host (active or suspended) */
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
    ZSA_MSG_CMD_BLE_DISCONNECT = 0x29,  /* 0 B: same as ZSA_MSG_CMD_BLE_BANK
                                         * op DISCONNECT */
    ZSA_MSG_CMD_BLE_UNPAIR   = 0x2A,  /* 0 B: Legacy, ignored. Use
                                       * ZSA_MSG_CMD_BLE_BANK op FORGET. */
    ZSA_MSG_CMD_BLE_SWITCH_HOST = 0x2B, /* 0 B: Legacy, ignored. Use
                                         * ZSA_MSG_CMD_BLE_BANK op CONNECT. */
    ZSA_MSG_CMD_RIGHT_TELEMETRY = 0x2C, /* <=ZSA_RIGHT_TLM_SIZE B: struct
                                         * zsa_right_telemetry (see
                                         * zsa_right_telemetry.h), the right
                                         * half's own wired 0x54 answer relayed
                                         * by the STM32 while it masters the
                                         * TRRS bus — the only path carrying the
                                         * right's battery to the left in wired
                                         * mode, where the right's radio is off.
                                         * Level: resent after every successful
                                         * read (1 s); the left calls it stale
                                         * after ZSA_RIGHT_TLM_RELAY_STALE_MS
                                         * without one. Not in ignition's copy
                                         * of this header (frozen at
                                         * voyager2-2.0.1; ignition never sends
                                         * it). */
    ZSA_MSG_CMD_TRRS_STATE   = 0x2D,  /* 1 B: struct zsa_trrs_state — level,
                                       * not a lease: the TRRS cable between
                                       * the halves is plugged (the STM32 got
                                       * an ACK from the real right expander).
                                       * Sent on every edge, at link-up, and
                                       * refreshed every ZSA_TRRS_STATE_INTERVAL_MS.
                                       * The left runs ESB only while unplugged.
                                       * Not in ignition's copy of this header
                                       * (frozen at voyager2-2.0.1; ignition
                                       * never sends it). */
    ZSA_MSG_CMD_SCAN_DIAG    = 0x2E,  /* 16 B: struct zsa_scan_diag — STM32
                                       * main-loop and link counters, sent
                                       * every ZSA_SCAN_DIAG_INTERVAL_MS. The
                                       * left folds it into its BLE telemetry
                                       * read (bat_telemetry) next to its own
                                       * struct zsa_nrf_diag. Not in ignition's
                                       * copy of this header (frozen at
                                       * voyager2-2.0.1; ignition never sends
                                       * it). */
    ZSA_MSG_CMD_CHARGE_LIMITS = 0x26, /* 4 B: struct zsa_charge_limits — the
                                       * VBUS input current limit and the
                                       * charge current to apply, both in mA.
                                       * The STM32 is the charging policy
                                       * point; the left nRF only detects CC
                                       * capability (EVT_PWR_BUDGET) and
                                       * executes this command. Was
                                       * CMD_VBUS_ILIM (1 B level) — same ID,
                                       * wider payload; see struct below. */
    ZSA_MSG_CMD_NAV_BOOST    = 0x2F,  /* 1 B: struct zsa_nav_boost — whether
                                       * the left nav connector needs the
                                       * left boost: a nav device answered on
                                       * the STM32's own bus, or the STM32 is
                                       * running a probe window. Sent on every
                                       * change and at link-up. The left drops
                                       * it on LINK_UP from the STM32 and on
                                       * reboot, and until it hears one again
                                       * treats a nav-capable build as
                                       * needing the boost. Not in ignition's
                                       * copy of this header (frozen at
                                       * voyager2-2.0.1; ignition never sends
                                       * it). */

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

    /* STM32 -> nRF: state (0x40-0x4F) */
    ZSA_MSG_CMD_NAV_STATE    = 0x40,  /* 4 B: struct zsa_nav_state — the nav
                                       * devices the STM32 reports per half,
                                       * as in its raw-HID telemetry. Sent on
                                       * every change and as a level every
                                       * ZSA_NAV_STATE_INTERVAL_MS. Not in
                                       * ignition's copy of this header. */
    ZSA_MSG_CMD_BLE_BANK     = 0x41,  /* 2 B: struct zsa_ble_bank_cmd — act on
                                       * one host bank (pair, connect,
                                       * disconnect, forget). Not in ignition's
                                       * copy of this header. */

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
                                       * USB source current budget from CC1/CC2,
                                       * DEFAULT or HIGH. Sent on CC/VBUS change
                                       * and at link-up, and refreshed every
                                       * second. */
    ZSA_MSG_EVT_VBUS_STATE   = 0x8B,  /* 1 B: struct zsa_vbus_state — nPM1300
                                       * VBUS presence. Gates the STM32's USB
                                       * D+ pullup (PA15). Sent on VBUS change
                                       * and at link-up. */
    ZSA_MSG_EVT_DFU_STATE    = 0x8C,  /* 4 B: struct zsa_dfu_progress — a
                                       * firmware update is running through
                                       * this half (nRF SMP or the STM32
                                       * relay), plus which half's image and
                                       * what phase, for the status LEDs, and
                                       * whether to stay awake. Sent on every
                                       * change and re-sent as a level. Older
                                       * senders send 1 or 3 B. */
    ZSA_MSG_EVT_LINK_STATE   = 0x8D,  /* 2 B: struct zsa_link_state — the two
                                       * nRF halves' view of each other over
                                       * ESB. Distinct from EVT_DFU_STATE: a
                                       * protocol mismatch is a half-working
                                       * keyboard, not an update in progress,
                                       * and must not read as one. Sent as a
                                       * level. */
    ZSA_MSG_EVT_POWER_VERDICT = 0x8E, /* 1 B: struct zsa_power_verdict — the
                                       * nRF's boot-time power verdict; the
                                       * STM32 keeps the key matrix dark until
                                       * it arrives. Re-sent when it changes
                                       * (VBUS attach). Not known to ignition. */
    ZSA_MSG_EVT_BATTERY_DETAIL = 0x8F, /* 26 B: struct zsa_battery_detail_event
                                        * — both halves' cell voltage, current,
                                        * temperature, time estimates and
                                        * charger registers. What EVT_BATTERY
                                        * carries as one percentage, spelled
                                        * out. Sent as a level alongside
                                        * EVT_BATTERY. Not known to ignition. */

    /* STM32 -> nRF: DFU responses (0x90-0x9F) */
    ZSA_MSG_DFU_STATUS       = 0x90,  /* 2 B: state + error code */

    /* STM32 -> nRF: DFU announcement (0xA0) */
    ZSA_MSG_DFU_READY        = 0xA0,  /* 0 B: bootloader is in DFU mode */

    /* Shared (bidirectional): log line (0xB0) */
    ZSA_MSG_LOG_LINE         = 0xB0,  /* src(1)+lvl(1)+ts_ms(4 LE)+text(<=120) */

    /* nRF -> STM32: events continued (0xC0-0xCF). The 0x80-0x8F block is full;
     * 0x86-0x88 in it stay reserved rather than being reused. */
    ZSA_MSG_EVT_BLE_IDENTITY = 0xC0,  /* 9 B: struct zsa_ble_identity — the
                                       * Bluetooth address this board answers
                                       * to on the host it is currently talking
                                       * to, plus which of its host slots that
                                       * is. One identity per slot, so the
                                       * address changes with the slot. Sent as
                                       * a level alongside EVT_BATTERY_DETAIL
                                       * and on QUERY_BATTERY. Not known to
                                       * ignition. */
    ZSA_MSG_EVT_PMIC_RAILS   = 0xC1,  /* 14 B: struct zsa_pmic_rails_event —
                                       * both halves' PMIC input rails, so the
                                       * cable path carries what the BLE detail
                                       * read carries. Sent as a level
                                       * alongside EVT_BLE_IDENTITY and on
                                       * QUERY_BATTERY. Not known to
                                       * ignition. */
    ZSA_MSG_EVT_ORYX_FROM_HOST = 0xC2, /* ZSA_ORYX_REPORT_LEN B: one Oryx
                                       * raw-HID OUT report from the BLE host,
                                       * zero padded. Other lengths are
                                       * dropped. Not known to ignition. */
    ZSA_MSG_EVT_BLE_BANKS    = 0xC3,  /* 4 B: struct zsa_ble_banks — host bank
                                       * table and link state. Sent on every
                                       * change and as a level every
                                       * ZSA_BLE_BANKS_INTERVAL_MS. Not known
                                       * to ignition. */
    ZSA_MSG_EVT_DIAG         = 0xC4,  /* 28 B: struct zsa_diag_page — the answer
                                       * to QUERY_DIAG. Not known to
                                       * ignition. */
    ZSA_MSG_EVT_RADIO_INFO   = 0xC5,  /* ZSA_RADIO_INFO_LEN B: struct
                                       * zsa_radio_info — the answer to
                                       * QUERY_RADIO_INFO. Not known to
                                       * ignition. */
    ZSA_MSG_EVT_HOST_SLOTS   = 0xC6,  /* ZSA_HOST_SLOTS_LEN B: struct
                                       * zsa_host_slots — the answer to
                                       * QUERY_HOST_SLOTS. Not known to
                                       * ignition. */
};

/* ---- Payload structures ---- */

/* ZSA_MSG_ORYX_TO_HOST / ZSA_MSG_EVT_ORYX_FROM_HOST: one Oryx raw-HID report. */
#define ZSA_ORYX_REPORT_LEN 32

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

/* No usable state of charge: the fuel gauge has not converged, or the half was
 * never heard from. Never send 0 for that -- it reads as a flat battery.
 * Applies to both SOC fields below. */
#define ZSA_BAT_SOC_UNKNOWN 0xFFu

/* Payload length before `right_flags` was appended. Left firmware that
 * predates it sends this; a reader must accept both lengths. */
#define ZSA_BATTERY_STATUS_LEN_V1 3u

/* ZSA_MSG_EVT_BATTERY (4 bytes) */
struct __attribute__((packed)) zsa_battery_status {
    uint8_t soc;         /* left half SOC, 0-100, or ZSA_BAT_SOC_UNKNOWN */
    uint8_t flags;       /* left half; bit 0: charging, bit 1: USB present */
    uint8_t right_soc;   /* right half SOC, 0-100, or ZSA_BAT_SOC_UNKNOWN */
    uint8_t right_flags; /* right half, same bits as `flags`; meaningful only
                          * while right_soc is known */
};

#define ZSA_BAT_FLAG_CHARGING 0x01
#define ZSA_BAT_FLAG_USB      0x02

/* One half's charging detail. Every field has its own "could not read it"
 * value, and none of them is a reading the field can really take: 0 mA and a
 * current nobody measured must not arrive as the same number. Also carried
 * over ESB (wireless_i2c_reversed_proto.h), over the wired 0x54 read
 * (zsa_right_telemetry.h) and over BLE. */
#define ZSA_BAT_DETAIL_MV_UNKNOWN   0u       /* voltage_mv */
#define ZSA_BAT_DETAIL_I16_UNKNOWN  (-32768) /* current_ma, temp_cdeg */
#define ZSA_BAT_DETAIL_MIN_UNKNOWN  0xFFFFu  /* tte_min, ttf_min */
#define ZSA_BAT_DETAIL_REG_UNKNOWN  0xFFu    /* chg_status, chg_error, chg_sensor */

struct __attribute__((packed)) zsa_battery_detail {
    uint16_t voltage_mv;  /* cell voltage */
    int16_t  current_ma;  /* positive = into the cell (charging), negative = discharge */
    int16_t  temp_cdeg;   /* NTC battery temperature, 0.01 degC */
    uint16_t tte_min;     /* fuel-gauge time to empty, 0..0xFFFE minutes;
                           * 0xFFFF is unknown, and so is any estimate past
                           * 0xFFFE minutes (~45 days) */
    uint16_t ttf_min;     /* fuel-gauge time to full, same range and cap */
    uint8_t  chg_status;  /* nPM1300 CHARGER.BCHGCHARGESTATUS, raw */
    uint8_t  chg_error;   /* nPM1300 CHARGER.BCHGERRREASON, raw (latched fault) */
    uint8_t  chg_sensor;  /* nPM1300 CHARGER.BCHGERRSENSOR, raw */
};

#define ZSA_BAT_DETAIL_SIZE 13u

_Static_assert(sizeof(struct zsa_battery_detail) == ZSA_BAT_DETAIL_SIZE,
               "zsa_battery_detail is a wire layout");

/* Sender side: a block that claims nothing, ready for the fields that read. */
static inline void zsa_battery_detail_init(struct zsa_battery_detail *d)
{
    d->voltage_mv = ZSA_BAT_DETAIL_MV_UNKNOWN;
    d->current_ma = ZSA_BAT_DETAIL_I16_UNKNOWN;
    d->temp_cdeg  = ZSA_BAT_DETAIL_I16_UNKNOWN;
    d->tte_min    = ZSA_BAT_DETAIL_MIN_UNKNOWN;
    d->ttf_min    = ZSA_BAT_DETAIL_MIN_UNKNOWN;
    d->chg_status = ZSA_BAT_DETAIL_REG_UNKNOWN;
    d->chg_error  = ZSA_BAT_DETAIL_REG_UNKNOWN;
    d->chg_sensor = ZSA_BAT_DETAIL_REG_UNKNOWN;
}

/* One half's PMIC input rails: what the charger is being fed, next to what the
 * cell is doing. `flags` is the whole block's validity marker -- a sender too
 * old to fill the seat leaves it zeroed, which reads as "not present" rather
 * than as 0 V. Also carried over ESB (wireless_i2c_reversed_proto.h), over the
 * wired 0x54 read (zsa_right_telemetry.h) and over BLE. */
#define ZSA_PMIC_RAILS_PRESENT 0x01u /* flags bit 0: the block carries a reading */
/* flags bit 1: the boost-converter state in bit 2 is meaningful. A sender that
 * does not report it leaves both clear. */
#define ZSA_PMIC_RAILS_BOOST_KNOWN 0x02u
/* flags bit 2: the firmware has the boost converter disabled (bit 1 set). */
#define ZSA_PMIC_RAILS_BOOST_OFF 0x04u
/* flags bit 3: the half is holding its charge current below the rate it was
 * commanded, because the charger kept restarting its cycle at that rate. */
#define ZSA_PMIC_RAILS_CHARGE_REDUCED 0x08u

/* `vbusin_status` bits, nPM1300 VBUSIN.VBUSINSTATUS raw. */
#define ZSA_PMIC_RAILS_VBUS_PRESENT   0x01u
#define ZSA_PMIC_RAILS_VBUS_CUR_LIM   0x02u
#define ZSA_PMIC_RAILS_VBUS_OVP       0x04u
#define ZSA_PMIC_RAILS_VBUS_UNDERVOLT 0x08u
#define ZSA_PMIC_RAILS_VBUS_SUSPENDED 0x10u
#define ZSA_PMIC_RAILS_VBUS_OUT       0x20u

#define ZSA_PMIC_RAILS_MV_UNKNOWN  0u    /* vbus_mv, vsys_mv */
#define ZSA_PMIC_RAILS_REG_UNKNOWN 0xFFu /* vbusin_status, chg_enable */

struct __attribute__((packed)) zsa_pmic_rails {
    uint8_t  flags;         /* ZSA_PMIC_RAILS_PRESENT, else the rest means nothing */
    uint16_t vbus_mv;       /* nPM1300 ADC VBUS */
    uint16_t vsys_mv;       /* nPM1300 ADC VSYS */
    uint8_t  vbusin_status; /* VBUSIN.VBUSINSTATUS raw (base 0x02, offset 0x07) */
    uint8_t  chg_enable;    /* CHARGER.BCHGENABLESET readback (base 0x03, offset 0x04) */
};

#define ZSA_PMIC_RAILS_SIZE 7u

_Static_assert(sizeof(struct zsa_pmic_rails) == ZSA_PMIC_RAILS_SIZE,
               "zsa_pmic_rails is a wire layout");

/* Sender side: a block that claims nothing. */
static inline void zsa_pmic_rails_init(struct zsa_pmic_rails *r)
{
    r->flags         = 0u;
    r->vbus_mv       = ZSA_PMIC_RAILS_MV_UNKNOWN;
    r->vsys_mv       = ZSA_PMIC_RAILS_MV_UNKNOWN;
    r->vbusin_status = ZSA_PMIC_RAILS_REG_UNKNOWN;
    r->chg_enable    = ZSA_PMIC_RAILS_REG_UNKNOWN;
}

/* ZSA_MSG_EVT_BATTERY_DETAIL (26 bytes): left half then right half. The right
 * block is all-unknown whenever the ESB link is not carrying it (wired mode,
 * link down) — the STM32 has the right's own wired read for that case. */
struct __attribute__((packed)) zsa_battery_detail_event {
    struct zsa_battery_detail left;
    struct zsa_battery_detail right;
};

_Static_assert(sizeof(struct zsa_battery_detail_event) == 2u * ZSA_BAT_DETAIL_SIZE,
               "EVT_BATTERY_DETAIL is a wire layout");

/* ZSA_MSG_EVT_BLE_IDENTITY (9 bytes): the local Bluetooth identity in use.
 * Each host slot owns its own identity, so this is both an address and a "which
 * host am I on". The address is the local one the host is bonded to, not the
 * host's. Unknown before the identity table exists, which is why addr_type has
 * a sentinel of its own — a zero address is a real address. */
#define ZSA_BLE_ADDR_TYPE_UNKNOWN 0xFFu

struct __attribute__((packed)) zsa_ble_identity {
    uint8_t slot;       /* host slot of this identity, 0-based */
    uint8_t slot_count; /* host slots the board keeps */
    uint8_t addr_type;  /* 0 public, 1 random (bt_addr_le_t.type);
                         * ZSA_BLE_ADDR_TYPE_UNKNOWN when nothing is known,
                         * and then slot/addr mean nothing either */
    uint8_t addr[6];    /* bt_addr_t.val order, least significant byte first */
};

#define ZSA_BLE_IDENTITY_SIZE 9u

_Static_assert(sizeof(struct zsa_ble_identity) == ZSA_BLE_IDENTITY_SIZE,
               "zsa_ble_identity is a wire layout");

/* ZSA_MSG_EVT_PMIC_RAILS (14 bytes): left half then right half. A half with
 * nothing to report sends a zsa_pmic_rails_init() block, so `flags` decides
 * per half whether the reading exists. */
struct __attribute__((packed)) zsa_pmic_rails_event {
    struct zsa_pmic_rails left;
    struct zsa_pmic_rails right;
};

#define ZSA_PMIC_RAILS_EVENT_SIZE (2u * ZSA_PMIC_RAILS_SIZE)

_Static_assert(sizeof(struct zsa_pmic_rails_event) == ZSA_PMIC_RAILS_EVENT_SIZE,
               "EVT_PMIC_RAILS is a wire layout");

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
    uint8_t connected; /* 0 = no USB host, 1 = USB host present (enumerated, active or suspended, VBUS present) */
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

/* ZSA_MSG_CMD_NAV_BOOST (1 byte) */
struct __attribute__((packed)) zsa_nav_boost {
    uint8_t needed; /* 0 = no left nav device, 1 = left nav present or probing */
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

/* ZSA_MSG_CMD_NAV_STATE (4 bytes). Each field is a ZSA_NAV_CAP_* mask, or
 * ZSA_NAV_STATE_UNKNOWN. `bus` holds devices found on the bus both halves
 * share while the TRRS cable is plugged, where the side cannot be told; 0
 * when the bus is not shared. The receiver treats the state as unknown once
 * ZSA_NAV_STATE_STALE_MS passes without one. */
#define ZSA_NAV_STATE_UNKNOWN     0xFF
#define ZSA_NAV_STATE_INTERVAL_MS 1000
#define ZSA_NAV_STATE_STALE_MS    3000
struct __attribute__((packed)) zsa_nav_state {
    uint8_t build; /* devices this build can drive */
    uint8_t left;
    uint8_t right;
    uint8_t bus;
};
#define ZSA_NAV_STATE_SIZE 4

/* ZSA_MSG_CMD_BLE_BANK (2 bytes). */
#define ZSA_BLE_BANK_COUNT 5u
#define ZSA_BLE_BANK_NONE  0xFFu

enum zsa_ble_bank_op {
    ZSA_BLE_BANK_OP_PAIR       = 1, /* empty bank: discoverable for it, 2 min */
    ZSA_BLE_BANK_OP_CONNECT    = 2, /* stored bank: advertise for its host */
    ZSA_BLE_BANK_OP_DISCONNECT = 3, /* drop the link, stop advertising */
    ZSA_BLE_BANK_OP_FORGET     = 4, /* delete the bond, bank becomes empty */
    ZSA_BLE_BANK_OP_CANCEL     = 5, /* end pairing on that bank, else no-op */
};

struct __attribute__((packed)) zsa_ble_bank_cmd {
    uint8_t op;   /* enum zsa_ble_bank_op */
    uint8_t bank; /* 0-based, < ZSA_BLE_BANK_COUNT */
};
#define ZSA_BLE_BANK_CMD_SIZE 2u

/* ZSA_MSG_EVT_BLE_BANKS (4 bytes). The receiver treats the table as unknown
 * once ZSA_BLE_BANKS_STALE_MS passes without one. */
#define ZSA_BLE_BANKS_INTERVAL_MS 1000
#define ZSA_BLE_BANKS_STALE_MS    3000

enum zsa_ble_bank_state {
    ZSA_BLE_BANK_STATE_IDLE       = 0, /* not advertising, not connected */
    ZSA_BLE_BANK_STATE_CONNECTING = 1, /* advertising for `active`'s host */
    ZSA_BLE_BANK_STATE_PAIRING    = 2, /* discoverable for `active` */
    ZSA_BLE_BANK_STATE_CONNECTED  = 3, /* `active`'s host is connected */
    ZSA_BLE_BANK_STATE_OFF        = 4, /* BLE stack off (USB host attached) */
};

struct __attribute__((packed)) zsa_ble_banks {
    uint8_t stored; /* bit n set: bank n holds a bond */
    uint8_t active; /* bank `state` applies to, or ZSA_BLE_BANK_NONE */
    uint8_t state;  /* enum zsa_ble_bank_state */
    uint8_t rsvd;   /* 0 */
};
#define ZSA_BLE_BANKS_SIZE 4u

_Static_assert(sizeof(struct zsa_ble_bank_cmd) == ZSA_BLE_BANK_CMD_SIZE,
               "zsa_ble_bank_cmd is a wire layout");
_Static_assert(sizeof(struct zsa_ble_banks) == ZSA_BLE_BANKS_SIZE,
               "zsa_ble_banks is a wire layout");

/* struct zsa_nav_motion removed with ZSA_MSG_EVT_NAV_MOTION (0x86, reserved). */

/* struct zsa_nav_report_raw removed with ZSA_MSG_EVT_NAV_REPORT_RAW (0x87,
 * reserved). Trackpad data flows over the emulated-i2c pad mirror instead. */

/* ZSA_MSG_EVT_DIGITIZER_MODE (1 byte) */
struct __attribute__((packed)) zsa_digitizer_mode {
    uint8_t mode; /* 0 = Mouse (boot default), 3 = Windows Precision Touchpad */
};

/* ZSA_MSG_EVT_PWR_BUDGET (2 bytes, LE) — detected USB source current budget.
 * Carries mA (not a bool) so 1.5 A vs 3 A differentiation or a dynamic
 * per-side split later needs no protocol change. The left's CC detect
 * distinguishes default vs high class, so it sends DEFAULT or HIGH.
 *
 * "No source" is not a budget value. With nothing attached the left reports
 * DEFAULT, which is the default charging profile; VBUS presence travels in
 * EVT_VBUS_STATE. */
#define ZSA_PWR_BUDGET_DEFAULT_MA  500u
#define ZSA_PWR_BUDGET_HIGH_MA    1500u
struct __attribute__((packed)) zsa_pwr_budget {
    uint16_t budget_ma;
};

/* ZSA_MSG_EVT_VBUS_STATE (1 byte) */
struct __attribute__((packed)) zsa_vbus_state { uint8_t present; };

/* ZSA_MSG_EVT_DFU_STATE (4 bytes) — `active` mirrors the nRF's DFU lock, so
 * it covers both the mcumgr SMP path and the STM32 relay; QMK drops keys and
 * holds the idle state off while it is set. `target`/`phase` describe the
 * left half's own image for the status LEDs (the right half indicates
 * itself). Named _progress because the zsa_dfu_state tag is taken by the
 * DFU_STATUS enum below. Sent as a level: the nRF re-sends it every second,
 * so QMK ages it out if the link dies mid-update. A 1-byte payload (older
 * senders) carries only `active`; receivers treat it as target NONE.
 * `flags` is wider than `active`: KEEP_AWAKE holds the STM32 out of Sleep for
 * the whole update, including the waits between steps, without dropping keys.
 * Payloads shorter than 4 bytes carry no flags. */
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
#define ZSA_DFU_FLAG_KEEP_AWAKE 0x01u /* an update session or step is under way */
struct __attribute__((packed)) zsa_dfu_progress {
    uint8_t active; /* 0 = idle, 1 = update in progress */
    uint8_t target; /* enum zsa_dfu_target */
    uint8_t phase;  /* enum zsa_dfu_phase */
    uint8_t flags;  /* ZSA_DFU_FLAG_* */
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

/* ZSA_MSG_EVT_POWER_VERDICT (1 byte) */
#define ZSA_POWER_CRITICAL      (1u << 0) /* do not light the matrix; request ship mode */
#define ZSA_POWER_VSYSLOW_RESET (1u << 1) /* last PMIC reset was a VSYS brownout */
struct __attribute__((packed)) zsa_power_verdict {
    uint8_t flags;
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
 * FLOOR is the default per-half profile: the startup limit in both halves'
 * devicetree (vbus-limit-microamp), where the PMIC returns by itself on VBUS
 * removal, and what a default 500 mA-class or unidentified source gets.
 * Nothing may lower it further and nothing may skip past it.
 *
 * Two halves at the floor add up to 600 mA of limit, more than a 500 mA-class
 * source advertises. A limit is not a draw: each half takes charge plus system,
 * about 150 mA plus up to 150 mA, and a bench measurement on a laptop port read
 * 300 to 458 mA for the pair.
 *
 * MAX and STEP are the nPM1300's VBUSINILIM register: 100 mA steps up to
 * 1.5 A, which is also why a 3 A source buys nothing over a 1.5 A one. */
#define ZSA_VBUS_LIMIT_FLOOR_MA  300u
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
 * At the input limit the nPM1300 feeds the system first and reduces the charge
 * current to stay inside it (datasheet §6.2.10); only once the charger is at
 * zero does it supplement the system from the pack. So the input limit, not
 * this reserve, is what actually bounds a half, and overcommitting the reserve
 * costs charge current rather than draining the cell. */
#define ZSA_SYSTEM_DRAW_MA 150u

/* Charge current bounds, in mA. MAX is the pack's rated charge current and
 * matches zsa,charge-max-microamp in both halves' devicetree; MIN is the
 * nPM1300's smallest programmable ICHG. current-microamp is a different
 * number, the startup charge current, which is FLOOR minus SYSTEM_DRAW.
 * Policy's belief about the cell — each nRF clamps to the devicetree it was
 * actually built with. */
#define ZSA_CHARGE_CURRENT_MIN_MA  32u
#define ZSA_CHARGE_CURRENT_MAX_MA 700u

/* Right-half ceilings, in mA. The right is fed from the left over the TRRS
 * cable, a path measured at about 1 ohm on 2026-09-04: 0.6 V lost at 0.6 A for
 * the pair, leaving the right PMIC 4.28 V at 490 mA, below the ~4.3 V it needs
 * before it drops to trickle. These caps keep the right's total draw near the
 * 0.5 A the cable can carry. The left, wired straight to VBUS, is not capped. */
#define ZSA_RIGHT_VBUS_LIMIT_MAX_MA 500u
#define ZSA_RIGHT_CHARGE_MAX_MA     300u

/* Which half an answer is for. The two are no longer interchangeable. */
enum zsa_charge_half {
    ZSA_CHARGE_HALF_LEFT  = 0,
    ZSA_CHARGE_HALF_RIGHT = 1,
};

/* The whole policy: what a source advertising budget_ma entitles ONE HALF to.
 * This is the left's answer, and the generic one; the right's is this capped
 * by zsa_charge_limits_for_half() below.
 *
 * Derived rather than tabulated, so a source class nobody has written down yet
 * still gets a sensible answer — the same reason EVT_PWR_BUDGET carries mA.
 * The input limit is the largest step the PMIC can program at or below this
 * half's share, never under the floor and never over what the register can
 * express. The charge current is whatever is left of that after the half has
 * fed itself, inside the cell's own limits.
 *
 * Two profiles come out of it in practice, and they now differ per half. A
 * default 500 mA-class source, or one that has not identified itself, lands on
 * the floor: 300 mA in, 150 mA into the cell, on both halves. A source
 * advertising 1.5 A or more gets the left 700 mA in and 550 mA into the cell,
 * and the right 500 mA in and 300 mA into the cell.
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

/* The same policy, narrowed to the half it is for: the derivation above, then
 * the right half's cable ceilings. Never below the floor or the PMIC's minimum
 * charge current, so a capped answer is still one the hardware can be set to.
 *
 * Pure and total like the derivation it wraps, and exhaustively tested
 * alongside it. */
static inline struct zsa_charge_limits zsa_charge_limits_for_half(uint16_t budget_ma,
                                                                 enum zsa_charge_half half)
{
    struct zsa_charge_limits out = zsa_charge_limits_for_budget(budget_ma);

    if (half != ZSA_CHARGE_HALF_RIGHT) {
        return out;
    }

    if (out.vbus_limit_ma > ZSA_RIGHT_VBUS_LIMIT_MAX_MA) {
        out.vbus_limit_ma = ZSA_RIGHT_VBUS_LIMIT_MAX_MA;
    }
    if (out.vbus_limit_ma < ZSA_VBUS_LIMIT_FLOOR_MA) {
        out.vbus_limit_ma = ZSA_VBUS_LIMIT_FLOOR_MA;
    }
    if (out.charge_ma > ZSA_RIGHT_CHARGE_MAX_MA) {
        out.charge_ma = ZSA_RIGHT_CHARGE_MAX_MA;
    }
    if (out.charge_ma < ZSA_CHARGE_CURRENT_MIN_MA) {
        out.charge_ma = ZSA_CHARGE_CURRENT_MIN_MA;
    }
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

/* ZSA_MSG_CMD_RIGHT_TELEMETRY — the payload is a struct zsa_right_telemetry
 * (zsa_right_telemetry.h; not included here, that header includes this one).
 * How long the left trusts the last relayed copy: the STM32 sends one per
 * successful wired read, so silence means the cable read stopped. */
#define ZSA_RIGHT_TLM_RELAY_STALE_MS 3000u

/* ZSA_MSG_CMD_TRRS_STATE (1 byte) */
struct __attribute__((packed)) zsa_trrs_state {
    uint8_t plugged; /* 0 = cable unplugged, 1 = plugged */
};
/* Refresh cadence for the level while it is unchanged. */
#define ZSA_TRRS_STATE_INTERVAL_MS 1000

/* ---- Diagnostics counters ---- */

/* ZSA_MSG_CMD_SCAN_DIAG (16 bytes), STM32 -> left, once a second.
 * Window fields cover the last ZSA_SCAN_DIAG_INTERVAL_MS; cumulative fields
 * wrap at 65535 and are compared as differences. A gap is the time between
 * two consecutive keyboard_task() passes. */
#define ZSA_SCAN_DIAG_VERSION     1u
#define ZSA_SCAN_DIAG_INTERVAL_MS 1000u

#define ZSA_SCAN_DIAG_FLAG_WIRED_RIGHT 0x01u /* STM32 owns the right bus (cable) */
#define ZSA_SCAN_DIAG_FLAG_USB_HOST    0x02u /* reports go to a USB host */

struct __attribute__((packed)) zsa_scan_diag {
    uint8_t  version;          /* ZSA_SCAN_DIAG_VERSION */
    uint8_t  len;              /* bytes after this one: 14 */
    uint16_t scans;            /* passes in the window */
    uint16_t max_gap_ms;       /* longest gap in the window */
    uint16_t gaps_over_5ms;    /* gaps > 5 ms in the window */
    uint16_t gaps_over_20ms;   /* gaps > 20 ms in the window */
    uint16_t right_read_errors;/* cumulative right-half expander read failures */
    uint16_t kbd_reports;      /* cumulative keyboard reports sent to the left nRF */
    uint16_t flags;            /* ZSA_SCAN_DIAG_FLAG_* */
};

#define ZSA_SCAN_DIAG_SIZE 16u
#define ZSA_SCAN_DIAG_LEN  14u

_Static_assert(sizeof(struct zsa_scan_diag) == ZSA_SCAN_DIAG_SIZE,
               "zsa_scan_diag is a wire layout");

/* Left nRF counters, served in the BLE telemetry read after zsa_scan_diag.
 * Not carried over UART. All cumulative except the connection parameters and
 * the keyboard latency, which are current values. */
#define ZSA_NRF_DIAG_VERSION 1u

struct __attribute__((packed)) zsa_nrf_diag {
    uint8_t  version;          /* ZSA_NRF_DIAG_VERSION */
    uint8_t  len;              /* bytes after this one: 26 */
    uint16_t uart_frames;      /* frames parsed from the STM32 */
    uint16_t uart_bad_frames;  /* parser resyncs: CRC, escape, length */
    uint16_t uart_rx_errors;   /* total: RX stopped, whatever the reason, plus
                                * no free RX buffer and ring overflow */
    uint16_t kbd_reports;      /* keyboard reports received from the STM32 */
    uint16_t kbd_dropped;      /* keyboard reports dropped, queue full */
    uint16_t ble_send_errors;  /* failed HID notifications, all report types */
    uint16_t esb_hops;         /* right-half link channel hops */
    uint16_t esb_timeouts;     /* right-half link snapshot timeouts */
    uint16_t conn_interval;    /* granted BLE interval, 1.25 ms units, 0 = no host */
    uint16_t conn_latency;     /* granted BLE peripheral latency */
    uint16_t uart_rx_overruns; /* RX stopped with UART_ERROR_OVERRUN */
    uint16_t uart_rx_line_errors;/* RX stopped with framing, parity, break or
                                  * noise */
    uint16_t kbd_latency_max_us; /* longest report wait from UART arrival to
                                  * BLE send, last full second, 65535 =
                                  * capped */
};

#define ZSA_NRF_DIAG_SIZE 28u
#define ZSA_NRF_DIAG_LEN  26u

_Static_assert(sizeof(struct zsa_nrf_diag) == ZSA_NRF_DIAG_SIZE,
               "zsa_nrf_diag is a wire layout");

/* ---- Left diagnostic record (QUERY_DIAG / EVT_DIAG) ---- */

/* zsa_diag_query.record */
#define ZSA_DIAG_REC_NOW  0u /* this boot, refreshed every second */
#define ZSA_DIAG_REC_PREV 1u /* the boot before this one */

/* Record bytes per page: page n carries bytes [n * 24, n * 24 + 24). */
#define ZSA_DIAG_PAGE_DATA 24u

/* zsa_diag_page.status */
#define ZSA_DIAG_PAGE_OK     0u
#define ZSA_DIAG_PAGE_NO_REC 1u /* no such record */
#define ZSA_DIAG_PAGE_RANGE  2u /* unknown record id or page past the end */

/* ZSA_MSG_QUERY_DIAG (2 bytes) */
struct __attribute__((packed)) zsa_diag_query {
    uint8_t record; /* ZSA_DIAG_REC_* */
    uint8_t page;
};

/* ZSA_MSG_EVT_DIAG (28 bytes) */
struct __attribute__((packed)) zsa_diag_page {
    uint8_t record;    /* as queried */
    uint8_t page;      /* as queried */
    uint8_t status;    /* ZSA_DIAG_PAGE_* */
    uint8_t total_len; /* record size in bytes; 0 unless status is OK */
    uint8_t data[ZSA_DIAG_PAGE_DATA]; /* zero past the end of the record */
};

#define ZSA_DIAG_QUERY_SIZE 2u
#define ZSA_DIAG_PAGE_SIZE  28u

_Static_assert(sizeof(struct zsa_diag_query) == ZSA_DIAG_QUERY_SIZE,
               "zsa_diag_query is a wire layout");
_Static_assert(sizeof(struct zsa_diag_page) == ZSA_DIAG_PAGE_SIZE,
               "zsa_diag_page is a wire layout");

/* ---- Radio firmware (QUERY_RADIO_INFO / EVT_RADIO_INFO) ---- */

#define ZSA_RADIO_INFO_FORMAT 1u

/* zsa_radio_info.flags */
#define ZSA_RADIO_INFO_F_LEFT_VALID      0x01u /* left version and hash read */
#define ZSA_RADIO_INFO_F_LEFT_CONFIRMED  0x02u /* MCUboot has the left image confirmed */
#define ZSA_RADIO_INFO_F_RIGHT_VALID     0x04u /* right_tag is the right's current report */
#define ZSA_RADIO_INFO_F_RIGHT_CONFIRMED 0x08u /* the right reports its image confirmed */

#define ZSA_RADIO_INFO_HASH_LEN 8u

/* ZSA_MSG_EVT_RADIO_INFO (26 bytes). Hashes are the leading bytes of the
 * running image's hash TLV, as img_mgmt_read_info() reads it. */
struct __attribute__((packed)) zsa_radio_info {
    uint8_t  format;        /* ZSA_RADIO_INFO_FORMAT */
    uint8_t  flags;         /* ZSA_RADIO_INFO_F_* */
    uint8_t  left_major;    /* left MCUboot header version, slot 0 */
    uint8_t  left_minor;
    uint16_t left_revision;
    uint32_t left_build_num;
    uint8_t  left_hash[ZSA_RADIO_INFO_HASH_LEN];
    uint8_t  right_tag[ZSA_RADIO_INFO_HASH_LEN]; /* zero unless RIGHT_VALID */
};

#define ZSA_RADIO_INFO_LEN 26u

_Static_assert(sizeof(struct zsa_radio_info) == ZSA_RADIO_INFO_LEN,
               "zsa_radio_info is a wire layout");

/* ---- Host slots (QUERY_HOST_SLOTS / EVT_HOST_SLOTS) ---- */

#define ZSA_HOST_SLOTS_MAX 5u

/* zsa_host_slot_addr.flags */
#define ZSA_HOST_SLOT_F_PAIRED    0x01u /* the slot holds a host */
#define ZSA_HOST_SLOT_F_CONNECTED 0x02u /* the slot's host is connected now */

/* One host slot: the board's own address on that slot's identity. */
struct __attribute__((packed)) zsa_host_slot_addr {
    uint8_t flags;     /* ZSA_HOST_SLOT_F_* */
    uint8_t addr_type; /* 0 public, 1 random (bt_addr_le_t.type);
                        * ZSA_BLE_ADDR_TYPE_UNKNOWN when unknown */
    uint8_t addr[6];   /* bt_addr_t.val order, least significant byte first */
};

/* ZSA_MSG_EVT_HOST_SLOTS (41 bytes). Entries from slot_count on are zero. */
struct __attribute__((packed)) zsa_host_slots {
    uint8_t slot_count; /* host slots the board keeps */
    struct zsa_host_slot_addr slot[ZSA_HOST_SLOTS_MAX];
};

#define ZSA_HOST_SLOTS_LEN 41u

_Static_assert(sizeof(struct zsa_host_slot_addr) == 8u,
               "zsa_host_slot_addr is a wire layout");
_Static_assert(sizeof(struct zsa_host_slots) == ZSA_HOST_SLOTS_LEN,
               "zsa_host_slots is a wire layout");

/* zsa_left_diag.threads[] order. */
enum zsa_diag_thread_id {
    ZSA_DIAG_THREAD_MAIN = 0, /* main status loop: time since its last pass */
    ZSA_DIAG_THREAD_HID_WQ,   /* hid_wq: wait of the queued heartbeat item */
    ZSA_DIAG_THREAD_SYS_WQ,   /* system work queue: same */
    ZSA_DIAG_THREAD_UART_RX,  /* uart_stm_rx: time in the batch being dispatched */
    ZSA_DIAG_THREAD_COUNT,
};

struct __attribute__((packed)) zsa_diag_thread {
    uint32_t stall_ms;     /* current wait, 0 = none */
    uint32_t max_stall_ms; /* longest wait this boot, current one included */
};

#define ZSA_LEFT_DIAG_VERSION 1u

/* zsa_left_diag.flags */
#define ZSA_LEFT_DIAG_F_STM32_LINK  0x01u /* LINK_UP handshake done */
#define ZSA_LEFT_DIAG_F_STM32_SLEEP 0x02u /* the STM32 reported Sleep */
#define ZSA_LEFT_DIAG_F_STM32_USB   0x04u /* the STM32 reported a USB host */
#define ZSA_LEFT_DIAG_F_TRRS        0x08u /* the STM32 reported the cable plugged */
#define ZSA_LEFT_DIAG_F_BLE_CONN    0x10u /* a BLE host is connected */
#define ZSA_LEFT_DIAG_F_UART_RX_OFF 0x20u /* UART RX re-enable failed */
#define ZSA_LEFT_DIAG_F_VBUS        0x40u /* VBUS present */

/* An age that has no event to count from. */
#define ZSA_DIAG_AGE_NEVER 0xFFFFFFFFu

/* The left nRF's diagnostic record. Lives in RAM that survives a pin or soft
 * reset; at boot the stored copy becomes ZSA_DIAG_REC_PREV. Counters are this
 * boot's, uint16 and wrapping. */
struct __attribute__((packed)) zsa_left_diag {
    uint8_t  version;             /* ZSA_LEFT_DIAG_VERSION */
    uint8_t  len;                 /* bytes after this one: 88 */
    uint16_t boot_count;          /* app boots since the record was last lost */
    uint32_t uptime_s;
    uint32_t resetreas;           /* RESET.RESETREAS at boot, 0 = power-on */
    uint8_t  flags;               /* ZSA_LEFT_DIAG_F_* */
    uint8_t  ble_slot;            /* connected host slot, 0xFF = none */
    uint8_t  esb_link;            /* right link: 0 absent, 1 OK, 2 mismatch */
    uint8_t  stm32_idle_step;     /* last CMD_IDLE_STATE step */
    uint16_t conn_interval;       /* 1.25 ms units, 0 = no host */
    uint16_t conn_latency;
    uint16_t conn_timeout;        /* 10 ms units */
    uint16_t uart_frames;         /* as in zsa_nrf_diag */
    uint16_t uart_bad_frames;
    uint16_t uart_rx_errors;
    uint16_t uart_rx_overruns;
    uint16_t uart_rx_line_errors;
    uint16_t uart_ring_drops;     /* RX ring overflows */
    uint16_t uart_rx_restarts;    /* RX re-enabled after it stopped */
    uint16_t kbd_reports;
    uint16_t kbd_dropped;
    uint16_t ble_send_errors;     /* failed HID input reports */
    uint16_t notify_errors;       /* failed battery notifications */
    uint16_t esb_hops;
    uint16_t esb_timeouts;
    uint16_t hid_send_max_ms;     /* longest single HID report send call */
    uint32_t uart_rx_age_ms;      /* since the last byte from the STM32 */
    uint32_t esb_rx_age_ms;       /* since the last packet from the right half */
    struct zsa_diag_thread threads[ZSA_DIAG_THREAD_COUNT];
};

#define ZSA_LEFT_DIAG_SIZE 90u
#define ZSA_LEFT_DIAG_LEN  88u

_Static_assert(sizeof(struct zsa_left_diag) == ZSA_LEFT_DIAG_SIZE,
               "zsa_left_diag is a wire layout");
_Static_assert(ZSA_LEFT_DIAG_SIZE <= 4u * ZSA_DIAG_PAGE_DATA,
               "zsa_left_diag fits four pages");

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
    uint16_t             resyncs; /* frames abandoned: bad escape, length or CRC */
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
    p->resyncs = 0;
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
            p->resyncs++;
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
                p->resyncs++;
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
            if (byte != expected) {
                p->resyncs++;
                return 0;
            }
            return 1;
        }

        default:
            p->state = ZSA_PARSE_HUNT;
            return 0;
    }
}

#endif /* ZSA_UART_PROTO_H */
