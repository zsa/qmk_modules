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
    ZSA_MSG_CMD_PAIR         = 0x21,
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
    ZSA_MSG_CMD_VBUS_ILIM    = 0x26,  /* 1 B: struct zsa_vbus_ilim — set the
                                       * left half's VBUS input current limit.
                                       * The STM32 is the charging policy
                                       * point; the left nRF only detects CC
                                       * capability (EVT_PWR_BUDGET) and
                                       * executes this command. */

    /* nRF -> STM32: DFU commands (0x30-0x3F) */
    ZSA_MSG_DFU_ERASE        = 0x30,  /* 4 B: addr LE32 */
    ZSA_MSG_DFU_SETADDR      = 0x31,  /* 4 B: addr LE32 */
    ZSA_MSG_DFU_DATA         = 0x32,  /* <=64 B: firmware chunk */
    ZSA_MSG_DFU_MANIFEST     = 0x33,  /* 0 B: finalize + reset */
    ZSA_MSG_DFU_ABORT        = 0x34,  /* 0 B: abort transfer */
    ZSA_MSG_DFU_MASS_ERASE   = 0x35,  /* 4 B: end addr LE32 (0 = full app region) */
    ZSA_MSG_DFU_QUERY        = 0x36,  /* 0 B: nRF asks if STM32 is in DFU mode */

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
    ZSA_PAIR_ACTIVE = 1,
    ZSA_PAIR_DONE   = 2,
    ZSA_PAIR_FAILED = 3,
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
 * only distinguishes default vs high class, so it sends DEFAULT or HIGH. */
#define ZSA_PWR_BUDGET_DEFAULT_MA  500u
#define ZSA_PWR_BUDGET_HIGH_MA    1500u
struct __attribute__((packed)) zsa_pwr_budget {
    uint16_t budget_ma;
};

/* ZSA_MSG_EVT_VBUS_STATE (1 byte) */
struct __attribute__((packed)) zsa_vbus_state { uint8_t present; };

/* ZSA_MSG_CMD_VBUS_ILIM (1 byte) — level, not raw mA: the actual current
 * values live in each half's devicetree (current-microamp). Lowering happens
 * via the nPM1300's own auto-revert to the startup limit on VBUS removal;
 * DEFAULT exists for future policy and is a no-op today. */
#define ZSA_VBUS_ILIM_DEFAULT 0
#define ZSA_VBUS_ILIM_HIGH    1
struct __attribute__((packed)) zsa_vbus_ilim {
    uint8_t level;
};

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
