/*
 * zsa_uart_proto.h -- ZSA keyboard UART protocol definition
 *
 * Shared between STM32 (QMK) and nRF54 (Zephyr).
 * No dependencies on either framework.
 *
 * Wire format:
 *   SYNC(0xAA) | TYPE(1B) | LEN(1B) | PAYLOAD(0-64B) | CRC8(1B)
 *
 * CRC-8/CCITT (polynomial 0x07) computed over TYPE + LEN + PAYLOAD.
 */

#ifndef ZSA_UART_PROTO_H
#define ZSA_UART_PROTO_H

#include <stdint.h>

/* ---- Wire format constants ---- */

#define ZSA_SYNC_BYTE      0xAA
#define ZSA_MAX_PAYLOAD    64
#define ZSA_HEADER_SIZE    3   /* SYNC + TYPE + LEN */
#define ZSA_FRAME_OVERHEAD 4   /* SYNC + TYPE + LEN + CRC */

/* ---- Message types ---- */

enum zsa_msg_type {
    /* STM32 -> nRF: HID reports (0x01-0x0F) */
    ZSA_MSG_HID_KEYBOARD     = 0x01,
    ZSA_MSG_HID_MOUSE        = 0x02,
    ZSA_MSG_HID_SYSTEM       = 0x03,

    /* STM32 -> nRF: queries (0x10-0x1F) */
    ZSA_MSG_QUERY_BLE_STATUS = 0x10,
    ZSA_MSG_QUERY_BATTERY    = 0x11,
    ZSA_MSG_QUERY_PAIR_STATE = 0x12,

    /* STM32 -> nRF: commands (0x20-0x2F) */
    ZSA_MSG_CMD_HIBERNATE    = 0x20,
    ZSA_MSG_CMD_PAIR         = 0x21,
    ZSA_MSG_CMD_IDLE_STATE   = 0x22,

    /* nRF -> STM32: events / query responses (0x80-0x8F) */
    ZSA_MSG_EVT_BLE_STATUS   = 0x80,
    ZSA_MSG_EVT_BATTERY      = 0x81,
    ZSA_MSG_EVT_PAIR_STATE   = 0x82,
    ZSA_MSG_EVT_LED_STATE    = 0x83,
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

/* ZSA_MSG_CMD_IDLE_STATE (1 byte) */
struct __attribute__((packed)) zsa_idle_state {
    uint8_t step; /* 0=active, 1=dim, 2=leds_off, 3=slow_scan, 4=deep */
};

/* ---- Parser ---- */

enum zsa_parse_state {
    ZSA_PARSE_HUNT,
    ZSA_PARSE_TYPE,
    ZSA_PARSE_LEN,
    ZSA_PARSE_PAYLOAD,
    ZSA_PARSE_CRC,
};

struct zsa_frame {
    uint8_t type;
    uint8_t len;
    uint8_t payload[ZSA_MAX_PAYLOAD];
};

struct zsa_parser {
    enum zsa_parse_state state;
    struct zsa_frame     frame;
    uint8_t              idx;
};

/* ---- CRC-8/CCITT (polynomial 0x07) ---- */

static inline uint8_t zsa_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* ---- Frame encoding ---- */

/*
 * Encode a frame into buf. Returns total frame size.
 * buf must be at least len + ZSA_FRAME_OVERHEAD bytes.
 */
static inline uint8_t zsa_frame_encode(uint8_t *buf, uint8_t type, const uint8_t *payload, uint8_t len) {
    buf[0] = ZSA_SYNC_BYTE;
    buf[1] = type;
    buf[2] = len;
    for (uint8_t i = 0; i < len; i++) {
        buf[3 + i] = payload[i];
    }
    /* CRC over TYPE + LEN + PAYLOAD */
    buf[3 + len] = zsa_crc8(&buf[1], (uint8_t)(2 + len));
    return (uint8_t)(ZSA_FRAME_OVERHEAD + len);
}

/* ---- Frame parsing ---- */

static inline void zsa_parser_init(struct zsa_parser *p) {
    p->state = ZSA_PARSE_HUNT;
}

/*
 * Feed one byte to the parser.
 * Returns 1 when a complete valid frame is ready in p->frame, 0 otherwise.
 */
static inline int zsa_parser_feed(struct zsa_parser *p, uint8_t byte) {
    switch (p->state) {
        case ZSA_PARSE_HUNT:
            if (byte == ZSA_SYNC_BYTE) {
                p->state = ZSA_PARSE_TYPE;
            }
            return 0;

        case ZSA_PARSE_TYPE:
            p->frame.type = byte;
            p->state      = ZSA_PARSE_LEN;
            return 0;

        case ZSA_PARSE_LEN:
            p->frame.len = byte;
            if (byte > ZSA_MAX_PAYLOAD) {
                p->state = ZSA_PARSE_HUNT;
                return 0;
            }
            if (byte == 0) {
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
            /* Compute CRC over TYPE + LEN + PAYLOAD */
            uint8_t tmp[2 + ZSA_MAX_PAYLOAD];
            tmp[0] = p->frame.type;
            tmp[1] = p->frame.len;
            for (uint8_t i = 0; i < p->frame.len; i++) {
                tmp[2 + i] = p->frame.payload[i];
            }
            uint8_t expected = zsa_crc8(tmp, (uint8_t)(2 + p->frame.len));

            p->state = ZSA_PARSE_HUNT;
            return (byte == expected) ? 1 : 0;
        }
    }

    p->state = ZSA_PARSE_HUNT;
    return 0;
}

#endif /* ZSA_UART_PROTO_H */
