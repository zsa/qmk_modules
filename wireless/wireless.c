/*
 * ZSA Wireless module — implements bluetooth.h weak functions
 * to route HID reports over UART to the nRF54 wireless chip.
 *
 * Runtime switching between USB and wireless is handled by QMK's
 * CONNECTION_ENABLE subsystem (connection.c / host.c).
 */

#include <stddef.h>
#include "wireless.h"
#include "bluetooth.h"
#include "report.h"
#include "uart.h"
#include "usb_util.h"
#include "keyboard.h"
#include "timer.h"
#include "wait.h"
#include "connection.h"
#ifdef RGB_MATRIX_ENABLE
#    include "rgb_matrix.h"
#endif

/* ---- Cached nRF state ---- */

static bool    nrf_ble_connected = true;
static uint8_t nrf_ble_device_idx;
static uint8_t nrf_battery_soc;
static uint8_t nrf_battery_flags;
static uint8_t nrf_led_state;

/* ---- Parser instance ---- */

static struct zsa_parser rx_parser;

/* ---- Helpers ---- */

static void send_frame(uint8_t type, const uint8_t *payload, uint8_t len) {
    uint8_t buf[1 + (ZSA_FRAME_OVERHEAD - 1 + ZSA_MAX_PAYLOAD) * 2];
    uint8_t frame_len = zsa_frame_encode(buf, type, payload, len);
    uart_transmit(buf, frame_len);
}

static void send_command(uint8_t type) {
    send_frame(type, NULL, 0);
}

static void handle_rx_frame(const struct zsa_frame *frame) {
    switch (frame->type) {
        case ZSA_MSG_EVT_BLE_STATUS:
            if (frame->len >= sizeof(struct zsa_ble_status)) {
                const struct zsa_ble_status *s = (const struct zsa_ble_status *)frame->payload;
                nrf_ble_connected  = s->connected != 0;
                nrf_ble_device_idx = s->device_idx;
            }
            break;

        case ZSA_MSG_EVT_BATTERY:
            if (frame->len >= sizeof(struct zsa_battery_status)) {
                const struct zsa_battery_status *b = (const struct zsa_battery_status *)frame->payload;
                nrf_battery_soc   = b->soc;
                nrf_battery_flags = b->flags;
            }
            break;

        case ZSA_MSG_EVT_PAIR_STATE:
            /* TODO: expose pairing state when needed */
            break;

        case ZSA_MSG_EVT_LED_STATE:
            if (frame->len >= sizeof(struct zsa_led_state)) {
                const struct zsa_led_state *l = (const struct zsa_led_state *)frame->payload;
                nrf_led_state = l->leds;
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

uint8_t zsa_wireless_battery_soc(void) {
    return nrf_battery_soc;
}

uint8_t zsa_wireless_battery_flags(void) {
    return nrf_battery_flags;
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

/* ---- Idle power management ---- */

static enum zsa_idle_step idle_step    = ZSA_IDLE_ACTIVE;
static uint8_t            saved_rgb_val = 255;

enum zsa_idle_step zsa_wireless_idle_step(void) {
    return idle_step;
}

static void send_idle_state(enum zsa_idle_step step) {
    struct zsa_idle_state payload = { .step = (uint8_t)step };
    send_frame(ZSA_MSG_CMD_IDLE_STATE, (const uint8_t *)&payload, sizeof(payload));
}

static void idle_power_enter_step(enum zsa_idle_step step) {
#ifdef RGB_MATRIX_ENABLE
    if (step >= ZSA_IDLE_DIM && idle_step < ZSA_IDLE_DIM) {
        saved_rgb_val = rgb_matrix_get_val();
        rgb_matrix_sethsv_noeeprom(rgb_matrix_get_hue(), rgb_matrix_get_sat(), ZSA_IDLE_DIM_BRIGHTNESS);
    }
    if (step >= ZSA_IDLE_LEDS_OFF && idle_step < ZSA_IDLE_LEDS_OFF) {
        rgb_matrix_disable_noeeprom();
    }
#endif
    idle_step = step;
    send_idle_state(step);
}

static void idle_power_wake(void) {
#ifdef RGB_MATRIX_ENABLE
    if (idle_step >= ZSA_IDLE_LEDS_OFF) {
        rgb_matrix_enable_noeeprom();
    }
    if (idle_step >= ZSA_IDLE_DIM) {
        rgb_matrix_sethsv_noeeprom(rgb_matrix_get_hue(), rgb_matrix_get_sat(), saved_rgb_val);
    }
#endif
    idle_step = ZSA_IDLE_ACTIVE;
    send_idle_state(ZSA_IDLE_ACTIVE);
}

static void idle_power_task(void) {
    static bool     was_bluetooth  = false;
    static uint32_t bt_switch_time = 0;

    /* Only manage idle power when running wirelessly */
    if (connection_get_host() != CONNECTION_HOST_BLUETOOTH) {
        was_bluetooth = false;
        if (idle_step != ZSA_IDLE_ACTIVE) {
            idle_power_wake();
        }
        return;
    }

    /* Just switched to bluetooth — reset idle baseline */
    if (!was_bluetooth) {
        was_bluetooth  = true;
        bt_switch_time = timer_read32();
        if (idle_step != ZSA_IDLE_ACTIVE) {
            idle_power_wake();
        }
        return;
    }

    /* Use the later of last input activity or bluetooth switch time */
    uint32_t since_input  = last_input_activity_elapsed();
    uint32_t since_switch = timer_elapsed32(bt_switch_time);
    uint32_t idle_ms      = (since_input < since_switch) ? since_input : since_switch;

    enum zsa_idle_step target = ZSA_IDLE_ACTIVE;
    if      (idle_ms > ZSA_IDLE_DEEP_MS)      target = ZSA_IDLE_DEEP;
    else if (idle_ms > ZSA_IDLE_SLOW_SCAN_MS) target = ZSA_IDLE_SLOW_SCAN;
    else if (idle_ms > ZSA_IDLE_LEDS_OFF_MS)  target = ZSA_IDLE_LEDS_OFF;
    else if (idle_ms > ZSA_IDLE_DIM_MS)        target = ZSA_IDLE_DIM;

    if (target != idle_step) {
        if (target > idle_step) {
            idle_power_enter_step(target);
        } else {
            idle_power_wake();
        }
    }

    if (idle_step >= ZSA_IDLE_SLOW_SCAN) {
        wait_ms(ZSA_IDLE_SCAN_DELAY_MS);
    }
}

/* ---- bluetooth.h weak function implementations ---- */

void bluetooth_init(void) {
    uart_init(ZSA_WIRELESS_BAUD_RATE);
    zsa_parser_init(&rx_parser);

    /* Query initial BLE state so we don't start in an unknown state */
    zsa_wireless_query_ble_status();
}

void bluetooth_task(void) {
    while (uart_available()) {
        uint8_t byte = uart_read();
        if (zsa_parser_feed(&rx_parser, byte)) {
            handle_rx_frame(&rx_parser.frame);
        }
    }

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
    send_frame(ZSA_MSG_HID_KEYBOARD, payload, sizeof(payload));
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

    send_frame(ZSA_MSG_HID_MOUSE, start, len);
}

void bluetooth_send_consumer(uint16_t usage) {
    uint8_t payload[3];
    payload[0] = REPORT_ID_CONSUMER;
    payload[1] = (uint8_t)(usage & 0xFF);
    payload[2] = (uint8_t)(usage >> 8);
    send_frame(ZSA_MSG_HID_SYSTEM, payload, sizeof(payload));
}

void bluetooth_send_system(uint16_t usage) {
    uint8_t payload[3];
    payload[0] = REPORT_ID_SYSTEM;
    payload[1] = (uint8_t)(usage & 0xFF);
    payload[2] = (uint8_t)(usage >> 8);
    send_frame(ZSA_MSG_HID_SYSTEM, payload, sizeof(payload));
}

void bluetooth_send_raw_hid(uint8_t *data, uint8_t length) {
    /* Raw HID not supported over wireless */
    (void)data;
    (void)length;
}
