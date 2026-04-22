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
#include "matrix.h"
#ifdef ZSA_BLE_LOG
#include "log_tap.h"
#endif
#ifdef RGB_MATRIX_ENABLE
#    include "rgb_matrix.h"
#endif

/* keyboard.c — not in keyboard.h but we need it to reset the idle timer */
void last_matrix_activity_trigger(void);

/* ---- Link state ---- */

static bool     link_ready = false;
static uint32_t last_link_up_ms = 0;
#define LINK_UP_INTERVAL_MS 100

/* ---- Cached nRF state ---- */

static bool    nrf_ble_connected = true;
static uint8_t nrf_ble_device_idx;
static uint8_t nrf_battery_soc;
static uint8_t nrf_battery_flags;
static uint8_t nrf_led_state;

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
    uart_transmit(buf, frame_len);
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

static void handle_rx_frame(const struct zsa_frame *frame) {
    switch (frame->type) {
        case ZSA_MSG_LINK_UP:
            link_ready = true;
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

        case ZSA_MSG_EVT_LED_RAIL:
            if (frame->len >= sizeof(struct zsa_led_rail_state)) {
                const struct zsa_led_rail_state *r = (const struct zsa_led_rail_state *)frame->payload;
#ifdef RGB_MATRIX_ENABLE
                if ((r->rails & ZSA_LED_RAIL_LEFT) && idle_step < ZSA_IDLE_LEDS_OFF) {
                    rgb_matrix_init();
                }
#endif
            }
            break;

        case ZSA_MSG_QUERY_USB_STATE:
            send_usb_state(usb_connected_state());
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

static void idle_power_task(void) {
    static bool     was_bluetooth  = false;
    static uint32_t bt_switch_time = 0;

    /* Only manage idle power when running wirelessly */
    if (usb_connected_state()) {
        if (was_bluetooth) {
            was_bluetooth = false;
            send_usb_state(true);
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
        if (idle_step != ZSA_IDLE_ACTIVE) {
            idle_power_wake();
        }
        return;
    }

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

        /* handle_rx_frame may have woken us (e.g. EVT_BLE_STATUS) */
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

/* ---- bluetooth.h weak function implementations ---- */

void bluetooth_init(void) {
    uart_init(ZSA_WIRELESS_BAUD_RATE);
    zsa_parser_init(&rx_parser);
    link_ready = false;
    last_link_up_ms = timer_read32();
#ifdef ZSA_BLE_LOG
    zsa_log_tap_init();
    zsa_log("zsa_wireless: init (ZSA_BLE_LOG=on)");
#endif
}

void bluetooth_task(void) {
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
