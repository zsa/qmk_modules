// Copyright 2024 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H
#include <string.h>
#include "oryx.h"
#include "oryx_telemetry.h"
#include "oryx_charge_telemetry.h"
#include "oryx_pmic_rails.h"
#include "oryx_scan_diag.h"
#include "oryx_diag.h"
#include "action_util.h"
#if defined(NAVIGATOR_TRACKPAD_DEBUG)
#    include "navigator_trackpad_common.h"
#endif

#ifdef COMMUNITY_MODULE_AUTOMOUSE_ENABLE
#    include <automouse.h>
#endif

ASSERT_COMMUNITY_MODULES_MIN_API_VERSION(1, 1, 1);

uint8_t current_layer = 0;

rawhid_state_t rawhid_state = {
    .paired             = false,
    .rgb_control        = false,
    .status_led_control = false,
};

#if defined(PROTOCOL_LUFA)
bool send_report(uint8_t endpoint, void *report, size_t size);
#    include "usb_descriptor.h"
#    define RAW_EP_NAME RAW_IN_EPNUM
#elif defined(PROTOCOL_CHIBIOS)
#    include "usb_endpoints.h"
#    define RAW_EP_NAME USB_ENDPOINT_IN_RAW
bool send_report(usb_endpoint_in_lut_t endpoint, void *report, size_t size);
#endif

#ifdef RGB_MATRIX_ENABLE
RGB webhid_leds[RGB_MATRIX_LED_COUNT];
#endif

/* Transport the paired session lives on, set by PAIRING_INIT. */
static oryx_transport_t session_transport = ORYX_TRANSPORT_USB;
/* Where events go: the arrival transport while a command is handled, the
 * session transport otherwise. */
static oryx_transport_t send_transport = ORYX_TRANSPORT_USB;

/* Weak default: no BLE transport. Overridden by the wireless module. */
__attribute__((weak)) bool oryx_ble_send(const uint8_t *data) {
    (void)data;
    return false;
}

void raw_hid_send_oryx(uint8_t *data, uint8_t length) {
    if (length != RAW_EPSIZE) {
        return;
    }
    if (send_transport == ORYX_TRANSPORT_BLE) {
        if (!oryx_ble_send(data) && session_transport == ORYX_TRANSPORT_BLE) {
            rawhid_state.paired = false;
        }
        return;
    }
    if (!send_report(RAW_EP_NAME, data, length)) {
#if !defined(PROTOCOL_LUFA)
        if (session_transport == ORYX_TRANSPORT_USB) {
            rawhid_state.paired = false;
        }
#endif
    }
}

void oryx_ble_link_down(void) {
    if (session_transport == ORYX_TRANSPORT_BLE) {
        rawhid_state.paired = false;
    }
}

void oryx_error(uint8_t code) {
    uint8_t event[RAW_EPSIZE] = {0};
    event[0] = ORYX_EVT_ERROR;
    event[1] = code;
    raw_hid_send_oryx(event, RAW_EPSIZE);
}

void oryx_layer_event(void) {
    uint8_t layer;
    uint8_t event[RAW_EPSIZE] = {0};
    layer    = get_highest_layer(layer_state | default_layer_state);
    event[0] = ORYX_EVT_LAYER;
    event[1] = layer;
    event[2] = ORYX_STOP_BIT;
    raw_hid_send_oryx(event, sizeof(event));
}

void pairing_failed_event(void) {
    rawhid_state.paired = false;
    uint8_t event[RAW_EPSIZE] = {0};
    event[0] = ORYX_EVT_PAIRING_FAILED;
    event[1] = ORYX_STOP_BIT;
    raw_hid_send_oryx(event, sizeof(event));
}

void pairing_success_event(void) {
    rawhid_state.paired = true;
    uint8_t event[RAW_EPSIZE] = {0};
    event[0] = ORYX_EVT_PAIRING_SUCCESS;
    event[1] = ORYX_STOP_BIT;
    raw_hid_send_oryx(event, sizeof(event));
    oryx_layer_event();
}

void toggle_smart_layer(void) {
    uint8_t event[RAW_EPSIZE] = {0};
    event[0] = ORYX_EVT_TOGGLE_SMART_LAYER;
    event[1] = ORYX_STOP_BIT;
    raw_hid_send_oryx(event, sizeof(event));
}

void trigger_smart_layer(void) {
    uint8_t event[RAW_EPSIZE] = {0};
    event[0] = ORYX_EVT_TRIGGER_SMART_LAYER;
    event[1] = ORYX_STOP_BIT;
    raw_hid_send_oryx(event, sizeof(event));
}

void set_webhid_effect(void) {
#if defined(RGB_MATRIX_ENABLE) && !defined(PROTOCOL_LUFA) && defined(RGB_MATRIX_CUSTOM_KB)
    rgb_matrix_mode_noeeprom(RGB_MATRIX_CUSTOM_oryx_webhid_effect);
    rawhid_state.rgb_control = true;
#endif
}

void clear_webhid_effect(void) {
#if defined(RGB_MATRIX_ENABLE) && !defined(PROTOCOL_LUFA)
    // Clear the pattern
    for (uint8_t i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
        webhid_leds[i] = (RGB){.r = 0, .g = 0, .b = 0};
    }
    rgb_matrix_reload_from_eeprom();
    rawhid_state.rgb_control = false;
#endif
}

/* Weak default: a board with nothing to report says so, rather than reporting
 * zeroes. Overridden by whoever actually knows (Voyager 2: the wireless
 * module, wireless.c). */
__attribute__((weak)) void oryx_telemetry_fill(oryx_telemetry_t *tlm) {
    oryx_telemetry_reset(tlm);
}

__attribute__((weak)) void oryx_charge_telemetry_fill(oryx_charge_telemetry_t *ctlm) {
    oryx_charge_telemetry_reset(ctlm);
}

__attribute__((weak)) void oryx_pmic_rails_fill(oryx_pmic_rails_t *rails) {
    oryx_pmic_rails_reset(rails);
}

__attribute__((weak)) void oryx_scan_diag_fill(oryx_scan_diag_t *diag) {
    oryx_scan_diag_reset(diag);
}

__attribute__((weak)) void oryx_diag_fill(oryx_diag_t *d, uint8_t record, uint8_t page, uint8_t flags) {
    (void)record;
    (void)page;
    (void)flags;
    oryx_diag_reset(d);
}

static void oryx_handle_command(uint8_t *data, uint8_t length) {
    uint8_t  command = data[0];
    uint8_t *param   = &data[1];

    switch (command) {
        case ORYX_CMD_GET_FW_VERSION: {
            uint8_t event[RAW_EPSIZE] = {0};
            uint8_t fw_version_size = sizeof(SERIAL_NUMBER);
            uint8_t stop[1];

            event[0] = ORYX_EVT_GET_FW_VERSION;
            stop[0]  = ORYX_STOP_BIT;

            memcpy(event + 1, SERIAL_NUMBER, fw_version_size);
            memcpy(event + fw_version_size, stop, 1);

            raw_hid_send_oryx(event, RAW_EPSIZE);
            break;
        }

        case ORYX_GET_PROTOCOL_VERSION: {
            uint8_t event[RAW_EPSIZE] = {0};
            event[0] = ORYX_EVT_GET_PROTOCOL_VERSION;
            event[1] = ORYX_PROTOCOL_VERSION;
            event[2] = ORYX_STOP_BIT;

            raw_hid_send_oryx(event, RAW_EPSIZE);
            break;
        }

        case ORYX_CMD_GET_TELEMETRY: {
            uint8_t event[RAW_EPSIZE] = {0};
            event[0] = ORYX_EVT_TELEMETRY;
            /* No ORYX_STOP_BIT: the payload is binary and fills the report,
             * so a terminator would be indistinguishable from a data byte.
             * oryx_telemetry_reset() writes every byte, including the
             * reserved tail. */
            oryx_telemetry_fill((oryx_telemetry_t *)&event[1]);
            raw_hid_send_oryx(event, RAW_EPSIZE);
            break;
        }

        case ORYX_CMD_GET_CHARGE_TELEMETRY: {
            uint8_t event[RAW_EPSIZE] = {0};
            event[0] = ORYX_EVT_CHARGE_TELEMETRY;
            /* No ORYX_STOP_BIT: the payload is binary and fills the report,
             * so a terminator would be indistinguishable from a data byte.
             * oryx_charge_telemetry_reset() writes every byte, including the
             * reserved tail. */
            oryx_charge_telemetry_fill((oryx_charge_telemetry_t *)&event[1]);
            raw_hid_send_oryx(event, RAW_EPSIZE);
            break;
        }

        case ORYX_CMD_GET_PMIC_RAILS: {
            uint8_t event[RAW_EPSIZE] = {0};
            event[0] = ORYX_EVT_PMIC_RAILS;
            /* No ORYX_STOP_BIT: the payload is binary and fills the report,
             * so a terminator would be indistinguishable from a data byte.
             * oryx_pmic_rails_reset() writes every byte, including the
             * reserved tail. */
            oryx_pmic_rails_fill((oryx_pmic_rails_t *)&event[1]);
            raw_hid_send_oryx(event, RAW_EPSIZE);
            break;
        }

        case ORYX_CMD_GET_SCAN_DIAG: {
            uint8_t event[RAW_EPSIZE] = {0};
            event[0] = ORYX_EVT_SCAN_DIAG;
            /* Binary payload, no ORYX_STOP_BIT; the fill writes every byte. */
            oryx_scan_diag_fill((oryx_scan_diag_t *)&event[1]);
            raw_hid_send_oryx(event, RAW_EPSIZE);
            break;
        }

        case ORYX_CMD_GET_DIAG: {
            uint8_t event[RAW_EPSIZE] = {0};
            event[0] = ORYX_EVT_DIAG;
            /* Binary payload, no ORYX_STOP_BIT; the fill writes every byte. */
            oryx_diag_fill((oryx_diag_t *)&event[1], param[0], param[1], param[2]);
            raw_hid_send_oryx(event, RAW_EPSIZE);
            break;
        }

#if defined(NAVIGATOR_TRACKPAD_DEBUG)
        case ORYX_CMD_DEBUG_PAD_POWER: {
            uint8_t event[RAW_EPSIZE] = {0};
            event[0] = ORYX_EVT_DEBUG_PAD_POWER;
            event[1] = param[0];
#    if defined(NAVIGATOR_TRACKPAD_DEBUG_TIMING)
            if (param[0] == NAVIGATOR_TRACKPAD_DEBUG_TIMING_RESULT) {
                navigator_trackpad_debug_timing_result(&event[2]);
                raw_hid_send_oryx(event, RAW_EPSIZE);
                break;
            }
#    endif
            event[2] = navigator_trackpad_debug_action(param[0], param[1]);
            event[3] = navigator_trackpad_debug_paused();
            event[4] = navigator_trackpad_debug_pad_up();
            event[5] = navigator_trackpad_debug_boost_mode();
            event[6] = navigator_trackpad_debug_boost_sent();
            raw_hid_send_oryx(event, RAW_EPSIZE);
            break;
        }
#endif

        case ORYX_CMD_PAIRING_INIT:
            session_transport = send_transport;
            pairing_success_event();

        case ORYX_CMD_PAIRING_VALIDATE:
            break; // Keeping this for backwards compatibility with older versions of Wally / Keymapp

        case ORYX_CMD_DISCONNECT:
            // BLE only: sent by the nRF when the host unsubscribes from events
            if (send_transport != ORYX_TRANSPORT_BLE) {
                oryx_error(ORYX_ERR_UNKNOWN_COMMAND);
                break;
            }
            if (session_transport == ORYX_TRANSPORT_BLE) {
                rawhid_state.paired = false;
            }
            break;

        case ORYX_SET_LAYER:
            // The first param's byte is on / off
            // The second param's byte is the layer number
            if (rawhid_state.paired == true && send_transport == session_transport) {
                if (param[0] == 0) {
                    layer_off(param[1]);
                } else {
                    layer_move(param[1]);
                }
            }
            break;

        case ORYX_RGB_CONTROL:
#if defined(RGB_MATRIX_ENABLE) && !defined(PROTOCOL_LUFA)
            if (param[0] == 0) {
                clear_webhid_effect();
            } else {
                set_webhid_effect();
            }
            uint8_t event[RAW_EPSIZE] = {0};
            event[0] = ORYX_EVT_RGB_CONTROL;
            event[1] = rawhid_state.rgb_control;
            raw_hid_send_oryx(event, RAW_EPSIZE);
#else
            oryx_error(ORYX_ERR_RGB_MATRIX_NOT_ENABLED);
#endif
            break;

        case ORYX_SET_RGB_LED:
#if defined(RGB_MATRIX_ENABLE) && !defined(PROTOCOL_LUFA)
            webhid_leds[param[0]] = (RGB){.r = param[1], .g = param[2], .b = param[3]};
            if (rawhid_state.rgb_control == false) {
                set_webhid_effect();
            }
#else
            oryx_error(ORYX_ERR_RGB_MATRIX_NOT_ENABLED);
#endif
            break;
        case ORYX_SET_RGB_LED_ALL:
#if defined(RGB_MATRIX_ENABLE) && !defined(PROTOCOL_LUFA)
            for (uint8_t i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
                webhid_leds[i] = (RGB){.r = param[0], .g = param[1], .b = param[2]};
            }
            if (rawhid_state.rgb_control == false) {
                set_webhid_effect();
            }
#else
            oryx_error(ORYX_ERR_RGB_MATRIX_NOT_ENABLED);
#endif
            break;
        case ORYX_SET_STATUS_LED:
            rawhid_state.status_led_control = true; // Eagerly take control of the status LEDs
            switch (param[0]) {
                case 0:
#ifdef STATUS_LED_1
                    STATUS_LED_1(param[1]);
#else
                    oryx_error(ORYX_ERR_STATUS_LED_OUT_OF_RANGE);
#endif
                    break;
                case 1:
#ifdef STATUS_LED_2
                    STATUS_LED_2(param[1]);
#else
                    oryx_error(ORYX_ERR_STATUS_LED_OUT_OF_RANGE);
#endif
                    break;
                case 2:
#ifdef STATUS_LED_3
                    STATUS_LED_3(param[1]);
#else
                    oryx_error(ORYX_ERR_STATUS_LED_OUT_OF_RANGE);
#endif
                    break;
                case 3:
#ifdef STATUS_LED_4
                    STATUS_LED_4(param[1]);
#else
                    oryx_error(ORYX_ERR_STATUS_LED_OUT_OF_RANGE);
#endif
                    break;
                case 4:
#ifdef STATUS_LED_5
                    STATUS_LED_5(param[1]);
#else
                    oryx_error(ORYX_ERR_STATUS_LED_OUT_OF_RANGE);
#endif
                    break;
                case 5:
#ifdef STATUS_LED_6
                    STATUS_LED_6(param[1]);
#else
                    oryx_error(ORYX_ERR_STATUS_LED_OUT_OF_RANGE);
#endif
                    break;

                default:
                    oryx_error(ORYX_ERR_STATUS_LED_OUT_OF_RANGE);
                    break;
            }
            break;
        case ORYX_UPDATE_BRIGHTNESS: {
#if defined(RGB_MATRIX_ENABLE) && !defined(PROTOCOL_LUFA)
            if (param[0]) {
                rgb_matrix_increase_val_noeeprom();
            } else {
                rgb_matrix_decrease_val_noeeprom();
            }
#else
            oryx_error(ORYX_ERR_RGB_MATRIX_NOT_ENABLED);
#endif
            break;
        }
        case ORYX_STATUS_LED_CONTROL: {
            rawhid_state.status_led_control = param[0];
            if (!param[0]) {
#ifdef STATUS_LED_1
                STATUS_LED_1(0);
#endif
#ifdef STATUS_LED_2
                STATUS_LED_2(0);
#endif
#ifdef STATUS_LED_3
                STATUS_LED_3(0);
#endif
#ifdef STATUS_LED_4
                STATUS_LED_4(0);
#endif
#ifdef STATUS_LED_5
                STATUS_LED_5(0);
#endif
#ifdef STATUS_LED_6
                STATUS_LED_6(0);
#endif
            }
            uint8_t event[RAW_EPSIZE] = {0};
            event[0] = ORYX_EVT_STATUS_LED_CONTROL;
            event[1] = rawhid_state.status_led_control;
            raw_hid_send_oryx(event, RAW_EPSIZE);
            break;
        }
        case ORYX_SET_AUTOMOUSE: // param[0] = device index, param[1] = enabled
#ifdef COMMUNITY_MODULE_AUTOMOUSE_ENABLE
            automouse_set_device_enabled(param[0], param[1]);
            // fall through to report the resulting state
#else
            oryx_error(ORYX_ERR_UNKNOWN_COMMAND);
            break;
#endif
        case ORYX_GET_AUTOMOUSE:
#ifdef COMMUNITY_MODULE_AUTOMOUSE_ENABLE
        {
            uint8_t event[RAW_EPSIZE] = {0};
            event[0] = ORYX_EVT_AUTOMOUSE;
            event[1] = automouse_get_device_enabled(AUTOMOUSE_DEVICE_TRACKBALL);
            event[2] = automouse_get_device_enabled(AUTOMOUSE_DEVICE_TRACKPAD);
            event[3] = ORYX_STOP_BIT;
            raw_hid_send_oryx(event, RAW_EPSIZE);
        }
#else
            oryx_error(ORYX_ERR_UNKNOWN_COMMAND);
#endif
            break;
        default:
            oryx_error(ORYX_ERR_UNKNOWN_COMMAND);
    }
}

static void oryx_dispatch(uint8_t *data, uint8_t length, oryx_transport_t from) {
    send_transport = from;
    oryx_handle_command(data, length);
    send_transport = session_transport;
}

void raw_hid_receive(uint8_t *data, uint8_t length) {
    oryx_dispatch(data, length, ORYX_TRANSPORT_USB);
}

void oryx_receive_ble(const uint8_t *data, uint8_t length) {
    if (length != RAW_EPSIZE) {
        return;
    }
    uint8_t report[RAW_EPSIZE];
    memcpy(report, data, RAW_EPSIZE);
    oryx_dispatch(report, RAW_EPSIZE, ORYX_TRANSPORT_BLE);
}

bool pre_process_record_oryx(uint16_t keycode, keyrecord_t *record) {
    if (!pre_process_record_oryx_kb(keycode, record)) {
        return true;
    }
    // While paired, the keyboard sends keystrokes positions to the host
    if (rawhid_state.paired == true) {
        uint8_t event[RAW_EPSIZE] = {0};
        event[0] = record->event.pressed ? ORYX_EVT_KEYDOWN : ORYX_EVT_KEYUP;
        event[1] = record->event.key.col;
        event[2] = record->event.key.row;
        event[3] = ORYX_STOP_BIT;
        raw_hid_send_oryx(event, sizeof(event));
    }
    return true;
}

layer_state_t layer_state_set_oryx(layer_state_t state) {
    state = layer_state_set_oryx_kb(state);
    if (rawhid_state.paired) {
        uint8_t layer = get_highest_layer(state | default_layer_state);
        // Some layer actions (OSL) trigger the layer state change thrice,
        // so we need to check if the layer has actually changed
        if (current_layer == layer) {
            return state;
        }
        current_layer = layer;
#if defined(PROTOCOL_LUFA)
        // Required for Atmel Boards
        wait_ms(10);
#endif
        uint8_t event[RAW_EPSIZE] = {0};
        event[0] = ORYX_EVT_LAYER;
        event[1] = current_layer;
        event[2] = ORYX_STOP_BIT;
        raw_hid_send_oryx(event, sizeof(event));
    }
    return state;
}

layer_state_t default_layer_state_set_oryx(layer_state_t state) {
    layer_state_set_oryx(state | layer_state);
    return default_layer_state_set_oryx_kb(state);
}
