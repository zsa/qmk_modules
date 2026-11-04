// Copyright 2024 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once
/*
The Oryx Webhid protocol

Each HID packet is a series of bytes.  The first byte is the packet type is the command. The rest of the bytes are the
params.

Before sending a packet, the host needs to be paired or should request a pairing code.

The pairing code is a sequence of key positions derived from Oryx's firmware version code stored in the FIRMWARE_VERSION
define.

Once the host has paired, it can freely use the commands define in the Oryx_Command_Code enum for which the board will
always respond with a Oryx_Event_Code or a Oryx_Error_Code.
*/

#include "quantum.h"
#include "raw_hid.h"

#ifndef RAW_EPSIZE
#    define RAW_EPSIZE 32
#endif

#define ORYX_PROTOCOL_VERSION 0x05
#define ORYX_STOP_BIT -2

enum Oryx_Command_Code {
    ORYX_CMD_GET_FW_VERSION,
    ORYX_CMD_PAIRING_INIT,
    ORYX_CMD_PAIRING_VALIDATE,
    ORYX_CMD_DISCONNECT,
    ORYX_SET_LAYER,
    ORYX_RGB_CONTROL,
    ORYX_SET_RGB_LED,
    ORYX_SET_STATUS_LED,
    ORYX_UPDATE_BRIGHTNESS,
    ORYX_SET_RGB_LED_ALL,
    ORYX_STATUS_LED_CONTROL,
    ORYX_SET_AUTOMOUSE,
    ORYX_GET_AUTOMOUSE,
    /* Ask the board about itself: battery per half, what nav this build
     * supports, what nav is actually attached. Answered with
     * ORYX_EVT_TELEMETRY carrying an oryx_telemetry_t. Unpaired-safe, like
     * the version queries: it reads state, it changes nothing. */
    ORYX_CMD_GET_TELEMETRY,
    /* Ask what each cell is doing: voltage, current, temperature, the fuel
     * gauge's time estimates and the charger's status and fault registers.
     * Answered with ORYX_EVT_CHARGE_TELEMETRY carrying an
     * oryx_charge_telemetry_t. Unpaired-safe, like the other queries. */
    ORYX_CMD_GET_CHARGE_TELEMETRY,
    /* Ask what each charger is being fed: VBUS and VSYS, the nPM1300's
     * input-status bits and whether charging is enabled. Answered with
     * ORYX_EVT_PMIC_RAILS carrying an oryx_pmic_rails_t. Unpaired-safe, like
     * the other queries. */
    ORYX_CMD_GET_PMIC_RAILS,
    /* Ask how regular the main loop is: passes in the last window, the worst
     * gap between two of them, failed right-half expander reads and keyboard
     * reports handed to the wireless half. Answered with ORYX_EVT_SCAN_DIAG
     * carrying an oryx_scan_diag_t. Unpaired-safe, like the other queries. */
    ORYX_CMD_GET_SCAN_DIAG,
    /* Ask for one page of a field diagnostic record (oryx_diag.h): the
     * keyboard MCU's or the left nRF's, this boot's or the previous one.
     * Answered with ORYX_EVT_DIAG carrying an oryx_diag_t. Unpaired-safe,
     * like the other queries. */
    ORYX_CMD_GET_DIAG,
#if defined(NAVIGATOR_TRACKPAD_DEBUG)
    /* Test builds only. Trackpad power-measurement action, param[0]: 0 status,
     * 1 force sleep, 2 cancel force sleep, 3 pause pad access, 4 resume pad
     * access, 5 short timers, 6 factory timers, 7 hold left boost claim on,
     * 8 left boost claim normal, 9 hold left boost claim off, 10 start a
     * power cycle timing (param[1] off time in 10 ms units, 0 = 1 s),
     * 11 power cycle timing result. Answered with ORYX_EVT_DEBUG_PAD_POWER.
     * Unpaired-safe. */
    ORYX_CMD_DEBUG_PAD_POWER = 0xF0,
#endif
    ORYX_GET_PROTOCOL_VERSION = 0xFE,
};

enum Oryx_Event_Code {
    ORYX_EVT_GET_FW_VERSION,
    ORYX_EVT_PAIRING_INPUT,
    ORYX_EVT_PAIRING_KEY_INPUT,
    ORYX_EVT_PAIRING_FAILED,
    ORYX_EVT_PAIRING_SUCCESS,
    ORYX_EVT_LAYER,
    ORYX_EVT_KEYDOWN,
    ORYX_EVT_KEYUP,
    ORYX_EVT_RGB_CONTROL,
    ORYX_EVT_TOGGLE_SMART_LAYER,
    ORYX_EVT_TRIGGER_SMART_LAYER,
    ORYX_EVT_STATUS_LED_CONTROL,
    ORYX_EVT_AUTOMOUSE,
    /* Answer to ORYX_CMD_GET_TELEMETRY: an oryx_telemetry_t in bytes 1..31.
     * Binary, so it carries no ORYX_STOP_BIT — a 0xFE inside it is data. */
    ORYX_EVT_TELEMETRY,
    /* Answer to ORYX_CMD_GET_CHARGE_TELEMETRY: an oryx_charge_telemetry_t in
     * bytes 1..31. Binary, so it carries no ORYX_STOP_BIT — a 0xFE inside it
     * is data. */
    ORYX_EVT_CHARGE_TELEMETRY,
    /* Answer to ORYX_CMD_GET_PMIC_RAILS: an oryx_pmic_rails_t in bytes 1..31.
     * Binary, so it carries no ORYX_STOP_BIT — a 0xFE inside it is data. */
    ORYX_EVT_PMIC_RAILS,
    /* Answer to ORYX_CMD_GET_SCAN_DIAG: an oryx_scan_diag_t in bytes 1..31.
     * Binary, so it carries no ORYX_STOP_BIT — a 0xFE inside it is data. */
    ORYX_EVT_SCAN_DIAG,
    /* Answer to ORYX_CMD_GET_DIAG: an oryx_diag_t in bytes 1..31. Binary, so
     * it carries no ORYX_STOP_BIT — a 0xFE inside it is data. */
    ORYX_EVT_DIAG,
#if defined(NAVIGATOR_TRACKPAD_DEBUG)
    /* Answer to ORYX_CMD_DEBUG_PAD_POWER: byte 1 action, byte 2 write status
     * (0 OK, CGEN6_* error bits, 0x80 pad not up on its real address, 0x81
     * unknown action), byte 3 paused, byte 4 pad up on its real address,
     * byte 5 left boost override (0 none, 1 held on, 2 held off), byte 6
     * last boost claim sent to the nRF. Action 10 status: 0 started, 0x82
     * busy or paused, 0x83 no nRF link. Action 11: bytes 2..23 as
     * documented at navigator_trackpad_debug_timing_result(). */
    ORYX_EVT_DEBUG_PAD_POWER = 0xF0,
#endif
    ORYX_EVT_GET_PROTOCOL_VERSION = 0XFE,
    ORYX_EVT_ERROR                = 0xFF,
};

enum Oryx_Error_Code {
    ORYX_ERR_PAIRING_INIT_FAILED,
    ORYX_ERR_PAIRING_INPUT_FAILED,
    ORYX_ERR_PAIRING_KEY_INPUT_FAILED,
    ORYX_ERR_PAIRING_FAILED,
    ORYX_ERR_RGB_MATRIX_NOT_ENABLED,
    ORYX_ERR_STATUS_LED_OUT_OF_RANGE,
    ORYX_ERR_UNKNOWN_COMMAND = 0xFF,
};

extern bool oryx_state_live_training_enabled;

typedef struct {
    bool paired;
    bool rgb_control;
    bool status_led_control;
} rawhid_state_t;

extern rawhid_state_t rawhid_state;

typedef enum {
    ORYX_TRANSPORT_USB,
    ORYX_TRANSPORT_BLE,
} oryx_transport_t;

/* Sends one RAW_EPSIZE report to the BLE host; false if not sent. Weak in
 * oryx.c, overridden by the wireless module. */
bool oryx_ble_send(const uint8_t *data);
/* Feeds one RAW_EPSIZE report received from the BLE host. */
void oryx_receive_ble(const uint8_t *data, uint8_t length);
/* Ends a BLE session when the BLE link drops. */
void oryx_ble_link_down(void);

void oryx_error(uint8_t code);
void pairing_failed_event(void);
void pairing_succesful_event(void);
void toggle_smart_layer(void);
void trigger_smart_layer(void);
void set_webhid_effect(void);

void oryx_layer_event(void);

#if defined(RGB_MATRIX_ENABLE) && !defined(KEYBOARD_ergodox_ez_glow)
extern RGB webhid_leds[RGB_MATRIX_LED_COUNT];
#endif
