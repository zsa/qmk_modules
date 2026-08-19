// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

// Supervisor for the devices hanging off the shared I2C bus. It owns presence,
// configuration, verification and recovery; drivers only touch a device while
// busdev_is_up() is true.

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "i2c_master.h"

#define BUSDEV_MAX_DEVICES 6
#define BUSDEV_NONE 0xFF

// Probe cadence for an absent device, and the slower cadence used after
// BUSDEV_PROBE_BACKOFF_AFTER consecutive failed probes.
#ifndef BUSDEV_PROBE_INTERVAL_MS
#    define BUSDEV_PROBE_INTERVAL_MS 250
#endif
#ifndef BUSDEV_PROBE_BACKOFF_MS
#    define BUSDEV_PROBE_BACKOFF_MS 1000
#endif
#ifndef BUSDEV_PROBE_BACKOFF_AFTER
#    define BUSDEV_PROBE_BACKOFF_AFTER 4
#endif

// Reported suspects, without a scheduled verify passing in between, that take
// the device down instead of reconfiguring it again.
#ifndef BUSDEV_MAX_SUSPECT_STREAK
#    define BUSDEV_MAX_SUSPECT_STREAK 3
#endif

// Consecutive I2C timeouts, across all devices, that make the bus suspect.
#ifndef BUSDEV_BUS_TIMEOUT_STREAK
#    define BUSDEV_BUS_TIMEOUT_STREAK 2
#endif
// Minimum spacing between two bus-clear attempts.
#ifndef BUSDEV_BUS_CLEAR_INTERVAL_MS
#    define BUSDEV_BUS_CLEAR_INTERVAL_MS 500
#endif

typedef uint8_t busdev_id_t;

typedef enum {
    BUSDEV_ABSENT,
    BUSDEV_SETTLING,
    BUSDEV_CONFIGURING,
    BUSDEV_VERIFYING,
    BUSDEV_UP,
} busdev_state_t;

typedef enum {
    BUSDEV_STEP_MORE,
    BUSDEV_STEP_DONE,
    BUSDEV_STEP_FAIL,
} busdev_step_t;

enum {
    BUSDEV_REASON_NONE,
    BUSDEV_REASON_PROBE_ACK,
    BUSDEV_REASON_NAK,
    BUSDEV_REASON_TIMEOUT,
    BUSDEV_REASON_BUS_DEAD,
    BUSDEV_REASON_INIT_FAILED,
    BUSDEV_REASON_VERIFY_MISMATCH,
    BUSDEV_REASON_VERIFY_OK,
    BUSDEV_REASON_IMPOSSIBLE_KEY,
    BUSDEV_REASON_BAD_ID,
    BUSDEV_REASON_BAD_REPORT_ID,
    BUSDEV_REASON_PEER,             // a device sharing the power rail was reset
    BUSDEV_REASON_TRANSPORT_SWITCH, // USB <-> BLE; the device may have moved
    BUSDEV_REASON_OWNERSHIP,        // the half of the bus this device sits on changed hands
    BUSDEV_REASON_SLEEP,
    BUSDEV_REASON_USER,
};

typedef struct {
    const char *name;
    uint8_t     addr;               // 8-bit QMK address (7-bit << 1)
    uint8_t     addr_alt;           // second address to probe, 0 = none
    uint16_t    settle_ms;          // wait after the first ACK before configuring
    uint16_t    verify_interval_ms; // 0 = never verify while up
    uint16_t    probe_timeout_ms;   // used by the default probe
    uint8_t     max_err_streak;     // consecutive report_error() before ABSENT

    // The address to probe now, overriding the addr/addr_alt alternation. For
    // devices whose address is decided elsewhere, e.g. by bus ownership.
    uint8_t (*select_addr)(void);
    // NULL => default: i2c_ping_address(addr, probe_timeout_ms).
    i2c_status_t (*probe)(uint8_t addr);
    // Called with step = 0, 1, 2, ... until DONE/FAIL. *delay_ms (default 0)
    // requests a wait before the next step. Each call does a bounded amount of
    // I2C and never blocks on wait_ms.
    busdev_step_t (*init_step)(uint8_t addr, uint8_t step, uint16_t *delay_ms);
    // NULL => always true. Reads configuration back and compares.
    bool (*verify)(uint8_t addr);
    // Optional; called after every transition.
    void (*on_state)(busdev_id_t id, busdev_state_t old_state, busdev_state_t new_state, uint8_t reason);
} busdev_desc_t;

// Registers a device. desc must be static. Returns BUSDEV_NONE when the
// registry is full; registering the same desc twice returns the existing id.
busdev_id_t busdev_register(const busdev_desc_t *desc);

// All id-taking calls accept BUSDEV_NONE and do nothing.
bool busdev_is_up(busdev_id_t id);
// The address that last ACKed, or desc->addr if none ever did. Also the
// address handed to init_step() and verify().
uint8_t        busdev_addr(busdev_id_t id);
busdev_state_t busdev_state(busdev_id_t id);
// Descriptor name, "?" for BUSDEV_NONE. For logging.
const char *busdev_name(busdev_id_t id);

// Bus layer state: true once BUSDEV_BUS_TIMEOUT_STREAK transactions in a row
// timed out, cleared by the next successful one.
bool    busdev_bus_suspect(void);
uint8_t busdev_bus_timeout_streak(void);

void busdev_report_error(busdev_id_t id, i2c_status_t status);
// Data plausibility failure. Ignored while the device is already being
// reconfigured; takes it down after BUSDEV_MAX_SUSPECT_STREAK reports.
void busdev_report_suspect(busdev_id_t id, uint8_t reason);
void busdev_force_down(busdev_id_t id, uint8_t reason);
void busdev_force_down_all(uint8_t reason);

// One bus action per call. Driven by housekeeping_task_busdev().
void busdev_task(void);

// Weak, empty by default (dprintf on CONSOLE builds).
void busdev_on_transition(busdev_id_t id, busdev_state_t old_state, busdev_state_t new_state, uint8_t reason);
