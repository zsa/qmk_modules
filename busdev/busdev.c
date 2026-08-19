// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#include "busdev.h"
#include "timer.h"
#include "wait.h"
#include "debug.h"

#ifdef PROTOCOL_CHIBIOS
#    include <hal.h>
#    include "gpio.h"

// Same pin configuration as platforms/chibios/drivers/i2c_master.c.
#    ifndef I2C_DRIVER
#        define I2C_DRIVER I2CD1
#    endif
#    ifndef I2C1_SCL_PIN
#        define I2C1_SCL_PIN B6
#    endif
#    ifndef I2C1_SDA_PIN
#        define I2C1_SDA_PIN B7
#    endif
#    ifndef I2C1_SCL_PAL_MODE
#        ifdef USE_GPIOV1
#            define I2C1_SCL_PAL_MODE PAL_MODE_ALTERNATE_OPENDRAIN
#        else
#            define I2C1_SCL_PAL_MODE 4
#        endif
#    endif
#    ifndef I2C1_SDA_PAL_MODE
#        ifdef USE_GPIOV1
#            define I2C1_SDA_PAL_MODE PAL_MODE_ALTERNATE_OPENDRAIN
#        else
#            define I2C1_SDA_PAL_MODE 4
#        endif
#    endif

// Half period of the recovery clock.
#    define BUSDEV_BUS_CLEAR_HALF_US 5
#    define BUSDEV_BUS_CLEAR_PULSES 9
#endif

typedef struct {
    const busdev_desc_t *desc;
    busdev_state_t       state;
    uint8_t              step;         // init_step index
    uint8_t              err_streak;   // consecutive report_error()
    uint8_t              probe_fails;  // consecutive failed probes, for backoff
    uint8_t              verify_fails; // consecutive verify mismatches
    uint8_t              suspects;     // report_suspect() since the last passing scheduled verify
    uint8_t              last_reason;
    uint32_t             deadline;
} busdev_dev_t;

static busdev_dev_t devices[BUSDEV_MAX_DEVICES];
static uint8_t      device_count;
static uint8_t      round_robin;

static uint8_t  bus_timeout_streak;
static bool     bus_suspect;
static uint32_t bus_clear_time;
static bool     bus_cleared_once;

// -- bus layer ---------------------------------------------------------------

#ifdef PROTOCOL_CHIBIOS
static bool bus_sda_low(void) {
    return palReadLine(I2C1_SDA_PIN) == PAL_LOW;
}

// Frees a slave that is holding SDA low: clock it out, then issue a STOP. The
// next i2c_* call re-runs i2cStart.
static void bus_clear(void) {
    i2cStop(&I2C_DRIVER);

    palSetLineMode(I2C1_SCL_PIN, PAL_MODE_OUTPUT_OPENDRAIN);
    palSetLineMode(I2C1_SDA_PIN, PAL_MODE_OUTPUT_OPENDRAIN);
    palSetLine(I2C1_SCL_PIN);
    palSetLine(I2C1_SDA_PIN);
    wait_us(BUSDEV_BUS_CLEAR_HALF_US);

    for (uint8_t i = 0; i < BUSDEV_BUS_CLEAR_PULSES && palReadLine(I2C1_SDA_PIN) == PAL_LOW; i++) {
        palClearLine(I2C1_SCL_PIN);
        wait_us(BUSDEV_BUS_CLEAR_HALF_US);
        palSetLine(I2C1_SCL_PIN);
        wait_us(BUSDEV_BUS_CLEAR_HALF_US);
    }

    // STOP: SDA rises while SCL is high.
    palClearLine(I2C1_SDA_PIN);
    wait_us(BUSDEV_BUS_CLEAR_HALF_US);
    palSetLine(I2C1_SCL_PIN);
    wait_us(BUSDEV_BUS_CLEAR_HALF_US);
    palSetLine(I2C1_SDA_PIN);
    wait_us(BUSDEV_BUS_CLEAR_HALF_US);

#    if defined(USE_GPIOV1)
    palSetLineMode(I2C1_SCL_PIN, I2C1_SCL_PAL_MODE);
    palSetLineMode(I2C1_SDA_PIN, I2C1_SDA_PAL_MODE);
#    else
    palSetLineMode(I2C1_SCL_PIN, PAL_MODE_ALTERNATE(I2C1_SCL_PAL_MODE) | PAL_OUTPUT_TYPE_OPENDRAIN);
    palSetLineMode(I2C1_SDA_PIN, PAL_MODE_ALTERNATE(I2C1_SDA_PAL_MODE) | PAL_OUTPUT_TYPE_OPENDRAIN);
#    endif
}
#else
static bool bus_sda_low(void) {
    return false;
}
static void bus_clear(void) {}
#endif

// Feeds a transaction result into the bus layer. Timeouts count against the
// bus, NAKs against the device only.
static void bus_note(i2c_status_t status) {
    if (status == I2C_STATUS_SUCCESS) {
        bus_timeout_streak = 0;
        bus_suspect        = false;
        return;
    }
    if (status != I2C_STATUS_TIMEOUT) {
        return;
    }
    if (bus_timeout_streak < UINT8_MAX) {
        bus_timeout_streak++;
    }
    if (!bus_suspect && bus_timeout_streak >= BUSDEV_BUS_TIMEOUT_STREAK) {
        bus_suspect = true;
        busdev_force_down_all(BUSDEV_REASON_BUS_DEAD);
    }
}

static void bus_clear_if_stuck(uint32_t now) {
    if (!bus_suspect || !bus_sda_low()) {
        return;
    }
    if (bus_cleared_once && !timer_expired32(now, bus_clear_time + BUSDEV_BUS_CLEAR_INTERVAL_MS)) {
        return;
    }
    bus_clear_time   = now;
    bus_cleared_once = true;
    bus_clear();
}

// -- registry ----------------------------------------------------------------

busdev_id_t busdev_register(const busdev_desc_t *desc) {
    if (desc == NULL) {
        return BUSDEV_NONE;
    }
    for (uint8_t i = 0; i < device_count; i++) {
        if (devices[i].desc == desc) {
            return i;
        }
    }
    if (device_count >= BUSDEV_MAX_DEVICES) {
        return BUSDEV_NONE;
    }

    busdev_id_t id      = device_count++;
    busdev_dev_t *d     = &devices[id];
    d->desc             = desc;
    d->state            = BUSDEV_ABSENT;
    d->step             = 0;
    d->err_streak       = 0;
    d->probe_fails      = 0;
    d->verify_fails     = 0;
    d->suspects         = 0;
    d->last_reason      = BUSDEV_REASON_NONE;
    d->deadline         = timer_read32();
    return id;
}

static busdev_dev_t *dev_of(busdev_id_t id) {
    if (id >= device_count) {
        return NULL;
    }
    return &devices[id];
}

bool busdev_is_up(busdev_id_t id) {
    busdev_dev_t *d = dev_of(id);
    return d != NULL && d->state == BUSDEV_UP;
}

uint8_t busdev_addr(busdev_id_t id) {
    busdev_dev_t *d = dev_of(id);
    return d != NULL ? d->desc->addr : 0;
}

busdev_state_t busdev_state(busdev_id_t id) {
    busdev_dev_t *d = dev_of(id);
    return d != NULL ? d->state : BUSDEV_ABSENT;
}

// -- transitions -------------------------------------------------------------

__attribute__((weak)) void busdev_on_transition(busdev_id_t id, busdev_state_t old_state, busdev_state_t new_state, uint8_t reason) {
    dprintf("busdev: %s %u->%u reason %u\n", devices[id].desc->name, (uint8_t)old_state, (uint8_t)new_state, reason);
}

static void set_state(uint8_t index, busdev_state_t new_state, uint8_t reason) {
    busdev_dev_t  *d         = &devices[index];
    busdev_state_t old_state = d->state;

    d->state       = new_state;
    d->last_reason = reason;

    if (d->desc->on_state != NULL) {
        d->desc->on_state(index, old_state, new_state, reason);
    }
    busdev_on_transition(index, old_state, new_state, reason);
}

static void to_absent(uint8_t index, uint8_t reason, uint32_t now) {
    busdev_dev_t *d = &devices[index];

    d->step         = 0;
    d->err_streak   = 0;
    d->verify_fails = 0;
    d->suspects     = 0;
    d->deadline     = now + BUSDEV_PROBE_INTERVAL_MS;
    set_state(index, BUSDEV_ABSENT, reason);
}

static void to_configuring(uint8_t index, uint8_t reason, uint32_t now) {
    busdev_dev_t *d = &devices[index];

    d->step       = 0;
    d->err_streak = 0;
    d->deadline   = now;
    set_state(index, BUSDEV_CONFIGURING, reason);
}

// -- scheduling --------------------------------------------------------------

static void do_probe(uint8_t index, uint32_t now) {
    busdev_dev_t        *d    = &devices[index];
    const busdev_desc_t *desc = d->desc;

    i2c_status_t status = desc->probe != NULL ? desc->probe(desc->addr) : i2c_ping_address(desc->addr, desc->probe_timeout_ms);
    bus_note(status);

    if (status == I2C_STATUS_SUCCESS) {
        d->probe_fails = 0;
        d->deadline    = now + desc->settle_ms;
        set_state(index, BUSDEV_SETTLING, BUSDEV_REASON_PROBE_ACK);
        return;
    }

    if (status == I2C_STATUS_TIMEOUT) {
        bus_clear_if_stuck(now);
    }
    if (d->probe_fails < UINT8_MAX) {
        d->probe_fails++;
    }
    d->deadline = now + (d->probe_fails >= BUSDEV_PROBE_BACKOFF_AFTER ? BUSDEV_PROBE_BACKOFF_MS : BUSDEV_PROBE_INTERVAL_MS);
}

static void do_init_step(uint8_t index, uint32_t now) {
    busdev_dev_t *d     = &devices[index];
    uint16_t      delay = 0;

    busdev_step_t result = d->desc->init_step != NULL ? d->desc->init_step(d->desc->addr, d->step, &delay) : BUSDEV_STEP_DONE;

    // The callback may have reported an error that already took the device down.
    if (d->state != BUSDEV_CONFIGURING) {
        return;
    }

    switch (result) {
        case BUSDEV_STEP_MORE:
            d->step++;
            d->deadline = now + delay;
            break;
        case BUSDEV_STEP_DONE:
            d->deadline = now;
            set_state(index, BUSDEV_VERIFYING, BUSDEV_REASON_NONE);
            break;
        case BUSDEV_STEP_FAIL:
            if (d->probe_fails < UINT8_MAX) {
                d->probe_fails++;
            }
            to_absent(index, BUSDEV_REASON_INIT_FAILED, now);
            break;
    }
}

static void do_verify(uint8_t index, uint32_t now) {
    busdev_dev_t  *d    = &devices[index];
    busdev_state_t from = d->state;

    bool ok = d->desc->verify != NULL ? d->desc->verify(d->desc->addr) : true;

    if (d->state != from) {
        return;
    }

    if (ok) {
        d->verify_fails = 0;
        d->err_streak   = 0;
        d->deadline     = now + d->desc->verify_interval_ms;
        if (from == BUSDEV_UP) {
            // A scheduled verify passed: the device is behaving again.
            d->suspects = 0;
        } else {
            set_state(index, BUSDEV_UP, BUSDEV_REASON_VERIFY_OK);
        }
        return;
    }

    d->verify_fails++;
    if (d->verify_fails >= 2) {
        to_absent(index, BUSDEV_REASON_VERIFY_MISMATCH, now);
    } else {
        to_configuring(index, BUSDEV_REASON_VERIFY_MISMATCH, now);
    }
}

static bool device_due(const busdev_dev_t *d, uint32_t now) {
    switch (d->state) {
        case BUSDEV_ABSENT:
        case BUSDEV_CONFIGURING:
        case BUSDEV_VERIFYING:
            return timer_expired32(now, d->deadline);
        case BUSDEV_UP:
            return d->desc->verify != NULL && d->desc->verify_interval_ms != 0 && timer_expired32(now, d->deadline);
        default:
            return false;
    }
}

void busdev_task(void) {
    uint32_t now = timer_read32();

    // Bookkeeping only: no bus access, so it does not consume the action slot.
    for (uint8_t i = 0; i < device_count; i++) {
        if (devices[i].state == BUSDEV_SETTLING && timer_expired32(now, devices[i].deadline)) {
            to_configuring(i, BUSDEV_REASON_NONE, now);
        }
    }

    for (uint8_t n = 0; n < device_count; n++) {
        uint8_t       i = (round_robin + n) % device_count;
        busdev_dev_t *d = &devices[i];
        if (!device_due(d, now)) {
            continue;
        }
        round_robin = (i + 1) % device_count;

        switch (d->state) {
            case BUSDEV_ABSENT:
                do_probe(i, now);
                break;
            case BUSDEV_CONFIGURING:
                do_init_step(i, now);
                break;
            case BUSDEV_VERIFYING:
            case BUSDEV_UP:
                do_verify(i, now);
                break;
            default:
                break;
        }
        return;
    }
}

// -- driver-facing events ----------------------------------------------------

void busdev_report_error(busdev_id_t id, i2c_status_t status) {
    busdev_dev_t *d = dev_of(id);
    if (d == NULL) {
        return;
    }

    bus_note(status);

    if (d->state != BUSDEV_UP) {
        return;
    }

    uint8_t limit = d->desc->max_err_streak != 0 ? d->desc->max_err_streak : 1;
    if (d->err_streak < UINT8_MAX) {
        d->err_streak++;
    }
    if (d->err_streak >= limit) {
        to_absent(id, status == I2C_STATUS_TIMEOUT ? BUSDEV_REASON_TIMEOUT : BUSDEV_REASON_NAK, timer_read32());
    }
}

void busdev_report_suspect(busdev_id_t id, uint8_t reason) {
    busdev_dev_t *d = dev_of(id);
    if (d == NULL || d->state == BUSDEV_ABSENT || d->state == BUSDEV_CONFIGURING || d->state == BUSDEV_VERIFYING) {
        return;
    }

    uint32_t now = timer_read32();
    if (d->suspects < UINT8_MAX) {
        d->suspects++;
    }
    // A device that keeps misbehaving after reconfiguration goes down, so the
    // probe cadence and backoff apply instead of a rewrite every few scans.
    if (d->suspects >= BUSDEV_MAX_SUSPECT_STREAK) {
        to_absent(id, reason, now);
    } else {
        to_configuring(id, reason, now);
    }
}

void busdev_force_down(busdev_id_t id, uint8_t reason) {
    busdev_dev_t *d = dev_of(id);
    if (d == NULL || d->state == BUSDEV_ABSENT) {
        return;
    }
    to_absent(id, reason, timer_read32());
}

void busdev_force_down_all(uint8_t reason) {
    for (uint8_t i = 0; i < device_count; i++) {
        busdev_force_down(i, reason);
    }
}

// -- module hooks ------------------------------------------------------------

void housekeeping_task_busdev(void) {
    busdev_task();
}

void suspend_power_down_busdev(void) {
    // The main loop, and with it housekeeping, is stopped during USB suspend.
    // Devices stay powered, so keep supervising them from here: the suspend
    // loop scans the matrix to detect a wake, and that needs the expander up.
    busdev_task();
}
