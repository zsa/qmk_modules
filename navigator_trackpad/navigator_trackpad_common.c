// Copyright 2025 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

// Common hardware layer for Navigator trackpad
// Shared by both mouse and PTP implementations

#include <stdint.h>
#include <sys/types.h>
#include "navigator_trackpad_common.h"
#include "i2c_master.h"
#include "busdev.h"
#include "quantum.h"
#include "timer.h"
#include <string.h>

// Shared globals
static uint16_t current_cpi = DEFAULT_CPI_TICK;

busdev_id_t navigator_trackpad_dev = BUSDEV_NONE;
// Address the supervisor is currently using for this device: the real trackpad
// (0x58, left-plugged) or the left-nRF wireless mirror (0x5C, right-plugged).
static uint8_t trackpad_addr = NAVIGATOR_TRACKPAD_ADDRESS;

#ifdef COMMUNITY_MODULE_WIRELESS_ENABLE
#    include "wireless.h"

// Strong override of the wireless module's weak probe hook, so telemetry can
// report what the last probe actually found rather than what this build can
// drive. The latched address says which of the two answered, which is as close
// to a side-of-the-board signal as the STM32 gets.
uint8_t zsa_nav_trackpad_probe_state(void) {
    busdev_state_t state = busdev_state(navigator_trackpad_dev);
    if (state == BUSDEV_ABSENT) {
        return ZSA_NAV_PROBE_NONE;
    }
    if (state != BUSDEV_UP) {
        return ZSA_NAV_PROBE_PENDING;
    }
    return (busdev_addr(navigator_trackpad_dev) == NAVIGATOR_TRACKPAD_ADDRESS) ? ZSA_NAV_PROBE_LOCAL : ZSA_NAV_PROBE_MIRROR;
}
#endif

// Failed transaction. A timeout while up takes the pad down at once; other
// errors count toward NAVIGATOR_TRACKPAD_MAX_ERR_STREAK.
static void trackpad_io_error(i2c_status_t status) {
    busdev_report_error(navigator_trackpad_dev, status);
    if (status == I2C_STATUS_TIMEOUT && busdev_is_up(navigator_trackpad_dev)) {
        busdev_force_down(navigator_trackpad_dev, BUSDEV_REASON_TIMEOUT);
    }
}

#ifdef COMMUNITY_MODULE_WIRELESS_ENABLE
// A report packet a Sleep poll read, handed to the next report read.
static uint8_t sleep_packet[CGEN6_MAX_PACKET_SIZE];
static bool    sleep_packet_valid;

static bool take_sleep_packet(uint8_t *data, uint16_t cnt) {
    if (!sleep_packet_valid) {
        return false;
    }
    sleep_packet_valid = false;
    if (cnt != CGEN6_MAX_PACKET_SIZE || trackpad_addr != NAVIGATOR_TRACKPAD_ADDRESS) {
        return false;
    }
    memcpy(data, sleep_packet, cnt);
    return true;
}
#else
#    define take_sleep_packet(data, cnt) false
#endif

// I2C communication functions
i2c_status_t cirque_gen6_read_report(uint8_t *data, uint16_t cnt) {
    if (take_sleep_packet(data, cnt)) {
        return I2C_STATUS_SUCCESS;
    }
    i2c_status_t res = i2c_receive(trackpad_addr, data, cnt, NAVIGATOR_TRACKPAD_TIMEOUT);
    if (res != I2C_STATUS_SUCCESS) {
        trackpad_io_error(res);
        return res;
    }
    busdev_report_ok(navigator_trackpad_dev);
    wait_us(cnt * 15);
    return res;
}

void cirque_gen6_clear(void) {
    uint8_t buf[CGEN6_MAX_PACKET_SIZE];
    for (uint8_t i = 0; i < 5; i++) {
        wait_ms(1);
        if (cirque_gen6_read_report(buf, CGEN6_MAX_PACKET_SIZE) != I2C_STATUS_SUCCESS) {
            break;
        }
    }
}

uint8_t cirque_gen6_read_memory(uint32_t addr, uint8_t *data, uint16_t cnt, bool fast_read) {
    uint8_t  cksum = 0;
    uint8_t  res   = CGEN6_SUCCESS;
    uint8_t  len[2];
    uint16_t read = 0;

    uint8_t preamble[8] = {0x01, 0x09, (uint8_t)(addr & (uint32_t)0x000000FF), (uint8_t)((addr & 0x0000FF00) >> 8), (uint8_t)((addr & 0x00FF0000) >> 16), (uint8_t)((addr & 0xFF000000) >> 24), (uint8_t)(cnt & 0x00FF), (uint8_t)((cnt & 0xFF00) >> 8)};

    // Read the length of the data + 3 bytes (first 2 bytes for the length and the last byte for the checksum)
    // Create a buffer to store the data
    uint8_t buf[cnt + 3];
    i2c_status_t status = i2c_transmit_and_receive(trackpad_addr, preamble, 8, buf, cnt + 3, NAVIGATOR_TRACKPAD_TIMEOUT);
    if (status != I2C_STATUS_SUCCESS) {
        res |= CGEN6_I2C_FAILED;
        trackpad_io_error(status);
    }

    // Read the data length
    for (uint8_t i = 0; i < 2; i++) {
        cksum += len[i] = buf[i];
        read++;
    }

    // Populate the data buffer
    for (uint16_t i = 2; i < cnt + 2; i++) {
        cksum += data[i - 2] = buf[i];
        read++;
    }

    if (!fast_read) {
        // Check the checksum
        if (cksum != buf[read]) {
            res |= CGEN6_CKSUM_FAILED;
        }

        // Check the length (incremented first to account for the checksum)
        if (++read != (len[0] | (len[1] << 8))) {
            res |= CGEN6_LEN_MISMATCH;
        }

        wait_ms(1);
    } else {
        wait_us(250);
    }

    return res;
}

uint8_t cirque_gen6_write_memory(uint32_t addr, uint8_t *data, uint16_t cnt) {
    uint8_t res   = CGEN6_SUCCESS;
    uint8_t cksum = 0, i = 0;
    uint8_t preamble[8] = {0x00, 0x09, (uint8_t)(addr & 0x000000FF), (uint8_t)((addr & 0x0000FF00) >> 8), (uint8_t)((addr & 0x00FF0000) >> 16), (uint8_t)((addr & 0xFF000000) >> 24), (uint8_t)(cnt & 0x00FF), (uint8_t)((cnt & 0xFF00) >> 8)};

    uint8_t buf[cnt + 9];
    // Calculate the checksum
    for (; i < 8; i++) {
        cksum += buf[i] = preamble[i];
    }

    for (i = 0; i < cnt; i++) {
        cksum += buf[i + 8] = data[i];
    }

    buf[cnt + 8] = cksum;

    i2c_status_t status = i2c_transmit(trackpad_addr, buf, cnt + 9, NAVIGATOR_TRACKPAD_TIMEOUT);
    if (status != I2C_STATUS_SUCCESS) {
        res |= CGEN6_I2C_FAILED;
        trackpad_io_error(status);
    }

    wait_ms(1);

    return res;
}

// Register access functions
uint8_t cirque_gen6_read_reg(uint32_t addr, bool fast_read) {
    uint8_t data;
    uint8_t res = cirque_gen6_read_memory(addr, &data, 1, fast_read);
    if (res != CGEN6_SUCCESS) {
        printf("Failed to read 8bits from register at address 0x%08X with error 0x%02X\n", (u_int)addr, res);
        return 0;
    }
    return data;
}

uint16_t cirque_gen6_read_reg_16(uint32_t addr) {
    uint8_t buf[2];
    uint8_t res = cirque_gen6_read_memory(addr, buf, 2, false);
    if (res != CGEN6_SUCCESS) {
        printf("Failed to read 16bits from register at address 0x%08X with error 0x%02X\n", (u_int)addr, res);
        return 0;
    }
    return (buf[1] << 8) | buf[0];
}

uint32_t cirque_gen6_read_reg_32(uint32_t addr) {
    uint8_t buf[4];
    uint8_t res = cirque_gen6_read_memory(addr, buf, 4, false);
    if (res != CGEN6_SUCCESS) {
        printf("Failed to read 32bits from register at address 0x%08X with error 0x%02X\n", (u_int)addr, res);
        return 0;
    }
    return (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
}

uint8_t cirque_gen6_write_reg(uint32_t addr, uint8_t data) {
    return cirque_gen6_write_memory(addr, &data, 1);
}

uint8_t cirque_gen6_write_reg_16(uint32_t addr, uint16_t data) {
    uint8_t buf[2] = {data & 0xFF, (data >> 8) & 0xFF};
    return cirque_gen6_write_memory(addr, buf, 2);
}

uint8_t cirque_gen6_write_reg_32(uint32_t addr, uint32_t data) {
    uint8_t buf[4] = {data & 0xFF, (data >> 8) & 0xFF, (data >> 16) & 0xFF, (data >> 24) & 0xFF};
    return cirque_gen6_write_memory(addr, buf, 4);
}

// Configuration functions
uint8_t cirque_gen6_set_relative_mode(void) {
    uint8_t feed_config4 = cirque_gen6_read_reg(CGEN6_FEED_CONFIG4, false);
    feed_config4 &= 0xF3;
    return cirque_gen6_write_reg(CGEN6_FEED_CONFIG4, feed_config4);
}

uint8_t cirque_gen6_set_ptp_mode(void) {
    uint8_t feed_config4 = cirque_gen6_read_reg(CGEN6_FEED_CONFIG4, false);
    feed_config4 &= 0xF7;
    feed_config4 |= 0x04;
    return cirque_gen6_write_reg(CGEN6_FEED_CONFIG4, feed_config4);
}

uint8_t cirque_gen6_swap_xy(bool set) {
    uint8_t xy_config = cirque_gen6_read_reg(CGEN6_XY_CONFIG, false);
    if (set) {
        xy_config |= 0x04;
    } else {
        xy_config &= ~0x04;
    }
    return cirque_gen6_write_reg(CGEN6_XY_CONFIG, xy_config);
}

uint8_t cirque_gen6_invert_y(bool set) {
    uint8_t xy_config = cirque_gen6_read_reg(CGEN6_XY_CONFIG, false);
    if (set) {
        xy_config |= 0x02;
    } else {
        xy_config &= ~0x02;
    }
    return cirque_gen6_write_reg(CGEN6_XY_CONFIG, xy_config);
}

uint8_t cirque_gen6_invert_x(bool set) {
    uint8_t xy_config = cirque_gen6_read_reg(CGEN6_XY_CONFIG, false);
    if (set) {
        xy_config |= 0x01;
    } else {
        xy_config &= ~0x01;
    }
    return cirque_gen6_write_reg(CGEN6_XY_CONFIG, xy_config);
}

uint8_t cirque_gen6_enable_logical_scaling(bool set) {
    uint8_t xy_config = cirque_gen6_read_reg(CGEN6_XY_CONFIG, false);
    if (set) {
        xy_config &= ~0x08;
    } else {
        xy_config |= 0x08;
    }
    return cirque_gen6_write_reg(CGEN6_XY_CONFIG, xy_config);
}

// Motion detection - returns true if data ready, false on no motion or I2C failure
bool cirque_gen6_has_motion(void) {
    uint8_t data;
    uint8_t res = cirque_gen6_read_memory(CGEN6_I2C_DR, &data, 1, true);
    if (res != CGEN6_SUCCESS) {
        return false;
    }
    return data != 0;
}

// Feed mode written by trackpad_init_step() and expected back by
// trackpad_verify().
#define CGEN6_FEED_CONFIG4_MODE_MASK 0x0C
#define CGEN6_FEED_CONFIG4_PTP 0x04

// The pad reads back in PTP mode. False on a failed read.
static bool trackpad_feed_is_ptp(void) {
    uint8_t feed_config4;
    if (cirque_gen6_read_memory(CGEN6_FEED_CONFIG4, &feed_config4, 1, false) != CGEN6_SUCCESS) {
        return false;
    }
    return (feed_config4 & CGEN6_FEED_CONFIG4_MODE_MASK) == CGEN6_FEED_CONFIG4_PTP;
}

// Report reading - fills provided report struct on CGEN6_READ_OK.
cgen6_read_t cirque_gen_6_read_report(cgen6_report_t *report) {
    uint8_t packet[CGEN6_MAX_PACKET_SIZE];
    if (cirque_gen6_read_report(packet, CGEN6_MAX_PACKET_SIZE) != I2C_STATUS_SUCCESS) {
        return CGEN6_READ_FAILED;
    }

    uint8_t report_id = packet[2];

    // PTP mode report
    if (report_id == CGEN6_PTP_REPORT_ID) {
        report->fingers[0].confidence = packet[3] & 0x01;
        report->fingers[0].tip        = (packet[3] & 0x02) >> 1;
        report->fingers[0].id         = (packet[3] & 0xFC) >> 2;
        report->fingers[0].x          = packet[5] << 8 | packet[4];
        report->fingers[0].y          = packet[7] << 8 | packet[6];
        report->fingers[1].confidence = packet[8] & 0x01;
        report->fingers[1].tip        = (packet[8] & 0x02) >> 1;
        report->fingers[1].id         = (packet[8] & 0xFC) >> 2;
        report->fingers[1].x          = packet[10] << 8 | packet[9];
        report->fingers[1].y          = packet[12] << 8 | packet[11];
        report->scan_time             = packet[14] << 8 | packet[13];
        report->contact_count         = packet[15];
        report->buttons               = packet[16];
#if defined(NAVIGATOR_TRACKPAD_DEBUG_TIMING)
        navigator_trackpad_debug_note_report(report, true);
#endif

        return CGEN6_READ_OK;
    }
    // Mouse/relative mode report. The pad is always configured for PTP, so a
    // mouse report means it lost its configuration, unless it still reads back
    // in PTP mode: then the report was queued before the mode write (the pad
    // powers up in its own mode and init writes PTP mode without reading the
    // queue out afterwards).
    else if (report_id == CGEN6_MOUSE_REPORT_ID) {
#if defined(NAVIGATOR_TRACKPAD_DEBUG_TIMING)
        navigator_trackpad_debug_note_report(report, false);
#endif
        report->buttons     = packet[3];
        report->xDelta      = packet[4];
        report->yDelta      = packet[5];
        report->scrollDelta = packet[6];
        report->panDelta    = packet[7];
        if (!trackpad_feed_is_ptp()) {
            busdev_report_suspect(navigator_trackpad_dev, BUSDEV_REASON_BAD_REPORT_ID);
        }
        return CGEN6_READ_OK;
    }

    // Unknown or empty report - no valid data
    return CGEN6_READ_EMPTY;
}

// Dumps sensor identification to the console. Enable with NAVIGATOR_TRACKPAD_DEBUG.
#if defined(NAVIGATOR_TRACKPAD_DEBUG)
#    if defined(NAVIGATOR_TRACKPAD_DEBUG_TIMING)
static bool timing_running(void);
#    else
#        define timing_running() false
#    endif

// Reads a 16-bit register and prints it raw and scaled to `unit`. Read-only.
static void trackpad_dump_power_16(const char *name, uint32_t addr, uint32_t scale, const char *unit) {
    uint8_t buf[2];
    uint8_t res = cirque_gen6_read_memory(addr, buf, 2, false);
    if (res != CGEN6_SUCCESS) {
        printf("Touchpad %s (0x%08X): read error 0x%02X\n", name, (u_int)addr, res);
        return;
    }
    uint16_t val = (buf[1] << 8) | buf[0];
    printf("Touchpad %s (0x%08X): 0x%04X = %u %s\n", name, (u_int)addr, val, (u_int)(val * scale), unit);
}

// Reads an 8-bit register into *val and prints it raw. Read-only.
static bool trackpad_dump_power_8(const char *name, uint32_t addr, uint8_t *val) {
    uint8_t res = cirque_gen6_read_memory(addr, val, 1, false);
    if (res != CGEN6_SUCCESS) {
        printf("Touchpad %s (0x%08X): read error 0x%02X\n", name, (u_int)addr, res);
        return false;
    }
    printf("Touchpad %s (0x%08X): 0x%02X\n", name, (u_int)addr, *val);
    return true;
}

// Dumps the power management block and SYS_CONFIG1. Read-only.
static void trackpad_dump_power(void) {
    uint8_t hdr[8];
    uint8_t res = cirque_gen6_read_memory(CGEN6_POWER_HEADER, hdr, 8, false);
    if (res != CGEN6_SUCCESS) {
        printf("Touchpad power header (0x%08X): read error 0x%02X\n", (u_int)CGEN6_POWER_HEADER, res);
    } else {
        uint32_t base  = (uint32_t)CGEN6_POWER_HEADER;
        bool     valid = hdr[0] == (base & 0xFF) && hdr[1] == ((base >> 8) & 0xFF) && hdr[2] == ((base >> 16) & 0xFF) && hdr[3] == ((base >> 24) & 0xFF) && (hdr[4] | hdr[5] | hdr[6] | hdr[7]) != 0;
        printf("Touchpad power header (0x%08X): %02X %02X %02X %02X %02X %02X %02X %02X (%s)\n", (u_int)base, hdr[0], hdr[1], hdr[2], hdr[3], hdr[4], hdr[5], hdr[6], hdr[7], valid ? "valid" : "invalid");
    }

    trackpad_dump_power_16("idle sleep wait", CGEN6_POWER_IDLE_SLEEP_WAIT, 100, "ms");
    trackpad_dump_power_16("idle sleep time", CGEN6_POWER_IDLE_SLEEP_TIME, 100, "us");
    trackpad_dump_power_16("deep sleep wait", CGEN6_POWER_DEEP_SLEEP_WAIT, 100, "ms");
    trackpad_dump_power_16("deep sleep time", CGEN6_POWER_DEEP_SLEEP_TIME, 100, "us");

    uint8_t settings;
    if (trackpad_dump_power_8("power settings", CGEN6_POWER_SETTINGS, &settings)) {
        printf("  disable deep sleep: %u\n", (settings & CGEN6_POWER_SETTING_DISABLE_DEEP_SLEEP) ? 1 : 0);
        printf("  disable idle sleep: %u\n", (settings & CGEN6_POWER_SETTING_DISABLE_IDLE_SLEEP) ? 1 : 0);
        printf("  don't wake from touch: %u\n", (settings & CGEN6_POWER_SETTING_DONT_WAKE_FROM_TOUCH) ? 1 : 0);
        printf("  don't wake from button: %u\n", (settings & CGEN6_POWER_SETTING_DONT_WAKE_FROM_BUTTON) ? 1 : 0);
        printf("  other bits: 0x%02X\n", settings & 0xF0);
    }

    uint8_t val;
    trackpad_dump_power_8("power cmd", CGEN6_POWER_CMD, &val);
    trackpad_dump_power_8("sys config1", CGEN6_SYS_CONFIG1, &val);
}

static void trackpad_dump_info(void) {
    uint8_t  hardwareId  = cirque_gen6_read_reg(CGEN6_HARDWARE_ID, false);
    uint8_t  firmwareId  = cirque_gen6_read_reg(CGEN6_FIRMWARE_ID, false);
    uint16_t vendorId    = cirque_gen6_read_reg_16(CGEN6_VENDOR_ID);
    uint16_t productId   = cirque_gen6_read_reg_16(CGEN6_PRODUCT_ID);
    uint16_t versionId   = cirque_gen6_read_reg_16(CGEN6_FIRMWARE_REV);
    uint32_t firmwareRev = cirque_gen6_read_reg_32(CGEN6_FIRMWARE_REV);

    printf("Touchpad Hardware ID: 0x%02X\n", hardwareId);
    printf("Touchpad Firmware ID: 0x%02X\n", firmwareId);
    printf("Touchpad Vendor ID: 0x%04X\n", vendorId);
    printf("Touchpad Product ID: 0x%04X\n", productId);
    printf("Touchpad Version ID: 0x%04X\n", versionId);

    uint32_t revision           = firmwareRev & 0x00ffffff;
    bool     uncommittedVersion = firmwareRev & 0x80000000;
    bool     branchVersion      = firmwareRev & 0x40000000;
    uint8_t  developerId        = firmwareRev & 0x3f000000;

    printf("Touchpad Firmware Revision: 0x%08X\n", (u_int)revision);
    printf("Touchpad Uncommitted Version: %s\n", uncommittedVersion ? "true" : "false");
    printf("Touchpad Branch Version: %s\n", branchVersion ? "true" : "false");
    printf("Touchpad Developer ID: %d\n", developerId);

    trackpad_dump_power();
}

// Repeats the dump while the pad is up on its real address and no contact is
// tracked by the caller.
void navigator_trackpad_debug_task(void) {
    static uint32_t last_dump = 0;

    if (navigator_trackpad_debug_paused() || timing_running()) {
        return;
    }
    if (!busdev_is_up(navigator_trackpad_dev) || busdev_addr(navigator_trackpad_dev) != NAVIGATOR_TRACKPAD_ADDRESS || trackpad_addr != NAVIGATOR_TRACKPAD_ADDRESS) {
        return;
    }
    if (timer_elapsed32(last_dump) < NAVIGATOR_TRACKPAD_DEBUG_DUMP_MS) {
        return;
    }
    last_dump = timer_read32();

    printf("Touchpad dump at %u ms, addr 0x%02X\n", (u_int)last_dump, trackpad_addr);
    trackpad_dump_info();
}

// Power-measurement actions.
static bool debug_paused;

bool navigator_trackpad_debug_paused(void) {
    return debug_paused;
}

void navigator_trackpad_debug_set_pause(bool paused) {
    debug_paused = paused;
    printf("Touchpad access %s\n", debug_paused ? "PAUSED" : "resumed");
}

void navigator_trackpad_debug_toggle_pause(void) {
    navigator_trackpad_debug_set_pause(!debug_paused);
}

bool navigator_trackpad_debug_pad_up(void) {
    return busdev_is_up(navigator_trackpad_dev) && busdev_addr(navigator_trackpad_dev) == NAVIGATOR_TRACKPAD_ADDRESS && trackpad_addr == NAVIGATOR_TRACKPAD_ADDRESS;
}

// True when the pad is up on its real address; prints why not otherwise.
static bool debug_pad_ready(const char *action) {
    if (navigator_trackpad_debug_pad_up()) {
        return true;
    }
    printf("Touchpad %s: skipped, pad not up at 0x%02X\n", action, NAVIGATOR_TRACKPAD_ADDRESS);
    return false;
}

static uint8_t debug_write_power_cmd(const char *action, uint8_t cmd) {
    if (!debug_pad_ready(action)) {
        return NAVIGATOR_TRACKPAD_DEBUG_NOT_UP;
    }
    uint8_t res = cirque_gen6_write_reg(CGEN6_POWER_CMD, cmd);
    printf("Touchpad %s: power cmd 0x%02X write 0x%02X\n", action, cmd, res);
    return res;
}

uint8_t navigator_trackpad_debug_force_sleep(void) {
    return debug_write_power_cmd("force sleep", CGEN6_POWER_CMD_FORCE_SLEEP);
}

uint8_t navigator_trackpad_debug_cancel_sleep(void) {
    return debug_write_power_cmd("cancel sleep", CGEN6_POWER_CMD_CANCEL_FORCE_SLEEP);
}

// Idle wait 1 s; the other timers are left as they are.
uint8_t navigator_trackpad_debug_short_timers(void) {
    if (!debug_pad_ready("short timers")) {
        return NAVIGATOR_TRACKPAD_DEBUG_NOT_UP;
    }
    uint8_t res = cirque_gen6_write_reg_16(CGEN6_POWER_IDLE_SLEEP_WAIT, 10);
    printf("Touchpad short timers: idle wait 10 write 0x%02X\n", res);
    return res;
}

// Values read from the pad at power-up.
uint8_t navigator_trackpad_debug_factory_timers(void) {
    if (!debug_pad_ready("factory timers")) {
        return NAVIGATOR_TRACKPAD_DEBUG_NOT_UP;
    }
    uint8_t r_iw = cirque_gen6_write_reg_16(CGEN6_POWER_IDLE_SLEEP_WAIT, 100);
    uint8_t r_it = cirque_gen6_write_reg_16(CGEN6_POWER_IDLE_SLEEP_TIME, 300);
    uint8_t r_dw = cirque_gen6_write_reg_16(CGEN6_POWER_DEEP_SLEEP_WAIT, 1200);
    uint8_t r_dt = cirque_gen6_write_reg_16(CGEN6_POWER_DEEP_SLEEP_TIME, 1000);
    uint8_t r_st = cirque_gen6_write_reg(CGEN6_POWER_SETTINGS, 0x00);
    printf("Touchpad factory timers: idle wait 0x%02X, idle time 0x%02X, deep wait 0x%02X, deep time 0x%02X, settings 0x%02X\n", r_iw, r_it, r_dw, r_dt, r_st);
    return r_iw | r_it | r_dw | r_dt | r_st;
}

uint8_t navigator_trackpad_debug_boost_mode(void) {
#    ifdef COMMUNITY_MODULE_WIRELESS_ENABLE
    return zsa_nav_boost_debug_mode();
#    else
    return 0;
#    endif
}

bool navigator_trackpad_debug_boost_sent(void) {
#    ifdef COMMUNITY_MODULE_WIRELESS_ENABLE
    return zsa_nav_boost_debug_sent();
#    else
    return false;
#    endif
}

#    if defined(NAVIGATOR_TRACKPAD_DEBUG_TIMING)
extern bool zsa_trrs_plugged(void);

static void put_u16(uint8_t *out, uint16_t v) {
    out[0] = v & 0xFF;
    out[1] = v >> 8;
}

// Power cycle timing (action 10). Run from housekeeping_task_navigator_trackpad().
enum {
    TIMING_OFF_SEND, // claim-off requested, not sent yet
    TIMING_OFF,      // claim-off sent, waiting the off time
    TIMING_ON_SEND,  // claim-on requested, not sent yet
    TIMING_ON,       // claim-on sent, measuring
};
#        define TIMING_SEND_MS 200

static struct {
    uint8_t  state;
    uint8_t  phase;
    uint8_t  sends; // zsa_nav_boost_debug_sends() when the override was set
    uint8_t  flags;
    uint8_t  init_runs;
    uint8_t  reports;
    uint16_t off_ms;
    uint16_t off_actual;
    uint16_t pings;
    uint16_t t_ack;
    uint16_t t_up;
    uint16_t t_report;
    uint16_t t_step[3];
    uint16_t t_init_done;
    uint32_t phase_ms; // phase start; claim-off sent in TIMING_OFF; claim-on sent in TIMING_ON
    uint32_t ping_ms;
} timing = {.state = NAVIGATOR_TRACKPAD_DEBUG_TIMING_IDLE};

static bool timing_running(void) {
    return timing.state == NAVIGATOR_TRACKPAD_DEBUG_TIMING_RUNNING;
}

static bool timing_measuring(void) {
    return timing_running() && timing.phase == TIMING_ON;
}

static uint16_t timing_since_on(void) {
    uint32_t ms = timer_elapsed32(timing.phase_ms);
    return ms < NAVIGATOR_TRACKPAD_DEBUG_TIMING_NONE ? (uint16_t)ms : NAVIGATOR_TRACKPAD_DEBUG_TIMING_NONE - 1;
}

static void timing_finish(uint8_t state) {
    zsa_nav_boost_debug_set(ZSA_NAV_BOOST_DEBUG_NONE);
    timing.state = state;
    printf("Touchpad timing %s: ack %u, up %u, report %u, init %u/%u/%u/%u, flags 0x%02X, runs %u, reports %u, off %u, pings %u\n", state == NAVIGATOR_TRACKPAD_DEBUG_TIMING_DONE ? "done" : "timeout", timing.t_ack, timing.t_up, timing.t_report, timing.t_step[0], timing.t_step[1], timing.t_step[2], timing.t_init_done, timing.flags, timing.init_runs, timing.reports, timing.off_actual, timing.pings);
}

// True once a claim requested at timing.sends went out with value `on`.
static bool timing_claim_sent(bool on) {
    return zsa_nav_boost_debug_sends() != timing.sends && zsa_nav_boost_debug_sent() == on;
}

static uint8_t timing_start(uint8_t arg) {
    if (timing_running() || debug_paused) {
        return NAVIGATOR_TRACKPAD_DEBUG_BUSY;
    }
    if (!zsa_wireless_link_ready()) {
        return NAVIGATOR_TRACKPAD_DEBUG_NO_LINK;
    }
    memset(&timing, 0, sizeof(timing));
    timing.t_ack       = NAVIGATOR_TRACKPAD_DEBUG_TIMING_NONE;
    timing.t_up        = NAVIGATOR_TRACKPAD_DEBUG_TIMING_NONE;
    timing.t_report    = NAVIGATOR_TRACKPAD_DEBUG_TIMING_NONE;
    timing.t_step[0]   = NAVIGATOR_TRACKPAD_DEBUG_TIMING_NONE;
    timing.t_step[1]   = NAVIGATOR_TRACKPAD_DEBUG_TIMING_NONE;
    timing.t_step[2]   = NAVIGATOR_TRACKPAD_DEBUG_TIMING_NONE;
    timing.t_init_done = NAVIGATOR_TRACKPAD_DEBUG_TIMING_NONE;
    timing.off_ms      = (arg != 0 ? arg : 100) * 10;
    timing.state       = NAVIGATOR_TRACKPAD_DEBUG_TIMING_RUNNING;
    timing.phase       = TIMING_OFF_SEND;
    timing.phase_ms    = timer_read32();
    timing.sends       = zsa_nav_boost_debug_sends();
    zsa_nav_boost_debug_set(ZSA_NAV_BOOST_DEBUG_OFF);
    printf("Touchpad timing: off %u ms\n", timing.off_ms);
    return CGEN6_SUCCESS;
}

// First ACK at the real address: the supervisor probes it next, moving the
// device off the mirror first if it is set up there.
static void timing_ack(bool by_supervisor) {
    timing.t_ack = timing_since_on();
    if (by_supervisor) {
        timing.flags |= NAVIGATOR_TRACKPAD_DEBUG_TIMING_SUP_ACK;
        return;
    }
    if (busdev_state(navigator_trackpad_dev) != BUSDEV_ABSENT && busdev_addr(navigator_trackpad_dev) != NAVIGATOR_TRACKPAD_ADDRESS) {
        busdev_force_down(navigator_trackpad_dev, BUSDEV_REASON_PROBE_ACK);
    }
    busdev_probe_now(navigator_trackpad_dev, NAVIGATOR_TRACKPAD_ADDRESS);
}

static void timing_task(void) {
    if (!timing_running()) {
        return;
    }
    switch (timing.phase) {
        case TIMING_OFF_SEND:
            if (timing_claim_sent(false)) {
                timing.phase    = TIMING_OFF;
                timing.phase_ms = zsa_nav_boost_debug_sent_ms();
            } else if (timer_elapsed32(timing.phase_ms) >= TIMING_SEND_MS) {
                timing.flags |= NAVIGATOR_TRACKPAD_DEBUG_TIMING_NO_CLAIM;
                timing_finish(NAVIGATOR_TRACKPAD_DEBUG_TIMING_TIMEOUT);
            }
            break;

        case TIMING_OFF:
            if (timer_elapsed32(timing.phase_ms) >= timing.off_ms) {
                timing.off_actual = timer_elapsed32(timing.phase_ms);
                timing.phase      = TIMING_ON_SEND;
                timing.phase_ms   = timer_read32();
                timing.sends      = zsa_nav_boost_debug_sends();
                zsa_nav_boost_debug_set(ZSA_NAV_BOOST_DEBUG_ON);
            }
            break;

        case TIMING_ON_SEND:
            if (timing_claim_sent(true)) {
                uint32_t sent     = zsa_nav_boost_debug_sent_ms();
                timing.off_actual = timing.off_actual + (uint16_t)(sent - timing.phase_ms);
                timing.phase      = TIMING_ON;
                timing.phase_ms   = sent;
                timing.ping_ms    = sent - NAVIGATOR_TRACKPAD_DEBUG_TIMING_PING_MS;
                if (zsa_wireless_vbus_present() || zsa_trrs_plugged()) {
                    timing.flags |= NAVIGATOR_TRACKPAD_DEBUG_TIMING_RAIL_FED;
                }
                if (navigator_trackpad_debug_pad_up()) {
                    timing.flags |= NAVIGATOR_TRACKPAD_DEBUG_TIMING_WAS_UP;
                }
            } else if (timer_elapsed32(timing.phase_ms) >= TIMING_SEND_MS) {
                timing.flags |= NAVIGATOR_TRACKPAD_DEBUG_TIMING_NO_CLAIM;
                timing_finish(NAVIGATOR_TRACKPAD_DEBUG_TIMING_TIMEOUT);
            }
            break;

        case TIMING_ON:
            if (timer_elapsed32(timing.phase_ms) >= NAVIGATOR_TRACKPAD_DEBUG_TIMING_WINDOW_MS) {
                timing_finish(timing.t_up != NAVIGATOR_TRACKPAD_DEBUG_TIMING_NONE ? NAVIGATOR_TRACKPAD_DEBUG_TIMING_DONE : NAVIGATOR_TRACKPAD_DEBUG_TIMING_TIMEOUT);
            } else if (timing.t_ack == NAVIGATOR_TRACKPAD_DEBUG_TIMING_NONE && timer_elapsed32(timing.ping_ms) >= NAVIGATOR_TRACKPAD_DEBUG_TIMING_PING_MS) {
                timing.ping_ms = timer_read32();
                timing.pings++;
                if (i2c_ping_address(NAVIGATOR_TRACKPAD_ADDRESS, NAVIGATOR_TRACKPAD_PROBE_TIMEOUT) == I2C_STATUS_SUCCESS) {
                    timing_ack(false);
                }
            }
            break;
    }
}

void housekeeping_task_navigator_trackpad(void) {
    timing_task();
}

static void timing_note_state(busdev_state_t new_state) {
    if (!timing_measuring() || busdev_addr(navigator_trackpad_dev) != NAVIGATOR_TRACKPAD_ADDRESS) {
        return;
    }
    if (new_state == BUSDEV_SETTLING && timing.t_ack == NAVIGATOR_TRACKPAD_DEBUG_TIMING_NONE) {
        timing_ack(true);
    }
    if (new_state == BUSDEV_UP && timing.t_up == NAVIGATOR_TRACKPAD_DEBUG_TIMING_NONE) {
        timing.t_up = timing_since_on();
    }
}

static void timing_note_init(uint8_t addr, uint8_t step) {
    if (!timing_measuring() || addr != NAVIGATOR_TRACKPAD_ADDRESS || timing.t_up != NAVIGATOR_TRACKPAD_DEBUG_TIMING_NONE) {
        return;
    }
    if (step == 0 && timing.init_runs < UINT8_MAX) {
        timing.init_runs++;
    }
    timing.t_step[step < 2 ? step : 2] = timing_since_on();
}

static void timing_note_init_done(uint8_t addr) {
    if (!timing_measuring() || addr != NAVIGATOR_TRACKPAD_ADDRESS || timing.t_up != NAVIGATOR_TRACKPAD_DEBUG_TIMING_NONE) {
        return;
    }
    timing.t_init_done = timing_since_on();
}

void navigator_trackpad_debug_note_report(const cgen6_report_t *report, bool ptp) {
    if (!timing_measuring() || trackpad_addr != NAVIGATOR_TRACKPAD_ADDRESS) {
        return;
    }
    if (!ptp) {
        timing.flags |= NAVIGATOR_TRACKPAD_DEBUG_TIMING_MOUSE;
        return;
    }
    if (timing.t_report == NAVIGATOR_TRACKPAD_DEBUG_TIMING_NONE) {
        timing.t_report = timing_since_on();
        if (report->fingers[0].tip) {
            timing.flags |= NAVIGATOR_TRACKPAD_DEBUG_TIMING_FIRST_TIP;
        }
    }
    if (report->fingers[0].tip || report->fingers[1].tip) {
        timing.flags |= NAVIGATOR_TRACKPAD_DEBUG_TIMING_ANY_TIP;
    }
    if (timing.reports < UINT8_MAX) {
        timing.reports++;
    }
}

void navigator_trackpad_debug_timing_result(uint8_t *out) {
    out[0] = timing.state;
    put_u16(&out[1], timing.t_ack);
    put_u16(&out[3], timing.t_up);
    put_u16(&out[5], timing.t_report);
    put_u16(&out[7], timing.t_step[0]);
    put_u16(&out[9], timing.t_step[1]);
    put_u16(&out[11], timing.t_step[2]);
    put_u16(&out[13], timing.t_init_done);
    out[15] = timing.flags;
    out[16] = timing.init_runs;
    out[17] = timing.reports;
    put_u16(&out[18], timing.off_actual);
    put_u16(&out[20], timing.pings);
}

// Actions 7, 8 and 9 end a running cycle.
static void timing_abort(void) {
    if (timing_running()) {
        timing.state = NAVIGATOR_TRACKPAD_DEBUG_TIMING_IDLE;
        printf("Touchpad timing aborted\n");
    }
}
#    else
#        define timing_abort()
#    endif

uint8_t navigator_trackpad_debug_action(uint8_t action, uint8_t arg) {
    switch (action) {
        case NAVIGATOR_TRACKPAD_DEBUG_STATUS:
            return CGEN6_SUCCESS;
        case NAVIGATOR_TRACKPAD_DEBUG_FORCE_SLEEP:
            return navigator_trackpad_debug_force_sleep();
        case NAVIGATOR_TRACKPAD_DEBUG_CANCEL_SLEEP:
            return navigator_trackpad_debug_cancel_sleep();
        case NAVIGATOR_TRACKPAD_DEBUG_PAUSE:
            navigator_trackpad_debug_set_pause(true);
            return CGEN6_SUCCESS;
        case NAVIGATOR_TRACKPAD_DEBUG_RESUME:
            navigator_trackpad_debug_set_pause(false);
            return CGEN6_SUCCESS;
        case NAVIGATOR_TRACKPAD_DEBUG_SHORT_TIMERS:
            return navigator_trackpad_debug_short_timers();
        case NAVIGATOR_TRACKPAD_DEBUG_FACTORY_TIMERS:
            return navigator_trackpad_debug_factory_timers();
#    ifdef COMMUNITY_MODULE_WIRELESS_ENABLE
        case NAVIGATOR_TRACKPAD_DEBUG_BOOST_ON:
            timing_abort();
            zsa_nav_boost_debug_set(ZSA_NAV_BOOST_DEBUG_ON);
            printf("Left boost claim held ON\n");
            return CGEN6_SUCCESS;
        case NAVIGATOR_TRACKPAD_DEBUG_BOOST_NORMAL:
            timing_abort();
            zsa_nav_boost_debug_set(ZSA_NAV_BOOST_DEBUG_NONE);
            printf("Left boost claim normal\n");
            return CGEN6_SUCCESS;
        case NAVIGATOR_TRACKPAD_DEBUG_BOOST_OFF:
            timing_abort();
            zsa_nav_boost_debug_set(ZSA_NAV_BOOST_DEBUG_OFF);
            printf("Left boost claim held OFF\n");
            return CGEN6_SUCCESS;
#    endif
#    if defined(NAVIGATOR_TRACKPAD_DEBUG_TIMING)
        case NAVIGATOR_TRACKPAD_DEBUG_CYCLE_TIMING:
            return timing_start(arg);
#    endif
        default:
            return NAVIGATOR_TRACKPAD_DEBUG_BAD_ACTION;
    }
}
#endif

static busdev_step_t trackpad_init_step(uint8_t addr, uint8_t step, uint16_t *delay_ms) {
    trackpad_addr = addr;
#if defined(NAVIGATOR_TRACKPAD_DEBUG_TIMING)
    timing_note_init(addr, step);
#endif

    switch (step) {
        case 0:
            cirque_gen6_clear();
#if defined(NAVIGATOR_TRACKPAD_DEBUG)
            // Skipped during a timing cycle: not part of the production init.
            if (!timing_running()) {
                trackpad_dump_info();
            }
#endif
            // Matches the post-clear settle the blocking init used to do.
            *delay_ms = 50;
            return BUSDEV_STEP_MORE;

        case 1:
            if (cirque_gen6_set_ptp_mode() != CGEN6_SUCCESS) {
                return BUSDEV_STEP_FAIL;
            }
            return BUSDEV_STEP_MORE;

        default:
            if (cirque_gen6_swap_xy(true) != CGEN6_SUCCESS) {
                return BUSDEV_STEP_FAIL;
            }
            if (cirque_gen6_invert_x(true) != CGEN6_SUCCESS) {
                return BUSDEV_STEP_FAIL;
            }
            if (cirque_gen6_invert_y(true) != CGEN6_SUCCESS) {
                return BUSDEV_STEP_FAIL;
            }
            // Disable scaling for raw coordinates
            if (cirque_gen6_enable_logical_scaling(false) != CGEN6_SUCCESS) {
                return BUSDEV_STEP_FAIL;
            }
#if defined(NAVIGATOR_TRACKPAD_DEBUG_TIMING)
            timing_note_init_done(addr);
#endif
            return BUSDEV_STEP_DONE;
    }
}

// XY_CONFIG is not adjacent to FEED_CONFIG4, so reading it costs a second
// transaction, and it cannot be lost without the feed mode being lost too.
static bool trackpad_verify(uint8_t addr) {
#if defined(NAVIGATOR_TRACKPAD_DEBUG)
    // Paused while up: pass without touching the bus.
    if (navigator_trackpad_debug_paused() && busdev_is_up(navigator_trackpad_dev)) {
        return true;
    }
#endif
    trackpad_addr = addr;
    return trackpad_feed_is_ptp();
}

#ifdef COMMUNITY_MODULE_WIRELESS_ENABLE
extern bool zsa_stm32_owns_right_bus(void);

// Next probe uses the other address; cleared on every drop.
static bool probe_other;

// First probe after a drop: the real pad while the STM32 owns the right bus,
// otherwise the address that last ACKed (the mirror ACKs whenever the left nRF
// runs ESB, so it cannot rule out a pad on the left half). Then alternates.
static uint8_t trackpad_select_addr(void) {
    uint8_t first = zsa_stm32_owns_right_bus() ? NAVIGATOR_TRACKPAD_ADDRESS : busdev_addr(navigator_trackpad_dev);
    uint8_t other = first == NAVIGATOR_TRACKPAD_ADDRESS ? NAVIGATOR_TRACKPAD_MIRROR_ADDRESS : NAVIGATOR_TRACKPAD_ADDRESS;
    uint8_t addr  = probe_other ? other : first;

    probe_other = !probe_other;
    return addr;
}

static void trackpad_on_state(busdev_id_t id, busdev_state_t old_state, busdev_state_t new_state, uint8_t reason) {
    if (new_state == BUSDEV_ABSENT) {
        probe_other        = false;
        sleep_packet_valid = false;
    }
#    if defined(NAVIGATOR_TRACKPAD_DEBUG_TIMING)
    timing_note_state(new_state);
#    endif
}

// Left nav probe window: the real pad address, next. Up on the mirror, which
// answers with or without a pad on the right, the real address is pinged
// directly and the device moves there only if the pad answers.
void zsa_nav_trackpad_probe_local(void) {
    if (busdev_is_up(navigator_trackpad_dev) && busdev_addr(navigator_trackpad_dev) != NAVIGATOR_TRACKPAD_ADDRESS) {
        if (i2c_ping_address(NAVIGATOR_TRACKPAD_ADDRESS, NAVIGATOR_TRACKPAD_PROBE_TIMEOUT) != I2C_STATUS_SUCCESS) {
            return;
        }
        busdev_force_down(navigator_trackpad_dev, BUSDEV_REASON_PROBE_ACK);
    }
    busdev_probe_now(navigator_trackpad_dev, NAVIGATOR_TRACKPAD_ADDRESS);
}

// The real pad answered and is being brought up.
bool zsa_nav_trackpad_local_pending(void) {
    busdev_state_t state = busdev_state(navigator_trackpad_dev);
    return state != BUSDEV_ABSENT && state != BUSDEV_UP && busdev_addr(navigator_trackpad_dev) == NAVIGATOR_TRACKPAD_ADDRESS;
}

// ---- Sleep polling (wireless.h, zsa_nav_sleep_ops_t) ----
//
// The pad keeps its own power modes; its deeper sleep is not used. A poll reads
// the report packet the normal path reads: with nothing pending the pad
// answers an empty report, and a packet with a report ID is kept whole for the
// next cirque_gen6_read_report(), so the touch goes through the normal PTP or
// mouse path.
static bool trackpad_sleep_enter(void) {
    return busdev_is_up(navigator_trackpad_dev) && busdev_addr(navigator_trackpad_dev) == NAVIGATOR_TRACKPAD_ADDRESS;
}

static bool trackpad_sleep_idle(void) {
    return !sleep_packet_valid;
}

static uint8_t trackpad_sleep_poll(void) {
#if defined(NAVIGATOR_TRACKPAD_DEBUG)
    // Paused: no bus access, report idle.
    if (navigator_trackpad_debug_paused()) {
        return ZSA_NAV_SLEEP_IDLE;
    }
#endif
    memset(sleep_packet, 0, sizeof(sleep_packet));
    bool ok = zsa_stop_i2c_read(NAVIGATOR_TRACKPAD_ADDRESS, sleep_packet, CGEN6_MAX_PACKET_SIZE);
    zsa_nav_sleep_note_raw(sleep_packet, 6);
    if (!ok) {
        return ZSA_NAV_SLEEP_ERROR;
    }
    uint8_t report_id = sleep_packet[2];
    if (report_id != CGEN6_PTP_REPORT_ID && report_id != CGEN6_MOUSE_REPORT_ID) {
        return ZSA_NAV_SLEEP_IDLE;
    }
    sleep_packet_valid = true;
    return ZSA_NAV_SLEEP_ACTIVITY;
}

static void trackpad_sleep_exit(void) {}

static const zsa_nav_sleep_ops_t trackpad_sleep_ops = {
    .enter = trackpad_sleep_enter,
    .idle  = trackpad_sleep_idle,
    .poll  = trackpad_sleep_poll,
    .exit  = trackpad_sleep_exit,
};

const zsa_nav_sleep_ops_t *zsa_nav_trackpad_sleep_ops(void) {
    return &trackpad_sleep_ops;
}
#endif

static const busdev_desc_t trackpad_desc = {
    .name               = "nav_trackpad",
    .addr               = NAVIGATOR_TRACKPAD_ADDRESS,
    .addr_alt           = NAVIGATOR_TRACKPAD_MIRROR_ADDRESS,
    .settle_ms          = 20,
    .verify_interval_ms = 1000,
    .probe_timeout_ms   = NAVIGATOR_TRACKPAD_PROBE_TIMEOUT,
    .max_err_streak     = NAVIGATOR_TRACKPAD_MAX_ERR_STREAK,
#ifdef COMMUNITY_MODULE_WIRELESS_ENABLE
    .select_addr        = trackpad_select_addr,
    .on_state           = trackpad_on_state,
#else
    .select_addr        = NULL,
    .on_state           = NULL,
#endif
    .probe              = NULL,
    .init_step          = trackpad_init_step,
    .verify             = trackpad_verify,
};

// Hands the pad over to the bus supervisor; presence and configuration are its
// job from here on.
void navigator_trackpad_device_init(void) {
    i2c_init();

    if (navigator_trackpad_dev == BUSDEV_NONE) {
        navigator_trackpad_dev = busdev_register(&trackpad_desc);
    }
}

// CPI management
void set_cirque_cpi(void) {
    // traverse the sequence by comparing the cpi_x value with the current cpi_x value
    // set the cpi to the next value in the sequence
    switch (current_cpi) {
        case CPI_1: {
            current_cpi = CPI_2;
            break;
        }
        case CPI_2: {
            current_cpi = CPI_3;
            break;
        }
        case CPI_3: {
            current_cpi = CPI_4;
            break;
        }
        case CPI_4: {
            current_cpi = CPI_5;
            break;
        }
        case CPI_5: {
            current_cpi = CPI_6;
            break;
        }
        case CPI_6: {
            current_cpi = CPI_7;
            break;
        }
        case CPI_7: {
            current_cpi = CPI_1;
            break;
        }
        default: {
            current_cpi = CPI_4;
            break;
        }
    }
}

uint16_t navigator_trackpad_get_cpi(void) {
    return current_cpi;
}

void navigator_trackpad_set_cpi(uint16_t cpi) {
    if (cpi == 0) { // Decrease one tick
        if (current_cpi > 1) {
            current_cpi--;
        }
    } else {
        if (current_cpi < CPI_TICKS) {
            current_cpi++;
        }
    }
    set_cirque_cpi();
}

// Helper function to count active fingers
uint8_t cirque_gen6_finger_count(cgen6_report_t *report) {
    uint8_t fingers = 0;
    if (report->fingers[0].tip) {
        fingers++;
    }
    if (report->fingers[1].tip) {
        fingers++;
    }
    return fingers;
}
