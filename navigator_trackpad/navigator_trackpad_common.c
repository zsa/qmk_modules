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

        return CGEN6_READ_OK;
    }
    // Mouse/relative mode report. The pad is always configured for PTP, so a
    // mouse report means it lost its configuration, unless it still reads back
    // in PTP mode: then the report was queued before the mode write (the pad
    // powers up in its own mode and init writes PTP mode without reading the
    // queue out afterwards).
    else if (report_id == CGEN6_MOUSE_REPORT_ID) {
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
}
#endif

static busdev_step_t trackpad_init_step(uint8_t addr, uint8_t step, uint16_t *delay_ms) {
    trackpad_addr = addr;

    switch (step) {
        case 0:
            cirque_gen6_clear();
#if defined(NAVIGATOR_TRACKPAD_DEBUG)
            trackpad_dump_info();
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
            return BUSDEV_STEP_DONE;
    }
}

// XY_CONFIG is not adjacent to FEED_CONFIG4, so reading it costs a second
// transaction, and it cannot be lost without the feed mode being lost too.
static bool trackpad_verify(uint8_t addr) {
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
