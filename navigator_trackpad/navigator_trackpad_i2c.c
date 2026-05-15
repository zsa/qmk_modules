// Copyright 2025 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

// I2C-side of the Cirque Gen 6 driver. Decode + dispatch live in the shared
// navigator_trackpad_core module; this file owns chip configuration and the
// raw-packet read.

#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include "navigator_trackpad_i2c.h"
#include "i2c_master.h"
#include "quantum.h"
#include "timer.h"

#ifndef NAVIGATOR_TRACKPAD_ADDRESS
#    define NAVIGATOR_TRACKPAD_ADDRESS 0x58
#endif

#ifndef NAVIGATOR_TRACKPAD_TIMEOUT
#    define NAVIGATOR_TRACKPAD_TIMEOUT 100
#endif

// Packet size (matches NAV_TRACKPAD_PACKET_LEN from the core header).
#define CGEN6_MAX_PACKET_SIZE 17

// C3 error codes when reading memory
#define CGEN6_SUCCESS      0x00
#define CGEN6_CKSUM_FAILED 0x01
#define CGEN6_LEN_MISMATCH 0x02
#define CGEN6_I2C_FAILED   0x03

// C3 register addresses
#define CGEN6_REG_BASE     0x20000800
#define CGEN6_HARDWARE_ID  (CGEN6_REG_BASE + 0x08)
#define CGEN6_FIRMWARE_ID  (CGEN6_REG_BASE + 0x09)
#define CGEN6_FIRMWARE_REV (CGEN6_REG_BASE + 0x10)
#define CGEN6_VENDOR_ID    (CGEN6_REG_BASE + 0x0A)
#define CGEN6_PRODUCT_ID   (CGEN6_REG_BASE + 0x0C)
#define CGEN6_VERSION_ID   (CGEN6_REG_BASE + 0x0E)
#define CGEN6_FEED_CONFIG4 0x200E000B
#define CGEN6_FEED_CONFIG3 0x200E000A
#define CGEN6_SYS_CONFIG1  0x20000008
#define CGEN6_XY_CONFIG    0x20080018
#define CGEN6_SFR_BASE     0x40000008
#define CGEN6_GPIO_BASE    0x00052000
#define CGEN6_I2C_DR       0x61010000

// Shared file-scope state with the rest of the wired shim.
bool trackpad_init = false;

// Low-level I2C read of the next pending HID-style packet.
static i2c_status_t cirque_gen6_read_report(uint8_t *data, uint16_t cnt) {
    i2c_status_t res = i2c_receive(NAVIGATOR_TRACKPAD_ADDRESS, data, cnt, NAVIGATOR_TRACKPAD_TIMEOUT);
    if (res != I2C_STATUS_SUCCESS) {
        return res;
    }
    wait_us(cnt * 15);
    return res;
}

static void cirque_gen6_clear(void) {
    uint8_t buf[CGEN6_MAX_PACKET_SIZE];
    for (uint8_t i = 0; i < 5; i++) {
        wait_ms(1);
        if (cirque_gen6_read_report(buf, CGEN6_MAX_PACKET_SIZE) != I2C_STATUS_SUCCESS) {
            break;
        }
    }
}

static uint8_t cirque_gen6_read_memory(uint32_t addr, uint8_t *data, uint16_t cnt, bool fast_read) {
    uint8_t  cksum = 0;
    uint8_t  res   = CGEN6_SUCCESS;
    uint8_t  len[2];
    uint16_t read = 0;

    uint8_t preamble[8] = {0x01, 0x09, (uint8_t)(addr & (uint32_t)0x000000FF), (uint8_t)((addr & 0x0000FF00) >> 8), (uint8_t)((addr & 0x00FF0000) >> 16), (uint8_t)((addr & 0xFF000000) >> 24), (uint8_t)(cnt & 0x00FF), (uint8_t)((cnt & 0xFF00) >> 8)};

    // Read the length of the data + 3 bytes (first 2 bytes for the length and the last byte for the checksum)
    // Create a buffer to store the data
    uint8_t buf[cnt + 3];
    if (i2c_transmit_and_receive(NAVIGATOR_TRACKPAD_ADDRESS, preamble, 8, buf, cnt + 3, NAVIGATOR_TRACKPAD_TIMEOUT) != I2C_STATUS_SUCCESS) {
        res |= CGEN6_I2C_FAILED;
        trackpad_init = false;
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

static uint8_t cirque_gen6_write_memory(uint32_t addr, uint8_t *data, uint16_t cnt) {
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

    if (i2c_transmit(NAVIGATOR_TRACKPAD_ADDRESS, buf, cnt + 9, NAVIGATOR_TRACKPAD_TIMEOUT) != I2C_STATUS_SUCCESS) {
        res |= CGEN6_I2C_FAILED;
        trackpad_init = false;
    }

    wait_ms(1);

    return res;
}

// Register access functions
static uint8_t cirque_gen6_read_reg(uint32_t addr, bool fast_read) {
    uint8_t data;
    uint8_t res = cirque_gen6_read_memory(addr, &data, 1, fast_read);
    if (res != CGEN6_SUCCESS) {
        printf("Failed to read 8bits from register at address 0x%08X with error 0x%02X\n", (u_int)addr, res);
        return 0;
    }
    return data;
}

static uint8_t cirque_gen6_write_reg(uint32_t addr, uint8_t data) {
    return cirque_gen6_write_memory(addr, &data, 1);
}

// Configuration functions
static uint8_t cirque_gen6_set_ptp_mode(void) {
    uint8_t feed_config4 = cirque_gen6_read_reg(CGEN6_FEED_CONFIG4, false);
    feed_config4 &= 0xF7;
    feed_config4 |= 0x04;
    return cirque_gen6_write_reg(CGEN6_FEED_CONFIG4, feed_config4);
}

static uint8_t cirque_gen6_swap_xy(bool set) {
    uint8_t xy_config = cirque_gen6_read_reg(CGEN6_XY_CONFIG, false);
    if (set) {
        xy_config |= 0x04;
    } else {
        xy_config &= ~0x04;
    }
    return cirque_gen6_write_reg(CGEN6_XY_CONFIG, xy_config);
}

static uint8_t cirque_gen6_invert_y(bool set) {
    uint8_t xy_config = cirque_gen6_read_reg(CGEN6_XY_CONFIG, false);
    if (set) {
        xy_config |= 0x02;
    } else {
        xy_config &= ~0x02;
    }
    return cirque_gen6_write_reg(CGEN6_XY_CONFIG, xy_config);
}

static uint8_t cirque_gen6_invert_x(bool set) {
    uint8_t xy_config = cirque_gen6_read_reg(CGEN6_XY_CONFIG, false);
    if (set) {
        xy_config |= 0x01;
    } else {
        xy_config &= ~0x01;
    }
    return cirque_gen6_write_reg(CGEN6_XY_CONFIG, xy_config);
}

static uint8_t cirque_gen6_enable_logical_scaling(bool set) {
    uint8_t xy_config = cirque_gen6_read_reg(CGEN6_XY_CONFIG, false);
    if (set) {
        xy_config &= ~0x08;
    } else {
        xy_config |= 0x08;
    }
    return cirque_gen6_write_reg(CGEN6_XY_CONFIG, xy_config);
}

// Public: read one raw 17-byte packet from the chip. Decode happens in core.
bool navigator_trackpad_i2c_read_packet(uint8_t out[NAV_TRACKPAD_PACKET_LEN]) {
    if (cirque_gen6_read_report(out, CGEN6_MAX_PACKET_SIZE) != I2C_STATUS_SUCCESS) {
        trackpad_init = false;
        return false;
    }
    return true;
}

// Device initialization
void navigator_trackpad_device_init(void) {
    i2c_init();
    i2c_status_t status = i2c_ping_address(NAVIGATOR_TRACKPAD_ADDRESS, NAVIGATOR_TRACKPAD_TIMEOUT);
    if (status != I2C_STATUS_SUCCESS) {
        trackpad_init = false;
        return;
    }
    cirque_gen6_clear();
    wait_ms(50);

    // Dump sensor info to the console if needed, just set NAVIGATOR_TRACKPAD_DEBUG to 1 in your config.h
 #if defined(NAVIGATOR_TRACKPAD_DEBUG)
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
#endif

    uint8_t res = CGEN6_SUCCESS;
    res = cirque_gen6_set_ptp_mode();

    if (res != CGEN6_SUCCESS) {
        trackpad_init = false;
        return;
    }

    cirque_gen6_swap_xy(true);
    cirque_gen6_invert_x(true);
    cirque_gen6_invert_y(true);
    cirque_gen6_enable_logical_scaling(false);  // Disable scaling for raw coordinates

    trackpad_init = true;
}
