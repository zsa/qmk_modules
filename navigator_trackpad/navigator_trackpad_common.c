// Copyright 2025 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

// Common hardware layer for Navigator trackpad
// Shared by both mouse and PTP implementations

#include <stdint.h>
#include <sys/types.h>
#include "navigator_trackpad_common.h"
#include "i2c_master.h"
#include "quantum.h"
#include "timer.h"

// Shared globals
uint16_t       current_cpi   = DEFAULT_CPI_TICK;
bool           trackpad_init = false;

// I2C communication functions
i2c_status_t cirque_gen6_read_report(uint8_t *data, uint16_t cnt) {
    i2c_status_t res = i2c_receive(NAVIGATOR_TRACKPAD_ADDRESS, data, cnt, NAVIGATOR_TRACKPAD_TIMEOUT);
    if (res != I2C_STATUS_SUCCESS) {
        return res;
    }
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

    if (i2c_transmit(NAVIGATOR_TRACKPAD_ADDRESS, buf, cnt + 9, NAVIGATOR_TRACKPAD_TIMEOUT) != I2C_STATUS_SUCCESS) {
        res |= CGEN6_I2C_FAILED;
        trackpad_init = false;
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
        trackpad_init = false;
        return false;
    }
    return data != 0;
}

// Report reading - fills provided report struct. Returns true on valid data, false on I2C failure or no data.
bool cirque_gen_6_read_report(cgen6_report_t *report) {
    uint8_t packet[CGEN6_MAX_PACKET_SIZE];
    if (cirque_gen6_read_report(packet, CGEN6_MAX_PACKET_SIZE) != I2C_STATUS_SUCCESS) {
        trackpad_init = false;
        return false;
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

        return true;
    }
    // Mouse/relative mode report
    else if (report_id == CGEN6_MOUSE_REPORT_ID) {
        report->buttons     = packet[3];
        report->xDelta      = packet[4];
        report->yDelta      = packet[5];
        report->scrollDelta = packet[6];
        report->panDelta    = packet[7];
        return true;
    }

    // Unknown or empty report - no valid data
    return false;
}

// Device initialization - returns true on success, false on failure
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

void restore_cpi(uint8_t cpi) {
    current_cpi = cpi;
    set_cirque_cpi();
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
