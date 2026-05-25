// Copyright 2025 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "i2c_master.h"

// Polling intervals (in ms)
#define NAVIGATOR_TRACKPAD_POLL_INTERVAL_MS 5    // Minimum interval between sensor queries
#define NAVIGATOR_TRACKPAD_PROBE_INTERVAL_MS 1000 // Interval for probing disconnected device

#ifndef NAVIGATOR_TRACKPAD_ADDRESS
#    define NAVIGATOR_TRACKPAD_ADDRESS 0x58
#endif

#ifndef NAVIGATOR_TRACKPAD_TIMEOUT
#    define NAVIGATOR_TRACKPAD_TIMEOUT 100
#endif

// Packet and report IDs
#define CGEN6_MAX_PACKET_SIZE 17
#define CGEN6_PTP_REPORT_ID 0x01
#define CGEN6_MOUSE_REPORT_ID 0x06
#define CGEN6_ABSOLUTE_REPORT_ID 0x09

// C3 error codes when reading memory
#define CGEN6_SUCCESS 0x00
#define CGEN6_CKSUM_FAILED 0x01
#define CGEN6_LEN_MISMATCH 0x02
#define CGEN6_I2C_FAILED 0x03

// C3 register addresses
#define CGEN6_REG_BASE 0x20000800
#define CGEN6_HARDWARE_ID CGEN6_REG_BASE + 0x08
#define CGEN6_FIRMWARE_ID CGEN6_REG_BASE + 0x09
#define CGEN6_FIRMWARE_REV CGEN6_REG_BASE + 0x10
#define CGEN6_VENDOR_ID CGEN6_REG_BASE + 0x0A
#define CGEN6_PRODUCT_ID CGEN6_REG_BASE + 0x0C
#define CGEN6_VERSION_ID CGEN6_REG_BASE + 0x0E
#define CGEN6_FEED_CONFIG4 0x200E000B
#define CGEN6_FEED_CONFIG3 0x200E000A
#define CGEN6_SYS_CONFIG1 0x20000008
#define CGEN6_XY_CONFIG 0x20080018
#define CGEN6_SFR_BASE 0x40000008
#define CGEN6_GPIO_BASE 0x00052000
#define CGEN6_I2C_DR 0x61010000

// CPI configuration
#define CPI_TICKS 7
#define DEFAULT_CPI_TICK 4
#define CPI_1 200
#define CPI_2 400
#define CPI_3 800
#define CPI_4 1024
#define CPI_5 1400
#define CPI_6 1800
#define CPI_7 2048

// Physical trackpad dimensions (in 0.01 inch units for HID descriptor).
// Navigator pad is ~40mm / 1.57" actual. We report a larger area (~3.5x = 5.5")
// so the OS treats cursor movement as faster than the tiny surface alone implies.
// CAP: do NOT inflate beyond ~3.5x. Higher values push the two contacts of a
// two-finger tap past Windows' tap-separation threshold, which silently breaks
// two-finger-tap right-click. 550 (~3.5x) is the empirical max that keeps the
// gesture reliable at natural finger spacing while preserving cursor speed.
#ifndef TRACKPAD_PHYSICAL_WIDTH
#    define TRACKPAD_PHYSICAL_WIDTH 550   // ~3.5x of 157 (1.57"/40mm actual)
#endif
#ifndef TRACKPAD_PHYSICAL_HEIGHT
#    define TRACKPAD_PHYSICAL_HEIGHT 550  // ~3.5x of 157 (1.57"/40mm actual)
#endif

// New-name aliases consumed by digitizer.h parametric defines
#ifndef DIGITIZER_TOUCHPAD_PHYSICAL_WIDTH
#    define DIGITIZER_TOUCHPAD_PHYSICAL_WIDTH TRACKPAD_PHYSICAL_WIDTH
#endif
#ifndef DIGITIZER_TOUCHPAD_PHYSICAL_HEIGHT
#    define DIGITIZER_TOUCHPAD_PHYSICAL_HEIGHT TRACKPAD_PHYSICAL_HEIGHT
#endif

// Logical coordinate range for HID descriptor (what we report to the OS)
#define TRACKPAD_LOGICAL_MAX 2048

// Rotation of the reported orientation, in degrees clockwise. Applies to both
// PTP (absolute) and mouse-fallback (relative) modes. The touch surface is
// circular, so any angle rotates cleanly about the center without clipping.
// Defined here (a compiled-in header), not in the module config.h, because the
// module config.h is force-included BEFORE the keymap config.h -- defining it
// there would lock the default in first and make keymap overrides a -Werror
// redefinition. Defining it here lets a keymap's config.h #define win normally.
#ifndef NAVIGATOR_TRACKPAD_ROTATION
#    define NAVIGATOR_TRACKPAD_ROTATION 0
#endif

// Pivot for absolute (PTP) rotation, in logical coordinates. Default is the
// center of the logical box. Override if the circle's center is found to map to
// a different point during hardware testing.
#ifndef NAVIGATOR_TRACKPAD_CENTER_X
#    define NAVIGATOR_TRACKPAD_CENTER_X (TRACKPAD_LOGICAL_MAX / 2)
#endif
#ifndef NAVIGATOR_TRACKPAD_CENTER_Y
#    define NAVIGATOR_TRACKPAD_CENTER_Y (TRACKPAD_LOGICAL_MAX / 2)
#endif

// Sensor coordinate range (measured empirically from Cirque Gen6)
// These define the actual usable touch area of the sensor
#define SENSOR_X_MIN 281
#define SENSOR_X_MAX 2018
#define SENSOR_Y_MIN 276
#define SENSOR_Y_MAX 2018

// Fixed-point multipliers for coordinate scaling (Q16 format)
// Precomputed as: (TRACKPAD_LOGICAL_MAX << 16) / (SENSOR_MAX - SENSOR_MIN)
// This avoids expensive division at runtime
#define SENSOR_SCALE_X_MULT 77176  // 2048 * 65536 / (2018 - 281)
#define SENSOR_SCALE_Y_MULT 76957  // 2048 * 65536 / (2018 - 276)

// Common finger structure (used by both mouse and PTP modes)
typedef struct {
    uint8_t  tip;
    uint8_t  confidence;
    uint8_t  id;
    uint16_t x;
    uint16_t y;
} cgen6_finger_t;

// Common report structure (used by both modes)
typedef struct {
    cgen6_finger_t fingers[2];
    uint8_t        buttons;
    uint8_t        contact_count; // Number of active contacts - used by PTP mode
    uint16_t       scan_time;     // Sensor timestamp (100μs units) - used by PTP mode
    int8_t         xDelta;        // Used by mouse mode
    int8_t         yDelta;        // Used by mouse mode
    int8_t         scrollDelta;   // Used by mouse mode
    int8_t         panDelta;      // Used by mouse mode
} cgen6_report_t;

// Low-level I2C functions
i2c_status_t cirque_gen6_read_report(uint8_t *data, uint16_t cnt);
void         cirque_gen6_clear(void);
uint8_t      cirque_gen6_read_memory(uint32_t addr, uint8_t *data, uint16_t cnt, bool fast_read);
uint8_t      cirque_gen6_write_memory(uint32_t addr, uint8_t *data, uint16_t cnt);

// Register access functions
uint8_t  cirque_gen6_read_reg(uint32_t addr, bool fast_read);
uint16_t cirque_gen6_read_reg_16(uint32_t addr);
uint32_t cirque_gen6_read_reg_32(uint32_t addr);
uint8_t  cirque_gen6_write_reg(uint32_t addr, uint8_t data);
uint8_t  cirque_gen6_write_reg_16(uint32_t addr, uint16_t data);
uint8_t  cirque_gen6_write_reg_32(uint32_t addr, uint32_t data);

// Configuration functions
uint8_t cirque_gen6_set_relative_mode(void);
uint8_t cirque_gen6_set_ptp_mode(void);
uint8_t cirque_gen6_swap_xy(bool set);
uint8_t cirque_gen6_invert_y(bool set);
uint8_t cirque_gen6_invert_x(bool set);
uint8_t cirque_gen6_enable_logical_scaling(bool set);

// Motion detection and report reading
// Returns true if motion data is ready, false otherwise (including I2C failure)
bool cirque_gen6_has_motion(void);
// Reads report data into provided report struct. Returns true on success, false on I2C failure.
bool cirque_gen_6_read_report(cgen6_report_t *report);

// Device initialization
void navigator_trackpad_device_init(void);

// CPI management
uint16_t navigator_trackpad_get_cpi(void);
void     navigator_trackpad_set_cpi(uint16_t cpi);

// Shared globals
extern bool           trackpad_init;

// Helper functions
uint8_t cirque_gen6_finger_count(cgen6_report_t *report);
