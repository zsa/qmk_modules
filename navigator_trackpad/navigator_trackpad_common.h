// Copyright 2025 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "i2c_master.h"
#include "busdev.h"

// Polling intervals (in ms)
#define NAVIGATOR_TRACKPAD_POLL_INTERVAL_MS 5    // Minimum interval between sensor queries

#ifndef NAVIGATOR_TRACKPAD_ADDRESS
#    define NAVIGATOR_TRACKPAD_ADDRESS 0x58
#endif

// Real trackpad = 0x2C<<1 = 0x58 (left-plugged, direct Cirque Gen6 on the
// STM32's bus, same as NAVIGATOR_TRACKPAD_ADDRESS above). Left-nRF wireless
// mirror = 0x2E<<1 = 0x5C (right-plugged, wireless; the left nRF emulates a
// register-compatible mirror of the real trackpad at this address).
#ifndef NAVIGATOR_TRACKPAD_MIRROR_ADDRESS
#    define NAVIGATOR_TRACKPAD_MIRROR_ADDRESS 0x5C
#endif

#ifndef NAVIGATOR_TRACKPAD_TIMEOUT
#    define NAVIGATOR_TRACKPAD_TIMEOUT 100
#endif

#ifndef NAVIGATOR_TRACKPAD_PROBE_TIMEOUT
#    define NAVIGATOR_TRACKPAD_PROBE_TIMEOUT 5
#endif

// Consecutive failed transactions (NAK/error, not timeout) while up before the
// pad is dropped: about 4 polls, 20 ms at NAVIGATOR_TRACKPAD_POLL_INTERVAL_MS.
#ifndef NAVIGATOR_TRACKPAD_MAX_ERR_STREAK
#    define NAVIGATOR_TRACKPAD_MAX_ERR_STREAK 4
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

// C3 power management registers
#define CGEN6_POWER_HEADER 0x200A0000
#define CGEN6_POWER_IDLE_SLEEP_WAIT 0x200A0008 // 16-bit, 100 ms units
#define CGEN6_POWER_IDLE_SLEEP_TIME 0x200A000A // 16-bit, 100 us units
#define CGEN6_POWER_DEEP_SLEEP_WAIT 0x200A000C // 16-bit, 100 ms units
#define CGEN6_POWER_DEEP_SLEEP_TIME 0x200A000E // 16-bit, 100 us units
#define CGEN6_POWER_SETTINGS 0x200A0010
#define CGEN6_POWER_CMD 0x200A0408

// CGEN6_POWER_SETTINGS bits
#define CGEN6_POWER_SETTING_DISABLE_DEEP_SLEEP 0x01
#define CGEN6_POWER_SETTING_DISABLE_IDLE_SLEEP 0x02
#define CGEN6_POWER_SETTING_DONT_WAKE_FROM_TOUCH 0x04
#define CGEN6_POWER_SETTING_DONT_WAKE_FROM_BUTTON 0x08

// CGEN6_POWER_CMD bits
#define CGEN6_POWER_CMD_FORCE_SLEEP 0x01
#define CGEN6_POWER_CMD_CANCEL_FORCE_SLEEP 0x02

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
typedef enum {
    CGEN6_READ_OK,     // report filled
    CGEN6_READ_EMPTY,  // transaction ok, no touch packet
    CGEN6_READ_FAILED, // I2C transaction failed
} cgen6_read_t;

// Reads report data into provided report struct.
cgen6_read_t cirque_gen_6_read_report(cgen6_report_t *report);

// Device initialization
void navigator_trackpad_device_init(void);

#if defined(NAVIGATOR_TRACKPAD_DEBUG)
#    ifndef NAVIGATOR_TRACKPAD_DEBUG_DUMP_MS
#        define NAVIGATOR_TRACKPAD_DEBUG_DUMP_MS 10000
#    endif
// Repeats the register dump every NAVIGATOR_TRACKPAD_DEBUG_DUMP_MS while up on
// NAVIGATOR_TRACKPAD_ADDRESS.
void navigator_trackpad_debug_task(void);

// Power-measurement actions. Each prints its result to the console.
// Pause stops all pad traffic (reports, dump, verify, sleep poll) until resumed.
// Write actions return CGEN6_SUCCESS, CGEN6_* error bits or
// NAVIGATOR_TRACKPAD_DEBUG_NOT_UP.
#    define NAVIGATOR_TRACKPAD_DEBUG_NOT_UP 0x80     // not up on NAVIGATOR_TRACKPAD_ADDRESS, nothing written
#    define NAVIGATOR_TRACKPAD_DEBUG_BAD_ACTION 0x81 // unknown action, nothing done
#    define NAVIGATOR_TRACKPAD_DEBUG_BUSY 0x82       // timing cycle running or pad access paused
#    define NAVIGATOR_TRACKPAD_DEBUG_NO_LINK 0x83    // no nRF link, the boost claim cannot be sent
bool    navigator_trackpad_debug_paused(void);
void    navigator_trackpad_debug_set_pause(bool paused);
void    navigator_trackpad_debug_toggle_pause(void);
bool    navigator_trackpad_debug_pad_up(void);
uint8_t navigator_trackpad_debug_force_sleep(void);
uint8_t navigator_trackpad_debug_cancel_sleep(void);
// Idle wait 1 s.
uint8_t navigator_trackpad_debug_short_timers(void);
// Idle wait 100, idle time 300, deep wait 1200, deep time 1000, settings 0x00.
uint8_t navigator_trackpad_debug_factory_timers(void);

// Actions for navigator_trackpad_debug_action() (Oryx ORYX_CMD_DEBUG_PAD_POWER).
enum {
    NAVIGATOR_TRACKPAD_DEBUG_STATUS,
    NAVIGATOR_TRACKPAD_DEBUG_FORCE_SLEEP,
    NAVIGATOR_TRACKPAD_DEBUG_CANCEL_SLEEP,
    NAVIGATOR_TRACKPAD_DEBUG_PAUSE,
    NAVIGATOR_TRACKPAD_DEBUG_RESUME,
    NAVIGATOR_TRACKPAD_DEBUG_SHORT_TIMERS,
    NAVIGATOR_TRACKPAD_DEBUG_FACTORY_TIMERS,
    NAVIGATOR_TRACKPAD_DEBUG_BOOST_ON,     // left boost claim held on (wireless module)
    NAVIGATOR_TRACKPAD_DEBUG_BOOST_NORMAL, // left boost claim back to normal
    NAVIGATOR_TRACKPAD_DEBUG_BOOST_OFF,    // left boost claim held off
    NAVIGATOR_TRACKPAD_DEBUG_CYCLE_TIMING, // power cycle timing, arg = off time in 10 ms units (0 = 1 s)
    NAVIGATOR_TRACKPAD_DEBUG_TIMING_RESULT, // last power cycle timing, see below
};
// Left boost override (0 none, 1 held on, 2 held off), and the last claim sent
// to the nRF. 0 without the wireless module.
uint8_t navigator_trackpad_debug_boost_mode(void);
bool    navigator_trackpad_debug_boost_sent(void);
// Runs one action with its argument byte; returns its write status (0 for
// status, pause, resume, and a started timing cycle).
uint8_t navigator_trackpad_debug_action(uint8_t action, uint8_t arg);

#    if defined(COMMUNITY_MODULE_WIRELESS_ENABLE)
#        define NAVIGATOR_TRACKPAD_DEBUG_TIMING
// Power cycle timing (action 10): boost claim held off for the off time, then
// held on for NAVIGATOR_TRACKPAD_DEBUG_TIMING_WINDOW_MS, then back to normal.
// Times are ms after the claim-on was handed to the UART.
#        define NAVIGATOR_TRACKPAD_DEBUG_TIMING_WINDOW_MS 1500
#        define NAVIGATOR_TRACKPAD_DEBUG_TIMING_PING_MS 2
#        define NAVIGATOR_TRACKPAD_DEBUG_TIMING_NONE 0xFFFF // not seen
enum {
    NAVIGATOR_TRACKPAD_DEBUG_TIMING_IDLE,    // never run, or aborted by action 7, 8 or 9
    NAVIGATOR_TRACKPAD_DEBUG_TIMING_RUNNING,
    NAVIGATOR_TRACKPAD_DEBUG_TIMING_DONE,    // pad up on 0x58 within the window
    NAVIGATOR_TRACKPAD_DEBUG_TIMING_TIMEOUT, // pad not up, or a claim never sent
};
// Flags byte.
#        define NAVIGATOR_TRACKPAD_DEBUG_TIMING_FIRST_TIP 0x01  // finger 0 tip in the first PTP report
#        define NAVIGATOR_TRACKPAD_DEBUG_TIMING_ANY_TIP 0x02    // any finger tip in any PTP report of the window
#        define NAVIGATOR_TRACKPAD_DEBUG_TIMING_SUP_ACK 0x04    // first ACK came from a supervisor probe, not a ping
#        define NAVIGATOR_TRACKPAD_DEBUG_TIMING_NO_CLAIM 0x08   // a claim was not sent within 200 ms
#        define NAVIGATOR_TRACKPAD_DEBUG_TIMING_RAIL_FED 0x10   // VBUS or TRRS cable: the nRF keeps the rail as it is
#        define NAVIGATOR_TRACKPAD_DEBUG_TIMING_WAS_UP 0x20     // pad already set up on 0x58 at claim-on
#        define NAVIGATOR_TRACKPAD_DEBUG_TIMING_MOUSE 0x40      // a mouse-mode report was read in the window
// Action 11 reply, written from event byte 2 (event byte 0 0xF0, byte 1 11);
// u16 little-endian, ms after claim-on, 0xFFFF not seen:
//   2      state (NAVIGATOR_TRACKPAD_DEBUG_TIMING_*)
//   3-4    t_ack: first ACK at 0x58
//   5-6    t_up: BUSDEV_UP on 0x58
//   7-8    t_report: first PTP report read by the report path
//   9-10   init step 0 start (clear, then 50 ms delay)
//   11-12  init step 1 start (PTP mode)
//   13-14  init step 2 start (XY config)
//   15-16  init done (verify follows)
//   17     flags (NAVIGATOR_TRACKPAD_DEBUG_TIMING_*)
//   18     init runs started in the window
//   19     PTP reports read in the window (saturates)
//   20-21  off time: claim-off sent to claim-on sent
//   22-23  pings sent before the first ACK
// Init times are those of the last init run before t_up.
void navigator_trackpad_debug_timing_result(uint8_t *out);
void navigator_trackpad_debug_note_report(const cgen6_report_t *report, bool ptp);
#    endif
#endif

// CPI management
uint16_t navigator_trackpad_get_cpi(void);
void     navigator_trackpad_set_cpi(uint16_t cpi);

// Shared globals
extern busdev_id_t navigator_trackpad_dev;

// Helper functions
uint8_t cirque_gen6_finger_count(cgen6_report_t *report);
