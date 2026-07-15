// Copyright 2025 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

// This is the QMK driver for the Navigator Trackball. It is comprised of two ICs:
// 1. The sci18is606 is a i2c to spi bridge that converts the i2c protocol to the spi protocol. It allows the trackball to
// be plugged using the TRRS jack used by ZSA keyboards or any other split keyboard.
// 2. The paw3805ek is a high-speed motion detection sensor. It is used to detect the motion of the trackball.

#include "i2c_master.h"
#include "navigator_trackball.h"
#include <stdint.h>
#include <stdio.h>
#include "quantum.h"

uint8_t current_cpi = NAVIGATOR_TRACKBALL_CPI;

uint8_t has_motion = 0;

/* BLE-NUS logger (viewed with ble_log.py). Provided by modules/zsa/wireless/
 * log_tap.c when ZSA_BLE_LOG is set; no-op stub otherwise. Declared locally to
 * avoid depending on the wireless module's include path. */
#ifdef ZSA_BLE_LOG
void zsa_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
#else
static inline void zsa_log(const char *fmt, ...) { (void)fmt; }
#endif

uint8_t trackball_init = 0;

deferred_token callback_token = 0;

// The address currently in use for the bridge chip: real bridge (0x50) or
// left-nRF wireless mirror (0x58). Selected by the dual probe in
// pointing_device_driver_init() and re-selected on every re-probe, so runtime
// side swaps (trackball moved between halves) are handled transparently.
static uint8_t nav_addr = NAVIGATOR_TRACKBALL_ADDRESS;

// The sequence of commands to configure and boot the paw3805ek sensor.
paw3805ek_reg_seq_t paw3805ek_configure_seq[] = {
    {0x06, 0x80},                 // Software reset
    {0x00, 0x00},                 // Request the sensor ID
    {0x09 | WRITE_REG_BIT, 0x5A}, // Disable the write protection
#ifdef MOUSE_EXTENDED_REPORT
    {0x19 | WRITE_REG_BIT, 0x30}, // Set the sensor orientation, set motion data length to 16 bits
#else
    {0x19 | WRITE_REG_BIT, 0x34}, // Set the sensor orientation, set motion data length to 8 bits
#endif
    //{0x26 | WRITE_REG_BIT, 0x10}, // Enable burst mode
    {0x09 | WRITE_REG_BIT, 0x00}, // Enable the write protection
};

// A wrapper function for i2c_transmit that adds the address of the bridge chip to the data.
i2c_status_t sci18is606_write(uint8_t *data, uint8_t length) {
    return i2c_transmit(nav_addr, data, length, NAVIGATOR_TRACKBALL_TIMEOUT);
}

// A wrapper function for i2c_receive that adds the address of the bridge chip to the data.
i2c_status_t sci18is606_read(uint8_t *data, uint8_t length) {
    return i2c_receive(nav_addr, data, length, NAVIGATOR_TRACKBALL_TIMEOUT);
}

// A wrapper function that allows to write and optionally read from the bridge chip.
i2c_status_t sci18is606_spi_tx(uint8_t *data, uint8_t length, bool read) {
    i2c_status_t status = sci18is606_write(data, length);
    wait_us(length * 15);
    // Read the SPI response if the command expects it
    if (read) {
        status = sci18is606_read(data, length);
    }
    if (status != I2C_STATUS_SUCCESS) {
        trackball_init = 0;
    }
    return status;
}

// Configure the bridge chip to enable SPI mode.
i2c_status_t sci18is606_configure(void) {
    uint8_t      spi_conf[2] = {SCI18IS606_CONF_SPI, SCI18IS606_CONF};
    i2c_status_t status      = sci18is606_write(spi_conf, 2);
    wait_ms(10);
    if (status != I2C_STATUS_SUCCESS) {
        trackball_init = 0;
    }
    return status;
}

bool paw3805ek_set_cpi(void) {

    paw3805ek_reg_seq_t cpi_reg_seq[] = {
        {0x09 | WRITE_REG_BIT, 0x5A}, // Disable write protection
        {0x0D | WRITE_REG_BIT, current_cpi},
        {0x0E | WRITE_REG_BIT, current_cpi},
        {0x09 | WRITE_REG_BIT, 0x00}, // Enable the write protection
    };

    // Run the spi sequence to configure the cpi.
    for (uint8_t i = 0; i < sizeof(cpi_reg_seq) / sizeof(paw3805ek_reg_seq_t); i++) {
        uint8_t buf[3];
        buf[0] = NCS_PIN;
        buf[1] = cpi_reg_seq[i].reg;
        buf[2] = cpi_reg_seq[i].data;
        if (sci18is606_spi_tx(buf, 3, true) != I2C_STATUS_SUCCESS) {
            return false;
        }
    }

    return true;
}

// Run the paw3805ek configuration sequence.
bool paw3805ek_configure(void) {
    for (uint8_t i = 0; i < sizeof(paw3805ek_configure_seq) / sizeof(paw3805ek_reg_seq_t); i++) {
        uint8_t buf[3];
        buf[0] = NCS_PIN;
        buf[1] = paw3805ek_configure_seq[i].reg;
        buf[2] = paw3805ek_configure_seq[i].data;
        if (sci18is606_spi_tx(buf, 3, true) != I2C_STATUS_SUCCESS) {
            zsa_log("[TB] paw3805ek cfg step %u SPI tx FAILED", i);
            return false;
        }
        // Wait for the sensor to restart after the software reset cmd
        wait_ms(1);

        // Check the sensor ID to validate the spi link after the reset
        if (i == 1) {
            zsa_log("[TB] paw3805ek ID read=0x%02X (expect 0x%02X)", buf[1], PAW3805EK_ID);
            if (buf[1] != PAW3805EK_ID) {
                return false;
            }
        }
    }

    return true;
}

// Assert the CS pin to read the motion register.
bool paw3805ek_has_motion(void) {
    uint8_t motion[3] = {0x01, 0x02, 0x00};
    if (sci18is606_spi_tx(motion, 3, true) != I2C_STATUS_SUCCESS) {
        return false;
    }
    return motion[1] & 0x80;
}

// Read the motion data from the paw3805ek sensor.
void paw3804ek_read_motion(report_mouse_t *mouse_report) {
#ifdef MOUSE_EXTENDED_REPORT
    uint8_t delta_x_l[2] = {0x01, 0x03};
    if (sci18is606_spi_tx(delta_x_l, 3, true) != I2C_STATUS_SUCCESS) {
        return;
    }

    uint8_t delta_y_l[2] = {0x01, 0x04};
    if (sci18is606_spi_tx(delta_y_l, 3, true) != I2C_STATUS_SUCCESS) {
        return;
    }

    uint8_t delta_x_h[2] = {0x01, 0x11};
    if (sci18is606_spi_tx(delta_x_h, 3, true) != I2C_STATUS_SUCCESS) {
        return;
    }

    uint8_t delta_y_h[2] = {0x01, 0x12};
    if (sci18is606_spi_tx(delta_y_h, 3, true) != I2C_STATUS_SUCCESS) {
        return;
    }

    mouse_report->x = (int16_t)((delta_x_h[1] << 8) | delta_x_l[1]);
    mouse_report->y = (int16_t)((delta_y_h[1] << 8) | delta_y_l[1]);
#else
    uint8_t delta_x[2] = {0x01, 0x03};
    if (sci18is606_spi_tx(delta_x, 3, true) != I2C_STATUS_SUCCESS) {
        return;
    }

    uint8_t delta_y[2] = {0x01, 0x04};
    if (sci18is606_spi_tx(delta_y, 3, true) != I2C_STATUS_SUCCESS) {
        return;
    }

    mouse_report->x = delta_x[1];
    mouse_report->y = delta_y[1];
#endif
}

// Deffered execution callback that periodically checks for motion.
uint32_t sci18is606_read_callback(uint32_t trigger_time, void *cb_arg) {
    if (!trackball_init) {
        pointing_device_driver_init();
        return NAVIGATOR_TRACKBALL_PROBE;
    }
    bool m = paw3805ek_has_motion();
    if (m) {
        has_motion = 1;
    }
    /* Heartbeat: how often we poll the 0x28 mirror and how often it reports the
     * motion bit set. motion_bit_set staying 0 while you move => the mirror is
     * serving no motion (left accumulator empty => upstream: right/ESB/left).
     * motion_bit_set climbing => the mirror has motion; look downstream. */
    static uint32_t polls, motion_bits, hb_last;
    polls++;
    if (m) {
        motion_bits++;
    }
    if (timer_elapsed32(hb_last) >= 1000) {
        hb_last = timer_read32();
        zsa_log("[TB] poll hb: polls=%u motion_bit_set=%u", (unsigned)polls, (unsigned)motion_bits);
    }
    return NAVIGATOR_TRACKBALL_READ;
}

// Override the weak custom driver functions
void pointing_device_driver_init(void) {
    i2c_init();

    // Probe the real bridge first (0x50, left-plugged); if it doesn't ACK,
    // fall back to the left-nRF wireless mirror (0x58, right-plugged). This
    // also re-runs on every re-probe (see sci18is606_read_callback), so a
    // trackball moved between halves at runtime is picked back up.
    nav_addr = NAVIGATOR_TRACKBALL_ADDRESS;
    i2c_status_t bridge = sci18is606_configure();
    if (bridge != I2C_STATUS_SUCCESS) {
        nav_addr = NAVIGATOR_TRACKBALL_MIRROR_ADDRESS;
        bridge = sci18is606_configure();
    }
    zsa_log("[TB] init: sci18is606_configure -> %d (nav_addr=0x%02X, %s)", (int)bridge, nav_addr,
            bridge == I2C_STATUS_SUCCESS ? "bridge OK" : "no bridge @0x50/@0x58");
    if (bridge == I2C_STATUS_SUCCESS) {
        bool cfg = paw3805ek_configure();
        zsa_log("[TB] init: paw3805ek_configure -> %d (%s)", (int)cfg,
                cfg ? "sensor OK" : "sensor init FAILED");
        trackball_init = 1;
        restore_cpi(current_cpi);
    }
    zsa_log("[TB] init: done, trackball_init=%d", trackball_init);

    // Always arm the periodic probe/read callback, even when the sensor did
    // not answer just now. On the wireless keyboard the responder at 0x28 is
    // the left nRF's emulated mirror, which only appears once a trackball is
    // actually plugged in — so a board that booted without one must keep
    // probing (NAVIGATOR_TRACKBALL_PROBE cadence, via the !trackball_init
    // branch of the callback) to pick it up on hot-plug. Without this, a
    // failed first init left no retry scheduled and the trackball stayed dead
    // until reboot.
    if (!callback_token) {
        callback_token = defer_exec(NAVIGATOR_TRACKBALL_READ, sci18is606_read_callback, NULL);
    }
}

report_mouse_t pointing_device_driver_get_report(report_mouse_t mouse_report) {
    if (!trackball_init) {
        return mouse_report;
    }

    if (has_motion) {
        has_motion = 0;
        paw3804ek_read_motion(&mouse_report);
        /* The actual deltas QMK reads from the mirror. Non-zero here means the
         * full path sensor->right->ESB->left accumulator->0x28 mirror->STM32
         * works and the cursor should move; if it still doesn't, the fault is in
         * QMK's pointing-device/report/HID layer, not the transport. */
        static uint32_t rd_last;
        if ((mouse_report.x != 0 || mouse_report.y != 0) &&
            timer_elapsed32(rd_last) >= 500) {
            rd_last = timer_read32();
            zsa_log("[TB] read: x=%d y=%d", (int)mouse_report.x, (int)mouse_report.y);
        }
    }
    return mouse_report;
}

uint16_t pointing_device_driver_get_cpi(void) {
    return current_cpi;
}

void restore_cpi(uint8_t cpi) {
    current_cpi = cpi;
    paw3805ek_set_cpi();
}

void pointing_device_driver_set_cpi(uint16_t cpi) {
    if (cpi == 0) { // Decrease one tick
        if (current_cpi > NAVIGATOR_TRACKBALL_CPI_TICK) {
            current_cpi -= NAVIGATOR_TRACKBALL_CPI_TICK;
            paw3805ek_set_cpi();
        }
    } else {
        if (current_cpi <= NAVIGATOR_TRACKBALL_CPI_MAX - NAVIGATOR_TRACKBALL_CPI_TICK) {
            current_cpi += NAVIGATOR_TRACKBALL_CPI_TICK;
            paw3805ek_set_cpi();
        }
    }
}
