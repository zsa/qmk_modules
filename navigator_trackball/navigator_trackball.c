// Copyright 2025 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

// This is the QMK driver for the Navigator Trackball. It is comprised of two ICs:
// 1. The sci18is606 is a i2c to spi bridge that converts the i2c protocol to the spi protocol. It allows the trackball to
// be plugged using the TRRS jack used by ZSA keyboards or any other split keyboard.
// 2. The paw3805ek is a high-speed motion detection sensor. It is used to detect the motion of the trackball.

#include "i2c_master.h"
#include "navigator_trackball.h"
#include "busdev.h"
#include <stdint.h>
#include <stdio.h>
#include "quantum.h"

static uint8_t current_cpi = NAVIGATOR_TRACKBALL_CPI;

uint8_t has_motion = 0;

deferred_token callback_token = 0;

static busdev_id_t nav_dev = BUSDEV_NONE;
// Address the supervisor is currently using for this device: the real bridge
// (0x50, left-plugged) or the left-nRF wireless mirror (0x5A, right-plugged).
static uint8_t nav_addr = NAVIGATOR_TRACKBALL_ADDRESS;

// The sequence of commands to configure and boot the paw3805ek sensor.
paw3805ek_reg_seq_t paw3805ek_configure_seq[] = {
    {0x06, 0x80},                 // Software reset
    {0x00, 0x00},                 // Request the sensor ID
    {0x09 | WRITE_REG_BIT, 0x5A},                                    // Disable the write protection
    {PAW3805EK_REG_ORIENTATION | WRITE_REG_BIT, PAW3805EK_ORIENTATION}, // Sensor orientation and motion data length
    //{0x26 | WRITE_REG_BIT, 0x10}, // Enable burst mode
    {0x09 | WRITE_REG_BIT, 0x00}, // Enable the write protection
};

#define PAW3805EK_SEQ_LEN (sizeof(paw3805ek_configure_seq) / sizeof(paw3805ek_reg_seq_t))
// Index in the sequence whose response carries the sensor ID.
#define PAW3805EK_ID_STEP 1

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
    if (read && status == I2C_STATUS_SUCCESS) {
        status = sci18is606_read(data, length);
    }
    if (status != I2C_STATUS_SUCCESS) {
        busdev_report_error(nav_dev, status);
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

// Presence test: the bridge SPI configuration write, which is also the first
// thing the device needs. A NAK means no bridge on that address.
static i2c_status_t nav_probe(uint8_t addr) {
    nav_addr = addr;

    uint8_t spi_conf[2] = {SCI18IS606_CONF_SPI, SCI18IS606_CONF};
    return i2c_transmit(addr, spi_conf, 2, NAVIGATOR_TRACKBALL_PROBE_TIMEOUT);
}

// Bridge SPI configuration, then one paw3805ek configuration entry per step,
// then the CPI restore. Self-sufficient: reached without a probe after a verify
// mismatch or a suspect.
// The mirror relays motion and CPI only; the sensor behind it is configured by
// the half it is plugged into, and it answers no other register.
static bool nav_on_mirror(uint8_t addr) {
    return addr != NAVIGATOR_TRACKBALL_ADDRESS;
}

static busdev_step_t nav_init_step(uint8_t addr, uint8_t step, uint16_t *delay_ms) {
    nav_addr = addr;

    if (nav_on_mirror(addr)) {
        return paw3805ek_set_cpi() ? BUSDEV_STEP_DONE : BUSDEV_STEP_FAIL;
    }

    if (step == 0) {
        uint8_t spi_conf[2] = {SCI18IS606_CONF_SPI, SCI18IS606_CONF};
        if (i2c_transmit(addr, spi_conf, 2, NAVIGATOR_TRACKBALL_TIMEOUT) != I2C_STATUS_SUCCESS) {
            return BUSDEV_STEP_FAIL;
        }
        *delay_ms = 10;
        return BUSDEV_STEP_MORE;
    }

    uint8_t seq = step - 1;
    if (seq < PAW3805EK_SEQ_LEN) {
        uint8_t buf[3];
        buf[0] = NCS_PIN;
        buf[1] = paw3805ek_configure_seq[seq].reg;
        buf[2] = paw3805ek_configure_seq[seq].data;
        if (sci18is606_spi_tx(buf, 3, true) != I2C_STATUS_SUCCESS) {
            return BUSDEV_STEP_FAIL;
        }
        // Validate the spi link with the sensor ID after the software reset.
        if (seq == PAW3805EK_ID_STEP && buf[1] != PAW3805EK_ID) {
            return BUSDEV_STEP_FAIL;
        }
        // Let the sensor restart after the software reset cmd.
        *delay_ms = 1;
        return BUSDEV_STEP_MORE;
    }

    if (!paw3805ek_set_cpi()) {
        return BUSDEV_STEP_FAIL;
    }
    return BUSDEV_STEP_DONE;
}

// Reads the sensor product ID through the bridge, exercising the whole path,
// then the orientation register, which a reset sensor no longer holds.
static bool nav_verify(uint8_t addr) {
    nav_addr = addr;

    if (nav_on_mirror(addr)) {
        return true;
    }

    uint8_t id[3] = {NCS_PIN, 0x00, 0x00};
    if (sci18is606_spi_tx(id, 3, true) != I2C_STATUS_SUCCESS) {
        return false;
    }
    if (id[1] != PAW3805EK_ID) {
        return false;
    }

    uint8_t orientation[3] = {NCS_PIN, PAW3805EK_REG_ORIENTATION, 0x00};
    if (sci18is606_spi_tx(orientation, 3, true) != I2C_STATUS_SUCCESS) {
        return false;
    }
    return orientation[1] == PAW3805EK_ORIENTATION;
}

static const busdev_desc_t nav_desc = {
    .name               = "nav_trackball",
    .addr               = NAVIGATOR_TRACKBALL_ADDRESS,
    .addr_alt           = NAVIGATOR_TRACKBALL_MIRROR_ADDRESS,
    .settle_ms          = 10,
    .verify_interval_ms = 500,
    .probe_timeout_ms   = NAVIGATOR_TRACKBALL_PROBE_TIMEOUT,
    .max_err_streak     = 1,
    .probe              = nav_probe,
    .init_step          = nav_init_step,
    .verify             = nav_verify,
    .on_state           = NULL,
};

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
    uint8_t delta_x_l[3] = {0x01, 0x03, 0x00};
    if (sci18is606_spi_tx(delta_x_l, 3, true) != I2C_STATUS_SUCCESS) {
        return;
    }

    uint8_t delta_y_l[3] = {0x01, 0x04, 0x00};
    if (sci18is606_spi_tx(delta_y_l, 3, true) != I2C_STATUS_SUCCESS) {
        return;
    }

    uint8_t delta_x_h[3] = {0x01, 0x11, 0x00};
    if (sci18is606_spi_tx(delta_x_h, 3, true) != I2C_STATUS_SUCCESS) {
        return;
    }

    uint8_t delta_y_h[3] = {0x01, 0x12, 0x00};
    if (sci18is606_spi_tx(delta_y_h, 3, true) != I2C_STATUS_SUCCESS) {
        return;
    }

    mouse_report->x = (int16_t)((delta_x_h[1] << 8) | delta_x_l[1]);
    mouse_report->y = (int16_t)((delta_y_h[1] << 8) | delta_y_l[1]);
#else
    uint8_t delta_x[3] = {0x01, 0x03, 0x00};
    if (sci18is606_spi_tx(delta_x, 3, true) != I2C_STATUS_SUCCESS) {
        return;
    }

    uint8_t delta_y[3] = {0x01, 0x04, 0x00};
    if (sci18is606_spi_tx(delta_y, 3, true) != I2C_STATUS_SUCCESS) {
        return;
    }

    mouse_report->x = delta_x[1];
    mouse_report->y = delta_y[1];
#endif
}

// Deffered execution callback that periodically checks for motion.
uint32_t sci18is606_read_callback(uint32_t trigger_time, void *cb_arg) {
    if (busdev_is_up(nav_dev) && paw3805ek_has_motion()) {
        has_motion = 1;
    }
    return NAVIGATOR_TRACKBALL_READ;
}

#ifdef COMMUNITY_MODULE_WIRELESS_ENABLE
#    include "wireless.h"

// Strong override of the wireless module's weak probe hook, so telemetry can
// report what the last probe actually found rather than what this build can
// drive. The latched address says which of the two answered, which is as close
// to a side-of-the-board signal as the STM32 gets.
uint8_t zsa_nav_trackball_probe_state(void) {
    if (!busdev_is_up(nav_dev)) {
        return ZSA_NAV_PROBE_NONE;
    }
    return (busdev_addr(nav_dev) == NAVIGATOR_TRACKBALL_ADDRESS) ? ZSA_NAV_PROBE_LOCAL : ZSA_NAV_PROBE_MIRROR;
}

// Left nav probe window: the real bridge address, next. Up on the mirror, which
// answers with or without a ball on the right, the real address is tried
// directly and the device moves there only if the bridge answers.
void zsa_nav_trackball_probe_local(void) {
    if (busdev_is_up(nav_dev) && busdev_addr(nav_dev) != NAVIGATOR_TRACKBALL_ADDRESS) {
        uint8_t spi_conf[2] = {SCI18IS606_CONF_SPI, SCI18IS606_CONF};
        if (i2c_transmit(NAVIGATOR_TRACKBALL_ADDRESS, spi_conf, 2, NAVIGATOR_TRACKBALL_PROBE_TIMEOUT) != I2C_STATUS_SUCCESS) {
            return;
        }
        busdev_force_down(nav_dev, BUSDEV_REASON_PROBE_ACK);
    }
    busdev_probe_now(nav_dev, NAVIGATOR_TRACKBALL_ADDRESS);
}

// The real bridge answered and is being brought up.
bool zsa_nav_trackball_local_pending(void) {
    busdev_state_t state = busdev_state(nav_dev);
    return state != BUSDEV_ABSENT && state != BUSDEV_UP && busdev_addr(nav_dev) == NAVIGATOR_TRACKBALL_ADDRESS;
}
#endif

// The bridge keeps ACKing across a transport switch, so only an explicit
// take-down makes the supervisor re-probe both addresses.
void zsa_nav_transport_changed(void) {
    busdev_force_down(nav_dev, BUSDEV_REASON_TRANSPORT_SWITCH);
}

// Override the weak custom driver functions
void pointing_device_driver_init(void) {
    i2c_init();

    if (nav_dev == BUSDEV_NONE) {
        nav_dev = busdev_register(&nav_desc);
    }

    if (!callback_token) {
        callback_token = defer_exec(NAVIGATOR_TRACKBALL_READ, sci18is606_read_callback, NULL);
    }
}

report_mouse_t pointing_device_driver_get_report(report_mouse_t mouse_report) {
    if (!busdev_is_up(nav_dev)) {
        return mouse_report;
    }

    if (has_motion) {
        has_motion = 0;
        paw3804ek_read_motion(&mouse_report);
    }
    return mouse_report;
}

uint16_t pointing_device_driver_get_cpi(void) {
    return current_cpi;
}

void pointing_device_driver_set_cpi(uint16_t cpi) {
    if (cpi == 0) { // Decrease one tick
        if (current_cpi > NAVIGATOR_TRACKBALL_CPI_TICK) {
            current_cpi -= NAVIGATOR_TRACKBALL_CPI_TICK;
        } else {
            return;
        }
    } else {
        if (current_cpi <= NAVIGATOR_TRACKBALL_CPI_MAX - NAVIGATOR_TRACKBALL_CPI_TICK) {
            current_cpi += NAVIGATOR_TRACKBALL_CPI_TICK;
        } else {
            return;
        }
    }

    if (busdev_is_up(nav_dev)) {
        paw3805ek_set_cpi();
    }
}
