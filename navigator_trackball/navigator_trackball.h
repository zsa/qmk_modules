// Copyright 2025 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "pointing_device.h"

#ifndef NAVIGATOR_TRACKBALL_ADDRESS
#    define NAVIGATOR_TRACKBALL_ADDRESS 0x50
#endif

// Real bridge = 0x28<<1 (left-plugged, direct SCI18IS606 on the STM32's bus).
// Left-nRF wireless mirror = 0x2C<<1 (right-plugged, wireless; the left nRF
// emulates a register-compatible mirror of the bridge at this address).
#ifndef NAVIGATOR_TRACKBALL_MIRROR_ADDRESS
#    define NAVIGATOR_TRACKBALL_MIRROR_ADDRESS 0x58
#endif

#ifndef NAVIGATOR_TRACKBALL_CPI
#    define NAVIGATOR_TRACKBALL_CPI 40
#endif

#ifndef NAVIGATOR_TRACKBALL_CPI_TICK
#    define NAVIGATOR_TRACKBALL_CPI_TICK 5
#endif

#define NAVIGATOR_TRACKBALL_CPI_MAX 125

#ifndef NAVIGATOR_TRACKBALL_TIMEOUT
#    define NAVIGATOR_TRACKBALL_TIMEOUT 100
#endif

#define NAVIGATOR_TRACKBALL_READ 7
#define NAVIGATOR_TRACKBALL_PROBE 1000

#define NCS_PIN 0x01
#define PAW3805EK_ID 0x31

#define SCI18IS606_CONF 0xDC //00001110b // MSB first, Mode 3, 155kHz

#define SCI18IS606_RW_SPI 0x00
#define SCI18IS606_CONF_SPI 0xF0
#define SCI18IS606_CLR_INT 0xF1
#define SCI18IS606_GET_ID 0xFE

#define WRITE_REG_BIT 0x80

typedef struct {
    uint8_t reg;
    uint8_t data;
} paw3805ek_reg_seq_t;
