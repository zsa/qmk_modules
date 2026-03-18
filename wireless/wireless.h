#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "zsa_uart_proto.h"

enum zsa_idle_step {
    ZSA_IDLE_ACTIVE    = 0,
    ZSA_IDLE_DIM       = 1,
    ZSA_IDLE_LEDS_OFF  = 2,
    ZSA_IDLE_SLOW_SCAN = 3,
    ZSA_IDLE_DEEP      = 4,
};

/* Cached nRF state, updated via UART events */
bool    zsa_wireless_ble_connected(void);
uint8_t zsa_wireless_battery_soc(void);
uint8_t zsa_wireless_battery_flags(void);

/* Send a query/command to the nRF */
void zsa_wireless_query_ble_status(void);
void zsa_wireless_query_battery(void);
void zsa_wireless_cmd_hibernate(void);
void zsa_wireless_cmd_pair(void);

/* Idle power management */
enum zsa_idle_step zsa_wireless_idle_step(void);
