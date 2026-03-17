#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "zsa_uart_proto.h"

/* Cached nRF state, updated via UART events */
bool    zsa_wireless_ble_connected(void);
uint8_t zsa_wireless_battery_soc(void);
uint8_t zsa_wireless_battery_flags(void);

/* Send a query/command to the nRF */
void zsa_wireless_query_ble_status(void);
void zsa_wireless_query_battery(void);
void zsa_wireless_cmd_hibernate(void);
void zsa_wireless_cmd_pair(void);
