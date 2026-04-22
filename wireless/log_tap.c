#ifdef ZSA_BLE_LOG

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "wireless.h"
#include "zsa_uart_proto.h"
#include "log_tap.h"

static char     line_buf[ZSA_LOG_MAX_TEXT];
static uint8_t  line_len;
/* Reserved for a future non-blocking TX path. `uart_transmit` is currently
 * blocking (ChibiOS chnWrite), so log spew backs up the UART queue rather
 * than drops. The flush-side marker emission is wired so that if/when a
 * producer bumps this counter, the user sees "log: dropped N lines". */
static uint32_t dropped_lines;

static void emit_line(uint8_t lvl, const char *text, uint16_t text_len) {
    uint8_t payload[ZSA_LOG_MAX_PAYLOAD];
    struct zsa_log_hdr hdr;
    zsa_log_hdr_pack(&hdr, ZSA_LOG_SRC_STM, lvl, 0u);
    memcpy(payload, &hdr, ZSA_LOG_HDR_LEN);

    uint16_t n = text_len > ZSA_LOG_MAX_TEXT ? ZSA_LOG_MAX_TEXT : text_len;
    memcpy(&payload[ZSA_LOG_HDR_LEN], text, n);

    zsa_wireless_send_frame(ZSA_MSG_LOG_LINE, payload,
                            (uint16_t)(ZSA_LOG_HDR_LEN + n));
}

static void flush(void) {
    if (line_len == 0) return;

    /* If the previous attempt dropped, prepend a marker first. */
    if (dropped_lines > 0) {
        char marker[64];
        int m = snprintf(marker, sizeof(marker),
                         "log: dropped %u lines", (unsigned)dropped_lines);
        if (m > 0) {
            uint16_t mlen = (m > (int)sizeof(marker) - 1)
                            ? (uint16_t)(sizeof(marker) - 1)
                            : (uint16_t)m;
            emit_line(ZSA_LOG_LVL_WRN, marker, mlen);
        }
        dropped_lines = 0;
    }

    emit_line(ZSA_LOG_LVL_INF, line_buf, line_len);
    line_len = 0;
}

/* QMK weak symbol — we override. */
int8_t sendchar(uint8_t c) {
    if (c == '\r') return 0;
    if (c == '\n') {
        flush();
        return 0;
    }
    if (line_len == sizeof(line_buf)) {
        flush();
    }
    line_buf[line_len++] = (char)c;
    return 0;
}

void zsa_log_tap_init(void) {
    line_len = 0;
    dropped_lines = 0;
}

#endif /* ZSA_BLE_LOG */
