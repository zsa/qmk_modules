/* TEMP DEBUG (ZSA_RTT_DEBUG): see rtt_debug.h. */
#ifdef ZSA_RTT_DEBUG

#include <stdbool.h>
#include <hal.h>
#include "rtt_debug.h"
#include "SEGGER_RTT.h"
#include "timer.h"
#include "zsa_uart_proto.h"

/* DWT cycle counter at SYSCLK. Deltas are computed on raw cycle
 * counts (wrap-safe for intervals < ~59 s) and converted to µs at the end. */
#define REG32(a)   (*(volatile uint32_t *)(a))
#define DEMCR      REG32(0xE000EDFCu)
#define DWT_CTRL   REG32(0xE0001000u)
#define DWT_CYCCNT REG32(0xE0001004u)
#define CPU_MHZ    (STM32_SYSCLK / 1000000u)

uint32_t zsa_rtt_cycles(void) {
    return DWT_CYCCNT;
}

static inline uint32_t cyc_to_us(uint32_t cycles) {
    return cycles / CPU_MHZ;
}

static struct {
    uint32_t loops;
    uint32_t dt_max_us;
    /* UART TX */
    uint32_t tx_mouse, tx_digi, tx_log, tx_other, tx_bytes;
    uint32_t tx_blk_max_us, tx_blk_sum_us;
    /* UART RX */
    uint32_t rx_pair, rx_ble, rx_rail, rx_dfu, rx_other;
    /* rgb driver flushes */
    uint32_t fl_n, fl_dur_max_us, fl_dur_sum_us, fl_gap_max_ms;
} w;

static uint32_t last_loop_cyc;
static bool     have_last_loop;
static uint32_t last_flush_ms;
static bool     have_last_flush;
static uint32_t last_dump_ms;

void zsa_rtt_init(void) {
    DEMCR |= (1u << 24);   /* TRCENA */
    DWT_CYCCNT = 0;
    DWT_CTRL |= 1u;        /* CYCCNTENA */
    SEGGER_RTT_printf(0, "\nzsa_rtt: up t=%u\n", (unsigned)timer_read32());
    last_dump_ms = timer_read32();
}

void zsa_rtt_tx(uint8_t type, uint16_t wire_len, uint32_t start_cycles) {
    uint32_t us = cyc_to_us(zsa_rtt_cycles() - start_cycles);
    switch (type) {
        case ZSA_MSG_HID_MOUSE:     w.tx_mouse++; break;
        case ZSA_MSG_HID_DIGITIZER: w.tx_digi++;  break;
        case ZSA_MSG_LOG_LINE:      w.tx_log++;   break;
        default:                    w.tx_other++; break;
    }
    w.tx_bytes += wire_len;
    w.tx_blk_sum_us += us;
    if (us > w.tx_blk_max_us) w.tx_blk_max_us = us;
}

void zsa_rtt_rx(uint8_t type) {
    switch (type) {
        case ZSA_MSG_EVT_PAIR_STATE: w.rx_pair++;  break;
        case ZSA_MSG_EVT_BLE_STATUS: w.rx_ble++;   break;
        case ZSA_MSG_EVT_LED_RAIL:   w.rx_rail++;  break;
        case ZSA_MSG_EVT_DFU_STATE:  w.rx_dfu++;   break;
        default:                     w.rx_other++; break;
    }
}

void zsa_rtt_event3(const char *name, int a, int b, int c) {
    SEGGER_RTT_printf(0, "EVT t=%u %s a=%d b=%d c=%d\n", (unsigned)timer_read32(), name, a, b, c);
}

void zsa_rtt_event(const char *name) {
    SEGGER_RTT_printf(0, "EVT t=%u %s\n", (unsigned)timer_read32(), name);
}

void zsa_rtt_flush_probe(uint32_t start_cycles) {
    uint32_t us  = cyc_to_us(zsa_rtt_cycles() - start_cycles);
    uint32_t now = timer_read32();
    if (have_last_flush) {
        uint32_t gap = now - last_flush_ms;
        if (gap > w.fl_gap_max_ms) w.fl_gap_max_ms = gap;
    }
    last_flush_ms   = now;
    have_last_flush = true;
    w.fl_n++;
    w.fl_dur_sum_us += us;
    if (us > w.fl_dur_max_us) w.fl_dur_max_us = us;
}

void zsa_rtt_loop_tick(void) {
    uint32_t cyc = zsa_rtt_cycles();
    if (have_last_loop) {
        uint32_t dt = cyc_to_us(cyc - last_loop_cyc);
        if (dt > w.dt_max_us) w.dt_max_us = dt;
    }
    last_loop_cyc  = cyc;
    have_last_loop = true;
    w.loops++;

    uint32_t now = timer_read32();
    if (now - last_dump_ms < 1000) {
        return;
    }
    last_dump_ms = now;

#ifdef ZSA_RTT_LOOP_STATS
    SEGGER_RTT_printf(0,
        "t=%u loops=%u dtmax=%uus | tx m=%u d=%u l=%u o=%u B=%u blkmax=%uus blksum=%uus"
        " | rx ps=%u bs=%u lr=%u df=%u o=%u | is31 n=%u durmax=%uus duravg=%uus gapmax=%ums\n",
        (unsigned)now, (unsigned)w.loops, (unsigned)w.dt_max_us,
        (unsigned)w.tx_mouse, (unsigned)w.tx_digi, (unsigned)w.tx_log, (unsigned)w.tx_other,
        (unsigned)w.tx_bytes, (unsigned)w.tx_blk_max_us, (unsigned)w.tx_blk_sum_us,
        (unsigned)w.rx_pair, (unsigned)w.rx_ble, (unsigned)w.rx_rail, (unsigned)w.rx_dfu,
        (unsigned)w.rx_other,
        (unsigned)w.fl_n, (unsigned)w.fl_dur_max_us,
        (unsigned)(w.fl_n ? w.fl_dur_sum_us / w.fl_n : 0),
        (unsigned)w.fl_gap_max_ms);
#endif

    w = (typeof(w)){0};
}

#endif /* ZSA_RTT_DEBUG */
