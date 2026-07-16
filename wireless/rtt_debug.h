/* TEMP DEBUG (ZSA_RTT_DEBUG): SEGGER RTT instrumentation for the wireless
 * LED-lag investigation. All hooks compile to no-ops unless the build sets
 * ZSA_RTT_DEBUG=yes. Read output with JLinkRTTClient attached to the STM32.
 * Remove this file, rtt/, rtt_debug.c and the hooks once the culprit is found. */
#pragma once

#include <stdint.h>

#ifdef ZSA_RTT_DEBUG
void     zsa_rtt_init(void);
/* Call once per keyboard_task() iteration; computes loop dt and emits a
 * 1 Hz stats line. */
void     zsa_rtt_loop_tick(void);
/* Raw DWT cycle counter (72 MHz), for stamping around blocking calls. */
uint32_t zsa_rtt_cycles(void);
/* Record one UART TX frame; start_cycles = zsa_rtt_cycles() taken before
 * the blocking uart_transmit(). */
void     zsa_rtt_tx(uint8_t type, uint16_t wire_len, uint32_t start_cycles);
/* Record one received UART frame by type. */
void     zsa_rtt_rx(uint8_t type);
/* One-shot timestamped event marker. */
void     zsa_rtt_event(const char *name);
/* Record one rgb driver flush; start_cycles taken before the flush. */
void     zsa_rtt_flush_probe(uint32_t start_cycles);
#else
static inline void     zsa_rtt_init(void) {}
static inline void     zsa_rtt_loop_tick(void) {}
static inline uint32_t zsa_rtt_cycles(void) { return 0; }
static inline void     zsa_rtt_tx(uint8_t type, uint16_t wire_len, uint32_t start_cycles) { (void)type; (void)wire_len; (void)start_cycles; }
static inline void     zsa_rtt_rx(uint8_t type) { (void)type; }
static inline void     zsa_rtt_event(const char *name) { (void)name; }
static inline void     zsa_rtt_flush_probe(uint32_t start_cycles) { (void)start_cycles; }
#endif
