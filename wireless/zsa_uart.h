#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <hal.h>

/* USART1 link to the left nRF (STM32F3): RX by DMA1 channel 5, circular, into
 * a ZSA_UART_RX_RING_SIZE ring; TX from a ZSA_UART_TX_QUEUE_SIZE queue fed to
 * the USART by its TXE interrupt. */

#define ZSA_UART_ERR_OVERRUN 0x01u
#define ZSA_UART_ERR_FRAMING 0x02u
#define ZSA_UART_ERR_NOISE 0x04u
#define ZSA_UART_ERR_PARITY 0x08u

/* Resets USART1, then starts RX and TX. Once per boot. */
void zsa_uart_init(uint32_t baud);

/* Queues len bytes; blocks while the TX queue is full. Thread context. */
void zsa_uart_transmit(const uint8_t *data, uint16_t len);

/* TX queue empty and the last byte off the wire. */
bool zsa_uart_tx_done(void);

/* Copies up to max received bytes into dst and returns the count. When the DMA
 * has overtaken the reader, *lapped is set, every unread byte is dropped and 0
 * is returned. */
uint16_t zsa_uart_read(uint8_t *dst, uint16_t max, bool *lapped);

/* Unread bytes in the ring, or a byte on the line. System locked. */
bool zsa_uart_rx_pendingI(void);

/* Drops every unread byte. */
void zsa_uart_rx_discard(void);

/* ZSA_UART_ERR_* flags set in USART1->ISR since the last call; clears them. */
uint8_t zsa_uart_errors_take(void);

/* Broadcast on an idle line after reception, and each time the DMA crosses
 * half or the end of the ring. */
event_source_t *zsa_uart_rx_event(void);
