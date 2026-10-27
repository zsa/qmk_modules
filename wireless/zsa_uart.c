/*
 * ZSA Wireless — USART1 driver for the link to the left nRF.
 *
 * RX: DMA1 channel 5 (the F303's fixed USART1_RX request) in circular mode
 * into rx_ring. The write position is rx_laps * ring size plus the bytes the
 * channel has written in the current lap (size - CNDTR); the reader keeps its
 * own running count, rx_tail. Both count modulo 2^32.
 *
 * TX: an output queue drained one byte per TXE interrupt.
 *
 * The USART interrupt also serves IDLE, which broadcasts rx_event; error
 * interrupts stay off and the error flags are read by zsa_uart_errors_take().
 */

#include <string.h>
#include "zsa_uart.h"
#include "gpio.h"
#include "chibios_config.h"

#if !defined(STM32F3XX)
#    error "zsa_uart: STM32F3 only"
#endif
#if !defined(STM32_DMA_REQUIRED)
#    error "zsa_uart: needs the ChibiOS DMA driver (STM32_DMA_REQUIRED)"
#endif
#if HAL_USE_SERIAL == TRUE && STM32_SERIAL_USE_USART1 == TRUE
#    error "zsa_uart: USART1 is also assigned to the ChibiOS serial driver"
#endif

#ifndef UART_TX_PIN
#    define UART_TX_PIN A9
#endif
#ifndef UART_RX_PIN
#    define UART_RX_PIN A10
#endif
#define ZSA_UART_PIN_AF 7U

#define RX_RING_MASK (ZSA_UART_RX_RING_SIZE - 1u)
_Static_assert((ZSA_UART_RX_RING_SIZE & RX_RING_MASK) == 0, "ZSA_UART_RX_RING_SIZE must be a power of two");
_Static_assert(ZSA_UART_RX_RING_SIZE <= 0xFFFFu, "ZSA_UART_RX_RING_SIZE must fit CNDTR");

#define RX_DMA_STREAM STM32_DMA_STREAM_ID(1, 5)
#define RX_DMA_MODE (STM32_DMA_CR_PL(3) | STM32_DMA_CR_DIR_P2M | STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_CIRC | STM32_DMA_CR_HTIE | STM32_DMA_CR_TCIE | STM32_DMA_CR_TEIE)

#define USART_ERR_FLAGS (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)

static uint8_t                   rx_ring[ZSA_UART_RX_RING_SIZE]; /* SRAM: DMA cannot reach CCM */
static const stm32_dma_stream_t *rx_dma;
static volatile uint32_t         rx_laps; /* TC interrupts served */
static uint32_t                  rx_tail; /* bytes consumed */
static event_source_t            rx_event;

static uint8_t        tx_buf[ZSA_UART_TX_QUEUE_SIZE];
static output_queue_t tx_queue;

/* Bytes written by the DMA. System locked. A TC flag still set has not been
 * counted in rx_laps yet; CNDTR is read again after it, past the reload. */
static uint32_t rx_headI(void) {
    uint32_t laps = rx_laps;
    uint32_t left = rx_dma->channel->CNDTR;
    if (rx_dma->dma->ISR & (STM32_DMA_ISR_TCIF << rx_dma->shift)) {
        laps++;
        left = rx_dma->channel->CNDTR;
    }
    return laps * ZSA_UART_RX_RING_SIZE + (ZSA_UART_RX_RING_SIZE - left);
}

/* (Re)starts the RX channel at the start of the ring. System locked. */
static void rx_armI(void) {
    dmaStreamDisable(rx_dma);
    dmaStreamClearInterrupt(rx_dma);
    dmaStreamSetPeripheral(rx_dma, &USART1->RDR);
    dmaStreamSetMemory0(rx_dma, rx_ring);
    dmaStreamSetTransactionSize(rx_dma, ZSA_UART_RX_RING_SIZE);
    dmaStreamSetMode(rx_dma, RX_DMA_MODE);
    rx_laps = 0;
    rx_tail = 0;
    dmaStreamEnable(rx_dma);
}

/* HT, TC and TE of the RX channel; a TE disables the channel, which
 * zsa_uart_read() re-arms. */
static void rx_dma_serve(void *param, uint32_t flags) {
    (void)param;
    osalSysLockFromISR();
    if (flags & STM32_DMA_ISR_TCIF) {
        rx_laps++;
    }
    chEvtBroadcastI(&rx_event);
    osalSysUnlockFromISR();
}

/* System locked (oqWriteTimeout). */
static void tx_notify(io_queue_t *qp) {
    (void)qp;
    USART1->CR1 |= USART_CR1_TXEIE;
}

OSAL_IRQ_HANDLER(STM32_USART1_HANDLER) {
    OSAL_IRQ_PROLOGUE();

    uint32_t isr = USART1->ISR;
    uint32_t cr1 = USART1->CR1;

    osalSysLockFromISR();
    if (isr & USART_ISR_IDLE) {
        USART1->ICR = USART_ICR_IDLECF;
        chEvtBroadcastI(&rx_event);
    }
    if ((cr1 & USART_CR1_TXEIE) && (isr & USART_ISR_TXE)) {
        msg_t b = oqGetI(&tx_queue);
        if (b < MSG_OK) {
            USART1->CR1 = cr1 & ~USART_CR1_TXEIE;
        } else {
            USART1->TDR = (uint8_t)b;
        }
    }
    osalSysUnlockFromISR();

    OSAL_IRQ_EPILOGUE();
}

void zsa_uart_init(uint32_t baud) {
    static bool initialised = false;
    if (initialised) {
        return;
    }
    initialised = true;

    chEvtObjectInit(&rx_event);
    oqObjectInit(&tx_queue, tx_buf, sizeof(tx_buf), tx_notify, NULL);

    /* Ignition hands over with USART1 enabled and its RXNE/IDLE interrupts on. */
    nvicDisableVector(STM32_USART1_NUMBER);
    rccEnableUSART1(true);
    rccResetUSART1();

    palSetLineMode(UART_TX_PIN, PAL_MODE_ALTERNATE(ZSA_UART_PIN_AF) | PAL_OUTPUT_TYPE_PUSHPULL | PAL_OUTPUT_SPEED_HIGHEST);
    palSetLineMode(UART_RX_PIN, PAL_MODE_ALTERNATE(ZSA_UART_PIN_AF) | PAL_OUTPUT_TYPE_PUSHPULL | PAL_OUTPUT_SPEED_HIGHEST);

    osalSysLock();
    rx_dma = dmaStreamAllocI(RX_DMA_STREAM, ZSA_UART_IRQ_PRIORITY, rx_dma_serve, NULL);
    if (rx_dma == NULL) {
        osalSysHalt("zsa_uart: DMA1 channel 5 taken");
    }
    rx_armI();
    osalSysUnlock();

    USART1->BRR = (STM32_USART1CLK + baud / 2u) / baud;
    USART1->CR2 = 0;
    USART1->CR3 = USART_CR3_DMAR;
    USART1->CR1 = USART_CR1_IDLEIE | USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    USART1->ICR = 0xFFFFFFFFu;
    nvicEnableVector(STM32_USART1_NUMBER, ZSA_UART_IRQ_PRIORITY);
}

void zsa_uart_transmit(const uint8_t *data, uint16_t len) {
    if (len == 0) {
        return;
    }
    oqWriteTimeout(&tx_queue, data, len, TIME_INFINITE);
}

bool zsa_uart_tx_done(void) {
    osalSysLock();
    bool empty = oqIsEmptyI(&tx_queue);
    osalSysUnlock();
    return empty && (USART1->ISR & USART_ISR_TC);
}

uint16_t zsa_uart_read(uint8_t *dst, uint16_t max, bool *lapped) {
    osalSysLock();
    if (!(rx_dma->channel->CCR & STM32_DMA_CR_EN)) {
        rx_armI();
        *lapped = true;
    }
    uint32_t tail  = rx_tail;
    uint32_t avail = rx_headI() - tail;
    osalSysUnlock();

    if (avail > ZSA_UART_RX_RING_SIZE) {
        zsa_uart_rx_discard();
        *lapped = true;
        return 0;
    }
    uint16_t n = avail < max ? (uint16_t)avail : max;
    if (n == 0) {
        return 0;
    }

    uint32_t pos   = tail & RX_RING_MASK;
    uint32_t first = ZSA_UART_RX_RING_SIZE - pos;
    if (first > n) {
        first = n;
    }
    memcpy(dst, &rx_ring[pos], first);
    memcpy(dst + first, rx_ring, n - first);

    /* The copy is good unless the DMA wrote past tail + ring size meanwhile. */
    osalSysLock();
    uint32_t head = rx_headI();
    osalSysUnlock();
    if (head - tail > ZSA_UART_RX_RING_SIZE) {
        zsa_uart_rx_discard();
        *lapped = true;
        return 0;
    }
    rx_tail = tail + n;
    return n;
}

bool zsa_uart_rx_pendingI(void) {
    return rx_headI() != rx_tail || (USART1->ISR & (USART_ISR_RXNE | USART_ISR_BUSY));
}

void zsa_uart_rx_discard(void) {
    osalSysLock();
    rx_tail = rx_headI();
    osalSysUnlock();
}

uint8_t zsa_uart_errors_take(void) {
    uint32_t isr = USART1->ISR & USART_ERR_FLAGS;
    uint8_t  f   = 0;
    if (isr & USART_ISR_ORE) f |= ZSA_UART_ERR_OVERRUN;
    if (isr & USART_ISR_FE) f |= ZSA_UART_ERR_FRAMING;
    if (isr & USART_ISR_NE) f |= ZSA_UART_ERR_NOISE;
    if (isr & USART_ISR_PE) f |= ZSA_UART_ERR_PARITY;
    /* ICR clear bits sit at the ISR flag positions. */
    USART1->ICR = isr;
    return f;
}

event_source_t *zsa_uart_rx_event(void) {
    return &rx_event;
}
