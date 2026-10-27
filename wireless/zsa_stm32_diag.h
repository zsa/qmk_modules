#pragma once

/* STM32 diagnostic record: ORYX_DIAG_REC_STM32 / ORYX_DIAG_REC_STM32_PREV.
 *
 * Refreshed every ZSA_STM32_DIAG_SAVE_MS into CCM SRAM (0x10000000), which
 * neither the QMK startup code nor ignition writes, so the last copy survives
 * a software, watchdog or pin reset and is served as the previous boot's
 * record. Counters are this boot's; uint16 ones saturate. */

#include <stdint.h>

#define ZSA_STM32_DIAG_VERSION 1u

/* zsa_stm32_diag.flags */
#define ZSA_STM32_DIAG_F_LINK      0x01u /* LINK_UP handshake done */
#define ZSA_STM32_DIAG_F_VBUS      0x02u /* the left nRF reports VBUS */
#define ZSA_STM32_DIAG_F_USB_HOST  0x04u /* a USB host enumerated */
#define ZSA_STM32_DIAG_F_BLE_CONN  0x08u /* the left nRF reports a BLE host */
#define ZSA_STM32_DIAG_F_TRRS      0x10u /* the TRRS cable is plugged */
#define ZSA_STM32_DIAG_F_RIGHT_LED 0x20u /* right LED driver up */
#define ZSA_STM32_DIAG_F_VBUS_SEEN 0x40u /* any EVT_VBUS_STATE this boot */

/* zsa_stm32_diag.transport */
#define ZSA_STM32_DIAG_TRANSPORT_PENDING 0u
#define ZSA_STM32_DIAG_TRANSPORT_BLE     1u
#define ZSA_STM32_DIAG_TRANSPORT_USB     2u

/* zsa_stm32_diag.right_link when no fresh EVT_LINK_STATE. */
#define ZSA_STM32_DIAG_RIGHT_LINK_UNKNOWN 0xFFu

/* An age with no event to count from. */
#define ZSA_STM32_DIAG_AGE_NEVER 0xFFFFFFFFu

struct __attribute__((packed)) zsa_stm32_diag {
    uint8_t  version;        /* ZSA_STM32_DIAG_VERSION */
    uint8_t  len;            /* bytes after this one: 46 */
    uint16_t boot_count;     /* boots since the record was last lost */
    uint32_t uptime_s;
    uint8_t  reset_flags;    /* RCC_CSR[31:24] at boot */
    uint8_t  flags;          /* ZSA_STM32_DIAG_F_* */
    uint8_t  transport;      /* ZSA_STM32_DIAG_TRANSPORT_* */
    uint8_t  idle_step;      /* enum zsa_idle_step */
    uint8_t  right_link;     /* EVT_LINK_STATE value or ..._UNKNOWN */
    uint8_t  power_gate;     /* 0 wait, 1 ok, 2 critical, 3 done */
    uint16_t link_ups;       /* LINK_UP frames received */
    uint16_t sd_overruns;    /* passes that saw USART1 ORE */
    uint16_t sd_framing;     /* USART1 FE */
    uint16_t sd_noise;       /* USART1 NE */
    uint16_t sd_parity;      /* USART1 PE */
    uint16_t sd_queue_full;  /* RX DMA ring laps (the old "RX queue full" slot) */
    uint16_t parser_resyncs; /* frames dropped: bad escape, length or CRC */
    uint16_t crc_errors;     /* of which bad CRC */
    uint16_t frame_aborts;   /* frames cut short by a new SYNC */
    uint32_t frames_rx;      /* frames received from the nRF */
    uint32_t frames_tx;      /* frames sent to the nRF */
    uint32_t rx_age_ms;      /* since the last frame from the nRF */
    uint32_t max_rx_gap_ms;  /* longest gap between frames, awake only */
};

#define ZSA_STM32_DIAG_SIZE 48u
#define ZSA_STM32_DIAG_LEN  46u

_Static_assert(sizeof(struct zsa_stm32_diag) == ZSA_STM32_DIAG_SIZE, "zsa_stm32_diag is a wire layout");

/* RCC_CSR[31:24] bits in zsa_stm32_diag.reset_flags (STM32F303 RM0316). */
#define ZSA_STM32_RESET_OBL  0x02u /* option byte loader */
#define ZSA_STM32_RESET_PIN  0x04u /* NRST pin */
#define ZSA_STM32_RESET_POR  0x08u /* power-on / power-down */
#define ZSA_STM32_RESET_SFT  0x10u /* software */
#define ZSA_STM32_RESET_IWDG 0x20u /* independent watchdog */
#define ZSA_STM32_RESET_WWDG 0x40u /* window watchdog */
#define ZSA_STM32_RESET_LPWR 0x80u /* low-power */
