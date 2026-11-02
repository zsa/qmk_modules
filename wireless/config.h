#pragma once

#ifndef ZSA_WIRELESS_BAUD_RATE
#    define ZSA_WIRELESS_BAUD_RATE 1000000
#endif

/* USART1 RX DMA ring (zsa_uart.c), a power of two. The longest main-loop stall
 * (~9 ms right-mirror flush) is ~900 B at 1 Mbaud. */
#ifndef ZSA_UART_RX_RING_SIZE
#    define ZSA_UART_RX_RING_SIZE 2048
#endif
/* USART1 TX queue; zsa_uart_transmit() blocks while it is full. */
#ifndef ZSA_UART_TX_QUEUE_SIZE
#    define ZSA_UART_TX_QUEUE_SIZE 64
#endif
/* NVIC priority of the USART1 and RX DMA interrupts. */
#ifndef ZSA_UART_IRQ_PRIORITY
#    define ZSA_UART_IRQ_PRIORITY 12
#endif

/* Power stages: Active -> Sleep after ZSA_SLEEP_TIMEOUT_MS without input ->
 * Hibernate (both halves in PMIC ship mode) after ZSA_HIBERNATE_TIMEOUT_MS
 * without input. Both count from the last input. Define ZSA_DEV_NO_SLEEP to
 * keep the board in Active. */
#ifndef ZSA_SLEEP_TIMEOUT_MS
#    define ZSA_SLEEP_TIMEOUT_MS 60000
#endif
#ifndef ZSA_HIBERNATE_TIMEOUT_MS
#    define ZSA_HIBERNATE_TIMEOUT_MS 600000
#endif

/* On the TRRS cable, spacing of the right-half ship command retries at the
 * hibernate step. */
#ifndef ZSA_HIBERNATE_RIGHT_RETRY_MS
#    define ZSA_HIBERNATE_RIGHT_RETRY_MS 1000
#endif

/* After boot, how long VBUS may be present without a USB host enumerating
 * before the transport is reported as BLE. Also how long after link-up the
 * STM32 waits for a first EVT_VBUS_STATE before deciding without it. */
#ifndef ZSA_USB_HANDOVER_GRACE_MS
#    define ZSA_USB_HANDOVER_GRACE_MS 5000
#endif

/* How long the STM32 stays running after a wake from STOP, or after the last
 * byte from the nRF, before it may stop again. Covers the debounce and the
 * first read of the mirrored right half and nav devices. */
#ifndef ZSA_SLEEP_WAKE_WINDOW_MS
#    define ZSA_SLEEP_WAKE_WINDOW_MS 50
#endif

/* Longest the STM32 waits for the black LED frame to reach both drivers
 * before its first STOP. */
#ifndef ZSA_SLEEP_LED_SETTLE_MS
#    define ZSA_SLEEP_LED_SETTLE_MS 1000
#endif

/* Left boost for the left nav connector (CMD_NAV_BOOST). Without a nav device
 * answering on the STM32's own bus, the left boost is off and a probe window
 * turns it on: at boot, on leaving Sleep, when a present device stops
 * answering, every ZSA_NAV_PROBE_INTERVAL_MS while Active, and every
 * ZSA_NAV_PROBE_SLEEP_INTERVAL_MS while in Sleep (the STM32 leaves STOP for
 * it; a device found wakes the board). A window lasts ZSA_NAV_PROBE_WINDOW_MS
 * from the request, longer while a device that answered is still being set
 * up. With VBUS (which feeds the connector) and the cable unplugged, the
 * real addresses are probed every ZSA_NAV_PROBE_INTERVAL_MS without a claim.
 * Not used in builds without nav modules. */
#ifndef ZSA_NAV_PROBE_INTERVAL_MS
#    define ZSA_NAV_PROBE_INTERVAL_MS 2000
#endif
#ifndef ZSA_NAV_PROBE_SLEEP_INTERVAL_MS
#    define ZSA_NAV_PROBE_SLEEP_INTERVAL_MS 30000
#endif
#ifndef ZSA_NAV_PROBE_WINDOW_MS
#    define ZSA_NAV_PROBE_WINDOW_MS 150
#endif

/* Wake filter for left nav motion read by a STOP poll (one poll per RTC
 * period, 16 ms), for devices that opt in (trackball). A poll moving
 * |dx| + |dy| >= ZSA_NAV_SLEEP_WAKE_COUNTS wakes at once. Smaller motion wakes
 * only once ZSA_NAV_SLEEP_WAKE_POLLS polls in a row saw motion, with the
 * counts of all of them delivered; an idle poll in between drops them. */
#ifndef ZSA_NAV_SLEEP_WAKE_COUNTS
#    define ZSA_NAV_SLEEP_WAKE_COUNTS 5
#endif
#ifndef ZSA_NAV_SLEEP_WAKE_POLLS
#    define ZSA_NAV_SLEEP_WAKE_POLLS 2
#endif

/* Minimum spacing between flushes of the right-half LED driver (0x77) while
 * wireless. That driver is the left nRF's TWIS emulation, not a real chip -
 * every flush blocks on a clock-stretched I2C transfer relayed over ESB
 * (~9 ms vs ~0.65 ms for the real left chip alone). The right half doesn't
 * need full matrix-scan-rate refresh, so this trades LED update latency for
 * CPU time back to the main loop. */
#ifndef ZSA_RIGHT_LED_INTERVAL_MS
#    define ZSA_RIGHT_LED_INTERVAL_MS 50
#endif

/* Send the mirrored right-half frame one 16-byte PWM block per main-loop pass
 * instead of all nine in one call. The nine back-to-back writes block the loop
 * ~9 ms, which pushes the trackpad fetch late. Set to 0 for the old
 * whole-frame flush. Only applies to the mirror, never to the real chip on the
 * cable and never to the left driver. */
#ifndef ZSA_RIGHT_LED_SLICED
#    define ZSA_RIGHT_LED_SLICED 1
#endif

/* Send the left half's frame one 16-byte PWM block per main-loop pass while
 * off USB. The nine back-to-back writes to the real left chip (0x74 on the
 * STM32's own bus) take ~3.6 ms on the wire and are the longest main-loop pass
 * in wireless mode, one per animation frame. On a USB host the whole-frame
 * flush is kept. Set to 0 for the old whole-frame flush everywhere. */
#ifndef ZSA_LEFT_LED_SLICED
#    define ZSA_LEFT_LED_SLICED 1
#endif

/* Main-loop period without VBUS: each pass ends by waiting for the next
 * ZSA_ACTIVE_PACE_US boundary of the system time, or a byte from the nRF.
 * 0 disables it. */
#ifndef ZSA_ACTIVE_PACE_US
#    define ZSA_ACTIVE_PACE_US 1000
#endif

/* ---- LED brightness policy (see wireless.c) ---- */

/* Ceiling on the effective RGB value while VBUS is present (see
 * zsa_wireless_led_val); the stored user setting is left untouched. 255 is no
 * cap; lower it to cap. */
#ifndef ZSA_WIRED_LED_VAL_CAP
#    define ZSA_WIRED_LED_VAL_CAP 255
#endif

/* ---- BLE status overlay (see status_layer.c) ---- */

/* Battery gauge geometry: six LEDs on the top row of each half, filling
 * left-to-right. Indices come from the board's rgb_matrix layout. */
#ifndef ZSA_STATUS_GAUGE_LEDS
#    define ZSA_STATUS_GAUGE_LEDS 6
#endif
#ifndef ZSA_STATUS_GAUGE_LEFT_FIRST
#    define ZSA_STATUS_GAUGE_LEFT_FIRST 0
#endif
#ifndef ZSA_STATUS_GAUGE_RIGHT_FIRST
#    define ZSA_STATUS_GAUGE_RIGHT_FIRST 26
#endif

/* Background for every LED on the layer that isn't part of a status readout,
 * including the unlit tail of each gauge. Written as a literal RGB level (not
 * scaled by the matrix brightness), so black stays black at any setting. */
#ifndef ZSA_STATUS_BG_R
#    define ZSA_STATUS_BG_R 0
#endif
#ifndef ZSA_STATUS_BG_G
#    define ZSA_STATUS_BG_G 0
#endif
#ifndef ZSA_STATUS_BG_B
#    define ZSA_STATUS_BG_B 0
#endif

/* Charging animation. Lit LEDs dip to ZSA_STATUS_BREATHE_MIN (0-255) at the
 * wave's trough rather than going dark, so the charge level stays readable.
 * Sweep period interpolates between these two bounds over the detected USB
 * source current budget (ZSA_PWR_BUDGET_DEFAULT_MA .. ZSA_PWR_BUDGET_HIGH_MA). */
#ifndef ZSA_STATUS_BREATHE_MIN
#    define ZSA_STATUS_BREATHE_MIN 60
#endif
#ifndef ZSA_STATUS_BREATHE_SLOW_MS
#    define ZSA_STATUS_BREATHE_SLOW_MS 4400
#endif
#ifndef ZSA_STATUS_BREATHE_FAST_MS
#    define ZSA_STATUS_BREATHE_FAST_MS 1400
#endif

/* ---- Firmware-update indication (see wireless.c) ---- */

/* How long a reported DFU state stays trusted. The nRF re-sends the level
 * every 1 s; past this the keyboard takes keys again, so a link that dies
 * mid-update cannot leave it permanently dead. */
#ifndef ZSA_DFU_STATE_STALE_MS
#    define ZSA_DFU_STATE_STALE_MS 5000
#endif

/* How long the reported DFU phase keeps driving the status LEDs without a
 * refresh. Far longer than the state above: the left nRF reboots into the
 * swap mid-update and re-asserts the phase once it is back, and the LEDs must
 * not drop out across that window. */
#ifndef ZSA_DFU_PHASE_STALE_MS
#    define ZSA_DFU_PHASE_STALE_MS 60000
#endif

/* How long a reported inter-half link state stays trusted. Same 1 s level
 * cadence as the pair state. */
#ifndef ZSA_LINK_STATE_STALE_MS
#    define ZSA_LINK_STATE_STALE_MS 5000
#endif

/* Right-half nav presence from the left nRF is reported this long after the
 * right half is seen linked (EVT_LINK_STATE OK) and the capability announce is
 * acked: the right starts probing on its first reply with NAV_ENABLED, and
 * retries every 1000 ms (NAV_PROBE_INTERVAL_MS in voyager_right ptx_main.c)
 * when a probe fails or its peripherals are not up yet. */
#ifndef ZSA_NAV_RIGHT_SETTLE_MS
#    define ZSA_NAV_RIGHT_SETTLE_MS 1500
#endif

/* ---- Low-battery boot guard (see wireless.c) ---- */

/* How long the boot power gate waits for the nRF's EVT_POWER_VERDICT before
 * lighting the matrix anyway: an nRF that is dead, in serial recovery, or
 * running a build that never sends one must not leave the keyboard dark. */
#ifndef ZSA_POWER_GATE_TIMEOUT_MS
#    define ZSA_POWER_GATE_TIMEOUT_MS 5000
#endif

/* Red status LED cadence on a critical verdict: ZSA_POWER_LED_PULSES long
 * pulses, then dark. Deliberately unlike the 500 ms protocol-mismatch blink. */
#ifndef ZSA_POWER_LED_ON_MS
#    define ZSA_POWER_LED_ON_MS 1000
#endif
#ifndef ZSA_POWER_LED_OFF_MS
#    define ZSA_POWER_LED_OFF_MS 500
#endif
#ifndef ZSA_POWER_LED_PULSES
#    define ZSA_POWER_LED_PULSES 5
#endif

/* Spacing of the CMD_HIBERNATE resend once the pulses are done, covering a
 * lost first frame. The nRF refuses to ship while USB is present, so a repeat
 * costs nothing. */
#ifndef ZSA_POWER_HIBERNATE_RETRY_MS
#    define ZSA_POWER_HIBERNATE_RETRY_MS 3000
#endif

/* ---- Bluetooth bank menu (BLE_MENU, see wireless.c) ---- */

/* Keys of each bank, as {row, col} matrix positions: one column of
 * ZSA_BLE_MENU_BANK_KEYS keys per bank, the first being the bank's number key.
 * An entry outside the matrix (e.g. {0xFF, 0xFF}) is unused. Defaults to the
 * columns under keys 1-5 on the Voyager 2 left half. */
#ifndef ZSA_BLE_MENU_BANK_KEYS
#    define ZSA_BLE_MENU_BANK_KEYS 4
#endif
#ifndef ZSA_BLE_MENU_KEYS
#    define ZSA_BLE_MENU_KEYS                \
        {                                    \
            {{0, 2}, {1, 2}, {2, 2}, {3, 2}}, \
            {{0, 3}, {1, 3}, {2, 3}, {3, 3}}, \
            {{0, 4}, {1, 4}, {2, 4}, {3, 4}}, \
            {{0, 5}, {1, 5}, {2, 5}, {3, 5}}, \
            {{0, 6}, {1, 6}, {2, 6}, {4, 4}}, \
        }
#endif

/* How long a bank's key must be held to forget that bank. */
#ifndef ZSA_BLE_FORGET_HOLD_MS
#    define ZSA_BLE_FORGET_HOLD_MS 2000
#endif

/* Idle time after which the open menu closes itself. */
#ifndef ZSA_BLE_MENU_IDLE_MS
#    define ZSA_BLE_MENU_IDLE_MS 60000
#endif

/* Blink period of the connecting and pairing banks. */
#ifndef ZSA_BLE_MENU_BLINK_MS
#    define ZSA_BLE_MENU_BLINK_MS 1000
#endif

/* Empty-bank color, #28a5cf. */
#ifndef ZSA_BLE_MENU_EMPTY_R
#    define ZSA_BLE_MENU_EMPTY_R 0x28
#endif
#ifndef ZSA_BLE_MENU_EMPTY_G
#    define ZSA_BLE_MENU_EMPTY_G 0xA5
#endif
#ifndef ZSA_BLE_MENU_EMPTY_B
#    define ZSA_BLE_MENU_EMPTY_B 0xCF
#endif

/* Level (0-255) of the five number keys, a flat dim gray, while the bank table
 * is unknown. */
#ifndef ZSA_BLE_MENU_UNKNOWN_VAL
#    define ZSA_BLE_MENU_UNKNOWN_VAL 40
#endif

/* How long a reported pair state stays trusted. The nRF refreshes the level
 * every 1 s; past this the state is treated as IDLE, so a dead link can't leave
 * the indicators stuck on. */
#ifndef ZSA_PAIR_STATE_STALE_MS
#    define ZSA_PAIR_STATE_STALE_MS 5000
#endif

/* ---- Diagnostic view (hold BLE_MENU, see wireless.c) ---- */

/* How long BLE_MENU must be held to open the diagnostic view. */
#ifndef ZSA_DIAG_HOLD_MS
#    define ZSA_DIAG_HOLD_MS 3000
#endif

/* The open view closes itself after this long. */
#ifndef ZSA_DIAG_VIEW_TIMEOUT_MS
#    define ZSA_DIAG_VIEW_TIMEOUT_MS 60000
#endif

/* Indicator keys, {row, col}, in the order of enum zsa_diag_key (wireless.h).
 * Defaults to the Voyager 2 left half: keys 1-5, Q-T, A-F and Z-B. */
#ifndef ZSA_DIAG_KEYS
#    define ZSA_DIAG_KEYS                                                  \
        {                                                                  \
            {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6},                        \
            {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6},                        \
            {2, 2}, {2, 3}, {2, 4}, {2, 5},                                \
            {3, 2}, {3, 3}, {3, 4}, {3, 5}, {4, 4},                        \
        }
#endif

/* Level (0-255) of an indicator whose state is unknown, a flat dim gray. */
#ifndef ZSA_DIAG_UNKNOWN_VAL
#    define ZSA_DIAG_UNKNOWN_VAL 40
#endif

/* An nRF frame within this long counts as the link being alive. */
#ifndef ZSA_DIAG_RX_FRESH_MS
#    define ZSA_DIAG_RX_FRESH_MS 2000
#endif

/* How often the STM32 diagnostic record is saved to CCM SRAM. */
#ifndef ZSA_STM32_DIAG_SAVE_MS
#    define ZSA_STM32_DIAG_SAVE_MS 1000
#endif
