#pragma once

#ifndef ZSA_WIRELESS_BAUD_RATE
#    define ZSA_WIRELESS_BAUD_RATE 1000000
#endif

/* Idle power-down (DIM/LEDS_OFF/DEEP) disabled by default for every build —
 * the board stays fully awake. Remove this define to restore normal idle
 * behavior. */
#ifndef ZSA_DEV_NO_SLEEP
#    define ZSA_DEV_NO_SLEEP
#endif

/* Idle power management defaults */
#ifndef ZSA_IDLE_DIM_MS
#    define ZSA_IDLE_DIM_MS 30000
#endif
#ifndef ZSA_IDLE_LEDS_OFF_MS
#    define ZSA_IDLE_LEDS_OFF_MS 120000
#endif
#ifndef ZSA_IDLE_DEEP_MS
#    define ZSA_IDLE_DEEP_MS 300000
#endif
#ifndef ZSA_IDLE_DIM_BRIGHTNESS
#    define ZSA_IDLE_DIM_BRIGHTNESS 128
#endif
#ifndef ZSA_IDLE_SCAN_DELAY_MS
#    define ZSA_IDLE_SCAN_DELAY_MS 50
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

/* ---- BLE pairing / discoverability indication (see wireless.c) ---- */

/* Half-period of the green status LED blink: slow while discoverable, fast
 * while an SMP pairing is running. */
#ifndef ZSA_PAIR_LED_SLOW_MS
#    define ZSA_PAIR_LED_SLOW_MS 500
#endif
#ifndef ZSA_PAIR_LED_FAST_MS
#    define ZSA_PAIR_LED_FAST_MS 125
#endif

/* How long BT_UNPR must be held before the bond-clearing command is sent. */
#ifndef ZSA_UNPAIR_HOLD_MS
#    define ZSA_UNPAIR_HOLD_MS 3000
#endif

/* How long a reported pair state stays trusted. The nRF refreshes the level
 * every 1 s; past this the state is treated as IDLE, so a dead link can't leave
 * the indicators stuck on. */
#ifndef ZSA_PAIR_STATE_STALE_MS
#    define ZSA_PAIR_STATE_STALE_MS 5000
#endif
