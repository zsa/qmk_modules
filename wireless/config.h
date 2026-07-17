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
