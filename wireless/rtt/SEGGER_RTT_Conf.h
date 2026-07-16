/* Minimal standalone SEGGER_RTT_Conf.h for the QMK/ChibiOS STM32 build.
 * Replaces the NCS/Zephyr Kconfig-based config the sources shipped with.
 * TEMP DEBUG — part of the ZSA_RTT_DEBUG instrumentation. */
#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

#define SEGGER_RTT_MAX_NUM_UP_BUFFERS    (2)
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS  (2)

#define BUFFER_SIZE_UP                   (2048)
#define BUFFER_SIZE_DOWN                 (16)

#define SEGGER_RTT_PRINTF_BUFFER_SIZE    (128u)

/* Don't block the firmware if the host isn't reading: skip on full buffer. */
#define SEGGER_RTT_MODE_DEFAULT          SEGGER_RTT_MODE_NO_BLOCK_SKIP

#define SEGGER_RTT_MEMCPY_USE_BYTELOOP   0

/* Cortex-M interrupt-safe locking via PRIMASK. */
#define SEGGER_RTT_LOCK()                                                  \
    {                                                                      \
        unsigned int _SEGGER_RTT__LockState;                               \
        __asm volatile("mrs %0, primask\n\t"                               \
                       "cpsid i"                                           \
                       : "=r"(_SEGGER_RTT__LockState)                      \
                       :                                                   \
                       : "memory");

#define SEGGER_RTT_UNLOCK()                                                \
        __asm volatile("msr primask, %0"                                   \
                       :                                                   \
                       : "r"(_SEGGER_RTT__LockState)                       \
                       : "memory");                                        \
    }

#endif
