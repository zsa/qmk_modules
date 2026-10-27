BLUETOOTH_DRIVER = custom

# USART1 link to the left nRF: RX by DMA, TX by interrupt.
SRC += modules/zsa/wireless/zsa_uart.c

# BLE status overlay rendering (battery gauges). Self-disables without
# RGB_MATRIX_ENABLE, so it is safe to build unconditionally.
SRC += modules/zsa/wireless/status_layer.c

ifeq ($(strip $(ZSA_BLE_LOG)), yes)
    OPT_DEFS += -DZSA_BLE_LOG
    SRC += modules/zsa/wireless/log_tap.c
endif

# TEMP DEBUG: SEGGER RTT instrumentation for the wireless LED-lag hunt.
# Enable with ZSA_RTT_DEBUG = yes in a keymap rules.mk; read with JLinkRTTClient.
ifeq ($(strip $(ZSA_RTT_DEBUG)), yes)
    OPT_DEFS += -DZSA_RTT_DEBUG
    SRC += modules/zsa/wireless/rtt_debug.c \
           modules/zsa/wireless/rtt/SEGGER_RTT.c \
           modules/zsa/wireless/rtt/SEGGER_RTT_printf.c
    VPATH += modules/zsa/wireless/rtt
endif
