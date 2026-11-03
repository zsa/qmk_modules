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
