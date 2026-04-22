UART_DRIVER_REQUIRED = yes
BLUETOOTH_DRIVER = custom

ifeq ($(strip $(ZSA_BLE_LOG)), yes)
    OPT_DEFS += -DZSA_BLE_LOG
    SRC += modules/zsa/wireless/log_tap.c
endif
