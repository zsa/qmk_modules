DIGITIZER_ENABLE = yes
DIGITIZER_MODE = touchpad
I2C_DRIVER_REQUIRED = yes

SRC += $(MODULE_PATH_NAVIGATOR_TRACKPAD)/navigator_trackpad.c
SRC += $(MODULE_PATH_NAVIGATOR_TRACKPAD)/navigator_trackpad_common.c
SRC += $(MODULE_PATH_NAVIGATOR_TRACKPAD)/navigator_trackpad_ptp.c
