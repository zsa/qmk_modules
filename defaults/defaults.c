#include <stdint.h>
#include "quantum.h"
#include "defaults.h"

bool process_record_defaults(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
#if defined(POINTING_DEVICE_DRIVER_navigator_trackpad)
        case NAVIGATOR_INC_CPI:
            if (record->event.pressed) {
                navigator_trackpad_set_cpi(1);
                keyboard_config.navigator_cpi = navigator_trackpad_get_cpi();
                eeconfig_update_kb(keyboard_config.raw);
            }
            return false;
        case NAVIGATOR_DEC_CPI:
            if (record->event.pressed) {
                navigator_trackpad_set_cpi(0);
                keyboard_config.navigator_cpi = navigator_trackpad_get_cpi();
                eeconfig_update_kb(keyboard_config.raw);
            }
            return false;
#endif
#if defined(POINTING_DEVICE_DRIVER_navigator_trackball)
        case NAVIGATOR_INC_CPI:
            if (record->event.pressed) {
                navigator_trackball_set_cpi(1);
                keyboard_config.navigator_cpi = navigator_trackball_get_cpi();
                eeconfig_update_kb(keyboard_config.raw);
            }
            return false;
        case NAVIGATOR_DEC_CPI:
            if (record->event.pressed) {
                navigator_trackball_set_cpi(0);
                keyboard_config.navigator_cpi = navigator_trackball_get_cpi();
                eeconfig_update_kb(keyboard_config.raw);
            }
            return false;
#endif
    }
    return true;
}
