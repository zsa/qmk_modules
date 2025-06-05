#include "quantum.h"
// List of ZSA specific keycodes, appended to the end of the keycode list.
enum zsa_keycodes {
    TOGGLE_LAYER_COLOR = QK_KB,
    LED_LEVEL,
    NAVIGATOR_INC_CPI,
    NAVIGATOR_DEC_CPI,
    ZSA_SAFE_RANGE
};
