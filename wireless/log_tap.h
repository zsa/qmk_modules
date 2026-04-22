#pragma once

#ifdef ZSA_BLE_LOG
void zsa_log_tap_init(void);
#else
static inline void zsa_log_tap_init(void) {}
#endif
