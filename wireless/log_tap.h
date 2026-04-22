#pragma once

#ifdef ZSA_BLE_LOG
void zsa_log_tap_init(void);
void zsa_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
#else
static inline void zsa_log_tap_init(void) {}
static inline void zsa_log(const char *fmt, ...) { (void)fmt; }
#endif
