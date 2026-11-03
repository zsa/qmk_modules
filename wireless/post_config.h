#pragma once

/* ZSA_SLEEP_DIAG pages ride the CMD_SCAN_DIAG window. */
#if defined(ZSA_SLEEP_DIAG) && !defined(ZSA_SCAN_DIAG)
#    define ZSA_SCAN_DIAG
#endif
