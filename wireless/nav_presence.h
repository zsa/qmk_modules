// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once
/* Which nav devices are attached to which half, as reported in the raw-HID
 * telemetry (oryx_telemetry_t nav_*) and in ZSA_MSG_CMD_NAV_STATE. No QMK
 * dependencies, so wireless/tests/nav_presence_test.c builds it on the host. */

#include <stdbool.h>
#include <stdint.h>
#include "zsa_uart_proto.h"

/* What the compiled-in nav driver found the last time it probed. The driver
 * tries the real device address first — a chip on the STM32's own I2C bus —
 * and only then the left nRF's mirror address, which exists solely to relay a
 * right-plugged device while wireless. Which one answered is the only
 * side-of-the-board signal QMK has.
 *
 * Note the ambiguity it does not resolve: the TRRS segments do not isolate, so
 * a right-plugged device is on the same net and answers at the same real
 * address whenever the cable is plugged. LOCAL means "on our bus", not "on the
 * left half". */
enum zsa_nav_probe {
    ZSA_NAV_PROBE_UNSUPPORTED = 0, /* no nav driver in this build: nothing looked */
    ZSA_NAV_PROBE_NONE,            /* looked, neither address answered */
    ZSA_NAV_PROBE_LOCAL,           /* the real device address answered */
    ZSA_NAV_PROBE_MIRROR,          /* only the left-nRF mirror answered */
    ZSA_NAV_PROBE_PENDING,         /* an address answered, the device is being set up */
};

struct zsa_nav_inputs {
    uint8_t build;        /* ZSA_NAV_CAP_* this build can drive */
    uint8_t ball;         /* enum zsa_nav_probe, trackball driver */
    uint8_t pad;          /* enum zsa_nav_probe, trackpad driver */
    bool    right_on_bus; /* TRRS cable plugged: one I2C net for both halves */
    /* Right half, from the left nRF. */
    bool    nrf_seen;      /* EVT_NAV_PRESENT received since the last LINK_UP */
    bool    link_fresh;    /* zsa_nav_link_fresh() */
    bool    right_linked;  /* that EVT_LINK_STATE said ZSA_LINK_STATE_OK */
    bool    right_settled; /* ZSA_NAV_RIGHT_SETTLE_MS passed since linked and announced */
    uint8_t nrf_device;    /* enum zsa_nav_device_type from EVT_NAV_PRESENT */
};

/* Age of the last EVT_LINK_STATE for nav purposes. The left nRF stops
 * refreshing it while the STM32 is in Sleep (it still sends a MISMATCH), so
 * the verdict held at Sleep entry is kept until Active, and aged from the wake
 * moment from then on. Times are timer_read32() milliseconds. */
struct zsa_nav_link_age {
    uint32_t ms;          /* last EVT_LINK_STATE, or the wake from Sleep */
    bool     seen;        /* any EVT_LINK_STATE received */
    bool     asleep;      /* the STM32 reported ZSA_IDLE_SLEEP */
    bool     sleep_fresh; /* fresh at Sleep entry, or refreshed in Sleep */
};

static inline bool zsa_nav_link_fresh(const struct zsa_nav_link_age *a, uint32_t now, uint32_t stale_ms) {
    if (!a->seen) {
        return false;
    }
    return a->asleep ? a->sleep_fresh : (uint32_t)(now - a->ms) < stale_ms;
}

/* An EVT_LINK_STATE arrived. */
static inline void zsa_nav_link_received(struct zsa_nav_link_age *a, uint32_t now) {
    a->ms   = now;
    a->seen = true;
    if (a->asleep) {
        a->sleep_fresh = true;
    }
}

/* The STM32 entered (true) or left (false) Sleep. */
static inline void zsa_nav_link_sleep(struct zsa_nav_link_age *a, bool asleep, uint32_t now, uint32_t stale_ms) {
    if (asleep == a->asleep) {
        return;
    }
    if (asleep) {
        a->sleep_fresh = zsa_nav_link_fresh(a, now, stale_ms);
    } else if (a->sleep_fresh) {
        a->ms = now;
    }
    a->asleep = asleep;
}

/* ZSA_NAV_CAP_* bit for an enum zsa_nav_device_type, 0 for none. */
static inline uint8_t zsa_nav_device_mask(uint8_t device_type) {
    switch (device_type) {
        case ZSA_NAV_DEVICE_TRACKBALL:
            return ZSA_NAV_CAP_TRACKBALL;
        case ZSA_NAV_DEVICE_TOUCHPAD:
            return ZSA_NAV_CAP_TRACKPAD;
        default:
            return 0;
    }
}

/* Fields: ZSA_NAV_CAP_* masks or ZSA_NAV_STATE_UNKNOWN.
 *   build: the build's mask.
 *   bus:   cable plugged: devices up at the real address, 0 when every driver
 *          found nothing, else unknown. Cable unplugged: 0.
 *   left:  cable unplugged: devices up at the real address, 0 when nothing
 *          answered or only the mirror did, unknown while a device is being
 *          set up or no driver looked. Cable plugged: 0 when the shared bus is
 *          empty, else unknown.
 *   right: cable unplugged: the left nRF's report while the right half is
 *          linked and settled, else unknown. Cable plugged: as left.
 * A build without nav support leaves left, right and bus unknown. */
static inline struct zsa_nav_state zsa_nav_presence(const struct zsa_nav_inputs *in) {
    struct zsa_nav_state out = {
        .build = in->build,
        .left  = ZSA_NAV_STATE_UNKNOWN,
        .right = ZSA_NAV_STATE_UNKNOWN,
        .bus   = ZSA_NAV_STATE_UNKNOWN,
    };

    /* No capability announced: the nRF stopped detecting and no driver looked. */
    if (in->build == 0) {
        return out;
    }

    uint8_t found   = 0;
    bool    looked  = false; /* at least one driver probed */
    bool    pending = false; /* a device is being set up */
    bool    empty   = true;  /* every driver that looked found nothing */
    const uint8_t probes[2] = {in->ball, in->pad};
    for (uint8_t i = 0; i < 2; i++) {
        switch (probes[i]) {
            case ZSA_NAV_PROBE_UNSUPPORTED:
                break;
            case ZSA_NAV_PROBE_NONE:
                looked = true;
                break;
            case ZSA_NAV_PROBE_LOCAL:
                looked = true;
                empty  = false;
                found |= i == 0 ? ZSA_NAV_CAP_TRACKBALL : ZSA_NAV_CAP_TRACKPAD;
                break;
            case ZSA_NAV_PROBE_PENDING:
                looked  = true;
                empty   = false;
                pending = true;
                break;
            default: /* MIRROR */
                looked = true;
                empty  = false;
                break;
        }
    }

    if (in->right_on_bus) {
        /* The mirror only answers while the left nRF runs ESB, which it does
         * not with the cable plugged: MIRROR here is a stale latch, unknown. */
        if (found != 0) {
            out.bus = found;
        } else if (looked && empty) {
            out.bus   = 0;
            out.left  = 0;
            out.right = 0;
        }
        return out;
    }

    out.bus = 0;

    if (found != 0) {
        out.left = found;
    } else if (looked && !pending) {
        out.left = 0;
    }

    if (in->nrf_seen && in->link_fresh && in->right_linked && in->right_settled) {
        out.right = zsa_nav_device_mask(in->nrf_device);
    }
    return out;
}
