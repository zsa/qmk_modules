// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Standalone host test for zsa_nav_presence() (nav_presence.h).
// Build & run from the module root:
//   gcc -Wall -Wextra -o /tmp/nav_presence_test wireless/tests/nav_presence_test.c
//   /tmp/nav_presence_test
//
// Enumerates builds x cable x device placements x right-link states, derives
// the probe states the drivers and the left nRF would report, and checks all
// four output bytes against the expected values worked out from the placement.
// Then the same with a device mid-setup, then fixed golden rows.

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../nav_presence.h"

#define BALL ZSA_NAV_CAP_TRACKBALL
#define PAD  ZSA_NAV_CAP_TRACKPAD
#define UNK  ZSA_NAV_STATE_UNKNOWN

static const struct {
    const char *name;
    uint8_t     mask;
} builds[] = {
    {"none", 0},
    {"ball", BALL},
    {"pad", PAD},
    {"both", BALL | PAD},
};

// Physical placement: one device per half at most.
static const struct {
    const char *name;
    uint8_t     left;  // ZSA_NAV_CAP_* on the left connector
    uint8_t     right; // ZSA_NAV_CAP_* on the right connector
} devices[] = {
    {"none", 0, 0},
    {"ball-left", BALL, 0},
    {"ball-right", 0, BALL},
    {"pad-left", PAD, 0},
    {"pad-right", 0, PAD},
    {"ball-left+pad-right", BALL, PAD},
    {"pad-left+ball-right", PAD, BALL},
};

enum link { LINK_SETTLED, LINK_UNSETTLED, LINK_DOWN, LINK_NEVER, LINK_COUNT };
static const char *link_names[LINK_COUNT] = {"up+settled", "up-not-settled", "down", "nrf-never-answered"};

static int checks;
static int failures;

static uint8_t device_type(uint8_t mask) {
    if (mask & BALL) return ZSA_NAV_DEVICE_TRACKBALL;
    if (mask & PAD) return ZSA_NAV_DEVICE_TOUCHPAD;
    return ZSA_NAV_DEVICE_NONE;
}

// What one driver reports. Cable plugged: one net, a device on either half
// answers at the real address. Unplugged: the left one answers at the real
// address, otherwise the mirror answers (the left nRF runs ESB).
static uint8_t probe_for(uint8_t build, uint8_t kind, bool plugged, uint8_t left, uint8_t right) {
    if (!(build & kind)) return ZSA_NAV_PROBE_UNSUPPORTED;
    if (left & kind) return ZSA_NAV_PROBE_LOCAL;
    if (plugged) return (right & kind) ? ZSA_NAV_PROBE_LOCAL : ZSA_NAV_PROBE_NONE;
    return ZSA_NAV_PROBE_MIRROR;
}

static struct zsa_nav_inputs inputs_for(uint8_t build, bool plugged, uint8_t left, uint8_t right, enum link link) {
    struct zsa_nav_inputs in = {
        .build        = build,
        .ball         = probe_for(build, BALL, plugged, left, right),
        .pad          = probe_for(build, PAD, plugged, left, right),
        .right_on_bus = plugged,
        // The left nRF only knows the right while ESB runs, i.e. unplugged.
        .nrf_device = plugged ? ZSA_NAV_DEVICE_NONE : device_type(right),
    };
    switch (link) {
        case LINK_SETTLED:
            in.nrf_seen = in.link_fresh = in.right_linked = in.right_settled = true;
            break;
        case LINK_UNSETTLED:
            in.nrf_seen = in.link_fresh = in.right_linked = true;
            break;
        case LINK_DOWN:
            in.nrf_seen = in.link_fresh = true; // stale type kept on purpose
            break;
        default:
            in.nrf_device = ZSA_NAV_DEVICE_NONE;
            break;
    }
    return in;
}

static void check(const char *what, const struct zsa_nav_inputs *in, struct zsa_nav_state want) {
    struct zsa_nav_state got = zsa_nav_presence(in);
    checks++;
    if (memcmp(&got, &want, sizeof(got)) != 0) {
        failures++;
        printf("FAIL %s: got {%02x %02x %02x %02x} want {%02x %02x %02x %02x}\n", what, got.build, got.left, got.right, got.bus, want.build, want.left, want.right, want.bus);
    }
}

// Expected bytes worked out from the placement, not from probe states.
// pending: the build's ball driver is mid-setup.
static struct zsa_nav_state expect_for(uint8_t build, bool plugged, uint8_t left, uint8_t right, enum link link, bool pending) {
    struct zsa_nav_state w = {.build = build, .left = UNK, .right = UNK, .bus = UNK};
    if (build == 0) return w;

    if (plugged) {
        uint8_t seen = (left | right) & build;
        if (pending) seen &= (uint8_t)~BALL;
        if (seen != 0) {
            w.bus = seen;
        } else if (!pending) {
            w.bus = w.left = w.right = 0;
        }
        return w;
    }

    w.bus           = 0;
    uint8_t on_left = left & build;
    if (pending) on_left &= (uint8_t)~BALL;
    if (on_left != 0) {
        w.left = on_left;
    } else if (!pending) {
        w.left = 0;
    }
    if (link == LINK_SETTLED) w.right = right;
    return w;
}

static void test_matrix(bool with_pending) {
    for (size_t b = 0; b < sizeof(builds) / sizeof(builds[0]); b++) {
        if (with_pending && !(builds[b].mask & BALL)) continue;
        for (int plugged = 0; plugged < 2; plugged++) {
            for (size_t d = 0; d < sizeof(devices) / sizeof(devices[0]); d++) {
                for (int l = 0; l < LINK_COUNT; l++) {
                    struct zsa_nav_inputs in = inputs_for(builds[b].mask, plugged, devices[d].left, devices[d].right, (enum link)l);
                    if (with_pending) in.ball = ZSA_NAV_PROBE_PENDING;
                    char what[128];
                    snprintf(what, sizeof(what), "build=%s cable=%s devices=%s link=%s%s", builds[b].name, plugged ? "plugged" : "unplugged", devices[d].name, link_names[l], with_pending ? " ball-mid-setup" : "");
                    check(what, &in, expect_for(builds[b].mask, plugged, devices[d].left, devices[d].right, (enum link)l, with_pending));
                }
            }
        }
    }
}

// Rows for the reported bugs, spelled out.
static void test_golden(void) {
    struct zsa_nav_inputs in;

    // A: cable plugged, ball on the shared bus: bus says so, sides unknown.
    in = (struct zsa_nav_inputs){.build = BALL | PAD, .ball = ZSA_NAV_PROBE_LOCAL, .pad = ZSA_NAV_PROBE_NONE, .right_on_bus = true};
    check("golden A plugged ball", &in, (struct zsa_nav_state){BALL | PAD, UNK, UNK, BALL});

    // A: cable plugged, nothing anywhere: all 0.
    in = (struct zsa_nav_inputs){.build = BALL, .ball = ZSA_NAV_PROBE_NONE, .pad = ZSA_NAV_PROBE_UNSUPPORTED, .right_on_bus = true};
    check("golden A plugged empty", &in, (struct zsa_nav_state){BALL, 0, 0, 0});

    // A: cable plugged, ball still latched on the mirror: unknown, not 0.
    in = (struct zsa_nav_inputs){.build = BALL, .ball = ZSA_NAV_PROBE_MIRROR, .pad = ZSA_NAV_PROBE_UNSUPPORTED, .right_on_bus = true};
    check("golden A plugged stale mirror", &in, (struct zsa_nav_state){BALL, UNK, UNK, UNK});

    // B: boot, nRF acked "none" but the right link is not up: right unknown.
    in = (struct zsa_nav_inputs){.build = BALL, .ball = ZSA_NAV_PROBE_MIRROR, .pad = ZSA_NAV_PROBE_UNSUPPORTED, .nrf_seen = true, .link_fresh = true, .right_linked = false, .nrf_device = ZSA_NAV_DEVICE_NONE};
    check("golden B right not linked", &in, (struct zsa_nav_state){BALL, 0, UNK, 0});

    // B: link up but not settled: unknown.
    in.right_linked = true;
    check("golden B right not settled", &in, (struct zsa_nav_state){BALL, 0, UNK, 0});

    // B: settled: the nRF's "none" counts.
    in.right_settled = true;
    check("golden B right settled none", &in, (struct zsa_nav_state){BALL, 0, 0, 0});

    // B: link state stale (nRF stopped refreshing): unknown again.
    in.link_fresh = false;
    check("golden B link stale", &in, (struct zsa_nav_state){BALL, 0, UNK, 0});

    // C: pad mid-setup unplugged, nothing else: left unknown, not 0.
    in = (struct zsa_nav_inputs){.build = PAD, .ball = ZSA_NAV_PROBE_UNSUPPORTED, .pad = ZSA_NAV_PROBE_PENDING};
    check("golden C pad pending unplugged", &in, (struct zsa_nav_state){PAD, UNK, UNK, 0});

    // C: pad mid-setup plugged: everything unknown.
    in.right_on_bus = true;
    check("golden C pad pending plugged", &in, (struct zsa_nav_state){PAD, UNK, UNK, UNK});

    // C: ball up, pad mid-setup, plugged: the ball is reported on the bus.
    in = (struct zsa_nav_inputs){.build = BALL | PAD, .ball = ZSA_NAV_PROBE_LOCAL, .pad = ZSA_NAV_PROBE_PENDING, .right_on_bus = true};
    check("golden C ball up pad pending plugged", &in, (struct zsa_nav_state){BALL | PAD, UNK, UNK, BALL});

    // Unplugged with ESB off (no mirror): NONE is a real 0 on the left.
    in = (struct zsa_nav_inputs){.build = BALL | PAD, .ball = ZSA_NAV_PROBE_NONE, .pad = ZSA_NAV_PROBE_NONE};
    check("golden unplugged both none", &in, (struct zsa_nav_state){BALL | PAD, 0, UNK, 0});

    // Nav build whose drivers are both absent: nothing looked.
    in = (struct zsa_nav_inputs){.build = BALL, .ball = ZSA_NAV_PROBE_UNSUPPORTED, .pad = ZSA_NAV_PROBE_UNSUPPORTED, .right_on_bus = true};
    check("golden no driver plugged", &in, (struct zsa_nav_state){BALL, UNK, UNK, UNK});
    in.right_on_bus = false;
    check("golden no driver unplugged", &in, (struct zsa_nav_state){BALL, UNK, UNK, 0});

    // Device type mapping.
    assert(zsa_nav_device_mask(ZSA_NAV_DEVICE_NONE) == 0);
    assert(zsa_nav_device_mask(ZSA_NAV_DEVICE_TRACKBALL) == BALL);
    assert(zsa_nav_device_mask(ZSA_NAV_DEVICE_TOUCHPAD) == PAD);
    assert(zsa_nav_device_mask(0x7F) == 0);
}

// Link-state ageing through Sleep, fed into zsa_nav_presence().
#define STALE 5000u
static void test_sleep_ageing(void) {
    struct zsa_nav_link_age age = {0};
    struct zsa_nav_inputs   in  = {.build = BALL, .ball = ZSA_NAV_PROBE_MIRROR, .pad = ZSA_NAV_PROBE_UNSUPPORTED, .nrf_seen = true, .right_linked = true, .right_settled = true, .nrf_device = ZSA_NAV_DEVICE_TRACKBALL};
    const struct zsa_nav_state known = {BALL, 0, BALL, 0};
    const struct zsa_nav_state unk   = {BALL, 0, UNK, 0};

    assert(!zsa_nav_link_fresh(&age, 0, STALE));
    zsa_nav_link_received(&age, 1000);

    // Awake, no new link state: ages out.
    in.link_fresh = zsa_nav_link_fresh(&age, 1000 + STALE - 1, STALE);
    check("ageing awake fresh", &in, known);
    in.link_fresh = zsa_nav_link_fresh(&age, 1000 + STALE, STALE);
    check("ageing awake stale", &in, unk);

    // Asleep with a stale link state: the verdict held at entry is kept.
    zsa_nav_link_received(&age, 10000);
    zsa_nav_link_sleep(&age, true, 11000, STALE);
    in.link_fresh = zsa_nav_link_fresh(&age, 10000 + 60000, STALE);
    check("ageing asleep stale keeps last", &in, known);

    // Awake again before a new link state: still known, aged from the wake.
    zsa_nav_link_sleep(&age, false, 80000, STALE);
    in.link_fresh = zsa_nav_link_fresh(&age, 80000 + 900, STALE);
    check("ageing woken before new state", &in, known);
    in.link_fresh = zsa_nav_link_fresh(&age, 80000 + STALE, STALE);
    check("ageing woken then stale", &in, unk);

    // Asleep, a MISMATCH arrives: the right is not linked, unknown.
    zsa_nav_link_received(&age, 90000);
    zsa_nav_link_sleep(&age, true, 90500, STALE);
    zsa_nav_link_received(&age, 120000);
    in.link_fresh   = zsa_nav_link_fresh(&age, 150000, STALE);
    in.right_linked = false; // nrf_link_state == ZSA_LINK_STATE_MISMATCH
    in.right_settled = false;
    check("ageing asleep mismatch", &in, unk);
    in.right_linked = true;
    in.right_settled = true;

    // Already stale when Sleep began: stays unknown in Sleep and after.
    struct zsa_nav_link_age old = {0};
    zsa_nav_link_received(&old, 0);
    zsa_nav_link_sleep(&old, true, STALE + 1, STALE);
    in.link_fresh = zsa_nav_link_fresh(&old, STALE + 2, STALE);
    check("ageing stale at sleep entry", &in, unk);
    zsa_nav_link_sleep(&old, false, STALE + 3, STALE);
    in.link_fresh = zsa_nav_link_fresh(&old, STALE + 4, STALE);
    check("ageing stale at sleep entry, woken", &in, unk);

    // A fresh state received in Sleep counts on waking.
    zsa_nav_link_sleep(&old, true, STALE + 5, STALE);
    zsa_nav_link_received(&old, STALE + 6);
    in.link_fresh = zsa_nav_link_fresh(&old, STALE + 7, STALE);
    check("ageing refreshed in sleep", &in, known);
}

int main(void) {
    test_matrix(false);
    int matrix = checks;
    test_matrix(true);
    int pending = checks - matrix;
    test_golden();
    int golden = checks - matrix - pending;
    test_sleep_ageing();
    int ageing = checks - matrix - pending - golden;

    printf("nav_presence: %d scenarios (%d matrix, %d mid-setup, %d golden, %d sleep ageing), %d failed\n", checks, matrix, pending, golden, ageing, failures);
    assert(failures == 0);
    return 0;
}
