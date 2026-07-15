// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Standalone host test for the pure contact-reconciliation logic.
// Build & run from the module root:
//   gcc -Wall -o /tmp/nt_contacts_test navigator_trackpad/tests/contacts_test.c
//   /tmp/nt_contacts_test
//
// Models the host (libinput / Windows PTP) side: the host believes a contact
// is down until it is explicitly reported with tip=0. We feed sensor frames
// through nt_reconcile_contacts, apply the emitted report to a simulated host,
// and assert the host is never left with a stuck contact.

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../navigator_trackpad_contacts.h"

// --- Simulated host belief: which host contact-ids are currently down ---
// Indexed by host_id. Sized to cover both the buggy port (which forwards raw
// 6-bit sensor ids) and the fixed policy (small host ids).
static bool host_down[256];

static void host_reset(void) { memset(host_down, 0, sizeof host_down); }

// Apply one emitted report the way the host would: tip=1 asserts the contact
// down (or keeps it down), tip=0 releases it.
static void host_apply(const nt_emit_list_t *e) {
    // The HID contact-id field is only 3 bits — the host can only distinguish
    // ids 0..7, and two live contacts must never collide onto one id.
    for (uint8_t i = 0; i < e->count; i++) {
        assert(e->items[i].host_id < 8 && "host_id must fit the 3-bit HID field");
        for (uint8_t j = i + 1; j < e->count; j++) {
            assert(e->items[i].host_id != e->items[j].host_id &&
                   "two contacts collided onto one host id in a single frame");
        }
    }
    for (uint8_t i = 0; i < e->count; i++) {
        host_down[e->items[i].host_id] = e->items[i].tip;
    }
}

static int host_count(void) {
    int n = 0;
    for (int i = 0; i < 256; i++) {
        if (host_down[i]) n++;
    }
    return n;
}

// The bug: three fingers (ids 10/20/30) swirling. The sensor only tracks two
// at a time and rotates which two it reports. When all fingers lift, the host
// must believe nothing is down. The host must also never believe more contacts
// are down than the transport can carry.
static void test_three_finger_swirl_no_stuck(void) {
    nt_contact_state_t st = {0};
    nt_emit_list_t     e;
    host_reset();

    // F1: fingers A(10) and B(20) on the pad.
    nt_sensor_contact_t f1[] = {{10, 100, 100, true}, {20, 200, 200, true}};
    nt_reconcile_contacts(&st, f1, 2, &e);
    host_apply(&e);
    assert(host_count() <= NT_MAX_CONTACTS);

    // F2: sensor swaps A out for C(30) while A is still physically down.
    nt_sensor_contact_t f2[] = {{20, 210, 210, true}, {30, 300, 300, true}};
    nt_reconcile_contacts(&st, f2, 2, &e);
    host_apply(&e);
    assert(host_count() <= NT_MAX_CONTACTS);  // buggy port leaves A stuck -> 3

    // F3: sensor now reports only C.
    nt_sensor_contact_t f3[] = {{30, 310, 310, true}};
    nt_reconcile_contacts(&st, f3, 1, &e);
    host_apply(&e);
    assert(host_count() <= NT_MAX_CONTACTS);

    // F4: all fingers lifted.
    nt_reconcile_contacts(&st, NULL, 0, &e);
    host_apply(&e);
    assert(host_count() == 0 && "a contact is stuck down after lift-off");
}

// One finger down then up leaves nothing stuck.
static void test_single_finger(void) {
    nt_contact_state_t st = {0};
    nt_emit_list_t     e;
    host_reset();

    nt_sensor_contact_t f1[] = {{7, 500, 500, true}};
    nt_reconcile_contacts(&st, f1, 1, &e);
    host_apply(&e);
    assert(host_count() == 1);

    nt_reconcile_contacts(&st, NULL, 0, &e);
    host_apply(&e);
    assert(host_count() == 0);
}

// A clean two-finger gesture: both down, both lift together.
static void test_two_finger_clean(void) {
    nt_contact_state_t st = {0};
    nt_emit_list_t     e;
    host_reset();

    nt_sensor_contact_t f1[] = {{3, 100, 100, true}, {4, 900, 900, true}};
    nt_reconcile_contacts(&st, f1, 2, &e);
    host_apply(&e);
    assert(host_count() == 2);

    nt_reconcile_contacts(&st, NULL, 0, &e);
    host_apply(&e);
    assert(host_count() == 0);
}

// Two fingers whose sensor ids alias under a 3-bit mask (1 and 9 -> both 1)
// must still map to distinct host ids.
static void test_no_host_id_alias(void) {
    nt_contact_state_t st = {0};
    nt_emit_list_t     e;
    host_reset();

    nt_sensor_contact_t f1[] = {{1, 100, 100, true}, {9, 900, 900, true}};
    nt_reconcile_contacts(&st, f1, 2, &e);
    host_apply(&e);  // asserts <8 and distinct internally
    assert(host_count() == 2);

    nt_reconcile_contacts(&st, NULL, 0, &e);
    host_apply(&e);
    assert(host_count() == 0);
}

int main(void) {
    test_single_finger();
    test_two_finger_clean();
    test_no_host_id_alias();
    test_three_finger_swirl_no_stuck();
    printf("All contact tests passed\n");
    return 0;
}
