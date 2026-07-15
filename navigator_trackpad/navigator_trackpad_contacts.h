// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Pure, host-testable contact reconciliation for the Navigator trackpad.
//
// The Cirque Gen6 sensor and our HID descriptor each track only two
// simultaneous contacts. The host (libinput / Windows PTP) tracks contacts by
// contact-id and only considers one lifted when it is explicitly reported with
// tip=0. The job here is to reconcile the contacts the host currently believes
// are down against the contacts the sensor reports this frame, and produce the
// fingers to emit — without ever leaving the host with a contact it thinks is
// down but we have stopped accounting for (a "stuck finger").

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifndef NT_MAX_CONTACTS
#    define NT_MAX_CONTACTS 2
#endif

// A contact as read from the sensor this frame, already scaled and rotated.
typedef struct {
    uint8_t  id;    // sensor's stable per-finger id (Cirque: 0..63)
    uint16_t x;
    uint16_t y;
    bool     conf;  // confidence (real contact vs noise)
} nt_sensor_contact_t;

// A contact the host currently believes is down. Keyed to the sensor by
// sensor_id; reported to the host with our own small stable host_id (the HID
// contact-id field is only 3 bits, so the raw 6-bit sensor id is never sent).
typedef struct {
    uint8_t  sensor_id;
    uint8_t  host_id;
    uint16_t x;
    uint16_t y;
    bool     conf;
} nt_host_contact_t;

// The set of contacts the host believes are down. Persisted across frames.
typedef struct {
    nt_host_contact_t items[NT_MAX_CONTACTS];
    uint8_t           count;
} nt_contact_state_t;

// One emitted finger in the outgoing HID report.
typedef struct {
    uint8_t  host_id;
    uint16_t x;
    uint16_t y;
    bool     tip;   // 1 = down, 0 = clean lift (release)
    bool     conf;
} nt_emit_contact_t;

typedef struct {
    nt_emit_contact_t items[NT_MAX_CONTACTS];
    uint8_t           count;
} nt_emit_list_t;

// Reconcile the host-believed-down set (st, updated in place) against the
// sensor contacts reported down this frame (cur, cur_n), producing the fingers
// to emit (out).
//
// Conservative 2-slot policy. The sensor and transport only track two contacts,
// so a third finger can never be faithfully represented — putting more than two
// fingers down is degraded input, and the only hard requirement is that we
// never strand a contact on the host:
//
//   1. Every contact the host believes down that the sensor no longer reports
//      is released this frame with tip=0. A contact is NEVER dropped from the
//      tracked set without first emitting its release, so the host can never be
//      left thinking a finger is still down (the "stuck finger" bug).
//   2. Continuing contacts keep their stable host_id and take updated position.
//   3. Brand-new contacts are picked up only as free slots allow. When a swap
//      fills both slots with a release plus a continuing contact, the new finger
//      simply waits for a later frame rather than evicting a live contact — this
//      keeps reported contacts calm instead of churning every frame.
//
// host_ids are assigned by us from [0, NT_MAX_CONTACTS) and kept stable for a
// contact's lifetime, so they always fit the 3-bit HID contact-id field and two
// live contacts never collide (the raw 6-bit sensor id is never forwarded).
static inline void nt_reconcile_contacts(nt_contact_state_t *st,
                                         const nt_sensor_contact_t *cur, uint8_t cur_n,
                                         nt_emit_list_t *out) {
    out->count = 0;
    if (cur_n > NT_MAX_CONTACTS) cur_n = NT_MAX_CONTACTS;

    bool               cur_used[NT_MAX_CONTACTS] = {0};
    nt_contact_state_t next                      = {0};

    // 1+2. Walk the host-believed-down set: continue contacts the sensor still
    //      reports, release (tip=0) the ones it no longer does.
    for (uint8_t i = 0; i < st->count; i++) {
        int match = -1;
        for (uint8_t j = 0; j < cur_n; j++) {
            if (!cur_used[j] && cur[j].id == st->items[i].sensor_id) { match = j; break; }
        }
        nt_emit_contact_t *e = &out->items[out->count++];
        e->host_id = st->items[i].host_id;
        if (match >= 0) {
            cur_used[match] = true;
            e->x    = cur[match].x;
            e->y    = cur[match].y;
            e->conf = cur[match].conf;
            e->tip  = true;
            // Keep the contact, with refreshed position, for next frame.
            nt_host_contact_t *k = &next.items[next.count++];
            k->sensor_id = st->items[i].sensor_id;
            k->host_id   = st->items[i].host_id;
            k->x         = cur[match].x;
            k->y         = cur[match].y;
            k->conf      = cur[match].conf;
        } else {
            // Vanished — emit a clean lift at its last known position and drop.
            e->x    = st->items[i].x;
            e->y    = st->items[i].y;
            e->conf = st->items[i].conf;
            e->tip  = false;
        }
    }

    // 3. Pick up brand-new contacts into any remaining slots.
    for (uint8_t j = 0; j < cur_n && out->count < NT_MAX_CONTACTS; j++) {
        if (cur_used[j]) continue;

        // Lowest host_id in [0, NT_MAX_CONTACTS) not held by a kept contact.
        uint8_t host_id = 0;
        for (uint8_t h = 0; h < NT_MAX_CONTACTS; h++) {
            bool taken = false;
            for (uint8_t k = 0; k < next.count; k++) {
                if (next.items[k].host_id == host_id) { taken = true; break; }
            }
            if (!taken) break;
            host_id++;
        }

        nt_host_contact_t *k = &next.items[next.count++];
        k->sensor_id = cur[j].id;
        k->host_id   = host_id;
        k->x         = cur[j].x;
        k->y         = cur[j].y;
        k->conf      = cur[j].conf;

        nt_emit_contact_t *e = &out->items[out->count++];
        e->host_id = host_id;
        e->x       = cur[j].x;
        e->y       = cur[j].y;
        e->tip     = true;
        e->conf    = cur[j].conf;
    }

    *st = next;
}
