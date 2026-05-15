// Copyright 2026 ZSA Technology Labs, Inc <contact@zsa.io>
// SPDX-License-Identifier: GPL-2.0-or-later

#include <string.h>
#include "navigator_trackpad_core.h"

#define CGEN6_PTP_REPORT_ID   0x01
#define CGEN6_MOUSE_REPORT_ID 0x06

bool navigator_trackpad_core_decode(const uint8_t bytes[NAV_TRACKPAD_PACKET_LEN],
                                    cgen6_report_t *report)
{
    if (!report) return false;
    memset(report, 0, sizeof(*report));

    uint8_t report_id = bytes[2];

    if (report_id == CGEN6_PTP_REPORT_ID) {
        report->fingers[0].confidence = bytes[3] & 0x01;
        report->fingers[0].tip        = (bytes[3] & 0x02) >> 1;
        report->fingers[0].id         = (bytes[3] & 0xFC) >> 2;
        report->fingers[0].x          = (uint16_t)bytes[5] << 8 | bytes[4];
        report->fingers[0].y          = (uint16_t)bytes[7] << 8 | bytes[6];
        report->fingers[1].confidence = bytes[8] & 0x01;
        report->fingers[1].tip        = (bytes[8] & 0x02) >> 1;
        report->fingers[1].id         = (bytes[8] & 0xFC) >> 2;
        report->fingers[1].x          = (uint16_t)bytes[10] << 8 | bytes[9];
        report->fingers[1].y          = (uint16_t)bytes[12] << 8 | bytes[11];
        report->scan_time             = (uint16_t)bytes[14] << 8 | bytes[13];
        report->contact_count         = bytes[15];
        report->buttons               = bytes[16];
        return true;
    }
    if (report_id == CGEN6_MOUSE_REPORT_ID) {
        report->buttons     = bytes[3];
        report->xDelta      = (int8_t)bytes[4];
        report->yDelta      = (int8_t)bytes[5];
        report->scrollDelta = (int8_t)bytes[6];
        report->panDelta    = (int8_t)bytes[7];
        return true;
    }
    return false;
}
