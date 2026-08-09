/*
 * zsa_right_telemetry.h -- what the right half says about itself, wired
 *
 * Shared between nRF54 (Zephyr, voyager_right/src/twis0.c) and STM32 (QMK,
 * keyboards/zsa/voyager_tng/voyager_tng.c). No dependencies on either
 * framework. Kept byte-identical in both trees, like zsa_uart_proto.h.
 *
 * ===========================================================================
 * WHY THIS EXISTS
 * ===========================================================================
 *
 * Wireless, the right half's state rides the ESB snapshot to the left, which
 * relays it to the STM32 as EVT_BATTERY. Wired, the right suspends ESB while
 * VBUS is present (ptx_main.c, vbus_state_worker), so that path carries
 * nothing and a plugged-in keyboard cannot report its own right half at all.
 *
 * This is the wired-mode source, and only that. The right's TWIS0 slave
 * (7-bit ZSA_RIGHT_TLM_I2C_ADDR on i2c21) sits on the TRRS net, stays enabled
 * regardless of VBUS, and the STM32 masters that bus whenever the cable is
 * plugged. Any I2C read from the address returns this struct from offset 0 --
 * there is no register pointer to set first, so a plain read is the whole
 * transaction. Writes to the address keep their existing meaning (0x04 is the
 * command register: bootloader hold, VBUS ILIM).
 *
 * ===========================================================================
 * ADDING A FIELD
 * ===========================================================================
 *
 * Append it where reserved[] starts, shrink reserved[] by the same size, and
 * raise ZSA_RIGHT_TLM_LEN. Leave ZSA_RIGHT_TLM_VERSION alone: `len` is what a
 * reader tests (ZSA_RIGHT_TLM_HAS), so an older STM32 reading a newer right
 * half stops at the fields it knows, and a newer STM32 reading an older right
 * half sees a short `len` and calls the missing fields unknown instead of
 * reading whatever was there. Raise the version only when a field that has
 * shipped changes meaning -- the one thing `len` cannot express. Never
 * renumber an offset and never reuse one.
 *
 * Every field carries its own "could not read it" value. A level we failed to
 * get must never arrive as 0: that reads as a flat battery.
 */

#ifndef ZSA_RIGHT_TELEMETRY_H
#define ZSA_RIGHT_TELEMETRY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 7-bit TWIS address of the right nRF. Also its I2C DFU / command endpoint. */
#define ZSA_RIGHT_TLM_I2C_ADDR 0x54u

/* Bumped only when a shipped field changes meaning. See ADDING A FIELD. */
#define ZSA_RIGHT_TLM_VERSION 1u

/* Bytes defined after the `len` byte itself. Grows as fields are appended. */
#define ZSA_RIGHT_TLM_LEN 2u

/* The right half has no usable estimate: the gauge has not converged yet, or
 * the PMIC did not answer. Never confuse this with 0%. */
#define ZSA_RIGHT_TLM_SOC_UNKNOWN 0xFFu

/* `flags` bits. Meaningful whenever the read itself succeeded -- both come
 * from state the nRF holds locally, so there is no "unknown" case for them. */
#define ZSA_RIGHT_TLM_FLAG_CHARGING 0x01u /* current actually entering the cell */
#define ZSA_RIGHT_TLM_FLAG_USB      0x02u /* VBUS present on the right half */

struct __attribute__((packed)) zsa_right_telemetry {
	uint8_t version;     /* ZSA_RIGHT_TLM_VERSION */
	uint8_t len;         /* defined bytes after this one; ZSA_RIGHT_TLM_LEN */
	uint8_t soc;         /* 0-100, or ZSA_RIGHT_TLM_SOC_UNKNOWN */
	uint8_t flags;       /* ZSA_RIGHT_TLM_FLAG_* */
	uint8_t reserved[4]; /* zero; room for four more one-byte fields */
};

#define ZSA_RIGHT_TLM_SIZE sizeof(struct zsa_right_telemetry)

/* True when the sender's `len` reaches far enough to cover `field`. The two
 * added bytes are `version` and `len`, which every version carries. */
#define ZSA_RIGHT_TLM_HAS(t, field)                                                                \
	(offsetof(struct zsa_right_telemetry, field) +                                             \
		 sizeof(((const struct zsa_right_telemetry *)0)->field) <=                         \
	 (size_t)(t)->len + 2u)

/* Sender side: a struct with every field at its unknown value, ready to have
 * the ones we can actually read filled in. */
static inline void zsa_right_telemetry_init(struct zsa_right_telemetry *t)
{
	t->version = ZSA_RIGHT_TLM_VERSION;
	t->len = ZSA_RIGHT_TLM_LEN;
	t->soc = ZSA_RIGHT_TLM_SOC_UNKNOWN;
	t->flags = 0u;
	for (size_t i = 0u; i < sizeof(t->reserved); ++i) {
		t->reserved[i] = 0u;
	}
}

/* Reader side. A struct from a right half whose major version we do not know
 * is refused whole: only `len` degrades safely, not the version. */
static inline bool zsa_right_telemetry_ok(const struct zsa_right_telemetry *t)
{
	return t->version == ZSA_RIGHT_TLM_VERSION;
}

/* State of charge in percent, or -1 when the right half could not read it,
 * did not send the field, or sent one out of range. */
static inline int zsa_right_telemetry_soc(const struct zsa_right_telemetry *t)
{
	if (!zsa_right_telemetry_ok(t) || !ZSA_RIGHT_TLM_HAS(t, soc)) {
		return -1;
	}
	if (t->soc == ZSA_RIGHT_TLM_SOC_UNKNOWN || t->soc > 100u) {
		return -1;
	}
	return (int)t->soc;
}

/* Charging/USB bits, or false when the field is not there -- so a caller
 * cannot read "not charging" out of a firmware that never said. */
static inline bool zsa_right_telemetry_flags(const struct zsa_right_telemetry *t, uint8_t *out)
{
	if (!zsa_right_telemetry_ok(t) || !ZSA_RIGHT_TLM_HAS(t, flags)) {
		return false;
	}
	*out = t->flags;
	return true;
}

#ifdef __cplusplus
}
#endif

#endif /* ZSA_RIGHT_TELEMETRY_H */
