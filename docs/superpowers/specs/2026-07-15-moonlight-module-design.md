# Moonlight — QMK Community Module for Room Lighting

**Date:** 2026-07-15
**Status:** Approved design, pending implementation plan
**Repo:** `zsa/qmk_modules` (this repo), new module `moonlight/`
**Firmware base:** `zsa/qmk_firmware`, branch `firmware25` (latest stable ZSA fork; supports community modules)

## Purpose

Upcycle broken ZSA keyboards as USB-powered room lights. A keyboard whose
matrix is too damaged for typing usually still has fully working LEDs and
power circuitry. Moonlight turns it into a lamp:

1. The LEDs turn on as soon as the board receives USB power — including from
   a dumb wall charger or power bank, with no computer and no USB enumeration.
2. A set of assignable keycodes controls the light: on/off, brightness, hue,
   preset colors, and animations (start/stop/next/faster/slower).
3. The board is **lamp-only by default**: it never sends keystrokes to a
   host, so a shorting matrix can't type garbage into a computer.

Moonlight is purely for light control. It is not a keyboard-lighting /
typing-feedback feature, and the README must say so up front.

## Why a community module

- Community modules (QMK ≥ 25 / ZSA `firmware25`) can declare keycodes,
  provide hooks (`keyboard_post_init`, `pre_process_record`,
  `process_record`), and ship a `config.h` that participates in the build's
  config chain. That covers every requirement with **zero changes to the
  firmware repo**.
- The previous plan (a userspace in the `qmk_for_lighting` fork of
  `firmware23`) is retired. `firmware23` was a mistake; it predates module
  support. The `qmk_for_lighting` repo is no longer needed.

## Git hygiene

All work must go in the community modules repo in a branch called `feat/moonlight`. Keep commits tidy and easy to review, batch related changes together. It is important that the whole change ships as a cohesive pull request that makes sense. Avoid overengineering.

## Target boards

Moonlander (`keyboards/zsa/moonlander`) and Voyager (`keyboards/zsa/voyager`)
on `firmware25`. The module itself is board-agnostic: it requires only
`RGB_MATRIX_ENABLE` and errors out at compile time without it. Other
rgb-matrix boards (e.g. Planck EZ) should work but are untested/out of scope.

## Module layout

```
moonlight/
├── qmk_module.json     # manifest: module_name "moonlight",
│                       # features { rgb_matrix: true }, keycodes list
├── config.h            # NO_USB_STARTUP_CHECK + tunable defaults
├── moonlight.c         # all behavior: hooks + keycode handlers
├── examples/
│   ├── voyager_lamp.json     # complete keymap.json, "modules": ["moonlight"]
│   └── moonlander_lamp.json
└── README.md
```

Users hand-edit a keymap.json (typically starting from an example), keep
`"modules": ["moonlight"]`, arrange the keycodes on whichever physical keys
still work, and build with `qmk compile`.

## Standalone power (works on a wall charger)

**Mechanism (verified against `firmware25` sources):**

- Boot does not wait for enumeration: `WAIT_FOR_USB` is not defined for
  these boards, so `protocol_pre_init()` proceeds immediately.
- The only thing that darkens the board without a host is the suspend trap
  in `tmk_core/protocol/chibios/chibios.c` (~line 184): the STM32F303 USB
  peripheral raises SUSP after ~3 ms of bus idle (always true on a charger),
  ChibiOS marks the driver `USB_SUSPENDED`, and the main loop spins in
  `suspend_power_down()` forever (remote wakeup is never negotiated without
  enumeration). This kills the LEDs and key scanning.
- That entire block is guarded by `#if !defined(NO_USB_STARTUP_CHECK)`.
  Moonlight's `config.h` defines **`NO_USB_STARTUP_CHECK`**. Module config.h
  files are added to the build's config chain (`build_keyboard.mk`,
  `config_h_community_module_appender`), and the define is purely additive.
- With the suspend block compiled out, no other path touches the LEDs:
  `usb_event_suspend_handler()` only records device state (no `SLEEP_LED` on
  these boards), and Voyager's `"sleep": true` (`RGB_MATRIX_SLEEP`) acts only
  through `suspend_power_down_quantum()`, which is now never called.

**Consequences (accepted, documented in README):**

- The board ignores USB suspend entirely: plugged into a computer that goes
  to sleep, the lamp stays on and keeps drawing full LED current.
- Drawing LED-level current from a port without enumerating exceeds the
  USB unconfigured-device budget (100 mA). Chargers don't care; this is a
  lamp, not a certified USB device.
- USB still enumerates normally when a real host is present, so reflashing
  keeps working.

## Keycodes and behavior model

The light is always in one of two shapes:

- **Steady** — `RGB_MATRIX_SOLID_COLOR` at the current HSV.
- **Animating** — any enabled, non-reactive rgb_matrix animation.

All persistent state lives in QMK's existing `rgb_matrix_config`
(enable/mode/HSV/speed), saved to EEPROM through the standard
eeprom-persisting API variants. The only module state is one RAM variable:
the last animation mode used (so "stop, then start" resumes the same
animation within a power session).

| Keycode | Alias | Behavior |
|---|---|---|
| `MOONLIGHT_ON` | `MNL_ON` | `rgb_matrix_enable()` |
| `MOONLIGHT_OFF` | `MNL_OFF` | `rgb_matrix_disable()` |
| `MOONLIGHT_BRIGHTER` | `MNL_BRI` | brightness up one step |
| `MOONLIGHT_DIMMER` | `MNL_DIM` | brightness down one step, floored (see guardrails) |
| `MOONLIGHT_HUE_UP` | `MNL_HUU` | hue wheel forward |
| `MOONLIGHT_HUE_DOWN` | `MNL_HUD` | hue wheel backward |
| `MOONLIGHT_ANIM_START` | `MNL_AST` | switch to last-used animation (default: breathing) |
| `MOONLIGHT_ANIM_STOP` | `MNL_ASP` | freeze into steady solid color at current HSV |
| `MOONLIGHT_ANIM_NEXT` | `MNL_ANX` | next animation, skipping reactive effects (runtime skip) |
| `MOONLIGHT_ANIM_FASTER` | `MNL_FST` | animation speed up |
| `MOONLIGHT_ANIM_SLOWER` | `MNL_SLW` | animation speed down |
| `MOONLIGHT_PRESET_1`…`_8` | `MNL_P1`…`MNL_P8` | steady mode + jump to preset HSV |

Notes:

- Hue/brightness changes apply in both shapes (they recolor animations too,
  where the effect uses the base HSV).
- `MOONLIGHT_ANIM_NEXT` was added beyond the original request because
  without it only one animation is ever reachable. No toggle keycode:
  on/off are discrete by design.
- Preset colors are defined per keymap in its `config.h`:
  `#define MOONLIGHT_PRESET_1 {HSV_CORAL}` (any `{h, s, v}` triple, so warm
  white via low saturation is possible). Unset slots get built-in defaults.
  Pressing a preset always lands in steady mode.

## Reactive-animation exclusion

Reactive effects make no sense on a lamp and must never be reachable via
`MOONLIGHT_ANIM_NEXT` / `MOONLIGHT_ANIM_START`. Because a module's config.h
is included *before* the keyboard's generated config, the module cannot
compile them out; instead the module skips them at runtime against a
compile-time table of mode IDs, each entry guarded by its `#ifdef` (only
enabled effects have enum values):

- `SOLID_REACTIVE_SIMPLE`, `SOLID_REACTIVE`, `SOLID_REACTIVE_WIDE`,
  `SOLID_REACTIVE_MULTIWIDE`, `SOLID_REACTIVE_CROSS`,
  `SOLID_REACTIVE_MULTICROSS`, `SOLID_REACTIVE_NEXUS`,
  `SOLID_REACTIVE_MULTINEXUS`, `SPLASH`, `MULTISPLASH`, `SOLID_SPLASH`,
  `SOLID_MULTISPLASH`
- `TYPING_HEATMAP` (framebuffer effect, but keypress-driven — reactive for
  our purposes)
- `SOLID_COLOR` is also skipped by the cycler: it is the "steady" state,
  reached via `MOONLIGHT_ANIM_STOP`, not part of the animation carousel.

Entries for effects a given board doesn't enable simply compile away via
their `#ifdef` guards, so the table is safe on any configuration.

## Power-up behavior

`keyboard_post_init_moonlight()` runs after EEPROM state is restored:

1. Force `rgb_matrix_enable()` — a lamp on a wall switch always comes on,
   even if it was off when unplugged.
2. If restored brightness is below `MOONLIGHT_MIN_BOOT_BRIGHTNESS`
   (default 40 of 255), raise it to that floor — never boot dark.
3. If the restored mode is reactive or out of range (e.g. the board was
   previously flashed with different firmware), snap to steady solid color.

Everything else (hue, speed, chosen animation) restores exactly as last set.

## Lamp-only mode

`pre_process_record_moonlight()` runs before any other keycode processing:

- Moonlight keycodes: pass through (handled in `process_record_moonlight()`).
- `QK_BOOT`: pass through, so a keymap can keep a flash key.
- Everything else: consumed (`return false`) — nothing ever reaches the
  host. Bootmagic (hold key while plugging in) is unaffected, as it runs
  before keymap processing.

Controlled by `MOONLIGHT_LAMP_ONLY`, **default on**. A keymap that wants
light control on a *working* keyboard can `#define MOONLIGHT_LAMP_ONLY 0`
in its config.h (keymap config is included after module config, so the
override works).

## Configuration surface (module `config.h` defaults)

| Define | Default | Meaning |
|---|---|---|
| `MOONLIGHT_LAMP_ONLY` | `1` | swallow all non-moonlight keycodes |
| `MOONLIGHT_MIN_BOOT_BRIGHTNESS` | `40` | brightness floor applied at power-up |
| `MOONLIGHT_MIN_BRIGHTNESS` | `16` | floor for `MOONLIGHT_DIMMER` (dark ≠ off; use `MNL_OFF`) |
| `MOONLIGHT_DEFAULT_ANIMATION` | breathing | animation used by first `MNL_AST` of a session; falls back to the first enabled non-reactive animation if breathing is disabled |
| `MOONLIGHT_PRESET_1`…`_8` | built-in palette | per-keymap HSV preset colors |

Plus the non-tunable `NO_USB_STARTUP_CHECK`.

## Error handling / guardrails

- `#error` at compile time if `RGB_MATRIX_ENABLE` is not set, with a message
  naming the module and the requirement.
- Brightness stepping clamps to `[MOONLIGHT_MIN_BRIGHTNESS, max_brightness]`
  (the boards already cap max via `rgb_matrix.max_brightness`).
- Animation cycling wraps modulo the enabled-effect count and re-skips until
  it lands on a valid non-reactive mode — this also protects against the
  out-of-bounds-mode class of bug previously patched in `firmware23`
  (stale EEPROM mode after flashing a build with fewer animations).
- All EEPROM writes go through QMK's debounced eeconfig API (as the existing
  RGB keycodes do), so holding a repeat key doesn't thrash flash-backed
  EEPROM emulation.

## Testing

Honest scope for firmware: no unit-test rig exists for community modules,
so coverage is compile verification plus structured hardware verification.
(This is a deliberate, documented deviation from the user's global 80 %
TDD rule — on-hardware firmware behavior is the wrong place to force it.)

1. **Build checks (automatable):** compile both `examples/*.json` against
   `zsa/qmk_firmware@firmware25`. The qmk_modules repo currently has no CI,
   so add a GitHub Action that checks out `firmware25`, clones this repo
   into `modules/`, and runs `qmk compile` on both example keymaps.
2. **Hardware checklist (in README):**
   - Boots lit from a dumb USB charger / power bank.
   - Every keycode behaves per the table above.
   - Power-cycle restores color/brightness/animation; light is ON after
     power-up even if turned off before unplugging.
   - Plugged into a computer: no keystrokes ever registered (lamp-only);
     light stays on when the computer sleeps.
   - Reflashing works (QK_BOOT key and/or physical reset).

## Out of scope

- ErgoDox EZ / Planck EZ support (module should largely work on any
  rgb_matrix board, but only Moonlander and Voyager are verified).
- Oryx integration or new Oryx-assignable keycodes.
- Pausing an animation mid-frame (stop = freeze into steady color).
- Any changes to `zsa/qmk_firmware` core.

## Subagents

Use subagents to implement this. Use smaller/weaker models as subagent subject to your judgment and review. Exercise autonomy, this spec is a goal.