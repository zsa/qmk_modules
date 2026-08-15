# moonlight

Moonlight turns a keyboard — typically one whose matrix is too broken to
type on — into a USB-powered room light. That's the whole feature.

**This is not a keyboard-lighting or typing-feedback module.** It doesn't
react to keypresses, it doesn't animate on your typing, and by default it
never sends a single keystroke to a computer. It just lights up and stays
lit, like a lamp.

## How it works

- The LEDs turn on as soon as the board gets USB power — a wall charger, a
  power bank, or a computer. No enumeration required, no host needed.
- Color, brightness, and animation state persist across power cycles
  (stored in the same EEPROM-backed `rgb_matrix_config` QMK already uses).
- The light always comes on at power-up, even if it was switched off before
  the board was last unplugged. A lamp on a wall switch doesn't remember
  "off".

## Keycodes

All keycodes act on key-down only; the release is swallowed.

| Keycode | Alias | Behavior |
|---|---|---|
| `MOONLIGHT_ON` | `MNL_ON` | Turn the light on |
| `MOONLIGHT_OFF` | `MNL_OFF` | Turn the light off |
| `MOONLIGHT_BRIGHTER` | `MNL_BRI` | Brightness up one step |
| `MOONLIGHT_DIMMER` | `MNL_DIM` | Brightness down one step, floored (see `MOONLIGHT_MIN_BRIGHTNESS` below — dim is not the same as off, use `MNL_OFF` for that) |
| `MOONLIGHT_HUE_UP` | `MNL_HUU` | Hue wheel forward |
| `MOONLIGHT_HUE_DOWN` | `MNL_HUD` | Hue wheel backward |
| `MOONLIGHT_ANIM_START` | `MNL_AST` | Switch to the last-used animation (default: breathing) |
| `MOONLIGHT_ANIM_STOP` | `MNL_ASP` | Freeze into steady solid color at the current HSV |
| `MOONLIGHT_ANIM_NEXT` | `MNL_ANX` | Next animation, skipping reactive/keypress-driven effects |
| `MOONLIGHT_ANIM_FASTER` | `MNL_FST` | Animation speed up |
| `MOONLIGHT_ANIM_SLOWER` | `MNL_SLW` | Animation speed down |
| `MOONLIGHT_PRESET_1` … `MOONLIGHT_PRESET_8` | `MNL_P1` … `MNL_P8` | Steady mode + jump to preset hue/sat (turns light on; brightness preserved) |

Notes:

- Hue and brightness changes apply whether the light is steady or
  animating — they recolor animations too, wherever the effect uses the
  base HSV.
- There's no on/off toggle keycode; `MNL_ON` and `MNL_OFF` are discrete by
  design.
- Presets change **color only**: pressing a preset applies its hue and
  saturation and always lands in steady mode, but keeps whatever brightness
  the light is currently at. The `v` (brightness) component in an
  `MOONLIGHT_PRESET_n_HSV` definition is accepted for convenience with
  QMK's `HSV_*` macros but is otherwise ignored — presets never cause a
  brightness jump.
- Presets also turn the light on, the same way `MNL_AST` does: pressing a
  preset while the light is off still lands you in that color, steady and
  lit, rather than leaving the light off.

## Quick start

From a `qmk_firmware` checkout on branch `firmware25`, with this repo
checked out at `modules/zsa` (i.e. `modules/zsa/moonlight/` is this
directory):

```sh
qmk compile modules/zsa/moonlight/examples/voyager_lamp.json
```

or, for a Moonlander (note the `revb` — bare `zsa/moonlander` isn't a
buildable target):

```sh
qmk compile modules/zsa/moonlight/examples/moonlander_lamp.json
```

Flash the resulting firmware with Keymapp, or directly with
`qmk flash modules/zsa/moonlight/examples/voyager_lamp.json`.

Both example keymaps lay out the full keycode set on a single layer (see
"Layer-switching keys" below for why it has to be one layer):

- Row 1 (top): `MNL_ON`, `MNL_OFF`, `MNL_BRI`, `MNL_DIM`, with `QK_BOOT`
  on the far top-right corner key so the board stays reflashable
- Row 2: `MNL_HUU`, `MNL_HUD`
- Row 3: `MNL_AST`, `MNL_ASP`, `MNL_ANX`, `MNL_FST`, `MNL_SLW`
- Row 4: `MNL_P1` through `MNL_P8`
- Every other position is `KC_NO`

In `examples/moonlander_lamp.json`, the flat `layers` array's grouping
around indices 54–71 (grouped 5/1/1/5/3/3 entries per line) looks odd if
you're skimming it — that's not a mistake. It mirrors the physical
argument order of the Moonlander's `LAYOUT` macro, including two
single-entry lines for the interleaved thumb keys. Leave the shape alone
if you hand-edit the file; only the values matter.

## Customization

Config defines, set in a keymap's own `config.h`. The module's own
`config.h` only carries `NO_USB_STARTUP_CHECK`; these tunables default in
`moonlight.c` behind `#ifndef` guards specifically so that a keymap-level
`config.h` define (processed after the module's) overrides them:

| Define | Default | Meaning |
|---|---|---|
| `MOONLIGHT_LAMP_ONLY` | `1` | Swallow every non-moonlight keycode (see below). Set to `0` on a working keyboard to allow normal typing alongside light control. |
| `MOONLIGHT_MIN_BOOT_BRIGHTNESS` | `40` | Brightness floor applied at power-up — the lamp never boots dark. |
| `MOONLIGHT_MIN_BRIGHTNESS` | `16` | Floor for `MOONLIGHT_DIMMER` — dimming stops here rather than going fully dark; use `MNL_OFF` to actually turn the light off. |
| `MOONLIGHT_DEFAULT_ANIMATION` | breathing | Animation used the first time `MNL_AST` runs in a power session. Falls back to the first enabled non-reactive animation if breathing isn't enabled on the board. |
| `MOONLIGHT_PRESET_1_HSV` … `MOONLIGHT_PRESET_8_HSV` | built-in palette (red, coral, gold, green, azure, blue, purple, white) | Per-keymap preset colors, e.g. `#define MOONLIGHT_PRESET_1_HSV {HSV_TEAL}`. Any `{h, s, v}` triple works, including `HSV_*` macros; only hue and saturation are actually applied (see the preset note above) — the `v` is accepted but ignored. Unset slots keep their default. |

If you hand-roll a keymap.json instead of starting from an example, keep
`"modules": ["zsa/defaults", "zsa/moonlight"]` — `zsa/defaults` is
required by ZSA board code and isn't optional, even on a lamp-only build.

To remap keys, edit the `layers` array in your keymap json (or in
`keymap.c` if you're not using QMK Configurator json format) and place any
of the keycodes/aliases above wherever a physical key still works — you
don't need to fill every position, unused keys can stay `KC_NO`.

### Layer-switching keys

In lamp-only mode (the default), **every** non-moonlight keycode is
swallowed — including layer-switching keys like `MO()`, `TT()`, or `DF()`.
A second layer is simply unreachable, so don't design a layout that
depends on one. Put every moonlight keycode you want on a single layer.

## Caveats

- **Ignores USB suspend.** When plugged into a computer that goes to
  sleep, the lamp stays lit and keeps drawing full LED current instead of
  powering down. This is intentional — it's what makes standalone
  operation on a dumb charger work at all.
- **Exceeds the USB unconfigured-device power budget by design.** Drawing
  LED-level current without ever enumerating goes over the 100 mA budget
  a compliant USB device is supposed to observe before configuration.
  Chargers and power banks don't enforce that budget, so this is fine in
  practice — this is a lamp, not a certified USB peripheral.
- **Reflashing still works normally.** Plugged into a real host, USB
  enumerates as usual, so `QK_BOOT` (included in both example keymaps) or
  the board's physical reset button both work exactly as they would on
  any other build.

## Hardware test checklist

After flashing, verify on real hardware:

1. Boots lit from a dumb USB charger / power bank (no computer attached).
2. Every keycode in the table above behaves as described.
3. Power-cycling restores color, brightness, and animation state, and the
   light is ON after power-up even if it was turned off before unplugging.
4. Plugged into a computer: no keystrokes are ever registered (lamp-only
   mode), and the light stays on when the computer goes to sleep.
5. Reflashing works, both via the `QK_BOOT` key and via the board's
   physical reset button.
