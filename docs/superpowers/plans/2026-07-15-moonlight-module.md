# Moonlight Module Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A `moonlight` QMK community module that turns broken ZSA keyboards into USB-powered room lights: LEDs on from any USB power source (no host needed), a full set of light-control keycodes, lamp-only by default.

**Architecture:** Pure community module in `zsa/qmk_modules` (no firmware-repo changes), built against `zsa/qmk_firmware@firmware25`. All behavior lives in `moonlight/moonlight.c` via module hooks (`keyboard_post_init_moonlight`, `pre_process_record_moonlight`, `process_record_moonlight`); keycodes are declared in `qmk_module.json`; the standalone-power fix is a single `NO_USB_STARTUP_CHECK` define in the module's `config.h`.

**Tech Stack:** QMK community modules API (≥ 1.0.0), rgb_matrix API, C, `qmk` CLI, GitHub Actions.

**Spec:** `docs/superpowers/specs/2026-07-15-moonlight-module-design.md` (same repo — read it first).

## Global Constraints

- All work in the `zsa/qmk_modules` repo on branch `feat/moonlight`; tidy, batched commits; one cohesive PR at the end. Avoid overengineering.
- Firmware base: `zsa/qmk_firmware`, branch `firmware25`. Zero changes to that repo.
- Module name `moonlight`; keycodes `MOONLIGHT_*` with `MNL_*` aliases exactly as tabled in the spec.
- Keycode manifest order is load-bearing: `MOONLIGHT_ON` must be first and `MOONLIGHT_PRESET_8` last, with no gaps, so `case MOONLIGHT_ON ... MOONLIGHT_PRESET_8:` range matches work.
- Target boards: `zsa/voyager` (LAYOUT, 52 keys) and `zsa/moonlander` (LAYOUT, 72 keys).
- "Tests" for this project are compile checks (no unit-test rig exists for community modules) — every task ends with a `qmk compile` verification. Hardware verification happens once at the end via the README checklist.

## Development environment (used by every task)

The firmware checkout at `~/Documents/github/qmk_firmware` vendors this repo as the submodule `modules/zsa`. **The submodule working tree is the primary dev checkout** — edits there are immediately buildable. Finished commits get pushed back to the canonical local repo (`~/Documents/github/qmk_modules`, remote name `outer`).

```bash
FW=~/Documents/github/qmk_firmware
MOD=$FW/modules/zsa          # dev checkout of qmk_modules (this repo)
OUTER=~/Documents/github/qmk_modules
```

Build command used throughout (run from anywhere; `qmk` knows the firmware path after Task 1):

```bash
qmk compile $MOD/moonlight/examples/voyager_lamp.json
```

---

### Task 1: Build environment on firmware25

**Files:**
- No repo files. Sets up `~/Documents/github/qmk_firmware` on `firmware25` and the module dev checkout.

**Interfaces:**
- Produces: a working `qmk compile` baseline for `zsa/voyager`; `$MOD` checked out on `feat/moonlight` with remote `outer` → `~/Documents/github/qmk_modules`.

- [ ] **Step 1: Switch firmware checkout to firmware25 and init submodules**

```bash
cd ~/Documents/github/qmk_firmware
git status --short   # must be clean; stop and report if not
git fetch origin
git checkout firmware25
qmk git-submodule    # inits/updates lib/* and modules/* submodules; takes a while
```

- [ ] **Step 2: Point qmk CLI at this checkout**

```bash
qmk config user.qmk_home=$HOME/Documents/github/qmk_firmware
qmk doctor | tail -5
```
Expected: `qmk doctor` ends with "QMK is ready to go" (warnings are fine, errors are not).

- [ ] **Step 3: Baseline compile (proves toolchain + firmware25 build)**

```bash
qmk compile -kb zsa/voyager -km default
```
Expected: ends with `[OK]` lines and produces `zsa_voyager_default.bin`. If this fails, fix the environment before proceeding — nothing else in the plan can work.

- [ ] **Step 4: Set up the module dev checkout on feat/moonlight**

```bash
cd ~/Documents/github/qmk_firmware/modules/zsa
git remote add outer ~/Documents/github/qmk_modules 2>/dev/null || true
git fetch outer
git checkout -b feat/moonlight outer/feat/moonlight
git log --oneline -2   # expect the two docs: commits (spec)
```

---

### Task 2: Module scaffold — manifest, config.h, compilable skeleton

**Files:**
- Create: `moonlight/qmk_module.json`
- Create: `moonlight/config.h`
- Create: `moonlight/moonlight.c`
- Create: `moonlight/examples/voyager_lamp.json` (minimal; finalized in Task 8)

**Interfaces:**
- Produces: keycodes `MOONLIGHT_ON` … `MOONLIGHT_PRESET_8` (aliases `MNL_ON` … `MNL_P8`) usable in keymaps and in C; module compiles into a build.

- [ ] **Step 1: Write the manifest**

`moonlight/qmk_module.json` — keycode order matters (see Global Constraints):

```json
{
    "module_name": "Moonlight",
    "maintainer": "ZSA",
    "license": "GPL-2.0-or-later",
    "features": {
        "rgb_matrix": true
    },
    "keycodes": [
        { "key": "MOONLIGHT_ON",          "aliases": ["MNL_ON"]  },
        { "key": "MOONLIGHT_OFF",         "aliases": ["MNL_OFF"] },
        { "key": "MOONLIGHT_BRIGHTER",    "aliases": ["MNL_BRI"] },
        { "key": "MOONLIGHT_DIMMER",      "aliases": ["MNL_DIM"] },
        { "key": "MOONLIGHT_HUE_UP",      "aliases": ["MNL_HUU"] },
        { "key": "MOONLIGHT_HUE_DOWN",    "aliases": ["MNL_HUD"] },
        { "key": "MOONLIGHT_ANIM_START",  "aliases": ["MNL_AST"] },
        { "key": "MOONLIGHT_ANIM_STOP",   "aliases": ["MNL_ASP"] },
        { "key": "MOONLIGHT_ANIM_NEXT",   "aliases": ["MNL_ANX"] },
        { "key": "MOONLIGHT_ANIM_FASTER", "aliases": ["MNL_FST"] },
        { "key": "MOONLIGHT_ANIM_SLOWER", "aliases": ["MNL_SLW"] },
        { "key": "MOONLIGHT_PRESET_1",    "aliases": ["MNL_P1"]  },
        { "key": "MOONLIGHT_PRESET_2",    "aliases": ["MNL_P2"]  },
        { "key": "MOONLIGHT_PRESET_3",    "aliases": ["MNL_P3"]  },
        { "key": "MOONLIGHT_PRESET_4",    "aliases": ["MNL_P4"]  },
        { "key": "MOONLIGHT_PRESET_5",    "aliases": ["MNL_P5"]  },
        { "key": "MOONLIGHT_PRESET_6",    "aliases": ["MNL_P6"]  },
        { "key": "MOONLIGHT_PRESET_7",    "aliases": ["MNL_P7"]  },
        { "key": "MOONLIGHT_PRESET_8",    "aliases": ["MNL_P8"]  }
    ]
}
```

- [ ] **Step 2: Write config.h**

`moonlight/config.h`:

```c
// Copyright 2026 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

// Keep running when USB never enumerates (dumb charger / power bank).
// Compiles out the suspend trap in tmk_core/protocol/chibios/chibios.c that
// would otherwise park the board in suspend_power_down() forever.
#define NO_USB_STARTUP_CHECK

// Swallow every non-moonlight keycode so a broken matrix can never type
// into a host. Keymaps for working keyboards may set this to 0.
#ifndef MOONLIGHT_LAMP_ONLY
#    define MOONLIGHT_LAMP_ONLY 1
#endif

// A lamp never boots dark: brightness floor applied at power-up.
#ifndef MOONLIGHT_MIN_BOOT_BRIGHTNESS
#    define MOONLIGHT_MIN_BOOT_BRIGHTNESS 40
#endif

// MOONLIGHT_DIMMER floor (dim ≠ off; MOONLIGHT_OFF turns the light off).
#ifndef MOONLIGHT_MIN_BRIGHTNESS
#    define MOONLIGHT_MIN_BRIGHTNESS 16
#endif
```

- [ ] **Step 3: Write the skeleton moonlight.c**

`moonlight/moonlight.c`:

```c
// Copyright 2026 ZSA Technology Labs, Inc <@zsa>
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Moonlight: light control for upcycled keyboards. Purely a lamp module —
// see README.md. All behavior is driven by rgb_matrix.

#include QMK_KEYBOARD_H

ASSERT_COMMUNITY_MODULES_MIN_API_VERSION(1, 0, 0);

#ifndef RGB_MATRIX_ENABLE
#    error "The moonlight module requires an rgb_matrix-enabled keyboard (RGB_MATRIX_ENABLE = yes)."
#endif
```

- [ ] **Step 4: Write the minimal example keymap (finalized in Task 8)**

`moonlight/examples/voyager_lamp.json` — one layer, mostly `KC_NO`, a few moonlight keys to prove keycode generation. Voyager `LAYOUT` takes 52 keys: 4 rows × 6 columns per half (rows interleave left/right), then 2 thumb keys per half.

```json
{
    "keyboard": "zsa/voyager",
    "keymap": "moonlight_lamp",
    "layout": "LAYOUT",
    "modules": ["zsa/moonlight"],
    "layers": [
        [
            "MNL_ON",  "MNL_OFF", "MNL_BRI", "MNL_DIM", "KC_NO", "KC_NO",
            "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO", "QK_BOOT",
            "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO", "KC_NO",
            "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO", "KC_NO",
            "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO", "KC_NO",
            "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO", "KC_NO",
            "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO", "KC_NO",
            "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO", "KC_NO",
            "KC_NO",   "KC_NO",
            "KC_NO",   "KC_NO"
        ]
    ]
}
```

- [ ] **Step 5: Compile to verify the scaffold**

```bash
qmk compile ~/Documents/github/qmk_firmware/modules/zsa/moonlight/examples/voyager_lamp.json
```
Expected: PASS (`[OK]`, `.bin` produced). Failure modes to fix here: manifest schema errors, module not found (path/name mismatch), keycode generation errors.

- [ ] **Step 6: Commit and push to outer**

```bash
cd ~/Documents/github/qmk_firmware/modules/zsa
git add moonlight/
git commit -m "feat: scaffold moonlight module (manifest, config, skeleton)"
git push outer feat/moonlight
```

---

### Task 3: Core light control — on/off, brightness, hue

**Files:**
- Modify: `moonlight/moonlight.c`

**Interfaces:**
- Consumes: keycodes from Task 2; QMK rgb_matrix API (`rgb_matrix_enable/disable`, `rgb_matrix_increase_val`, `rgb_matrix_get_hsv`, `rgb_matrix_sethsv`, `rgb_matrix_increase_hue`, `rgb_matrix_decrease_hue` — all EEPROM-persisting variants).
- Produces: `process_record_moonlight(uint16_t keycode, keyrecord_t *record)` handling the six core keycodes; later tasks extend its `switch`.

- [ ] **Step 1: Add the handler skeleton and core cases**

Append to `moonlight/moonlight.c`:

```c
bool process_record_moonlight(uint16_t keycode, keyrecord_t *record) {
    if (!process_record_moonlight_kb(keycode, record)) {
        return false;
    }

    // Moonlight keycodes act on press only; consume the release too.
    if (!record->event.pressed) {
        switch (keycode) {
            case MOONLIGHT_ON ... MOONLIGHT_PRESET_8:
                return false;
            default:
                return true;
        }
    }

    switch (keycode) {
        case MOONLIGHT_ON:
            rgb_matrix_enable();
            return false;
        case MOONLIGHT_OFF:
            rgb_matrix_disable();
            return false;
        case MOONLIGHT_BRIGHTER:
            rgb_matrix_increase_val(); // clamps at RGB_MATRIX_MAXIMUM_BRIGHTNESS
            return false;
        case MOONLIGHT_DIMMER: {
            HSV     hsv = rgb_matrix_get_hsv();
            uint8_t v   = hsv.v > MOONLIGHT_MIN_BRIGHTNESS + RGB_MATRIX_VAL_STEP
                              ? hsv.v - RGB_MATRIX_VAL_STEP
                              : MOONLIGHT_MIN_BRIGHTNESS;
            rgb_matrix_sethsv(hsv.h, hsv.s, v);
            return false;
        }
        case MOONLIGHT_HUE_UP:
            rgb_matrix_increase_hue();
            return false;
        case MOONLIGHT_HUE_DOWN:
            rgb_matrix_decrease_hue();
            return false;
        default:
            return true;
    }
}
```

- [ ] **Step 2: Compile**

```bash
qmk compile ~/Documents/github/qmk_firmware/modules/zsa/moonlight/examples/voyager_lamp.json
```
Expected: PASS.

- [ ] **Step 3: Commit and push**

```bash
cd ~/Documents/github/qmk_firmware/modules/zsa
git add moonlight/moonlight.c
git commit -m "feat: core light control (on/off, brightness, hue)"
git push outer feat/moonlight
```

---

### Task 4: Animation model — reactive skip table, start/stop/next/faster/slower

**Files:**
- Modify: `moonlight/moonlight.c`

**Interfaces:**
- Consumes: `process_record_moonlight` switch from Task 3.
- Produces: `moonlight_mode_is_lamp_safe(uint8_t mode)` and `moonlight_next_anim(uint8_t from)` (static helpers used by Task 6); `moonlight_last_anim` (static `uint8_t`, 0 = none yet).

- [ ] **Step 1: Add helpers ABOVE `process_record_moonlight`**

```c
// Modes that must never run on a lamp: reactive/keypress-driven effects,
// plus NONE and out-of-range. SOLID_COLOR is excluded from the carousel
// too — it is the "steady" state reached via MOONLIGHT_ANIM_STOP.
static bool moonlight_mode_is_lamp_safe(uint8_t mode) {
    switch (mode) {
        case RGB_MATRIX_NONE:
        case RGB_MATRIX_SOLID_COLOR:
#ifdef ENABLE_RGB_MATRIX_SOLID_REACTIVE_SIMPLE
        case RGB_MATRIX_SOLID_REACTIVE_SIMPLE:
#endif
#ifdef ENABLE_RGB_MATRIX_SOLID_REACTIVE
        case RGB_MATRIX_SOLID_REACTIVE:
#endif
#ifdef ENABLE_RGB_MATRIX_SOLID_REACTIVE_WIDE
        case RGB_MATRIX_SOLID_REACTIVE_WIDE:
#endif
#ifdef ENABLE_RGB_MATRIX_SOLID_REACTIVE_MULTIWIDE
        case RGB_MATRIX_SOLID_REACTIVE_MULTIWIDE:
#endif
#ifdef ENABLE_RGB_MATRIX_SOLID_REACTIVE_CROSS
        case RGB_MATRIX_SOLID_REACTIVE_CROSS:
#endif
#ifdef ENABLE_RGB_MATRIX_SOLID_REACTIVE_MULTICROSS
        case RGB_MATRIX_SOLID_REACTIVE_MULTICROSS:
#endif
#ifdef ENABLE_RGB_MATRIX_SOLID_REACTIVE_NEXUS
        case RGB_MATRIX_SOLID_REACTIVE_NEXUS:
#endif
#ifdef ENABLE_RGB_MATRIX_SOLID_REACTIVE_MULTINEXUS
        case RGB_MATRIX_SOLID_REACTIVE_MULTINEXUS:
#endif
#ifdef ENABLE_RGB_MATRIX_SPLASH
        case RGB_MATRIX_SPLASH:
#endif
#ifdef ENABLE_RGB_MATRIX_MULTISPLASH
        case RGB_MATRIX_MULTISPLASH:
#endif
#ifdef ENABLE_RGB_MATRIX_SOLID_SPLASH
        case RGB_MATRIX_SOLID_SPLASH:
#endif
#ifdef ENABLE_RGB_MATRIX_SOLID_MULTISPLASH
        case RGB_MATRIX_SOLID_MULTISPLASH:
#endif
#ifdef ENABLE_RGB_MATRIX_TYPING_HEATMAP
        case RGB_MATRIX_TYPING_HEATMAP: // framebuffer effect, keypress-driven
#endif
            return false;
        default:
            return mode < RGB_MATRIX_EFFECT_MAX;
    }
}

// Next lamp-safe animation after `from`, wrapping. Falls back to
// SOLID_COLOR if the board somehow has no lamp-safe animations.
static uint8_t moonlight_next_anim(uint8_t from) {
    uint8_t mode = from;
    for (uint8_t i = 0; i < RGB_MATRIX_EFFECT_MAX; i++) {
        mode = (mode + 1 < RGB_MATRIX_EFFECT_MAX) ? mode + 1 : 1;
        if (moonlight_mode_is_lamp_safe(mode)) {
            return mode;
        }
    }
    return RGB_MATRIX_SOLID_COLOR;
}

#if !defined(MOONLIGHT_DEFAULT_ANIMATION) && defined(ENABLE_RGB_MATRIX_BREATHING)
#    define MOONLIGHT_DEFAULT_ANIMATION RGB_MATRIX_BREATHING
#endif

static uint8_t moonlight_default_anim(void) {
#ifdef MOONLIGHT_DEFAULT_ANIMATION
    if (moonlight_mode_is_lamp_safe(MOONLIGHT_DEFAULT_ANIMATION)) {
        return MOONLIGHT_DEFAULT_ANIMATION;
    }
#endif
    return moonlight_next_anim(RGB_MATRIX_SOLID_COLOR);
}

// Last animation used this power session (0 = none yet), so
// stop-then-start resumes the same animation.
static uint8_t moonlight_last_anim = 0;
```

- [ ] **Step 2: Add the animation cases to the `switch` in `process_record_moonlight`** (before `default:`)

```c
        case MOONLIGHT_ANIM_START: {
            uint8_t target = moonlight_last_anim ? moonlight_last_anim : moonlight_default_anim();
            rgb_matrix_enable();
            rgb_matrix_mode(target);
            moonlight_last_anim = target;
            return false;
        }
        case MOONLIGHT_ANIM_STOP: {
            uint8_t cur = rgb_matrix_get_mode();
            if (moonlight_mode_is_lamp_safe(cur)) {
                moonlight_last_anim = cur; // resume point for the next START
            }
            rgb_matrix_mode(RGB_MATRIX_SOLID_COLOR);
            return false;
        }
        case MOONLIGHT_ANIM_NEXT: {
            uint8_t cur  = rgb_matrix_get_mode();
            uint8_t base = moonlight_mode_is_lamp_safe(cur)
                               ? cur
                               : (moonlight_last_anim ? moonlight_last_anim : RGB_MATRIX_SOLID_COLOR);
            uint8_t next = moonlight_next_anim(base);
            rgb_matrix_enable();
            rgb_matrix_mode(next);
            moonlight_last_anim = next;
            return false;
        }
        case MOONLIGHT_ANIM_FASTER:
            rgb_matrix_increase_speed();
            return false;
        case MOONLIGHT_ANIM_SLOWER:
            rgb_matrix_decrease_speed();
            return false;
```

- [ ] **Step 3: Compile**

```bash
qmk compile ~/Documents/github/qmk_firmware/modules/zsa/moonlight/examples/voyager_lamp.json
```
Expected: PASS. If a `RGB_MATRIX_*` case constant is undeclared, its `#ifdef ENABLE_...` guard name is wrong for firmware25 — check `quantum/rgb_matrix/rgb_matrix.h` enum and the `ENABLE_RGB_MATRIX_*` names in the generated `info_config.h` under `.build/`.

- [ ] **Step 4: Commit and push**

```bash
cd ~/Documents/github/qmk_firmware/modules/zsa
git add moonlight/moonlight.c
git commit -m "feat: animation start/stop/next/speed with reactive-effect exclusion"
git push outer feat/moonlight
```

---

### Task 5: Preset colors

**Files:**
- Modify: `moonlight/moonlight.c`

**Interfaces:**
- Consumes: `process_record_moonlight` switch; `HSV` type and `HSV_*` macros from `color.h` (available via `QMK_KEYBOARD_H`).
- Produces: `moonlight_presets[8]` (static `const HSV[]`); presets apply hue+sat only, brightness preserved.

- [ ] **Step 1: Add preset defaults and table ABOVE `process_record_moonlight`**

```c
// Preset palette. Keymaps override any slot in their config.h, e.g.
//     #define MOONLIGHT_PRESET_1_HSV {HSV_TEAL}
// Only hue and saturation are applied; current brightness is preserved
// (the v component is accepted for HSV_* macro convenience but ignored).
#ifndef MOONLIGHT_PRESET_1_HSV
#    define MOONLIGHT_PRESET_1_HSV {HSV_RED}
#endif
#ifndef MOONLIGHT_PRESET_2_HSV
#    define MOONLIGHT_PRESET_2_HSV {HSV_CORAL}
#endif
#ifndef MOONLIGHT_PRESET_3_HSV
#    define MOONLIGHT_PRESET_3_HSV {HSV_GOLD}
#endif
#ifndef MOONLIGHT_PRESET_4_HSV
#    define MOONLIGHT_PRESET_4_HSV {HSV_GREEN}
#endif
#ifndef MOONLIGHT_PRESET_5_HSV
#    define MOONLIGHT_PRESET_5_HSV {HSV_AZURE}
#endif
#ifndef MOONLIGHT_PRESET_6_HSV
#    define MOONLIGHT_PRESET_6_HSV {HSV_BLUE}
#endif
#ifndef MOONLIGHT_PRESET_7_HSV
#    define MOONLIGHT_PRESET_7_HSV {HSV_PURPLE}
#endif
#ifndef MOONLIGHT_PRESET_8_HSV
#    define MOONLIGHT_PRESET_8_HSV {HSV_WHITE}
#endif

static const HSV moonlight_presets[] = {
    MOONLIGHT_PRESET_1_HSV, MOONLIGHT_PRESET_2_HSV, MOONLIGHT_PRESET_3_HSV, MOONLIGHT_PRESET_4_HSV,
    MOONLIGHT_PRESET_5_HSV, MOONLIGHT_PRESET_6_HSV, MOONLIGHT_PRESET_7_HSV, MOONLIGHT_PRESET_8_HSV,
};
```

Note: the config macros are named `MOONLIGHT_PRESET_n_HSV` (not `MOONLIGHT_PRESET_n`) because the bare names are the keycode identifiers — they cannot also be object-like macros.

- [ ] **Step 2: Add the preset case to the `switch`** (before `default:`)

```c
        case MOONLIGHT_PRESET_1 ... MOONLIGHT_PRESET_8: {
            HSV preset = moonlight_presets[keycode - MOONLIGHT_PRESET_1];
            rgb_matrix_mode(RGB_MATRIX_SOLID_COLOR); // presets always land steady
            rgb_matrix_sethsv(preset.h, preset.s, rgb_matrix_get_hsv().v);
            return false;
        }
```

- [ ] **Step 3: Compile**

```bash
qmk compile ~/Documents/github/qmk_firmware/modules/zsa/moonlight/examples/voyager_lamp.json
```
Expected: PASS.

- [ ] **Step 4: Commit and push**

```bash
cd ~/Documents/github/qmk_firmware/modules/zsa
git add moonlight/moonlight.c
git commit -m "feat: preset color slots (hue/sat only, brightness preserved)"
git push outer feat/moonlight
```

---

### Task 6: Power-up behavior — always on, never dark, never reactive

**Files:**
- Modify: `moonlight/moonlight.c`

**Interfaces:**
- Consumes: `moonlight_mode_is_lamp_safe`, `moonlight_last_anim` (Task 4); `MOONLIGHT_MIN_BOOT_BRIGHTNESS` (Task 2).
- Produces: `keyboard_post_init_moonlight(void)`.

- [ ] **Step 1: Add the hook at the END of moonlight.c**

```c
void keyboard_post_init_moonlight(void) {
    keyboard_post_init_moonlight_kb();

    // A lamp on a wall switch always comes on.
    rgb_matrix_enable();

    // Never boot dark.
    HSV hsv = rgb_matrix_get_hsv();
    if (hsv.v < MOONLIGHT_MIN_BOOT_BRIGHTNESS) {
        rgb_matrix_sethsv(hsv.h, hsv.s, MOONLIGHT_MIN_BOOT_BRIGHTNESS);
    }

    // EEPROM may hold a reactive or out-of-range mode (previous firmware,
    // fewer animations, etc.). Snap those to steady; remember valid
    // animations as the START resume point.
    uint8_t mode = rgb_matrix_get_mode();
    if (mode == RGB_MATRIX_SOLID_COLOR) {
        // steady — nothing to do
    } else if (moonlight_mode_is_lamp_safe(mode)) {
        moonlight_last_anim = mode; // restored mid-animation; keep animating
    } else {
        rgb_matrix_mode(RGB_MATRIX_SOLID_COLOR);
    }
}
```

- [ ] **Step 2: Compile**

```bash
qmk compile ~/Documents/github/qmk_firmware/modules/zsa/moonlight/examples/voyager_lamp.json
```
Expected: PASS.

- [ ] **Step 3: Commit and push**

```bash
cd ~/Documents/github/qmk_firmware/modules/zsa
git add moonlight/moonlight.c
git commit -m "feat: power-up always on, brightness floor, reactive-mode snap"
git push outer feat/moonlight
```

---

### Task 7: Lamp-only mode

**Files:**
- Modify: `moonlight/moonlight.c`

**Interfaces:**
- Consumes: `MOONLIGHT_LAMP_ONLY` (Task 2); keycode range from Task 2.
- Produces: `pre_process_record_moonlight(uint16_t keycode, keyrecord_t *record)`.

- [ ] **Step 1: Add the hook ABOVE `process_record_moonlight`**

```c
// Lamp-only: nothing but moonlight controls (and QK_BOOT, for flashing)
// gets processed — a broken matrix can never type into a host.
bool pre_process_record_moonlight(uint16_t keycode, keyrecord_t *record) {
    if (!pre_process_record_moonlight_kb(keycode, record)) {
        return false;
    }
#if MOONLIGHT_LAMP_ONLY
    switch (keycode) {
        case MOONLIGHT_ON ... MOONLIGHT_PRESET_8:
        case QK_BOOT:
            return true;
        default:
            return false;
    }
#else
    return true;
#endif
}
```

- [ ] **Step 2: Compile**

```bash
qmk compile ~/Documents/github/qmk_firmware/modules/zsa/moonlight/examples/voyager_lamp.json
```
Expected: PASS.

- [ ] **Step 3: Verify the opt-out path also compiles**

```bash
cd ~/Documents/github/qmk_firmware/modules/zsa/moonlight
sed -i '' 's/#    define MOONLIGHT_LAMP_ONLY 1/#    define MOONLIGHT_LAMP_ONLY 0/' config.h
qmk compile ~/Documents/github/qmk_firmware/modules/zsa/moonlight/examples/voyager_lamp.json
sed -i '' 's/#    define MOONLIGHT_LAMP_ONLY 0/#    define MOONLIGHT_LAMP_ONLY 1/' config.h
git diff --stat   # must be empty
```
Expected: both compiles PASS; working tree clean afterwards.

- [ ] **Step 4: Commit and push**

```bash
cd ~/Documents/github/qmk_firmware/modules/zsa
git add moonlight/moonlight.c
git commit -m "feat: lamp-only mode swallows all non-moonlight keycodes"
git push outer feat/moonlight
```

---

### Task 8: Example keymaps for both boards

**Files:**
- Modify: `moonlight/examples/voyager_lamp.json` (full layout)
- Create: `moonlight/examples/moonlander_lamp.json`

**Interfaces:**
- Consumes: `MNL_*` aliases (Task 2).
- Produces: two ready-to-flash reference keymaps; the canonical key arrangement documented in the README (Task 9).

Key-order note: a keymap.json layer is a flat array in the same order as the board's `LAYOUT` macro arguments. Before finalizing, open `keyboards/zsa/voyager/keymaps/default/keymap.c` and `keyboards/zsa/moonlander/keymaps/default/keymap.c` in the firmware checkout and confirm the row structure assumed below (Voyager: rows interleave left/right, 6 per half-row, 4 rows, then 2+2 thumbs = 52; Moonlander: half-rows of 7,7,7,6,5 interleaved, then 4+4 thumbs = 72). `qmk compile` hard-fails on a count mismatch, which catches structural errors.

- [ ] **Step 1: Finalize voyager_lamp.json**

Replace the layers array of `moonlight/examples/voyager_lamp.json` with:

```json
    "layers": [
        [
            "MNL_ON",  "MNL_OFF", "MNL_BRI", "MNL_DIM", "KC_NO",   "KC_NO",
            "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "QK_BOOT",
            "MNL_HUU", "MNL_HUD", "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",
            "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",
            "MNL_AST", "MNL_ASP", "MNL_ANX", "MNL_FST", "MNL_SLW", "KC_NO",
            "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",
            "MNL_P1",  "MNL_P2",  "MNL_P3",  "MNL_P4",  "MNL_P5",  "MNL_P6",
            "MNL_P7",  "MNL_P8",  "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",
            "KC_NO",   "KC_NO",
            "KC_NO",   "KC_NO"
        ]
    ]
```

Layout logic (also goes in the README): left hand = controls (row 1: power/brightness, row 2: hue, row 3: animation), bottom row = presets spilling onto the right hand, `QK_BOOT` on the far top-right corner where it is hard to hit by accident.

- [ ] **Step 2: Create moonlander_lamp.json**

`moonlight/examples/moonlander_lamp.json`:

```json
{
    "keyboard": "zsa/moonlander",
    "keymap": "moonlight_lamp",
    "layout": "LAYOUT",
    "modules": ["zsa/moonlight"],
    "layers": [
        [
            "MNL_ON",  "MNL_OFF", "MNL_BRI", "MNL_DIM", "KC_NO",   "KC_NO",   "KC_NO",
            "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "QK_BOOT",
            "MNL_HUU", "MNL_HUD", "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",
            "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",
            "MNL_AST", "MNL_ASP", "MNL_ANX", "MNL_FST", "MNL_SLW", "KC_NO",   "KC_NO",
            "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",
            "MNL_P1",  "MNL_P2",  "MNL_P3",  "MNL_P4",  "MNL_P5",  "MNL_P6",
            "MNL_P7",  "MNL_P8",  "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",
            "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",
            "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",
            "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO",
            "KC_NO",   "KC_NO",   "KC_NO",   "KC_NO"
        ]
    ]
}
```

- [ ] **Step 3: Compile BOTH**

```bash
qmk compile ~/Documents/github/qmk_firmware/modules/zsa/moonlight/examples/voyager_lamp.json
qmk compile ~/Documents/github/qmk_firmware/modules/zsa/moonlight/examples/moonlander_lamp.json
```
Expected: both PASS. A key-count error means the row structure assumption was wrong — fix against the default keymap.c ordering.

- [ ] **Step 4: Commit and push**

```bash
cd ~/Documents/github/qmk_firmware/modules/zsa
git add moonlight/examples/
git commit -m "feat: reference lamp keymaps for voyager and moonlander"
git push outer feat/moonlight
```

---

### Task 9: Documentation — module README + root README entry

**Files:**
- Create: `moonlight/README.md`
- Modify: `README.md` (repo root, "Available modules" list)

**Interfaces:**
- Consumes: everything above (documents it).

- [ ] **Step 1: Write moonlight/README.md**

Content requirements (write full prose, not stubs; keycode table can be copied from the spec):

1. **Opening paragraph, verbatim intent:** Moonlight is purely for light control — it turns a keyboard (typically one too broken to type on) into a USB-powered room light. It is not a keyboard-lighting or typing-feedback feature; by default it never sends a single keystroke to a computer.
2. **How it works:** lights on from any USB power source (wall charger, power bank, computer — no enumeration needed); state persists across power cycles; the light always comes on at power-up.
3. **Keycode table:** all 19 keycodes with aliases and behavior (copy from spec §"Keycodes and behavior model", including the preset hue/sat-only note).
4. **Quick start:** clone firmware25, `qmk compile modules/zsa/moonlight/examples/voyager_lamp.json`, flash with Keymapp or `qmk flash`; diagram/description of the example layout from Task 8.
5. **Customization:** the `MOONLIGHT_*` config defines table (from spec §"Configuration surface", using the `_HSV` names from Task 5), how to remap keys in the json, `MOONLIGHT_LAMP_ONLY 0` for working keyboards. Note that in lamp-only mode layer-switching keys are swallowed too — arrange everything on one layer.
6. **Caveats:** ignores USB suspend (stays lit and drawing current when a host sleeps); exceeds the USB unconfigured-device power budget by design — fine on chargers; reflashing still works normally (`QK_BOOT` key or physical reset button).
7. **Hardware test checklist** (copy the five items from spec §"Testing").

- [ ] **Step 2: Add the root README entry**

In the repo root `README.md`, `Available modules` list, add:

```markdown
- `moonlight`: Turns a (broken) keyboard into a USB-powered room light —
  standalone power support, light-control keycodes, lamp-only by default
```

- [ ] **Step 3: Commit and push**

```bash
cd ~/Documents/github/qmk_firmware/modules/zsa
git add moonlight/README.md README.md
git commit -m "docs: moonlight README and root module listing"
git push outer feat/moonlight
```

---

### Task 10: CI — build both examples on GitHub Actions

**Files:**
- Create: `.github/workflows/build_moonlight.yml`

**Interfaces:**
- Consumes: example keymaps (Task 8).
- Produces: CI that fails if either example stops compiling against firmware25.

- [ ] **Step 1: Write the workflow**

`.github/workflows/build_moonlight.yml`:

```yaml
name: Build moonlight examples

on:
  push:
    branches: [main, "feat/**"]
    paths: ["moonlight/**", ".github/workflows/build_moonlight.yml"]
  pull_request:
    paths: ["moonlight/**", ".github/workflows/build_moonlight.yml"]

jobs:
  build:
    runs-on: ubuntu-latest
    container: ghcr.io/qmk/qmk_cli:latest
    strategy:
      matrix:
        example: [voyager_lamp, moonlander_lamp]
    steps:
      - name: Checkout firmware (firmware25)
        run: |
          git clone --depth 1 --branch firmware25 \
            https://github.com/zsa/qmk_firmware.git /qmk_firmware
          cd /qmk_firmware
          qmk config user.qmk_home=/qmk_firmware
          qmk git-submodule

      - name: Checkout this repo as modules/zsa
        uses: actions/checkout@v4
        with:
          path: modules_checkout

      - name: Overlay module checkout
        run: |
          rm -rf /qmk_firmware/modules/zsa
          cp -r "$GITHUB_WORKSPACE/modules_checkout" /qmk_firmware/modules/zsa

      - name: Build example
        run: |
          cd /qmk_firmware
          qmk compile modules/zsa/moonlight/examples/${{ matrix.example }}.json
```

- [ ] **Step 2: Push and verify the run**

```bash
cd ~/Documents/github/qmk_firmware/modules/zsa
git add .github/workflows/build_moonlight.yml
git commit -m "ci: compile moonlight examples against firmware25"
git push outer feat/moonlight
```

Then push `feat/moonlight` from `~/Documents/github/qmk_modules` to GitHub (`git push -u origin feat/moonlight`) and check the Actions run. CI environment quirks (container image contents, submodule flags) may need 1-2 fix iterations — that is expected; keep fixes in this task's commit scope with `ci:` messages.

---

### Task 11: Final verification and PR

**Files:** none (verification + PR).

- [ ] **Step 1: Spec sweep** — reread `docs/superpowers/specs/2026-07-15-moonlight-module-design.md` top to bottom; for each requirement confirm where it is implemented (file + function). Fix anything missed before proceeding.

- [ ] **Step 2: Clean rebuild of both examples**

```bash
cd ~/Documents/github/qmk_firmware
rm -rf .build
qmk compile modules/zsa/moonlight/examples/voyager_lamp.json
qmk compile modules/zsa/moonlight/examples/moonlander_lamp.json
```
Expected: both `.bin` files produced.

- [ ] **Step 3: Hardware verification (user-in-the-loop)** — flash a real board and run the README hardware checklist with the user. The wall-charger test is the headline feature; do not skip it. Record results in the PR description's test plan.

- [ ] **Step 4: Open the PR** — push `feat/moonlight` to `zsa/qmk_modules` origin, then open a PR titled `feat: moonlight — light-control module for upcycled keyboards`, body summarizing: purpose, the NO_USB_STARTUP_CHECK mechanism, keycode table, lamp-only default, and the test plan (compile CI + hardware checklist results, with unchecked boxes for anything not yet run on hardware). Get user approval on the PR body before creating it.
