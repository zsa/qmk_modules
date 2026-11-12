# QMK Modules

ZSA's QMK community modules

Starting from QMK version 25, some of the common code for ZSA's QMK fork were
moved to this repository.

## Enabling a module

List the modules you want in your keymap's `keymap.json`:

```json
{
    "modules": ["zsa/oryx", "zsa/defaults", "zsa/mousejiggler"]
}
```

Keycodes a module declares, and their aliases, can then be used in `keymap.c`
like any QMK keycode. ZSA keyboards (Voyager, Wireless Voyager, Moonlander)
already pull in `zsa/busdev` from their `keyboard.json`.

## Dependencies between modules

- `zsa/navigator_trackball` and `zsa/navigator_trackpad` need `zsa/defaults`,
  which declares the Navigator keycodes both of them handle, and `zsa/busdev`.
- `zsa/wireless` needs `zsa/busdev`.
- Optional links, used only when both modules are listed:
  - `zsa/oryx` lets Oryx/Keymapp read and set the `zsa/automouse` per-device
    enable state.
  - `zsa/navigator_trackpad` feeds trackpad motion to `zsa/automouse`.
  - `zsa/keycolors` stops drawing while `zsa/oryx` has taken over the LEDs, and
    follows the brightness cap of `zsa/wireless`.
  - `zsa/wireless` carries the `zsa/oryx` protocol over Bluetooth.

## Modules

### automouse

Turns on a mouse layer when the pointing device moves and turns it off again
after a timeout, or after the next tap in oneshot mode. It tracks the trackball
and the trackpad separately, each with its own layer, timeout, threshold and
oneshot setting.

```json
"modules": ["zsa/automouse"]
```
This module replaces QMK's built-in auto mouse
(`POINTING_DEVICE_AUTO_MOUSE_ENABLE`), and its API is different. QMK's
`set_auto_mouse_enable()`, `set_auto_mouse_layer()` and friends do not apply.
`automouse.h` exposes instead:

- `automouse_enable()`, `automouse_disable()`, `automouse_toggle()`,
  `automouse_is_enabled()`, `automouse_is_active()`
- `automouse_set_device_enabled()`, `automouse_get_device_enabled()`,
  `automouse_toggle_device()`, taking `AUTOMOUSE_DEVICE_TRACKBALL` or
  `AUTOMOUSE_DEVICE_TRACKPAD`
- `automouse_report_motion()`, to feed motion from a sensor outside QMK's
  pointing device pipeline

| Keycode | Alias | What it does |
| --- | --- | --- |
| `KC_AUTOMOUSE_TOGGLE` | `AM_TOGG` | Turns automouse on or off for both devices. Turning it on restores each device's compile-time default. |
| `KC_AUTOMOUSE_TOGGLE_TRACKBALL` | `AM_TBAL` | Turns automouse on or off for the trackball only. |
| `KC_AUTOMOUSE_TOGGLE_TRACKPAD` | `AM_TPAD` | Turns automouse on or off for the trackpad only. |

`AUTOMOUSE_LAYER` must be defined in `config.h`. Other settings
(`AUTOMOUSE_TIMEOUT`, `AUTOMOUSE_THRESHOLD`, `AUTOMOUSE_ONESHOT`, the
`_TRACKBALL` / `_TRACKPAD` variants, ...) are listed in `automouse/automouse.h`.

### busdev

Supervises the devices on the shared I2C bus: detects when they are plugged
in, configures and checks them, and recovers them after bus errors. The
Navigator and wireless modules register their devices with it. No keycodes.

```json
"modules": ["zsa/busdev"]
```

ZSA keyboards already enable it in their `keyboard.json`.

### defaults

ZSA's common settings: a default `SERIAL_NUMBER`, aliases for keycodes renamed
between QMK versions, the ZSA keyboard keycodes (`TOGGLE_LAYER_COLOR`,
`LED_LEVEL`, handled by the keyboard code) and the Navigator keycodes below.
The Navigator keycodes only do something when `zsa/navigator_trackball` or
`zsa/navigator_trackpad` is also enabled.

```json
"modules": ["zsa/defaults"]
```

| Keycode | Alias | Trackball (`zsa/navigator_trackball`) | Trackpad (`zsa/navigator_trackpad`) |
| --- | --- | --- | --- |
| `NAVIGATOR_INC_CPI` | `NV_CPIU` | Raises sensitivity by one step. | Raises sensitivity by one step (mouse fallback mode only). |
| `NAVIGATOR_DEC_CPI` | `NV_CPID` | Lowers sensitivity by one step. | Lowers sensitivity by one step (mouse fallback mode only). |
| `TRACKPAD_INC_CPI` | `TP_CPIU` | Not handled. | Same as `NAVIGATOR_INC_CPI`. |
| `TRACKPAD_DEC_CPI` | `TP_CPID` | Not handled. | Same as `NAVIGATOR_DEC_CPI`. |
| `NAVIGATOR_TURBO` | `NV_TRBO` | Speeds the cursor up while held (`NAVIGATOR_TURBO_MULTIPLIER`, default x3). | No effect. |
| `NAVIGATOR_AIM` | `NV_AIM` | Slows the cursor down while held (`NAVIGATOR_AIM_DIVIDER`, default /3). | No effect. |
| `TOGGLE_TURBO` | `NV_TTRB` | Turns turbo on or off; turning it on turns aim off. | No effect. |
| `TOGGLE_AIM` | `NV_TAIM` | Turns aim on or off; turning it on turns turbo off. | No effect. |
| `DRAG_SCROLL` | `NV_DSCR` | Ball movement scrolls instead of moving the cursor while held. | No effect. |
| `TOGGLE_SCROLL` | `NV_TSCR` | Turns scroll mode on or off. | No effect. |
| `TOGGLE_SCROLL_VERTICAL` | `NV_VSCR` | Turns vertical-only scrolling on or off (horizontal scroll is dropped). | Not handled. |
| `NAVIGATOR_CLEAR_SPEED` | `NV_CSPD` | Turns turbo and aim off, and ignores turbo/aim layers until the layer state changes. | No effect. |

### doubleclick

Adds a double-click key.

```json
"modules": ["zsa/doubleclick"]
```

| Keycode | Alias | What it does |
| --- | --- | --- |
| `KC_MS_DBL_CLICK` | | Clicks the left mouse button, then again after `MOUSE_DOUBLECLICK_INTERVAL_MS` (default 200 ms). |

### keycolors

Per-layer key colors from the `ledmap` table in the keymap (the format Oryx
generates). Only layers 1 and 2 are painted from the table. It respects the
keyboard's layer color toggle (`TOGGLE_LAYER_COLOR`). No keycodes.

```json
"modules": ["zsa/keycolors"]
```

### mousejiggler

Moves the mouse pointer back and forth by a tiny amount at a fixed interval,
to keep the computer awake.

```json
"modules": ["zsa/mousejiggler"]
```

Settings: `MOUSE_JIGGLER_INTERVAL_MS` (default 1000) and
`MOUSE_JIGGLER_MOVEMENT` (default 1). Status LED 1 is lit while the jiggler is
on, on keyboards that have it.

| Keycode | Alias | What it does |
| --- | --- | --- |
| `KC_MS_JIGGLER_TOGGLE` | `MJ_TOGG` | Turns the jiggler on or off. |

### navigator_trackball

Driver for the ZSA Navigator trackball (PAW3805EK sensor behind an I2C to SPI
bridge, plugged into the TRRS jack). Handles rotation, turbo, aim and drag
scroll. Its keycodes are declared in `zsa/defaults` (see the table there).

```json
"modules": ["zsa/navigator_trackball", "zsa/defaults"]
```

Settings are in `navigator_trackball/navigator.h` and
`navigator_trackball/navigator_trackball.h` (for example
`NAVIGATOR_TRACKBALL_ROTATION`, `NAVIGATOR_SCROLL_DIVIDER`,
`NAVIGATOR_TURBO_LAYERS`, `NAVIGATOR_AIM_LAYERS`,
`NAVIGATOR_DRAG_SCROLL_LAYERS`).

### navigator_trackpad

Driver for the ZSA Navigator trackpad. It reports as a precision touchpad
(QMK digitizer) and falls back to a regular mouse when the host does not
support that. Its keycodes are declared in `zsa/defaults` (see the table
there).

```json
"modules": ["zsa/navigator_trackpad", "zsa/defaults"]
```

Mouse fallback settings: `TRACKPAD_MOUSE_SENSITIVITY` and
`TRACKPAD_MOUSE_ACCELERATION` (see `navigator_trackpad/config.h`).

### oryx

The raw HID protocol that connects the keyboard to Oryx live training, Keymapp
live view and typ.ing live view: pairing, layer reporting, LED control and
status queries. No keycodes.

```json
"modules": ["zsa/oryx"]
```

### wireless

Bluetooth support for the Wireless Voyager: the link to the board's radio
chip, the Bluetooth host menu, sleep and power off, and the Bluetooth status
overlay. It only builds for that keyboard (STM32F3 with the radio on USART1).

```json
"modules": ["zsa/wireless"]
```

| Keycode | Alias | What it does |
| --- | --- | --- |
| `BLE_MENU` | `BT_MENU` | Tap: opens the Bluetooth host menu, which shows the 5 host slots on the keys. In the menu, tapping a slot pairs an empty slot, connects a stored one or disconnects the active one; holding it (`ZSA_BLE_FORGET_HOLD_MS`, default 2 s) forgets it; any other key closes the menu. Does nothing while plugged into a computer over USB. Holding `BLE_MENU` itself (`ZSA_DIAG_HOLD_MS`, default 3 s) shows a diagnostic view, with or without USB. |
| `ZSA_SHIP_MODE` | `ZSA_SHP` | Powers both halves off. Refused while USB is connected. |
