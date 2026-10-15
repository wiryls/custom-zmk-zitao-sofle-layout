# ZitaoTech Sofle · Personal Layout

ZMK v0.3.0 configuration for a left A320 touchpad and right TrackPoint.

Edit **`config/zitaotech_sofle.keymap`**. The board's fallback keymap remains the vendor default.

Generated diagrams are published to the [`draw` branch](../../tree/draw), separately from firmware source history.

## Build and flash

Push changes or run **Build ZMK firmware** in GitHub Actions. After a successful build, download its `firmware` artifact and flash both halves:

- Left: `lpm_view;left_bbtrackpad-zitaotech_sofle_left-zmk.uf2`
- Right: `lpm_view;right_trackpoint-zitaotech_sofle_right-zmk.uf2`

Do not use the alternative trackball, `settings_reset`, or `left-touchpad-fpd-diagnostics` images for normal updates. The opt-in diagnostic image logs only A320 sensor registers over USB and is intended for contact-detection experiments. Backlight and sensitivity settings are saved after a 1-second delay; wait at least 2 seconds before rebooting to test persistence.

Before regular use, verify both pointing devices, stationary touch/release detection, layer controls, fast movement and encoder rotation, BLE pairing indication, backlight modes and restart persistence. A320 shutter-based contact estimation is experimental; thresholds across fingers, pressure and lighting, and battery impact require measurement. Firmware build and hardware checks are required; source inspection alone is not validation.
