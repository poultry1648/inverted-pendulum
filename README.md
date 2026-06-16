# SKR Pico Firmware (C)

Bare-metal C project for the **BIGTREETECH SKR Pico v1.0** — a 3D-printer
control board built on the Raspberry Pi **RP2040** (dual Cortex-M0+).

Because the SKR Pico is an RP2040 board, the toolchain is the official
**Raspberry Pi Pico SDK** (CMake + `arm-none-eabi-gcc`). The SDK (v2.2.0) is
vendored as a git submodule under [lib/pico-sdk](lib/pico-sdk) so the build is
reproducible without a system-wide `PICO_SDK_PATH`.

## Layout

| Path | Purpose |
|------|---------|
| [src/skr_pico.h](src/skr_pico.h) | Board pin map (single source of truth) |
| [src/main.c](src/main.c) | Safe bring-up firmware skeleton |
| [CMakeLists.txt](CMakeLists.txt) | Build definition |
| [pico_sdk_import.cmake](pico_sdk_import.cmake) | Locates/initializes the SDK |
| [lib/pico-sdk](lib/pico-sdk) | Pico SDK (submodule) |

## Prerequisites

CMake and the ARM cross-compiler. On Debian/Ubuntu:

```bash
sudo apt install gcc-arm-none-eabi cmake ninja-build
```

> This machine has CMake but **not** `gcc-arm-none-eabi` yet — install it with
> the line above before building.

If you cloned this repo fresh, pull the SDK submodule too:

```bash
git submodule update --init --recursive
```

## Build

```bash
cmake -S . -B build -G Ninja
cmake --build build
```

Output firmware: `build/skr_pico_fw.uf2`.

## Flash

1. Hold **BOOT**, tap **RESET** (or plug in USB while holding BOOT) — the board
   mounts as a USB drive named `RPI-RP2`.
2. Copy the `.uf2` onto it:
   ```bash
   cp build/skr_pico_fw.uf2 /media/$USER/RPI-RP2/
   ```
   The board reboots into the new firmware automatically.

## Serial console

`stdio` is routed to the **USB-CDC** port (the board's USB-C). After flashing:

```bash
screen /dev/ttyACM0 115200    # or: minicom -D /dev/ttyACM0
```

You should see a status line once per second reporting both thermistor ADC
readings. Set `pico_enable_stdio_uart(... 1)` in `CMakeLists.txt` to also use a
hardware UART.

## What the skeleton does

`main.c` is intentionally a **safe** starting point, not a motion controller:

- heaters forced **OFF** and steppers **DISABLED** on boot,
- part-cooling fan attached to a PWM slice,
- both 100k NTC thermistors sampled on the RP2040 ADC,
- status printed over USB.

Raw ADC counts are reported as-is — converting to °C needs an NTC
(Beta/Steinhart-Hart) model for your specific thermistor, plus the board's
divider resistor. Add that next, along with TMC2209 UART setup (`gpio8/9`),
step/dir motion, and endstop handling. All pins are defined in
[src/skr_pico.h](src/skr_pico.h).

## Pin reference (RP2040 GPIO)

Steppers (STEP/DIR/EN): X `11/10/12`, Y `6/5/7`, Z `19/28/2`, E `14/13/15`.
Endstops: X `4`, Y `3`, Z `25`, probe/E0 `16`.
Heaters: hotend `23`, bed `21`. Thermistors: hotend `27` (ADC1), bed `26` (ADC0).
Fans: hotend `18`, part `17`, MCU `20`. TMC UART: TX `8` / RX `9`. RGB: `24`.
