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

## What the firmware does

On boot `main.c`:

- forces heaters **OFF** and all steppers **DISABLED** (fail-safe first),
- configures the **E-axis TMC2209 over UART** (UART1, TX `GP8` / RX `GP9`):
  internal current reference, 1/16 microstepping w/ 256-interpolation, moderate
  run current — see [src/tmc2209.c](src/tmc2209.c),
- enables the E driver and **continuously rotates the motor**, reversing every
  ~2 revolutions,
- samples both 100k NTC thermistors on the ADC and prints a status line over USB.

> **Motors are powered from the board's VM input (a stepper PSU), not USB.**
> With USB only, the driver configures fine but the motor will not turn. Make
> sure an E-axis stepper is plugged into the **E** port and VM power is applied.

### Tuning the motion

In [src/main.c](src/main.c): `STEP_LOW_US` sets the step rate (≈5 kHz default),
`MICROSTEPS` must match the value programmed in `tmc2209_configure()`, and the
run/hold current lives in the `IHOLD_IRUN` write in
[src/tmc2209.c](src/tmc2209.c) (`IRUN=16/31` ≈ half scale — lower it if the
motor runs hot, raise it for more torque).

### Not yet implemented

°C conversion (needs a Beta/Steinhart-Hart model for your thermistor + the
board divider), the other three axes, endstop/probe handling, and acceleration
ramping. All pins are defined in [src/skr_pico.h](src/skr_pico.h).

## Pin reference (RP2040 GPIO)

Steppers (STEP/DIR/EN): X `11/10/12`, Y `6/5/7`, Z `19/28/2`, E `14/13/15`.
Endstops: X `4`, Y `3`, Z `25`, probe/E0 `16`.
Heaters: hotend `23`, bed `21`. Thermistors: hotend `27` (ADC1), bed `26` (ADC0).
Fans: hotend `18`, part `17`, MCU `20`. TMC UART: TX `8` / RX `9`. RGB: `24`.
