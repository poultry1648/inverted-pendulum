# SKR Pico Inverted Pendulum

Bare-metal C firmware for the **BIGTREETECH SKR Pico v1.0** (Raspberry Pi
**RP2040**) that drives a belt-driven cart — the moving base of a cart-pole
inverted pendulum. The end goal is an **LQR balance controller**; this
repository currently implements the deterministic real-time motion core and a
serial/Web-Serial interface that the controller will plug into.

> **Status: work in progress.** The cart moves under a fixed-rate 1 kHz control
> loop with a *placeholder* controller. LQR, upright angle calibration, and
> homing are not implemented yet — see [Roadmap](#roadmap).

## Features

- **Dual-core, deterministic timing** — core 1 runs a self-clocked **1 kHz**
  control loop; core 0 handles USB stdio, command parsing and telemetry, so the
  control loop never blocks on USB.
- **Non-blocking step generation** — velocity and acceleration-limited,
  bidirectional, with a fractional step accumulator so sub-step velocity is not
  lost. Step pulses are spread evenly across each tick.
- **Safety** — a tilt / missed-deadline fault latches, zeroes the output and
  de-energizes the driver; a `reset` command clears it.
- **TMC2209 configuration** over the board's shared single-wire UART.
- **USB-CDC console** with a simple line protocol and 5 Hz telemetry.
- **Web Serial UI** (Vite) with live telemetry and cart jog controls.

## Hardware

- BIGTREETECH SKR Pico v1.0 (RP2040, dual Cortex-M0+).
- A GT2-belt cart on the **E** axis, driven by one onboard **TMC2209**.
- A **potentiometer on TH0** (GPIO26 / ADC0) at the pendulum pivot for angle
  sensing.
- Motors are powered from the board's **VM input (a stepper PSU), not USB**. A
  USB-only connection configures the driver but will not move the motor.

## Repository layout

| Path | Purpose |
|------|---------|
| [`src/skr_pico.h`](src/skr_pico.h) | Board pin map (single source of truth) |
| [`src/main.c`](src/main.c) | Core 0: USB stdio, TMC config, commands, telemetry, launches core 1 |
| [`src/control.c`](src/control.c) / [`src/control.h`](src/control.h) | Core 1 1 kHz loop, state estimation, placeholder controller, inter-core IPC |
| [`src/motion.c`](src/motion.c) / [`src/motion.h`](src/motion.h) | Axis geometry, limits, step generation, position, fault latch, pot read |
| [`src/tmc2209.c`](src/tmc2209.c) / [`src/tmc2209.h`](src/tmc2209.h) | TMC2209 single-wire UART config |
| [`web/`](web/) | Vite + Web Serial UI |
| [`scripts/flash_uf2.sh`](scripts/flash_uf2.sh) | Post-build copy of the `.uf2` to a mounted `RPI-RP2` drive |
| [`lib/pico-sdk`](lib/pico-sdk) | Raspberry Pi Pico SDK (git submodule) |
| [`AGENTS.md`](AGENTS.md) | Notes for contributors/agents working in this repo |

## Prerequisites

- CMake, Ninja and the ARM cross-compiler:

  ```bash
  sudo apt install gcc-arm-none-eabi cmake ninja-build
  ```

- The Pico SDK is vendored as a submodule. On a fresh clone:

  ```bash
  git submodule update --init --recursive
  ```

- Node.js for the web UI.

## Build

```bash
cmake -S . -B build -G Ninja
cmake --build build
```

Or with the bundled preset:

```bash
cmake --preset default
cmake --build --preset default
```

The firmware image is `build/skr_pico_fw.uf2`.

## Flash

1. Hold **BOOT**, tap **RESET** (or plug in USB while holding BOOT) so the board
   mounts as a USB drive named `RPI-RP2`.
2. Copy the image:

   ```bash
   cp build/skr_pico_fw.uf2 /media/$USER/RPI-RP2/
   ```

   The board reboots into the new firmware automatically.

The build also runs [`scripts/flash_uf2.sh`](scripts/flash_uf2.sh) after linking,
which copies the `.uf2` to a mounted `RPI-RP2` drive if one is present and is a
no-op otherwise — handy for the VS Code CMake "Build" button.

## Web UI

```bash
cd web
npm install
npm run dev        # http://localhost:5173
```

Chrome/Edge only (Web Serial). On Linux the user needs read/write access to
`/dev/ttyACM0`: join the `dialout` group (requires re-login) or install a udev
rule (takes effect immediately):

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", MODE="0666"
```

The UI shows cart position, pot angle and a full telemetry card, and sends the
same line protocol as the terminal. The move buttons jog relative to the last
reported position and clamp to the travel range parsed from the boot banner.

## Serial console / protocol

`stdio` is routed to the board's **USB-CDC** port:

```bash
screen /dev/ttyACM0 115200    # or: minicom -D /dev/ttyACM0
```

Commands (one per line):

| Command | Effect |
|---------|--------|
| `<number>` | Go to absolute position in mm (selects position-hold) |
| `go` / `start` | Resume from idle into position-hold |
| `stop` / `s` | Zero the output and select idle |
| `reset` | Clear a latched fault and reset timing stats |
| `mode idle\|hold\|vel\|square\|sine\|<n>` | Select the placeholder velocity source |
| `vel <mm/s>` | Constant-velocity magnitude |
| `amp <mm/s>` `freq <hz>` | Square/sine amplitude and frequency |
| `kp <gain>` | Position-hold proportional gain |
| `home` / `h` | *Not implemented yet* (carriage is assumed parked at 0 mm) |

Telemetry is printed on two lines:

```text
pos=  0.00 mm  TH0 raw= 1370  150.0 deg          # ~5 Hz, parsed by web/
dbg tick=12345 steps=0 jit=999/1000/1002 us miss=0 fault=0 mode=idle v=0.0 ...  # ~1 Hz
```

The `pos=`, `raw=` and `deg` tokens are a **parsing contract** with `web/`; keep
them if you change the status `printf`.

## Architecture

- **Core 0** ([`src/main.c`](src/main.c)) — fail-safe boot (heaters off, steppers
  disabled), TMC2209 configuration, then launches core 1. It parses commands,
  forwards them to core 1 through an SDK queue, and prints telemetry read from a
  race-free seqlock snapshot.
- **Core 1** ([`src/control.c`](src/control.c)) — owns the ADC and runs a
  self-clocked 1 kHz tick: sample pot → estimate state (`x`, `ẋ`, `θ`, `θ̇`) →
  produce a placeholder velocity → emit steps → check safety → publish
  telemetry. It contains no `printf`, USB or blocking calls.
- **Motion** ([`src/motion.c`](src/motion.c)) — converts a velocity command into
  signed, evenly-spaced step pulses; owns the position counter, velocity/accel
  clamps and the fault latch.
- **TMC2209** ([`src/tmc2209.c`](src/tmc2209.c)) — write-only register config
  (current, microstepping) over UART1.

The placeholder controller in [`src/control.c`](src/control.c) is isolated in
`placeholder_velocity()` and will be replaced by the LQR law without touching
the real-time plumbing.

## Configuration

- **Axis geometry, limits, pot** — [`src/motion.h`](src/motion.h): belt/pulley
  (`STEPS_PER_MM`), `AXIS_TRAVEL_MM`, `MAX_CART_VEL_MM_S`,
  `MAX_CART_ACCEL_MM_S2`, `MAX_TILT_DEG`, and the pot calibration constants.
- **`MICROSTEPS` must match `MRES`** programmed in
  `tmc2209_configure()`'s `CHOPCONF`, or steps/mm is silently wrong.
- **Run/hold current** — the `IHOLD_IRUN` write in
  [`src/tmc2209.c`](src/tmc2209.c) (`IRUN=16/31` ≈ half scale). Lower it if the
  motor runs hot, raise it for more torque.

## Roadmap

- [x] **Stage 1 — real-time motion core**: dual-core 1 kHz loop, non-blocking
      step generation, safety latch, telemetry, web UI.
- [ ] **Angle calibration**: `theta` is currently an absolute pot angle with no
      upright reference. Make it a signed deviation from vertical and use it for
      the tilt safety.
- [ ] **Homing**: open-loop hard-stop homing (creep into the left stop, zero,
      back off) — no limit switch required.
- [ ] **LQR**: derive the linearized cart-pole model, compute gains offline
      (`scipy.linalg.solve_continuous_are`), and replace the placeholder.
- [ ] **Swing-up** (optional): LQR only balances near upright.

## Pin reference (RP2040 GPIO)

- Steppers STEP/DIR/EN: X `11/10/12`, Y `6/5/7`, Z `19/28/2`, E `14/13/15`.
- Endstops: X `4`, Y `3`, Z `25`, probe/E0 `16`.
- Heaters: hotend `23`, bed `21`. Thermistors: hotend `27` (ADC1), bed `26` (ADC0).
- Fans: hotend `18`, part `17`, MCU `20`. TMC UART: TX `8` / RX `9`. RGB: `24`.
