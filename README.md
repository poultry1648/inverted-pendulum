# SKR Pico Inverted Pendulum

![Cart-pole balancing demo](docs/demo.avif)

Bare-metal C firmware that turns a **BIGTREETECH SKR Pico v1.0** (RP2040) into a
real-time controller for a **cart-pole inverted pendulum**. A belt-driven cart
runs along a rail and a potentiometer at the pivot measures the pole's angle; the
firmware reads that angle, estimates the full cart-pole state, and commands the
cart to keep the pole balanced upright with a **Linear-Quadratic Regulator
(LQR)**. A browser UI (Web Serial) provides live telemetry, jog controls,
calibration, and a live LQR design panel.

The project is built in stages. The deterministic motion core, angle
calibration, and the LQR balance law are implemented; hard-stop homing is not,
so the carriage is assumed parked at 0 mm on boot.

## Hardware

- **Controller** — BIGTREETECH SKR Pico v1.0, an RP2040 board (dual Cortex-M0+,
  133 MHz, 264 KB SRAM, 2 MB flash). Pin map: [`src/skr_pico.h`](src/skr_pico.h).
- **Actuator** — the board's **E axis**: a GT2 belt (2 mm pitch) on a 20-tooth
  pulley (40 mm/rev), driven by the onboard **TMC2209** stepper driver in 1/16
  microstepping (80 steps/mm).
- **Sensor** — a **potentiometer on the TH0 header** (GPIO26 = ADC0), mechanically
  coupled to the pendulum pivot, read by the RP2040's 12-bit ADC.
- **Power** — the stepper runs from the board's **VM input (a stepper PSU), not
  USB**. A USB-only connection configures the driver but will not move the motor.
- **Travel** — 350 mm soft-limited range, 0 mm = left end.

The firmware does not use any heater, fan, thermistor, endstop, or the other
three stepper axes; those pins are still driven to a safe state at boot.

## Solution

The board is treated as an ideal **cart-acceleration source**: the control input
`u` is a desired cart acceleration (m/s²), which the motion layer rate-limits and
turns into step pulses. The plant is the standard cart-pole linearized about
upright,

```
xddot       = u
thetadotdot = (g/l) * theta - (1/l) * u
```

with `l` the pivot-to-pole-center-of-mass distance. State is
`[x, xdot, theta, thetadot]`, where `theta` is signed from upright
(positive = pole tips RIGHT).

Key design choices:

- **Augmented LQR with a position-error integrator.** A fifth state
  `xi = integral of (x - x_ref)` is added so the law both balances and pulls the
  cart toward the middle of the rail. This also nulls the small steady creep
  caused by a constant upright-calibration bias, which would otherwise look like
  a constant acceleration and drive the cart to one end. The gains are computed
  offline in [`tools/lqr_design.py`](tools/lqr_design.py) (or live in the browser)
  and written to [`src/lqr_gains.h`](src/lqr_gains.h):

  ```
  u = -( K0*(x - x_ref) + K1*xdot + K2*theta + K3*thetadot + K4*xi )
  ```

- **Auto-arm state machine.** Selecting balance mode enters a *waiting* state
  (zero output, tilt trip bypassed) so the pole can be placed by hand. Once the
  pole is within 12° of upright and slower than 60°/s the controller *engages*
  on its own; a tilt beyond 30° disengages back to waiting so the operator can
  recatch. This is what makes hand-starting the pendulum practical.

- **Hard safety backstops.** A tilt beyond 45°, the cart reaching within 10 mm
  of either end while balancing, or a sustained run of missed control deadlines
  latches a fault: output goes to zero and the driver is de-energized until
  `reset`. Motors are never left floating.

- **Live tuning.** The LQR gains and centering limits are runtime values, so a
  design can be tried without reflashing: `set k0..k4 <gain>`,
  `set xiclamp|deadband|slew <value>`, with `gains` to read them back. The web
  UI's LQR panel solves Q/R → K in the browser and pushes all of it at once.

- **Signed angle calibration.** `theta` is measured from a runtime-captured
  upright reference with a signed degrees-per-count scale (`zero` and
  `cal <deg>`). Calibration lives in RAM only; it is not persisted to flash
  because a flash write stalls XIP and would fault the real-time core.

- **Simple, deterministic step generation.** Instead of per-step interrupts or a
  PIO program, the number of pulses for a tick is spread evenly across that
  tick's time span, with a fractional accumulator so sub-step velocity is never
  lost. Velocity and acceleration are clamped so a velocity step cannot stall
  the stepper.

The older placeholder modes (`hold`, `vel`, `square`, `sine`) remain for bench
testing alongside the LQR.

## Architecture

The firmware splits work across the RP2040's two cores so the control loop never
blocks on USB.

```
        core 0 (main.c)                          core 1 (control.c)
  USB-CDC stdio, command parsing            self-clocked 1 kHz control tick
  TMC2209 config, telemetry printf          sample pot -> estimate state
        |            ^                            |            ^
        | commands   | seqlock snapshot           | steps      | telemetry
        v            |                            v            |
   command queue (pico_util) ------------->  motion.c (step gen, position, safety)
```

- **Core 0 — [`src/main.c`](src/main.c):** fail-safe boot (heaters off, steppers
  disabled), TMC2209 configuration, then launches core 1. Parses USB lines,
  posts commands to core 1 through a queue, and prints telemetry read from a
  race-free seqlock snapshot. No real-time work.
- **Core 1 — [`src/control.c`](src/control.c):** owns the ADC and runs the
  self-clocked 1 kHz loop: sample pot → estimate `xdot`/`thetadot` with first-order
  low-pass filters → run the selected law → emit steps → check safety → publish
  telemetry. Contains no `printf`, USB, or blocking calls.
- **Motion — [`src/motion.c`](src/motion.c):** axis geometry, limits, the
  position counter, acceleration/velocity clamps, evenly-spaced pulse emission,
  the fault latch, and the potentiometer read/calibration.
- **TMC2209 — [`src/tmc2209.c`](src/tmc2209.c):** minimal write-only register
  configuration (current, microstepping) over the board's shared half-duplex
  single-wire UART (UART1, TX=GP8 / RX=GP9). TX echoes onto RX and is ignored.
- **LQR gains — [`src/lqr_gains.h`](src/lqr_gains.h):** power-on defaults,
  auto-generated by the design tools; live values live in `control.c`.

### Repository layout

| Path | Purpose |
|------|---------|
| [`src/skr_pico.h`](src/skr_pico.h) | Board pin map (single source of truth for every GPIO) |
| [`src/main.c`](src/main.c) | Core 0: USB stdio, TMC config, commands, telemetry |
| [`src/control.c`](src/control.c) / [`src/control.h`](src/control.h) | Core 1 1 kHz loop, state estimation, control laws, inter-core IPC |
| [`src/motion.c`](src/motion.c) / [`src/motion.h`](src/motion.h) | Geometry, limits, step generation, position, fault latch, pot read |
| [`src/tmc2209.c`](src/tmc2209.c) / [`src/tmc2209.h`](src/tmc2209.h) | TMC2209 single-wire UART config |
| [`src/lqr_gains.h`](src/lqr_gains.h) | Auto-generated LQR gain defaults |
| [`tools/lqr_design.py`](tools/lqr_design.py) | Offline LQR gain design; regenerates `lqr_gains.h` |
| [`web/`](web/) | Vite + Web Serial UI, including the browser LQR solver |
| [`scripts/flash_uf2.sh`](scripts/flash_uf2.sh) | Post-build copy of the `.uf2` to a mounted `RPI-RP2` drive |
| [`lib/pico-sdk`](lib/pico-sdk) | Raspberry Pi Pico SDK (git submodule) |

## Serial console and protocol

`stdio` is routed to the board's **USB-CDC** port (UART stdio is disabled):

```bash
screen /dev/ttyACM0 115200    # or: minicom -D /dev/ttyACM0
```

Commands, one per line:

| Command | Effect |
|---------|--------|
| `<number>` | Go to absolute position in mm (position-hold; in LQR, soft-recenters the cart) |
| `go` / `start` | Resume from idle into position-hold |
| `stop` / `s` | Zero the output and select idle (leaves balance mode) |
| `reset` | Clear a latched fault, reset timing stats, re-arm balance |
| `balance` / `bal` | Enter LQR balance mode; auto-arms near upright |
| `mode idle\|hold\|vel\|square\|sine\|lqr\|<n>` | Select the control law |
| `vel <mm/s>` / `amp <mm/s>` / `freq <hz>` / `kp <gain>` | Placeholder-mode parameters |
| `set k0..k4 <gain>` | Live LQR gain (k2 must stay negative) |
| `set xiclamp\|deadband\|slew <value>` | Cart-centering integrator clamp, deadband, reference slew |
| `gains` | Print the live gains and centering limits |
| `zero` | Average the pot for ~1 s and set that as upright (`theta = 0`) |
| `cal <deg>` | Pole held at a known angle; set the signed degrees-per-count scale |
| `calmode on\|off` | Suspend/restore the tilt safety while calibrating |
| `calstatus` | Print `upright_raw`, `deg_per_count`, and the live `theta` |
| `home` / `h` | *Not implemented yet* (carriage is assumed parked at 0 mm) |

Telemetry is printed on two lines. The `pos=`, `raw=`, and `deg` tokens are a
**parsing contract** with `web/`; keep them if you change the status `printf`.

```text
pos=  0.00 mm  TH0 raw= 1370   1.5 deg          # ~5 Hz, parsed by web/
dbg tick=12345 steps=0 jit=999/1000/1002 us miss=0 fault=0 mode=idle lqr=0 xref=175.00 xi=0.000 v=0.0 xd=0.0 thd=0.0  # ~1 Hz
```

### Angle calibration

`theta` is `0` when the pole is vertical, positive when it tilts toward **+x
(RIGHT)** and negative to the left. The tilt safety is only meaningful once the
upright reference and signed scale are calibrated:

1. `calmode on` — suspend the tilt safety so the pole can be swept through large
   angles without latching a fault.
2. Hold the pole vertical and still, then `zero` — stores the averaged raw value
   as upright.
3. Hold the pole at a known angle to the **RIGHT** (e.g. 45°), then `cal 45`.
4. Tilt right and confirm `theta` goes positive; recalibrate or negate
   `THETA_DEG_PER_COUNT` if the sign is flipped.
5. `calmode off` — restore the safety.

After `zero`/`cal`, the firmware prints values to paste back into
[`src/motion.h`](src/motion.h) as the compile-time defaults
(`THETA_UPRIGHT_RAW`, `THETA_DEG_PER_COUNT`). Calibration is **not** persisted to
flash by design.

## Web UI

A Vite app using the Web Serial API. Chrome/Edge only.

- Live cart position, signed pole tilt, and a full telemetry card.
- Jog buttons (±1/±10 mm) relative to the last reported position, clamped to the
  range parsed from the boot banner, plus go-to-0 / go-to-center / home.
- Drive controls (stop/go/reset, mode, placeholder parameters) and the complete
  angle-calibration workflow.
- A **Balance (LQR)** panel showing the auto-arm state.
- An **LQR design** panel: Q weights (log-spaced sliders) and `l`/`g` are solved
  to a gain vector in the browser using the same math as
  [`tools/lqr_design.py`](tools/lqr_design.py) — Ackermann-seeded Kleinman-Newton
  for the Riccati equation, with closed-loop poles reported live — then applied
  to the firmware with a sequence of `set` commands.
- A raw-command box for anything else.

The web solver fixes `R = 1` because `K` depends only on the `Q:R` ratio; keep
[`web/src/lqr.js`](web/src/lqr.js) and [`tools/lqr_design.py`](tools/lqr_design.py)
in sync if you change the model.

## Configuration

- **Axis geometry, limits, pot** — [`src/motion.h`](src/motion.h): `STEPS_PER_MM`,
  `AXIS_TRAVEL_MM`, `MAX_CART_VEL_MM_S`, `MAX_CART_ACCEL_MM_S2`, `MAX_TILT_DEG`,
  and the pot calibration constants.
- **Microstepping** — `MICROSTEPS` in `motion.h` must match the `MRES` value
  programmed in `tmc2209_configure()`'s `CHOPCONF`, or steps/mm is silently
  wrong.
- **Run/hold current** — the `IHOLD_IRUN` write in [`src/tmc2209.c`](src/tmc2209.c)
  (`IRUN=16/31` ≈ half scale). Lower it if the motor runs hot, raise it for more
  torque.
- **LQR gains** — regenerate [`src/lqr_gains.h`](src/lqr_gains.h) with the design
  tool, or tune live with `set`.

## Build and flash

Prerequisites (Debian/Ubuntu; `arm-none-eabi-gcc`, CMake, and Ninja):

```bash
sudo apt install gcc-arm-none-eabi cmake ninja-build
```

The Pico SDK is vendored as a git submodule. On a fresh clone:

```bash
git submodule update --init --recursive
```

Build:

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

Flash: hold **BOOT**, tap **RESET** (or plug in USB while holding BOOT) so the
board mounts as a USB drive named `RPI-RP2`, then copy the image:

```bash
cp build/skr_pico_fw.uf2 /media/$USER/RPI-RP2/
```

The board reboots into the new firmware automatically. The build also runs
[`scripts/flash_uf2.sh`](scripts/flash_uf2.sh) after linking, which copies the
`.uf2` to a mounted `RPI-RP2` drive if one is present and is a no-op otherwise —
handy for the VS Code CMake "Build" button.

Build and run the web UI:

```bash
cd web
npm install
npm run dev        # http://localhost:5173
```

On Linux the user needs read/write access to `/dev/ttyACM0`: either join the
`dialout` group (requires re-login) or install a udev rule (takes effect
immediately):

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", MODE="0666"
```
