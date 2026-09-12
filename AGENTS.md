# AGENTS.md

Bare-metal C firmware for the BIGTREETECH SKR Pico v1.0 (RP2040) using the
Raspberry Pi Pico SDK, vendored as a git submodule.

## Build

```bash
git submodule update --init --recursive   # first time only
cmake -S . -B build -G Ninja
cmake --build build
```

Output: `build/skr_pico_fw.uf2`. Toolchain: `arm-none-eabi-gcc` (already
installed). There is no test suite, lint, or typecheck step — a clean build is
the only verification.

## Architecture

- `src/skr_pico.h` — board pin map, the single source of truth for every GPIO.
- `src/main.c` — the app: reads a target coordinate (mm) over USB serial and
  moves the E-axis belt carriage there. At boot it runs open-loop hard-stop
  homing (`home_axis()`): it creeps LEFT into the stop, declares that 0 mm, and
  backs off `HOME_BACKOFF_MM`. There is no endstop. Sending `home` re-homes.
- `src/tmc2209.c/h` — minimal write-only TMC2209 config over single-wire UART.
- `src/control.c/h` — core 1 1 kHz loop: placeholder modes plus the cart-pole
  LQR with an auto-arm state machine. The LQR gains are runtime values (seeded
  from `src/lqr_gains.h`) and can be changed live with `set k0..k4 <gain>`
  (`gains` prints them); `xiclamp`/`deadband`/`slew` are tunable too.
- `web/` — Vite app (Web Serial) that shows cart position and pot angle and
  sends move commands; it speaks the same line protocol as the terminal.
- `web/src/lqr.js` — browser port of `tools/lqr_design.py`: turns Q weights
  (R fixed at 1) into K via Ackermann-seeded Kleinman-Newton, and reports the
  closed-loop poles. The "LQR design" panel solves live and applies via `set`.

## Web UI

```bash
cd web
npm install
npm run dev        # http://localhost:5173
```

Chrome/Edge only (Web Serial). On Linux the user needs read/write access to
`/dev/ttyACM0`; either join the `dialout` group (requires re-login) or install a
udev rule (takes effect immediately):

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", MODE="0666"
```

The UI parses the periodic `pos=<mm> mm  TH0 raw=<n>  <deg> deg` line and sends
targets as `<mm>\n`, exactly like typing into the terminal. The move buttons jog
by ±1/±10 mm relative to the last reported position and clamp to the travel
range parsed from the boot banner.

## Gotchas

- `README.md` describes an old behavior ("continuously rotates the motor").
  Trust the source: `main.c` now implements coordinate movement over USB.
- `MICROSTEPS` in `main.c` must match the `MRES` microstep value programmed in
  `tmc2209_configure()` (`CHOPCONF`), or steps/mm is wrong.
- TMC2209 `ENABLE` is active-low (HIGH = de-energized); heaters are low-side
  N-FETs (drive HIGH = on). Never leave heater pins floating.
- All four TMC2209s share one half-duplex UART line (UART1, TX=GP8/RX=GP9).
  `tmc2209.c` only writes registers; TX echoes onto RX and is ignored.
- stdio is USB-CDC only (`pico_enable_stdio_uart ... 0` in `CMakeLists.txt`).
- The periodic status line's `pos=`/`raw=`/`deg` tokens are a parsing contract
  with `web/`; keep them if you change the printf in `main.c`.
- Motors are powered from the board's VM input (stepper PSU), not USB — a USB
  connection alone configures the driver but won't turn the motor.
- `build/`, `.uf2`, `.elf`, `.bin`, `.hex`, `.dis`, `.map` are gitignored; the
  `build/` dir is a CMake artifact and `lib/pico-sdk` is a submodule.
- `set k2` is rejected unless negative: theta>0 (pole tips RIGHT) must command
  u>0. `src/lqr_gains.h` is now only the power-on default; live gains live in
  `control.c` (`g_lqr_k[]`) and reset to the header values on reboot.
- The web LQR solver fixes R=1 because K depends only on the Q:R ratio; keep
  `web/src/lqr.js` and `tools/lqr_design.py` in sync if you change the model.
