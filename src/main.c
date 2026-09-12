/*
 * SKR Pico v1.0 — single-axis belt gantry, fixed-rate control over USB.
 *
 * Core 0 (this file): USB stdio, TMC2209 config, command parsing, telemetry and
 * launching the core 1 real-time loop (control.c). Core 1 owns the ADC and the
 * deterministic 1 kHz step generation (motion.c).
 *
 * Conventions: carriage parked LEFT = 0 mm, coordinates increase RIGHT, DIR=1
 * (forward) drives LEFT. Motors run from the stepper PSU, not USB.
 *
 * Position is assumed true at boot; hard-stop homing is OUT OF SCOPE for this
 * stage (see the marked spot in main()).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "skr_pico.h"
#include "tmc2209.h"
#include "motion.h"
#include "control.h"

/* TMC2209 ENABLE is active-low: HIGH = de-energized. Heaters are low-side. */
static void steppers_disable(void) {
    const uint pins[] = { SKR_X_ENABLE_PIN, SKR_Y_ENABLE_PIN,
                          SKR_Z_ENABLE_PIN, SKR_E_ENABLE_PIN };
    for (size_t i = 0; i < count_of(pins); i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
        gpio_put(pins[i], 1);
    }
}

static void heaters_off(void) {
    const uint pins[] = { SKR_HOTEND_HEATER_PIN, SKR_BED_HEATER_PIN };
    for (size_t i = 0; i < count_of(pins); i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
        gpio_put(pins[i], 0);
    }
}

static const char *mode_name(uint8_t m) {
    static const char *names[] = { "idle", "hold", "vel", "square", "sine", "lqr" };
    return m <= CTRL_MODE_LQR ? names[m] : "?";
}

static int mode_from_name(const char *s) {
    if (!s) return -1;
    if (!strcmp(s, "idle")) return CTRL_MODE_IDLE;
    if (!strcmp(s, "hold") || !strcmp(s, "position")) return CTRL_MODE_POSITION_HOLD;
    if (!strcmp(s, "vel") || !strcmp(s, "const")) return CTRL_MODE_CONST_VEL;
    if (!strcmp(s, "square")) return CTRL_MODE_SQUARE_VEL;
    if (!strcmp(s, "sine")) return CTRL_MODE_SINE_VEL;
    if (!strcmp(s, "lqr") || !strcmp(s, "balance")) return CTRL_MODE_LQR;
    return s[0] >= '0' && s[0] <= '9' ? atoi(s) : -1;
}

/* Live-tunable LQR parameter names accepted by `set`. */
static int param_from_name(const char *s) {
    if (!s) return -1;
    if (!strcmp(s, "k0")) return PARAM_K0;
    if (!strcmp(s, "k1")) return PARAM_K1;
    if (!strcmp(s, "k2")) return PARAM_K2;
    if (!strcmp(s, "k3")) return PARAM_K3;
    if (!strcmp(s, "k4")) return PARAM_K4;
    if (!strcmp(s, "xiclamp") || !strcmp(s, "xi")) return PARAM_XI_CLAMP;
    if (!strcmp(s, "deadband") || !strcmp(s, "db")) return PARAM_DEADBAND;
    if (!strcmp(s, "slew")) return PARAM_SLEW;
    return -1;
}

static void handle_line(char *line) {
    char *cmd = strtok(line, " \t");
    char *arg = strtok(NULL, " \t");
    if (!cmd) return;

    if (!strcmp(cmd, "home") || !strcmp(cmd, "h")) {
        printf("home: not implemented yet (carriage assumed at 0 mm)\n");
    } else if (!strcmp(cmd, "stop") || !strcmp(cmd, "s")) {
        control_post_cmd(CMD_STOP, 0, 0);
        printf("stop\n");
    } else if (!strcmp(cmd, "go") || !strcmp(cmd, "start")) {
        control_post_cmd(CMD_GO, 0, 0);
        printf("go\n");
    } else if (!strcmp(cmd, "balance") || !strcmp(cmd, "bal")) {
        control_post_cmd(CMD_MODE, CTRL_MODE_LQR, 0);
        printf("balance: LQR armed, waiting for pole near upright...\n");
    } else if (!strcmp(cmd, "reset")) {
        control_post_cmd(CMD_RESET, 0, 0);
        printf("reset\n");
    } else if (!strcmp(cmd, "zero")) {
        control_post_cmd(CMD_ZERO, 0, 0);
        printf("zero: averaging pot for ~1 s, then theta=0...\n");
    } else if (!strcmp(cmd, "cal")) {
        if (!arg) { printf("usage: cal <deg>  (pole held at a known angle)\n"); return; }
        float deg = strtof(arg, NULL);
        if (fabsf(deg) < 0.01f) { printf("cal: angle must be non-zero\n"); return; }
        control_post_cmd(CMD_CAL, 0, deg);
        printf("cal: computing scale from %.2f deg...\n", deg);
    } else if (!strcmp(cmd, "calmode")) {
        bool on  = arg && (!strcmp(arg, "on")  || !strcmp(arg, "1"));
        bool off = arg && (!strcmp(arg, "off") || !strcmp(arg, "0"));
        if (!on && !off) { printf("usage: calmode on|off\n"); return; }
        control_post_cmd(CMD_CALMODE, 0, on ? 1.0f : 0.0f);
        printf("calmode %s (tilt safety %s)\n",
               on ? "on" : "off", on ? "suspended" : "active");
    } else if (!strcmp(cmd, "calstatus")) {
        telemetry_t t;
        if (control_get_telemetry(&t)) {
            printf("calstatus: upright_raw=%.1f deg_per_count=%.5f "
                   "theta=%.2f deg calmode=%s\n",
                   t.upright_raw, t.deg_per_count, t.theta_deg,
                   t.calmode ? "on" : "off");
        } else {
            printf("calstatus: telemetry busy, retry\n");
        }
    } else if (!strcmp(cmd, "set")) {
        char *val = strtok(NULL, " \t");
        if (!arg || !val) {
            printf("usage: set k0..k4|xiclamp|deadband|slew <value>\n");
            return;
        }
        int p = param_from_name(arg);
        if (p < 0) { printf("set: unknown param \"%s\"\n", arg); return; }
        char *end;
        float v = strtof(val, &end);
        if (end == val || !isfinite(v)) {
            printf("set: bad value \"%s\"\n", val);
            return;
        }
        if (p == PARAM_K2 && v >= 0.0f) {
            printf("set: k2 must be negative (theta>0 -> accelerate RIGHT)\n");
            return;
        }
        control_post_cmd(CMD_SET, (uint8_t)p, v);
        printf("%s = %.6f\n", arg, v);
    } else if (!strcmp(cmd, "gains")) {
        telemetry_t t;
        if (control_get_telemetry(&t)) {
            printf("gains k0=%.6f k1=%.6f k2=%.6f k3=%.6f k4=%.6f\n",
                   t.lqr_k[0], t.lqr_k[1], t.lqr_k[2], t.lqr_k[3], t.lqr_k[4]);
            printf("limits xiclamp=%.3f deadband=%.2f slew=%.2f\n",
                   t.xi_clamp, t.center_deadband_mm, t.x_ref_slew_mm_s);
        } else {
            printf("gains: telemetry busy, retry\n");
        }
    } else if (!strcmp(cmd, "mode")) {
        int m = mode_from_name(arg);
        if (m < 0) printf("usage: mode idle|hold|vel|square|sine|lqr|<n>\n");
        else {
            control_post_cmd(CMD_MODE, (uint8_t)m, 0);
            printf("mode %s\n", mode_name(m));
        }
    } else if (!strcmp(cmd, "vel") || !strcmp(cmd, "amp") ||
               !strcmp(cmd, "freq") || !strcmp(cmd, "kp")) {
        if (!arg) { printf("usage: %s <value>\n", cmd); return; }
        uint8_t type = !strcmp(cmd, "vel")  ? CMD_VEL
                     : !strcmp(cmd, "amp")  ? CMD_AMP
                     : !strcmp(cmd, "freq") ? CMD_FREQ : CMD_KP;
        control_post_cmd(type, 0, strtof(arg, NULL));
        printf("%s = %s\n", cmd, arg);
    } else {
        /* Bare number: absolute target + position-hold (web/ protocol). */
        char *end;
        float mm = strtof(cmd, &end);
        if (end == cmd) { printf("unknown: \"%s\"\n", cmd); return; }
        if (mm < 0.0f || mm > AXIS_TRAVEL_MM) {
            printf("out of range: %.2f mm (allowed 0..%.0f)\n", mm, AXIS_TRAVEL_MM);
            return;
        }
        control_post_cmd(CMD_TARGET, 0, mm);
        printf("target %.2f mm (position-hold)\n", mm);
    }
}

int main(void) {
    stdio_init_all();

    heaters_off();          /* fail safe before anything else */
    steppers_disable();

    tmc2209_uart_init();
    tmc2209_configure(TMC_ADDR_E);
    motion_init();
    motion_enable(true);

    /* FUTURE: home_axis() belongs here, before the control loop may move the
     * axis. Stage 1 assumes the carriage is parked at the left stop (x = 0). */

    printf("\nSKR Pico ready. Range 0..%.0f mm (0 = left).\n", AXIS_TRAVEL_MM);
    printf("  <number>   go to absolute mm (position-hold; recenters in LQR)\n");
    printf("  go|stop|reset\n");
    printf("  balance    LQR balance, auto-arms when the pole is near upright\n");
    printf("  mode hold|vel|square|sine|idle|lqr\n");
    printf("  vel <mm/s>  amp <mm/s>  freq <hz>  kp <gain>\n");
    printf("  set k0..k4 <gain> | xiclamp | deadband | slew   (live LQR tuning)\n");
    printf("  gains      print live LQR gains and limits\n");
    printf("Calibration (theta is signed from upright, +RIGHT):\n");
    printf("  calmode on|off   suspend/restore the tilt safety trip\n");
    printf("  zero             hold pole vertical; average ~1 s -> theta=0\n");
    printf("  cal <deg>        hold pole <deg> to the RIGHT; set signed scale\n");
    printf("  calstatus        print upright_raw, deg_per_count, live theta\n");

    control_start();

    char line[48];
    size_t n = 0;
    uint32_t last_status  = to_ms_since_boot(get_absolute_time());
    uint32_t last_dbg     = last_status;
    uint32_t last_cal_seq = 0;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        telemetry_t t;
        bool have = control_get_telemetry(&t);

        /* Calibration applied by core 1: print values to paste into motion.h. */
        if (have && t.cal_seq != last_cal_seq) {
            last_cal_seq = t.cal_seq;
            printf("cal done: upright_raw=%.1f  deg_per_count=%.6f\n",
                   t.upright_raw, t.deg_per_count);
            printf("  paste into src/motion.h:\n");
            printf("  #define THETA_UPRIGHT_RAW   %.1ff\n", t.upright_raw);
            printf("  #define THETA_DEG_PER_COUNT %.6ff\n", t.deg_per_count);
        }

        /* ~5 Hz status: KEEP the pos=/raw=/deg token contract with web/. */
        if (have && now - last_status >= 200) {
            last_status = now;
            printf("pos=%6.2f mm  TH0 raw=%4.0f  %5.1f deg\n",
                   t.x_mm, t.raw, t.theta_deg);
        }
        /* ~1 Hz diagnostics on a separate line so web/ parsing is untouched. */
        if (have && now - last_dbg >= 1000) {
            last_dbg = now;
            printf("dbg tick=%lu steps=%ld jit=%lu/%lu/%lu us miss=%lu fault=%lu "
                   "mode=%s lqr=%u xref=%.2f xi=%.3f v=%.1f xd=%.1f thd=%.1f\n",
                   (unsigned long)t.tick, (long)t.x_steps,
                   (unsigned long)t.period_min_us,
                   (unsigned long)t.period_mean_us, (unsigned long)t.period_max_us,
                   (unsigned long)t.missed, (unsigned long)t.fault,
                   mode_name(t.mode), (unsigned)t.lqr_state, t.x_ref_mm,
                   t.lqr_xi, t.v_cmd, t.xdot, t.thetadot);
        }

        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) continue;
        if (c == '\r' || c == '\n') {
            if (n == 0) continue;       /* swallow blank lines / CRLF */
            line[n] = '\0';
            n = 0;
            handle_line(line);
        } else if (n < sizeof(line) - 1) {
            line[n++] = (char)c;
        }
    }
}
