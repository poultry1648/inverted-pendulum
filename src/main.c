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
    static const char *names[] = { "idle", "hold", "vel", "square", "sine" };
    return m <= CTRL_MODE_SINE_VEL ? names[m] : "?";
}

static int mode_from_name(const char *s) {
    if (!s) return -1;
    if (!strcmp(s, "idle")) return CTRL_MODE_IDLE;
    if (!strcmp(s, "hold") || !strcmp(s, "position")) return CTRL_MODE_POSITION_HOLD;
    if (!strcmp(s, "vel") || !strcmp(s, "const")) return CTRL_MODE_CONST_VEL;
    if (!strcmp(s, "square")) return CTRL_MODE_SQUARE_VEL;
    if (!strcmp(s, "sine")) return CTRL_MODE_SINE_VEL;
    return s[0] >= '0' && s[0] <= '9' ? atoi(s) : -1;
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
    } else if (!strcmp(cmd, "reset")) {
        control_post_cmd(CMD_RESET, 0, 0);
        printf("reset\n");
    } else if (!strcmp(cmd, "mode")) {
        int m = mode_from_name(arg);
        if (m < 0) printf("usage: mode idle|hold|vel|square|sine|<n>\n");
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
    printf("  <number>   go to absolute mm (position-hold)\n");
    printf("  go|stop|reset\n");
    printf("  mode hold|vel|square|sine|idle\n");
    printf("  vel <mm/s>  amp <mm/s>  freq <hz>  kp <gain>\n");

    control_start();

    char line[48];
    size_t n = 0;
    uint32_t last_status = to_ms_since_boot(get_absolute_time());
    uint32_t last_dbg    = last_status;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        telemetry_t t;
        bool have = control_get_telemetry(&t);

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
                   "mode=%s v=%.1f xd=%.1f thd=%.1f\n",
                   (unsigned long)t.tick, (long)t.x_steps,
                   (unsigned long)t.period_min_us,
                   (unsigned long)t.period_mean_us, (unsigned long)t.period_max_us,
                   (unsigned long)t.missed, (unsigned long)t.fault,
                   mode_name(t.mode), t.v_cmd, t.xdot, t.thetadot);
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
