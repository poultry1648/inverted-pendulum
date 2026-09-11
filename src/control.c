/*
 * control.c — core 1 real-time loop and inter-core plumbing. See control.h.
 *
 * Self-clocked with get_absolute_time()/busy_wait_until(): each iteration waits
 * for the next absolute tick, measures the achieved period, does a fixed amount
 * of work and schedules the following tick. No printf, USB or blocking calls.
 */
#include "control.h"
#include "motion.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "pico/util/queue.h"
#include "hardware/timer.h"
#include "hardware/sync.h"
#include <math.h>

/* PLACEHOLDER (Stage 1): bench-only gains, replaced by the LQR law later. */
#define POS_HOLD_KP          4.0f
#define THETA_DOT_LPF_ALPHA  0.15f   /* 1st-order low-pass on dtheta/dt */
#define XDOT_LPF_ALPHA       0.25f   /* 1st-order low-pass on dx/dt */
#define TWO_PI               6.28318530718f

/* ---- Core 0 -> core 1 commands: SDK queue (core-safe, never blocks) ---- */
typedef struct { uint8_t type, mode; float value; } cmd_t;
static queue_t g_cmd_q;

void control_post_cmd(uint8_t type, uint8_t mode, float value) {
    cmd_t c = { type, mode, value };
    queue_try_add(&g_cmd_q, &c);
}

/* ---- Core 1 -> core 0 telemetry: compact seqlock ---------------------- */
static volatile uint32_t g_tel_seq;
static telemetry_t       g_tel;

static void telemetry_publish(const telemetry_t *t) {
    g_tel_seq++;
    __dmb();
    g_tel = *t;
    __dmb();
    g_tel_seq++;
}

bool control_get_telemetry(telemetry_t *out) {
    uint32_t s1 = g_tel_seq;
    if (s1 & 1u) return false;          /* writer mid-update */
    __dmb();
    *out = g_tel;
    __dmb();
    return g_tel_seq == s1;
}

/* ---- Core 1 controller state ------------------------------------------ */
static control_mode_t g_mode      = CTRL_MODE_IDLE;
static float          g_target_mm = 0.0f;
static float          g_kp        = POS_HOLD_KP;
static float          g_amp       = 50.0f;   /* mm/s */
static float          g_freq      = 0.5f;    /* Hz   */
static float          g_vel       = 50.0f;   /* mm/s */

static float g_x_prev     = 0.0f;
static float g_theta_prev = 0.0f;
static float g_thetadot   = 0.0f;
static float g_xdot       = 0.0f;

static uint32_t g_tick         = 0;
static uint32_t g_period_min   = 0xFFFFFFFFu;
static uint32_t g_period_max   = 0;
static uint64_t g_period_sum   = 0;
static uint32_t g_period_count = 0;
static uint32_t g_missed       = 0;   /* cumulative, for telemetry */
static uint32_t g_miss_streak  = 0;   /* consecutive, for the fault */
static bool     g_faulted      = false;

/* Calibration: `zero` averages raw for ~1 s on core 1, `cal` derives the signed
 * scale from a raw sample, and calmode suspends the tilt trip during the sweep.
 * cal_seq lets core 0 print the result once core 1 has applied it. */
#define ZERO_AVG_TICKS  CONTROL_HZ          /* ~1 s of samples */
static bool     g_calmode     = false;
static bool     g_zeroing     = false;
static float    g_zero_sum    = 0.0f;
static uint32_t g_zero_count  = 0;
static bool     g_cal_pending = false;
static float    g_cal_angle   = 0.0f;
static uint32_t g_cal_seq     = 0;

static void apply_cmd(uint8_t type, uint8_t mode, float value) {
    switch (type) {
    case CMD_STOP:   g_mode = CTRL_MODE_IDLE; break;
    case CMD_GO:     if (g_mode == CTRL_MODE_IDLE) g_mode = CTRL_MODE_POSITION_HOLD; break;
    case CMD_TARGET: g_target_mm = value; g_mode = CTRL_MODE_POSITION_HOLD; break;
    case CMD_MODE:   if (mode <= CTRL_MODE_SINE_VEL) g_mode = (control_mode_t)mode; break;
    case CMD_AMP:    g_amp  = value; break;
    case CMD_FREQ:   g_freq = value; break;
    case CMD_VEL:    g_vel  = value; break;
    case CMD_KP:     g_kp   = value; break;
    case CMD_RESET:
        g_faulted = false;
        g_missed = g_miss_streak = 0;
        g_period_min = 0xFFFFFFFFu;
        g_period_max = g_period_sum = g_period_count = 0;
        motion_fault_clear();
        break;
    case CMD_ZERO:
        g_zeroing = true;           /* accumulate raw over ZERO_AVG_TICKS */
        g_zero_sum = 0.0f;
        g_zero_count = 0;
        break;
    case CMD_CAL:
        g_cal_pending = true;       /* applied after the next raw sample */
        g_cal_angle = value;
        break;
    case CMD_CALMODE:
        g_calmode = (value != 0.0f);
        /* Entering calmode escapes a tilt fault so a bad default doesn't block
         * the sweep; deadline-miss faults still latch normally. */
        if (g_calmode && g_faulted) {
            g_faulted = false;
            motion_fault_clear();
        }
        break;
    }
}

/* PLACEHOLDER: swap this whole function for the LQR law later. */
static float placeholder_velocity(float x_mm, uint32_t tick) {
    float t = (float)tick * DT_S;
    switch (g_mode) {
    case CTRL_MODE_POSITION_HOLD: return g_kp * (g_target_mm - x_mm);
    case CTRL_MODE_CONST_VEL:     return g_vel;
    case CTRL_MODE_SQUARE_VEL:
        if (g_freq <= 0.0f) return 0.0f;
        return (fmodf(t * g_freq, 1.0f) < 0.5f) ? g_amp : -g_amp;
    case CTRL_MODE_SINE_VEL:      return g_amp * sinf(TWO_PI * g_freq * t);
    default:                      return 0.0f;
    }
}

static void control_core1_entry(void) {
    motion_pot_init();                  /* core 1 owns the ADC */

    float raw = 0.0f;
    float theta = motion_pot_read(&raw);
    g_theta_prev = theta;
    g_x_prev     = motion_pos_mm();

    absolute_time_t last = get_absolute_time();
    absolute_time_t next = delayed_by_us(last, DT_US);

    for (;;) {
        /* Self-clocked 1 kHz tick. */
        busy_wait_until(next);
        absolute_time_t now = get_absolute_time();
        uint32_t period = (uint32_t)absolute_time_diff_us(last, now);
        last = now;

        if (period < g_period_min) g_period_min = period;
        if (period > g_period_max) g_period_max = period;
        g_period_sum += period;
        g_period_count++;
        if (period > DT_US + DEADLINE_SLACK_US) {
            g_missed++;
            g_miss_streak++;
        } else {
            g_miss_streak = 0;
        }

        /* Next tick; resync (don't burst a backlog) if we fell behind. */
        next = delayed_by_us(next, DT_US);
        if (absolute_time_diff_us(now, next) <= 0) next = delayed_by_us(now, DT_US);

        cmd_t c;
        while (queue_try_remove(&g_cmd_q, &c)) apply_cmd(c.type, c.mode, c.value);

        /* Sample + estimate state. */
        theta = motion_pot_read(&raw);
        float x_mm = motion_pos_mm();
        g_xdot     += XDOT_LPF_ALPHA      * ((x_mm - g_x_prev) / DT_S - g_xdot);
        g_thetadot += THETA_DOT_LPF_ALPHA * ((theta - g_theta_prev) / DT_S - g_thetadot);
        g_x_prev     = x_mm;
        g_theta_prev = theta;

        /* Calibration: apply pending scale from the raw sample just taken. */
        if (g_cal_pending) {
            g_cal_pending = false;
            if (fabsf(g_cal_angle) > 0.01f) {
                motion_pot_set_scale(
                    (raw - motion_pot_upright_raw()) / g_cal_angle);
                g_cal_seq++;
            }
        }
        /* `zero`: finish the ~1 s average and capture it as upright. */
        if (g_zeroing) {
            g_zero_sum += raw;
            if (++g_zero_count >= ZERO_AVG_TICKS) {
                motion_pot_set_upright(g_zero_sum / (float)g_zero_count);
                g_zeroing = false;
                g_cal_seq++;
            }
        }

        /* Safety: latch, zero output, de-energize. calmode bypasses only the
         * tilt trip; missed deadlines still fault. */
        if (!g_faulted &&
            ((!g_calmode && fabsf(theta) > MAX_TILT_DEG) ||
             g_miss_streak > DEADLINE_MISS_FAULT)) {
            g_faulted = true;
            motion_fault_latch();
        }

        float v_cmd = g_faulted ? 0.0f : placeholder_velocity(x_mm, g_tick);

        /* Clamp, accumulate and emit evenly across the remaining tick. */
        motion_emit(v_cmd, get_absolute_time(), next);

        telemetry_t t = {
            .x_steps = motion_pos_steps(),
            .x_mm = motion_pos_mm(),
            .theta_deg = theta,
            .raw = raw,
            .thetadot = g_thetadot,
            .xdot = g_xdot,
            .v_cmd = v_cmd,
            .tick = g_tick,
            .period_min_us = g_period_min,
            .period_max_us = g_period_max,
            .period_mean_us = g_period_count ? (uint32_t)(g_period_sum / g_period_count) : 0,
            .missed = g_missed,
            .fault = g_faulted,
            .mode = (uint8_t)g_mode,
            .upright_raw = motion_pot_upright_raw(),
            .deg_per_count = motion_pot_deg_per_count(),
            .calmode = g_calmode ? 1 : 0,
            .cal_seq = g_cal_seq,
        };
        telemetry_publish(&t);
        g_tick++;
    }
}

void control_start(void) {
    queue_init(&g_cmd_q, sizeof(cmd_t), 8);
    multicore_launch_core1(control_core1_entry);
}
