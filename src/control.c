/*
 * control.c — core 1 real-time loop and inter-core plumbing. See control.h.
 *
 * Self-clocked with get_absolute_time()/busy_wait_until(): each iteration waits
 * for the next absolute tick, measures the achieved period, does a fixed amount
 * of work and schedules the following tick. No printf, USB or blocking calls.
 */
#include "control.h"
#include "motion.h"
#include "lqr_gains.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "pico/util/queue.h"
#include "hardware/timer.h"
#include "hardware/sync.h"
#include <math.h>

/* PLACEHOLDER (Stage 1): bench-only gains, kept for the non-LQR modes. */
#define POS_HOLD_KP          4.0f
#define THETA_DOT_LPF_ALPHA  0.15f   /* 1st-order low-pass on dtheta/dt */
#define XDOT_LPF_ALPHA       0.25f   /* 1st-order low-pass on dx/dt */
#define TWO_PI               6.28318530718f
#define DEG2RAD              0.017453292519943295f

/* ---- Cart-pole LQR auto-arm thresholds ---------------------------------
 * Selecting CTRL_MODE_LQR enters LQR_WAITING (output 0, tilt trip bypassed) so
 * the pole can be placed by hand. It engages automatically once the pole is
 * close enough to upright; a large tilt drops back to waiting (auto-recatch).
 * DISARM_TILT_DEG must exceed ARM_TILT_DEG for hysteresis; MAX_TILT_DEG in
 * motion.h stays a hard fault backstop while active. */
#define ARM_TILT_DEG      12.0f   /* |theta| below this -> engage */
#define ARM_RATE_DPS      60.0f   /* ...and |thetadot| below this (deg/s) */
#define DISARM_TILT_DEG   30.0f   /* |theta| above this -> disengage to waiting */

/* Cart centering. x_ref defaults to the middle of the travel range so the LQR
 * position term plus its xi integrator pull the cart back to center and null
 * any small upright-calibration bias (a constant theta offset otherwise looks
 * like a constant acceleration and makes the cart creep to one end).
 * LQR_XI_CLAMP bounds the position-error integral (m*s) against windup. The
 * steady xi that cancels a theta bias b is -LQR_K2*b/LQR_K4, so 3.0 covers
 * roughly a 4.5 deg zero error (its acceleration contribution is capped at
 * |LQR_K4|*3 ~= 2.5 m/s^2). Keeping it modest matters: a big windup lets a
 * disturbance launch the cart end to end. If the reported xi sits at the clamp
 * the cart will creep, which is the signal to re-zero or raise this.
 * CENTER_DEADBAND_MM freezes the integrator near the target so it cannot hunt,
 * and X_REF_SLEW_MM_S ramps x_ref instead of stepping it (a hard recenter after
 * a catch/disarm pumps the pole and causes a full-range limit cycle).
 *
 * The #defines below are the power-on defaults; the live values are the g_*
 * globals (tunable over USB via `set k0..k4|xiclamp|deadband|slew`). */
#define CART_CENTER_MM     (AXIS_TRAVEL_MM * 0.5f)
#define LQR_XI_CLAMP       3.0f
#define CENTER_DEADBAND_MM 15.0f
#define X_REF_SLEW_MM_S    40.0f

/* Live-tunable LQR parameters. K defaults come from lqr_gains.h. K2 must stay
 * negative (theta>0 -> accelerate RIGHT); SET_K2_MAX enforces it. */
static float g_lqr_k[5] = { LQR_K0, LQR_K1, LQR_K2, LQR_K3, LQR_K4 };
static float g_xi_clamp          = LQR_XI_CLAMP;
static float g_center_deadband_mm = CENTER_DEADBAND_MM;
static float g_x_ref_slew_mm_s    = X_REF_SLEW_MM_S;

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

/* LQR: velocity integrator, position-error integral, centering reference and
 * auto-arm state. */
static lqr_state_t g_lqr_state = LQR_OFF;
static float       g_lqr_v_cmd = 0.0f;   /* mm/s: integrates the accel command */
static float       g_lqr_xi    = 0.0f;   /* m*s: integral of (x - x_ref) */
static float       g_x_ref_mm  = 0.0f;   /* slewed cart reference */
static float       g_x_ref_target_mm = CART_CENTER_MM; /* desired reference */

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
static bool     g_travel_armed = false; /* clear of both stops at least once */

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
    case CMD_STOP:
        g_mode = CTRL_MODE_IDLE;
        g_lqr_state = LQR_OFF;          /* `stop` leaves balance mode */
        g_lqr_v_cmd = 0.0f;
        g_lqr_xi    = 0.0f;
        g_x_ref_target_mm = CART_CENTER_MM;
        break;
    case CMD_GO:     if (g_mode == CTRL_MODE_IDLE) g_mode = CTRL_MODE_POSITION_HOLD; break;
    case CMD_TARGET:
        g_target_mm = value;
        /* While balancing a numeric target is a soft cart recentering (the
         * reference slews toward it), not a mode switch; otherwise it selects
         * position-hold as before. */
        if (g_mode == CTRL_MODE_LQR) g_x_ref_target_mm = value;
        else                         g_mode = CTRL_MODE_POSITION_HOLD;
        break;
    case CMD_MODE:
        if (mode <= CTRL_MODE_LQR) {
            g_mode = (control_mode_t)mode;
            if (g_mode == CTRL_MODE_LQR) {
                g_lqr_state = LQR_WAITING;   /* re-arms on every entry */
                g_lqr_v_cmd = 0.0f;
                g_lqr_xi    = 0.0f;
                g_x_ref_target_mm = CART_CENTER_MM;
            } else {
                g_lqr_state = LQR_OFF;
            }
        }
        break;
    case CMD_AMP:    g_amp  = value; break;
    case CMD_FREQ:   g_freq = value; break;
    case CMD_VEL:    g_vel  = value; break;
    case CMD_KP:     g_kp   = value; break;
    case CMD_RESET:
        g_faulted = false;
        g_missed = g_miss_streak = 0;
        g_period_min = 0xFFFFFFFFu;
        g_period_max = g_period_sum = g_period_count = 0;
        g_travel_armed = false;   /* re-arm once the cart is clear again */
        motion_fault_clear();
        if (g_mode == CTRL_MODE_LQR) {   /* reset re-arms balance */
            g_lqr_state = LQR_WAITING;
            g_lqr_v_cmd = 0.0f;
            g_lqr_xi    = 0.0f;
            g_x_ref_target_mm = CART_CENTER_MM;
        }
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
    case CMD_SET:
        /* Live LQR tuning. Core 0 range-checks and reports; this is the final
         * guard on core 1 so a NaN or wrong-signed gain can never reach the
         * law. Out-of-range values are dropped. */
        if (mode >= PARAM_COUNT || !isfinite(value)) break;
        if (mode <= PARAM_K4) {
            if (mode == PARAM_K2 && value >= 0.0f) break;   /* sign contract */
            if (value < -500.0f || value > 500.0f) break;
            g_lqr_k[mode] = value;
        } else if (mode == PARAM_XI_CLAMP) {
            if (value < 0.0f || value > 100.0f) break;
            g_xi_clamp = value;
        } else if (mode == PARAM_DEADBAND) {
            if (value < 0.0f || value > 100.0f) break;
            g_center_deadband_mm = value;
        } else if (mode == PARAM_SLEW) {
            if (value < 0.0f || value > 1000.0f) break;
            g_x_ref_slew_mm_s = value;
        }
        break;
    }
}

/* Cart-pole LQR. u is a desired cart acceleration (m/s^2) computed from the SI
 * state; it is integrated into the velocity command motion_emit() consumes.
 * Sign check: theta > 0 (pole tipped RIGHT) must give u > 0 (accelerate RIGHT),
 * i.e. LQR_K2 < 0; tools/lqr_design.py asserts this. Inputs are converted here
 * (mm, deg) so K stays pure SI. */
static float lqr_velocity(float x_mm, float xdot_mm_s,
                          float theta_deg, float thetadot_deg_s) {
    if (g_lqr_state != LQR_ACTIVE) return 0.0f;

    /* Slew the reference toward its target instead of stepping: a hard recenter
     * pumps the pole and can start a full-range oscillation. */
    float dref = g_x_ref_target_mm - g_x_ref_mm;
    float step = g_x_ref_slew_mm_s * DT_S;
    g_x_ref_mm += fmaxf(-step, fminf(step, dref));

    float dx  = (x_mm - g_x_ref_mm) * 1e-3f;   /* m   */
    float xd  = xdot_mm_s * 1e-3f;             /* m/s */
    float th  = theta_deg * DEG2RAD;           /* rad */
    float thd = thetadot_deg_s * DEG2RAD;      /* rad/s */

    /* Position-error integral: drives steady-state cart error to zero (and so
     * rejects the theta-bias creep). Frozen inside the deadband (no hunting)
     * and clamped for anti-windup. */
    if (fabsf(dx) > g_center_deadband_mm * 1e-3f) {
        g_lqr_xi += dx * DT_S;
        if (g_lqr_xi >  g_xi_clamp) g_lqr_xi =  g_xi_clamp;
        if (g_lqr_xi < -g_xi_clamp) g_lqr_xi = -g_xi_clamp;
    }

    float u = -(g_lqr_k[0] * dx + g_lqr_k[1] * xd + g_lqr_k[2] * th
                + g_lqr_k[3] * thd + g_lqr_k[4] * g_lqr_xi);

    g_lqr_v_cmd += u * 1000.0f * DT_S;         /* m/s^2 -> mm/s */
    if (g_lqr_v_cmd >  MAX_CART_VEL_MM_S) g_lqr_v_cmd =  MAX_CART_VEL_MM_S;
    if (g_lqr_v_cmd < -MAX_CART_VEL_MM_S) g_lqr_v_cmd = -MAX_CART_VEL_MM_S;
    return g_lqr_v_cmd;
}

/* Auto-arm transitions, run after the safety trip: WAITING -> ACTIVE once the
 * pole is near upright and slow, ACTIVE -> WAITING (auto-recatch) on a large
 * tilt. MAX_TILT_DEG has already been enforced as the hard backstop while
 * ACTIVE by the time a disengage happens. */
static void lqr_update_state(float x_mm, float theta_deg, float thetadot_deg_s) {
    if (g_mode != CTRL_MODE_LQR) { g_lqr_state = LQR_OFF; return; }

    if (g_lqr_state == LQR_WAITING) {
        if (fabsf(theta_deg) < ARM_TILT_DEG &&
            fabsf(thetadot_deg_s) < ARM_RATE_DPS) {
            g_lqr_state = LQR_ACTIVE;
            g_lqr_v_cmd = 0.0f;               /* start from rest */
            g_lqr_xi    = 0.0f;               /* no inherited windup */
            g_x_ref_mm  = x_mm;               /* start where we are... */
            g_x_ref_target_mm = CART_CENTER_MM; /* ...and slew to center */
        }
    } else if (g_lqr_state == LQR_ACTIVE) {
        if (fabsf(theta_deg) > DISARM_TILT_DEG) {
            g_lqr_state = LQR_WAITING;   /* auto-recatch */
            g_lqr_v_cmd = 0.0f;
            g_lqr_xi    = 0.0f;
        }
    }
}

/* PLACEHOLDER: kept for the non-LQR modes. */
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

        /* Travel-limit safety: arm once the cart has been clear of both stops. */
        if (x_mm > AXIS_END_MARGIN_MM && x_mm < AXIS_TRAVEL_MM - AXIS_END_MARGIN_MM)
            g_travel_armed = true;

        /* Calibration: scale is degrees-per-count, so at the known angle A:
         *   A = (raw - upright) * scale  ->  scale = A / (raw - upright). */
        if (g_cal_pending) {
            g_cal_pending = false;
            float delta = raw - motion_pot_upright_raw();
            if (fabsf(g_cal_angle) > 0.01f && fabsf(delta) > 0.01f) {
                motion_pot_set_scale(g_cal_angle / delta);
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

        /* Safety: latch, zero output, de-energize. Evaluated before the LQR
         * state update so MAX_TILT_DEG is a real backstop while ACTIVE. The
         * generic tilt trip is bypassed while LQR is disarmed (waiting/off),
         * where the pole may be held far from upright by hand. calmode bypasses
         * the trip globally; missed deadlines still fault. */
        bool lqr_active = (g_mode == CTRL_MODE_LQR && g_lqr_state == LQR_ACTIVE);
        bool tilt_trip  = (!g_calmode && fabsf(theta) > MAX_TILT_DEG);
        if (g_mode == CTRL_MODE_LQR && !lqr_active) tilt_trip = false;
        /* Fault before the carriage can slam a hard stop while balancing. */
        bool travel_trip = (lqr_active && g_travel_armed &&
                            (x_mm < AXIS_END_MARGIN_MM ||
                             x_mm > AXIS_TRAVEL_MM - AXIS_END_MARGIN_MM));
        if (!g_faulted && (tilt_trip || travel_trip ||
                           g_miss_streak > DEADLINE_MISS_FAULT)) {
            g_faulted = true;
            motion_fault_latch();
            if (g_mode == CTRL_MODE_LQR) {
                g_lqr_state = LQR_WAITING;
                g_lqr_v_cmd = 0.0f;
                g_lqr_xi    = 0.0f;
            }
        }

        /* LQR auto-arm: engage near upright, disengage on a large tilt. Skipped
         * while faulted so a reset is required to recover. */
        if (!g_faulted) lqr_update_state(x_mm, theta, g_thetadot);

        float v_cmd;
        if (g_faulted) {
            v_cmd = 0.0f;
            g_lqr_v_cmd = 0.0f;
        } else if (g_mode == CTRL_MODE_LQR) {
            v_cmd = lqr_velocity(x_mm, g_xdot, theta, g_thetadot);
        } else {
            v_cmd = placeholder_velocity(x_mm, g_tick);
        }

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
            .lqr_state = (uint8_t)g_lqr_state,
            .x_ref_mm = g_x_ref_mm,
            .lqr_xi = g_lqr_xi,
            .lqr_k = { g_lqr_k[0], g_lqr_k[1], g_lqr_k[2], g_lqr_k[3], g_lqr_k[4] },
            .xi_clamp = g_xi_clamp,
            .center_deadband_mm = g_center_deadband_mm,
            .x_ref_slew_mm_s = g_x_ref_slew_mm_s,
        };
        telemetry_publish(&t);
        g_tick++;
    }
}

void control_start(void) {
    queue_init(&g_cmd_q, sizeof(cmd_t), 16);   /* Apply sends 8 at once */
    multicore_launch_core1(control_core1_entry);
}
