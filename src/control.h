/*
 * control.h — core 1 real-time loop and the core 0 <-> core 1 interface.
 *
 * Core 1 runs a self-clocked 1 kHz loop: sample pot, estimate state, run the
 * selected control law (placeholder modes or the cart-pole LQR), emit steps
 * (motion.c), check safety and publish a telemetry snapshot. Core 0 posts
 * commands and reads snapshots.
 *
 * CTRL_MODE_LQR drives the cart-pole LQR (gains in lqr_gains.h) through an
 * auto-arm state machine: selecting the mode waits until the pole is near
 * upright, then engages on its own; a large tilt disengages back to waiting.
 */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    CTRL_MODE_IDLE = 0,       /* v = 0 */
    CTRL_MODE_POSITION_HOLD,  /* v = Kp * (target - x), clamped */
    CTRL_MODE_CONST_VEL,      /* v = vel */
    CTRL_MODE_SQUARE_VEL,     /* +/- amp, freq Hz */
    CTRL_MODE_SINE_VEL,       /* amp * sin(2*pi*freq*t) */
    CTRL_MODE_LQR,            /* cart-pole LQR with auto-arm state machine */
} control_mode_t;

/* LQR auto-arm state (only meaningful while mode == CTRL_MODE_LQR). */
typedef enum {
    LQR_OFF = 0,     /* not balancing */
    LQR_WAITING = 1, /* armed to engage; output 0, tilt trip bypassed */
    LQR_ACTIVE = 2,  /* law running */
} lqr_state_t;

typedef enum {
    CMD_STOP = 0,   /* zero output, select idle */
    CMD_GO,         /* start/resume (idle -> position-hold) */
    CMD_TARGET,     /* value = absolute target mm, selects position-hold */
    CMD_MODE,       /* mode = control_mode_t */
    CMD_AMP,        /* value = square/sine amplitude mm/s */
    CMD_FREQ,       /* value = square/sine frequency Hz */
    CMD_VEL,        /* value = constant velocity mm/s */
    CMD_KP,         /* value = position-hold gain */
    CMD_RESET,      /* clear fault + reset stats */
    CMD_ZERO,       /* average pot ~1 s and set that raw as upright (theta=0) */
    CMD_CAL,        /* value = pole's known angle from upright; set signed scale */
    CMD_CALMODE,    /* value != 0 -> bypass the tilt trip while calibrating */
} control_cmd_type_t;

typedef struct {
    int32_t  x_steps;
    float    x_mm;
    float    theta_deg;
    float    raw;
    float    thetadot;
    float    xdot;
    float    v_cmd;
    uint32_t tick;
    uint32_t period_min_us;
    uint32_t period_max_us;
    uint32_t period_mean_us;
    uint32_t missed;
    uint32_t fault;
    uint8_t  mode;
    /* Calibration snapshot (cal_seq bumps when upright/scale change). */
    float    upright_raw;
    float    deg_per_count;
    uint8_t  calmode;
    uint32_t cal_seq;
    /* LQR auto-arm state, cart centering reference and its integral. */
    uint8_t  lqr_state;      /* lqr_state_t */
    float    x_ref_mm;
    float    lqr_xi;         /* m*s: integral of (x - x_ref); clamp = windup */
} telemetry_t;

void control_start(void);                       /* launch core 1 (call on core 0) */
void control_post_cmd(uint8_t type, uint8_t mode, float value);
bool control_get_telemetry(telemetry_t *out);   /* false if a read raced a write */

#endif /* CONTROL_H */
