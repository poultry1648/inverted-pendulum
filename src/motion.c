/*
 * motion.c — see motion.h. No printf, USB or sleep_us().
 *
 * Step generation spreads N pulses evenly across a tick: pulse i at offset
 * (i * span) / N, so the loop stays self-clocked and no per-step interrupt or
 * PIO FIFO is needed. Upgrade path if pulse timing ever needs offloading: a PIO
 * state machine fed the step count + period by this same code.
 */
#include "motion.h"
#include "skr_pico.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include <math.h>

static int32_t g_pos_steps = 0;   /* from the left end, positive = RIGHT */
static float   g_carry     = 0.0f;/* sub-step remainder (never lose velocity) */
static float   g_v_ramped  = 0.0f;/* acceleration-limited commanded velocity */
static bool    g_faulted   = false;

void motion_init(void) {
    const uint pins[] = { SKR_E_STEP_PIN, SKR_E_DIR_PIN, SKR_E_ENABLE_PIN };
    for (size_t i = 0; i < count_of(pins); i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
    }
    gpio_put(SKR_E_STEP_PIN, 0);
    gpio_put(SKR_E_DIR_PIN, 0);
    gpio_put(SKR_E_ENABLE_PIN, 1);   /* active-low: start de-energized */
    g_pos_steps = 0;
    g_carry = g_v_ramped = 0.0f;
    g_faulted = false;
}

void motion_enable(bool on) { gpio_put(SKR_E_ENABLE_PIN, on ? 0 : 1); }
bool motion_faulted(void)   { return g_faulted; }

void motion_fault_latch(void) {
    g_faulted = true;
    g_carry = g_v_ramped = 0.0f;
    gpio_put(SKR_E_ENABLE_PIN, 1);   /* de-energize */
}

void motion_fault_clear(void) {
    g_faulted = false;
    g_carry = g_v_ramped = 0.0f;
    gpio_put(SKR_E_ENABLE_PIN, 0);   /* re-energize */
}

int32_t motion_pos_steps(void) { return g_pos_steps; }
float   motion_pos_mm(void)    { return (float)g_pos_steps / STEPS_PER_MM; }

/* Emit n pulses with leading edges spread evenly across [t0, t1). Callers
 * guarantee n <= MOTION_MAX_STEPS_PER_TICK so STEP_HIGH_US always fits. */
static void emit_pulses(uint32_t n, absolute_time_t t0, absolute_time_t t1) {
    int64_t span = absolute_time_diff_us(t0, t1);
    if (span <= 0) span = DT_US;    /* fell behind: fall back to a full tick */
    for (uint32_t i = 0; i < n; i++) {
        uint32_t off = (uint32_t)(((uint64_t)i * (uint64_t)span) / n);
        busy_wait_until(delayed_by_us(t0, off));
        gpio_put(SKR_E_STEP_PIN, 1);
        busy_wait_until(delayed_by_us(t0, (uint64_t)off + STEP_HIGH_US));
        gpio_put(SKR_E_STEP_PIN, 0);
    }
}

int32_t motion_emit(float v, absolute_time_t tick_start, absolute_time_t tick_end) {
    if (g_faulted) return 0;

    /* Clamp velocity, then acceleration (per-tick delta). */
    if (v >  MAX_CART_VEL_MM_S) v =  MAX_CART_VEL_MM_S;
    if (v < -MAX_CART_VEL_MM_S) v = -MAX_CART_VEL_MM_S;
    float dv_max = MAX_CART_ACCEL_MM_S2 * DT_S;
    g_v_ramped += fmaxf(-dv_max, fminf(dv_max, v - g_v_ramped));

    /* Fractional step accumulator: truncate toward zero, keep the remainder. */
    float desired = g_v_ramped * STEPS_PER_MM * DT_S + g_carry;
    int32_t n = (int32_t)desired;
    int32_t n_abs = n < 0 ? -n : n;
    if (n_abs > MOTION_MAX_STEPS_PER_TICK) {   /* saturate, drop windup */
        n = n < 0 ? -(int32_t)MOTION_MAX_STEPS_PER_TICK
                  :  (int32_t)MOTION_MAX_STEPS_PER_TICK;
        n_abs = MOTION_MAX_STEPS_PER_TICK;
        g_carry = 0.0f;
    } else {
        g_carry = desired - (float)n;
    }
    if (n_abs == 0) return 0;

    gpio_put(SKR_E_DIR_PIN, n < 0 ? 1 : 0);   /* DIR=1 (forward) drives LEFT */
    emit_pulses((uint32_t)n_abs, tick_start, tick_end);
    g_pos_steps += n;
    return n;
}

/* Upright reference and signed scale. Runtime-calibratable; defaults come from
 * motion.h. NOTE: no flash persistence by design — flash writes stall XIP and
 * would fault the core 1 loop. Save calibration from core 0 before
 * control_start() (or with core 1 paused) if persistence is added later. */
static float g_upright_raw   = THETA_UPRIGHT_RAW;
static float g_deg_per_count = THETA_DEG_PER_COUNT;

void  motion_pot_set_upright(float raw)         { g_upright_raw = raw; }
void  motion_pot_set_scale(float deg_per_count) { g_deg_per_count = deg_per_count; }
float motion_pot_upright_raw(void)              { return g_upright_raw; }
float motion_pot_deg_per_count(void)            { return g_deg_per_count; }

void motion_pot_init(void) {
    adc_init();
    adc_gpio_init(SKR_BED_THERM_PIN);   /* TH0 = GPIO26 = ADC0 */
    adc_select_input(SKR_BED_THERM_ADC);
}

/* theta = (raw - upright) * deg_per_count: 0 when vertical, +RIGHT / -LEFT.
 * raw_out keeps the raw ADC value for telemetry. */
float motion_pot_read(float *raw_out) {
    uint32_t sum = 0;
    for (int i = 0; i < POT_SAMPLES; i++) sum += adc_read();
    float raw = (float)(sum / POT_SAMPLES);
    if (raw_out) *raw_out = raw;
    return (raw - g_upright_raw) * g_deg_per_count;
}
