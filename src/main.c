/*
 * SKR Pico v1.0 — single-axis belt gantry, "go to coordinate" over USB.
 *
 * The E driver drives a belt/pulley that moves a carriage along a linear axis.
 * Send a target coordinate in millimetres over the USB serial port (one number
 * per line) and the carriage moves there.
 *
 * Conventions (per the mechanical setup):
 *   - The carriage starts parked on the LEFT; that position is 0 mm.
 *   - Coordinates increase to the RIGHT (0 .. AXIS_TRAVEL_MM).
 *   - The motor's "forward" rotation (DIR=1) moves the carriage LEFT, so moving
 *     toward a larger coordinate requires the REVERSE direction.
 *
 * IMPORTANT: position is established by open-loop hard-stop homing at boot:
 * the carriage is crept into the LEFT stop, that position is declared 0, then
 * the carriage backs off to the right by HOME_BACKOFF_MM. There is no endstop.
 *
 * NOTE: motors are powered from the board's VM input (stepper PSU), not USB.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "skr_pico.h"
#include "tmc2209.h"

/* ---- Motor / driver ---- */
#define MICROSTEPS         16
#define FULL_STEPS_PER_REV 200
#define STEPS_PER_REV      (FULL_STEPS_PER_REV * MICROSTEPS)   /* 3200 */

/* ---- Belt geometry (adjust to your hardware!) ----
 * Default: GT2 belt (2 mm pitch) on a 20-tooth pulley => 40 mm per revolution. */
#define BELT_PITCH_MM      2.0f
#define PULLEY_TEETH       20
#define MM_PER_REV         (BELT_PITCH_MM * PULLEY_TEETH)      /* 40 mm */
#define STEPS_PER_MM       (STEPS_PER_REV / MM_PER_REV)        /* 80 steps/mm */

/* ---- Soft travel limit (no endstop, so this just rejects bad targets) ---- */
#define AXIS_TRAVEL_MM     350.0f

/* ---- Step pulse timing & acceleration ramp ----
 * Speed is set by the inter-pulse gap. We ramp the gap from STEP_START_US down
 * to STEP_MIN_US over the first STEP_RAMP steps (and back up over the last
 * STEP_RAMP), so the motor accelerates instead of slamming to top speed and
 * stalling. At 80 steps/mm: STEP_MIN_US=75 -> ~12.8 kHz -> ~160 mm/s cruise.
 * Lower STEP_MIN_US for more speed; if it stalls, raise it or lengthen STEP_RAMP. */
#define STEP_HIGH_US       3     /* TMC2209 needs ~100 ns; 3 µs is safe */
#define STEP_START_US      350   /* gap for the first/last step (slow ends) */
#define STEP_MIN_US        50    /* gap at cruise (top speed) */
#define STEP_RAMP          600   /* steps spent accelerating (and decelerating) */

/* ---- Open-loop hard-stop homing ----
 * There is no endstop, so homing deliberately drives the carriage into the
 * LEFT hard stop and lets the motor skip steps against it (open-loop). That
 * pressed position becomes 0 mm. The creep uses a slow constant inter-pulse gap
 * (no acceleration ramp) so the stall is gentle; HOME_OVERTRAVEL_MM must exceed
 * the full travel so the stop is reached from anywhere on the axis, and
 * HOME_MAX_STEPS hard-bounds the move so homing always terminates even if the
 * carriage jams. */
#define HOME_CREEP_GAP_US   600                             /* ~20 mm/s creep */
#define HOME_OVERTRAVEL_MM  (AXIS_TRAVEL_MM + 20.0f)        /* > full travel */
#define HOME_BACKOFF_MM     15.0f                           /* clear the stop */
#define HOME_MAX_STEPS      ((uint32_t)(HOME_OVERTRAVEL_MM * STEPS_PER_MM) + 1u)

/* Potentiometer on TH0 (GPIO26 = ADC0). Full-scale electrical rotation of the
 * pot, in degrees; adjust to match the actual part (most single-turn pots ~300). */
#define POT_ANGLE_DEG 300.0f
#define POT_SAMPLES    8

/* Calibration: the raw 12-bit ADC values observed at the pot's minimum and
 * maximum positions. Read the "raw=" number printed in the terminal at each
 * end of travel and put those values here; the angle then maps cleanly across
 * 0..POT_ANGLE_DEG. */
#define POT_RAW_MIN 20.0f
#define POT_RAW_MAX 2753.0f

/* Absolute carriage position, in microsteps, measured right from the left end. */
static int32_t g_pos_steps = 0;

/* TMC2209 ENABLE is active-low: HIGH = de-energized. */
static void steppers_disable(void) {
    const uint en_pins[] = {
        SKR_X_ENABLE_PIN, SKR_Y_ENABLE_PIN,
        SKR_Z_ENABLE_PIN, SKR_E_ENABLE_PIN,
    };
    for (size_t i = 0; i < count_of(en_pins); i++) {
        gpio_init(en_pins[i]);
        gpio_set_dir(en_pins[i], GPIO_OUT);
        gpio_put(en_pins[i], 1);
    }
}

/* Heaters are low-side N-FETs: LOW = off. Never leave these floating. */
static void heaters_off(void) {
    const uint heat_pins[] = { SKR_HOTEND_HEATER_PIN, SKR_BED_HEATER_PIN };
    for (size_t i = 0; i < count_of(heat_pins); i++) {
        gpio_init(heat_pins[i]);
        gpio_set_dir(heat_pins[i], GPIO_OUT);
        gpio_put(heat_pins[i], 0);
    }
}

static void e_axis_init(void) {
    gpio_init(SKR_E_STEP_PIN);
    gpio_set_dir(SKR_E_STEP_PIN, GPIO_OUT);
    gpio_put(SKR_E_STEP_PIN, 0);

    gpio_init(SKR_E_DIR_PIN);
    gpio_set_dir(SKR_E_DIR_PIN, GPIO_OUT);
    gpio_put(SKR_E_DIR_PIN, 0);
}

/* forward == true => motor "forward" (DIR=1) => carriage moves LEFT. */
static void e_set_dir(bool forward) {
    gpio_put(SKR_E_DIR_PIN, forward ? 1 : 0);
}

static void e_enable(bool on) {
    gpio_put(SKR_E_ENABLE_PIN, on ? 0 : 1); /* active-low */
}

/* Inter-pulse gap for step `idx` of a `count`-step move: ramps up to cruise
 * speed at the start and back down at the end. If the move is shorter than
 * 2*STEP_RAMP, this naturally yields a triangular profile that never reaches
 * full speed (so short moves stay safe too). */
static uint32_t step_gap_us(uint32_t idx, uint32_t count) {
    uint32_t from_start = idx;
    uint32_t from_end   = count - 1 - idx;
    uint32_t edge = (from_start < from_end) ? from_start : from_end;
    if (edge >= STEP_RAMP) return STEP_MIN_US;          /* cruising */
    uint32_t span = STEP_START_US - STEP_MIN_US;        /* linear ramp */
    return STEP_START_US - (span * edge) / STEP_RAMP;
}

/* Emit `count` step pulses with an acceleration/deceleration ramp. */
static void e_step(uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        gpio_put(SKR_E_STEP_PIN, 1);
        sleep_us(STEP_HIGH_US);
        gpio_put(SKR_E_STEP_PIN, 0);
        sleep_us(step_gap_us(i, count));
    }
}

/* Emit `count` step pulses at a fixed inter-pulse gap (no accel ramp). Used by
 * homing, where a slow constant creep into the stop is wanted. */
static void e_step_const(uint32_t count, uint32_t gap_us) {
    for (uint32_t i = 0; i < count; i++) {
        gpio_put(SKR_E_STEP_PIN, 1);
        sleep_us(STEP_HIGH_US);
        gpio_put(SKR_E_STEP_PIN, 0);
        sleep_us(gap_us);
    }
}

/* Move the carriage to an absolute coordinate (mm). Updates g_pos_steps. */
static void move_to_mm(float target_mm) {
    int32_t target_steps = (int32_t)lroundf(target_mm * STEPS_PER_MM);
    int32_t delta = target_steps - g_pos_steps;
    if (delta == 0) {
        printf("already at %.2f mm\n", target_mm);
        return;
    }

    /* delta > 0 => need to move RIGHT (larger coord) => reverse direction.
     * delta < 0 => move LEFT => forward direction. */
    bool forward = (delta < 0);
    e_set_dir(forward);
    e_step((uint32_t)labs(delta));

    g_pos_steps = target_steps;
    printf("moved %s to %.2f mm\n",
           forward ? "left" : "right", g_pos_steps / STEPS_PER_MM);
}

/* Open-loop hard-stop homing. Creeps LEFT into the stop for more than the full
 * travel, declares that pressed position 0, then backs off to the right by
 * HOME_BACKOFF_MM. Blocking; call with the E driver configured and enabled. */
static void home_axis(void) {
    printf("Homing: creeping left to hard stop (max %u steps)...\n",
           (unsigned)HOME_MAX_STEPS);
    e_set_dir(true);                                  /* forward => LEFT */
    e_step_const(HOME_MAX_STEPS, HOME_CREEP_GAP_US);

    /* The carriage is now pressed against the left stop: reference zero. */
    g_pos_steps = 0;
    printf("Homing: left stop reached, position zeroed.\n");

    /* Back off so the carriage is not resting on the stop. */
    int32_t backoff_steps = (int32_t)lroundf(HOME_BACKOFF_MM * STEPS_PER_MM);
    e_set_dir(false);                                 /* reverse => RIGHT */
    e_step_const((uint32_t)backoff_steps, HOME_CREEP_GAP_US);
    g_pos_steps = backoff_steps;
    printf("Homing: backed off to %.2f mm.\n", g_pos_steps / STEPS_PER_MM);
}

/* Sample the pot on TH0. Returns angle in degrees; also reports the raw
 * 12-bit ADC reading via `raw_out` for diagnostics. */
static float pot_read(float *raw_out) {
    uint32_t sum = 0;
    for (int i = 0; i < POT_SAMPLES; i++) {
        sum += adc_read();
    }
    float raw = (float)(sum / POT_SAMPLES);
    if (raw_out) *raw_out = raw;
    return (raw - POT_RAW_MIN) / (POT_RAW_MAX - POT_RAW_MIN) * POT_ANGLE_DEG;
}

int main(void) {
    stdio_init_all();

    /* Fail safe before anything else. */
    heaters_off();
    steppers_disable();

    /* Configure the E driver over UART, then bring up the step/dir pins. */
    tmc2209_uart_init();
    tmc2209_configure(TMC_ADDR_E);
    e_axis_init();
    e_enable(true);

    /* Establish a true zero by pressing into the left hard stop. */
    home_axis();

    /* TH0 = GPIO26 = ADC0. */
    adc_init();
    adc_gpio_init(SKR_BED_THERM_PIN);
    adc_select_input(SKR_BED_THERM_ADC);

    printf("\nSKR Pico ready. Range 0..%.0f mm (0 = left).\n",
           AXIS_TRAVEL_MM);
    printf("Potentiometer on TH0: angle printed as you turn it.\n");
    printf("Send a coordinate in mm and press enter to move the axis.\n");
    printf("Send 'home' to re-home against the left stop.\n");

    char line[32];
    size_t n = 0;
    uint32_t last = to_ms_since_boot(get_absolute_time());

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last >= 200) {
            last = now;
            float raw;
            float angle = pot_read(&raw);
            printf("pos=%6.2f mm  TH0 raw=%4.0f  %5.1f deg\n",
                   g_pos_steps / STEPS_PER_MM, raw, angle);
        }

        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) continue;
        if (c == '\r' || c == '\n') {
            if (n == 0) continue;               /* swallow blank lines / CRLF */
            line[n] = '\0';
            n = 0;

            /* "home" (or "h"/"H") re-runs open-loop homing on demand. */
            if (strcmp(line, "home") == 0 ||
                (line[1] == '\0' && (line[0] == 'h' || line[0] == 'H'))) {
                home_axis();
                continue;
            }

            char *end;
            float target_mm = strtof(line, &end);
            if (end == line) {
                printf("not a number: \"%s\"\n", line);
                continue;
            }
            if (target_mm < 0.0f || target_mm > AXIS_TRAVEL_MM) {
                printf("out of range: %.2f mm (allowed 0..%.0f)\n",
                       target_mm, AXIS_TRAVEL_MM);
                continue;
            }
            move_to_mm(target_mm);
        } else if (n < sizeof(line) - 1) {
            line[n++] = (char)c;
        }
    }
}
