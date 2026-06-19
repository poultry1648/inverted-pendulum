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
 * IMPORTANT: position is dead-reckoned from the 0 = left assumption. There is no
 * homing/endstop here, so power on with the carriage physically at the left end.
 *
 * NOTE: motors are powered from the board's VM input (stepper PSU), not USB.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
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
#define AXIS_TRAVEL_MM     200.0f

/* ---- Step pulse timing ---- */
#define STEP_HIGH_US       3     /* TMC2209 needs ~100 ns; 3 µs is safe */
#define STEP_LOW_US        200   /* gap between pulses -> ~5 kHz step rate */

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

/* Emit `count` step pulses at the configured rate. */
static void e_step(uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        gpio_put(SKR_E_STEP_PIN, 1);
        sleep_us(STEP_HIGH_US);
        gpio_put(SKR_E_STEP_PIN, 0);
        sleep_us(STEP_LOW_US);
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

/* Read one newline-terminated line from USB serial into buf (blocking). */
static void read_line(char *buf, size_t maxlen) {
    size_t n = 0;
    while (true) {
        int c = getchar_timeout_us(1000000);   /* 1 s slices; keep waiting */
        if (c == PICO_ERROR_TIMEOUT) continue;
        if (c == '\r' || c == '\n') {
            if (n == 0) continue;               /* swallow blank lines / CRLF */
            buf[n] = '\0';
            return;
        }
        if (n < maxlen - 1) buf[n++] = (char)c;
    }
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

    /* Carriage assumed parked at the left end == 0 mm. */
    g_pos_steps = 0;

    printf("\nSKR Pico belt axis ready. Range 0..%.0f mm (0 = left).\n",
           AXIS_TRAVEL_MM);
    printf("Send a target coordinate in mm and press enter.\n");

    char line[32];
    while (true) {
        printf("pos %.2f mm > ", g_pos_steps / STEPS_PER_MM);
        read_line(line, sizeof(line));

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
    }
}
