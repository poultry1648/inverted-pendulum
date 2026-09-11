/*
 * motion.h — axis geometry, limits, step generation and safety for the E belt.
 *
 * No blocking calls: converts a velocity command (mm/s) into signed microstep
 * pulses spread evenly across a fixed control tick. Owns the E step/dir/enable
 * pins, the position counter, velocity/acceleration clamps, the fractional step
 * accumulator and the fault latch. Driven from core 1 (control.c); no printf/USB.
 */
#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/time.h"

/* MICROSTEPS MUST stay in sync with MRES in tmc2209_configure()'s CHOPCONF
 * (currently 4 => 1/16), or steps/mm is silently wrong. */
#define MICROSTEPS         16
#define FULL_STEPS_PER_REV 200
#define STEPS_PER_REV      (FULL_STEPS_PER_REV * MICROSTEPS)   /* 3200 */

/* GT2 belt (2 mm pitch) on a 20-tooth pulley => 40 mm/rev. */
#define BELT_PITCH_MM      2.0f
#define PULLEY_TEETH       20
#define MM_PER_REV         (BELT_PITCH_MM * PULLEY_TEETH)      /* 40 mm */
#define STEPS_PER_MM       (STEPS_PER_REV / MM_PER_REV)        /* 80 steps/mm */

#define AXIS_TRAVEL_MM     350.0f      /* soft limit: rejects bad targets */

/* Fixed-rate control loop: self-clocked at CONTROL_HZ, DT_US per tick. */
#define CONTROL_HZ         1000
#define DT_US              1000
#define DT_S               (1.0f / (float)CONTROL_HZ)

/* A stepper told to jump velocity stalls, so the command is always rate-limited
 * by MAX_CART_ACCEL_MM_S2 before becoming steps; MAX_CART_VEL_MM_S is the cap. */
#define MAX_CART_VEL_MM_S    300.0f
#define MAX_CART_ACCEL_MM_S2 3000.0f

/* Safety: a tilt outside the plausible band, or a sustained run of missed
 * deadlines, latches a fault until reset. */
#define MAX_TILT_DEG          45.0f
#define DEADLINE_SLACK_US     200
#define DEADLINE_MISS_FAULT   10

/* TMC2209 needs ~100 ns high; 3 us is safe. STEP_MIN_GAP_US caps steps/tick so
 * the high time always fits. */
#define STEP_HIGH_US       3
#define STEP_MIN_GAP_US    (STEP_HIGH_US + 2)
#define MOTION_MAX_STEPS_PER_TICK (DT_US / STEP_MIN_GAP_US)

/* Potentiometer on TH0 (GPIO26 = ADC0): electrical rotation and the raw 12-bit
 * ADC values observed at the pot's min/max positions. */
#define POT_ANGLE_DEG 300.0f
#define POT_SAMPLES    8
#define POT_RAW_MIN 20.0f
#define POT_RAW_MAX 2753.0f

/* ---- Pendulum angle calibration -----------------------------------------
 * The pot reports an absolute 0..POT_ANGLE_DEG angle, but the controller needs
 * theta measured from upright and signed:
 *
 *     theta = (raw - THETA_UPRIGHT_RAW) * THETA_DEG_PER_COUNT
 *
 *   theta = 0   when the pole is vertical,
 *   theta > 0   when the pole tilts toward +x (RIGHT, increasing cart mm),
 *   theta < 0   when the pole tilts LEFT.
 *
 * THETA_UPRIGHT_RAW: raw 12-bit ADC value (0..4095) with the pole held vertical.
 *   Measure it with the `zero` command (averages the pot for ~1 s) and paste the
 *   printed value here. The default below is the pot's mid-travel, a reasonable
 *   starting point until calibrated.
 *
 * THETA_DEG_PER_COUNT: signed mechanical degrees per ADC count. The default is
 *   derived from the pot's electrical span (POT_ANGLE_DEG over the raw range) as
 *   a starting point; refine it with the `cal <deg>` command (hold the pole at a
 *   known angle, then paste the printed value here). The SIGN encodes the
 *   mounting direction: make it positive if a RIGHT tilt increases raw, negative
 *   if a RIGHT tilt decreases raw. Both values are overridable at runtime via
 *   motion_pot_set_upright()/motion_pot_set_scale(). */
#define THETA_UPRIGHT_RAW   2398 
#define THETA_DEG_PER_COUNT 0.252809 

void motion_init(void);                 /* E pins up, de-energized, x = 0 */
void motion_enable(bool on);            /* ENABLE is active-low */
void motion_fault_latch(void);          /* latch, zero output, de-energize */
void motion_fault_clear(void);          /* clear latch and re-energize */
bool motion_faulted(void);

int32_t motion_pos_steps(void);
float   motion_pos_mm(void);

/* Clamp v to the limits, convert to signed steps with a fractional accumulator,
 * set DIR and emit pulses evenly across [tick_start, tick_end). Returns signed
 * steps emitted (positive = RIGHT); 0 without stepping while faulted. */
int32_t motion_emit(float v_cmd_mm_s, absolute_time_t tick_start,
                    absolute_time_t tick_end);

void  motion_pot_init(void);            /* core 1 owns the ADC */

/* Returns theta in signed degrees from upright; raw is still reported via
 * raw_out for telemetry. Runtime calibration entry points below. */
float motion_pot_read(float *raw_out);

/* Runtime calibration (called from core 1; values also readable on core 0).
 * Persistence is deliberately NOT implemented here: a flash write stalls XIP
 * and would fault the core 1 control loop, so any future save belongs on core 0
 * before control_start() or with core 1 paused. */
void  motion_pot_set_upright(float raw);        /* capture theta = 0 */
void  motion_pot_set_scale(float deg_per_count);/* signed deg per ADC count */
float motion_pot_upright_raw(void);
float motion_pot_deg_per_count(void);

#endif /* MOTION_H */
