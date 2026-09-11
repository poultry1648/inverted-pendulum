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
float motion_pot_read(float *raw_out);

#endif /* MOTION_H */
