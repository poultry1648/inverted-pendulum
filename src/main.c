/*
 * SKR Pico v1.0 — spin the E (extruder) stepper.
 *
 * On boot it puts the board in a safe state (heaters OFF, all steppers
 * disabled), configures the E-axis TMC2209 over UART (current + 1/16
 * microstepping), then continuously rotates the motor, reversing every couple
 * of revolutions. Thermistor ADC readings are printed over USB before each
 * move.
 *
 * NOTE: motors are powered from the board's VM input (stepper PSU), NOT from
 * USB. With USB only, config succeeds but the motor will not turn.
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "skr_pico.h"
#include "tmc2209.h"

/* 1.8° motor (200 full steps/rev) at 1/16 microstepping. */
#define MICROSTEPS         16
#define FULL_STEPS_PER_REV 200
#define STEPS_PER_REV      (FULL_STEPS_PER_REV * MICROSTEPS)

#define STEP_HIGH_US       3     /* TMC2209 needs only ~100 ns; 3 µs is safe */
#define STEP_LOW_US        200   /* gap between pulses -> ~5 kHz step rate    */

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

static void e_set_dir(bool forward) {
    gpio_put(SKR_E_DIR_PIN, forward ? 1 : 0);
}

static void e_enable(bool on) {
    gpio_put(SKR_E_ENABLE_PIN, on ? 0 : 1); /* active-low */
}

/* Emit `count` step pulses on the E axis at the configured rate. */
static void e_step(uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        gpio_put(SKR_E_STEP_PIN, 1);
        sleep_us(STEP_HIGH_US);
        gpio_put(SKR_E_STEP_PIN, 0);
        sleep_us(STEP_LOW_US);
    }
}

static uint16_t therm_read_raw(uint adc_channel) {
    adc_select_input(adc_channel);
    return adc_read();
}

int main(void) {
    stdio_init_all();

    /* Fail safe before anything else. */
    heaters_off();
    steppers_disable();

    adc_init();
    adc_gpio_init(SKR_HOTEND_THERM_PIN);
    adc_gpio_init(SKR_BED_THERM_PIN);

    /* Configure the E driver over UART, then bring up the step/dir pins. */
    tmc2209_uart_init();
    tmc2209_configure(TMC_ADDR_E);
    e_axis_init();
    e_enable(true);

    const float adc_to_volts = 3.3f / (1 << 12);
    bool forward = true;

    while (true) {
        printf("E spin %-7s | hotend ADC=%u (%.3f V) | bed ADC=%u (%.3f V)\n",
               forward ? "forward" : "reverse",
               therm_read_raw(SKR_HOTEND_THERM_ADC),
               therm_read_raw(SKR_HOTEND_THERM_ADC) * adc_to_volts,
               therm_read_raw(SKR_BED_THERM_ADC),
               therm_read_raw(SKR_BED_THERM_ADC) * adc_to_volts);

        e_set_dir(forward);
        e_step(STEPS_PER_REV * 5);   /* ~2 revolutions */
        forward = !forward;
    }
}
