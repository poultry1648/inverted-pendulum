/*
 * SKR Pico v1.0 — bring-up firmware skeleton.
 *
 * This is a safe starting point, not a motion controller. On boot it:
 *   - puts the board in a known-safe state (heaters OFF, steppers DISABLED),
 *   - configures the part-cooling fan on a PWM slice,
 *   - reads both 100k NTC thermistors on the RP2040 ADC,
 *   - prints a status line over USB-CDC once per second.
 *
 * Build produces skr_pico_fw.uf2; flash via BOOTSEL (see README).
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "skr_pico.h"

/* TMC2209 ENABLE is active-low: drive HIGH to leave motors de-energized. */
static void steppers_disable(void) {
    const uint en_pins[] = {
        SKR_X_ENABLE_PIN, SKR_Y_ENABLE_PIN,
        SKR_Z_ENABLE_PIN, SKR_E_ENABLE_PIN,
    };
    for (size_t i = 0; i < count_of(en_pins); i++) {
        gpio_init(en_pins[i]);
        gpio_set_dir(en_pins[i], GPIO_OUT);
        gpio_put(en_pins[i], 1); /* disabled */
    }
}

/* Heaters are low-side N-FETs: drive LOW = off. Never leave these floating. */
static void heaters_off(void) {
    const uint heat_pins[] = { SKR_HOTEND_HEATER_PIN, SKR_BED_HEATER_PIN };
    for (size_t i = 0; i < count_of(heat_pins); i++) {
        gpio_init(heat_pins[i]);
        gpio_set_dir(heat_pins[i], GPIO_OUT);
        gpio_put(heat_pins[i], 0); /* off */
    }
}

/* Configure one GPIO as a PWM output and return its slice number. */
static uint fan_pwm_init(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 255);            /* 8-bit duty */
    pwm_set_gpio_level(pin, 0);          /* start off */
    pwm_set_enabled(slice, true);
    return slice;
}

/* Read a thermistor channel and return the raw 12-bit ADC value (0..4095). */
static uint16_t therm_read_raw(uint adc_channel) {
    adc_select_input(adc_channel);
    return adc_read();
}

int main(void) {
    stdio_init_all();

    /* Fail safe first, before anything else can go wrong. */
    heaters_off();
    steppers_disable();

    /* Thermistor inputs on the ADC. init_pin wires the GPIO to the ADC mux. */
    adc_init();
    adc_gpio_init(SKR_HOTEND_THERM_PIN);
    adc_gpio_init(SKR_BED_THERM_PIN);

    /* Part-cooling fan on PWM. */
    fan_pwm_init(SKR_FAN_PART_PIN);

    const float adc_to_volts = 3.3f / (1 << 12);

    while (true) {
        uint16_t hot_raw = therm_read_raw(SKR_HOTEND_THERM_ADC);
        uint16_t bed_raw = therm_read_raw(SKR_BED_THERM_ADC);

        printf("SKR Pico alive | hotend ADC=%u (%.3f V) | bed ADC=%u (%.3f V)\n",
               hot_raw, hot_raw * adc_to_volts,
               bed_raw, bed_raw * adc_to_volts);

        sleep_ms(1000);
    }
}
