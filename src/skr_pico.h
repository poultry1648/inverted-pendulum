/*
 * skr_pico.h — BIGTREETECH SKR Pico v1.0 pin map for the Raspberry Pi Pico SDK
 *
 * The SKR Pico is built around the RP2040 (dual Cortex-M0+). Every peripheral
 * on the board is wired to a fixed GPIO, so these macros are the single source
 * of truth for the rest of the firmware. Pin numbers are GPIO numbers as seen
 * by the SDK (gpioN), taken from BIGTREETECH's published pinout / Klipper cfg.
 *
 * ADC note (RP2040): ADC0=GPIO26, ADC1=GPIO27, ADC2=GPIO28, ADC3=GPIO29.
 */
#ifndef SKR_PICO_H
#define SKR_PICO_H

/* ---- Stepper drivers (4x onboard TMC2209) ----
 * Each axis has STEP / DIR / ENABLE. ENABLE is active-low on the TMC2209. */
#define SKR_X_STEP_PIN   11
#define SKR_X_DIR_PIN    10
#define SKR_X_ENABLE_PIN 12

#define SKR_Y_STEP_PIN   6
#define SKR_Y_DIR_PIN    5
#define SKR_Y_ENABLE_PIN 7

#define SKR_Z_STEP_PIN   19
#define SKR_Z_DIR_PIN    28
#define SKR_Z_ENABLE_PIN 2

#define SKR_E_STEP_PIN   14
#define SKR_E_DIR_PIN    13
#define SKR_E_ENABLE_PIN 15

/* ---- TMC2209 single-wire UART ----
 * All four drivers share one half-duplex UART line. TX/RX are tied together on
 * the board through resistors; gpio8 = TX, gpio9 = RX (PIO/soft-UART in Klipper).
 * Driver addresses are set by jumpers (MS1/MS2): X=0, Y=2, Z=1, E=3 by default. */
#define SKR_TMC_UART_TX_PIN 8
#define SKR_TMC_UART_RX_PIN 9

/* ---- Endstops / probe ----
 * X and Y endstop headers double as the TMC2209 DIAG pins for sensorless homing. */
#define SKR_X_ENDSTOP_PIN 4
#define SKR_Y_ENDSTOP_PIN 3
#define SKR_Z_ENDSTOP_PIN 25
#define SKR_PROBE_PIN     16   /* "E0-stop" header; also used for filament sensor */

/* ---- Heaters (low-side N-FET, drive HIGH = on) ---- */
#define SKR_HOTEND_HEATER_PIN 23
#define SKR_BED_HEATER_PIN    21

/* ---- Thermistors (100k NTC, on RP2040 ADC) ---- */
#define SKR_HOTEND_THERM_PIN  27   /* ADC1 */
#define SKR_BED_THERM_PIN      26   /* ADC0 */
#define SKR_HOTEND_THERM_ADC   1
#define SKR_BED_THERM_ADC      0

/* ---- Fans (PWM-capable) ---- */
#define SKR_FAN_HOTEND_PIN 18   /* heatbreak / hotend cooling fan (FAN1) */
#define SKR_FAN_PART_PIN   17   /* part cooling fan (FAN0) */
#define SKR_FAN_MCU_PIN    20   /* controller/board fan (FAN2) */

/* ---- Misc ---- */
#define SKR_NEOPIXEL_PIN   24   /* onboard RGB (WS2812), drive via PIO */

#endif /* SKR_PICO_H */
