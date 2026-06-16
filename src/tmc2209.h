/*
 * tmc2209.h — minimal single-wire UART config for the SKR Pico's TMC2209s.
 *
 * The four onboard drivers share one half-duplex UART line. On the RP2040 that
 * line lands on UART1 (TX=GP8, RX=GP9). Each driver has a fixed address set by
 * its MS1/MS2 jumpers; on the stock SKR Pico the extruder (E) driver is addr 3.
 *
 * This module only *writes* registers (enough to set current + microstepping so
 * step/dir motion works). TX echoes onto RX because the lines are bridged on the
 * board; since we never read back, that echo is simply ignored.
 */
#ifndef TMC2209_H
#define TMC2209_H

#include <stdint.h>

/* Driver UART addresses on the stock SKR Pico (MS1/MS2 jumpers). */
#define TMC_ADDR_X 0
#define TMC_ADDR_Y 2
#define TMC_ADDR_Z 1
#define TMC_ADDR_E 3

/* Register addresses (write = reg | 0x80). */
#define TMC_REG_GCONF       0x00
#define TMC_REG_IHOLD_IRUN  0x10
#define TMC_REG_TPOWERDOWN  0x11
#define TMC_REG_CHOPCONF    0x6C

/* Bring up UART1 on the TMC pins. Call once before configuring any driver. */
void tmc2209_uart_init(void);

/* Write one 32-bit register to the driver at `addr`. */
void tmc2209_write(uint8_t addr, uint8_t reg, uint32_t value);

/*
 * Apply a sane step/dir-capable config to one driver:
 *   - internal current reference (not VREF), UART-controlled microsteps,
 *   - 1/16 microstepping with 256-interpolation,
 *   - moderate run/hold current.
 * After this, toggling STEP with the driver enabled will move the motor.
 */
void tmc2209_configure(uint8_t addr);

#endif /* TMC2209_H */
