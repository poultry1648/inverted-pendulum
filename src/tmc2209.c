#include "tmc2209.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "skr_pico.h"

/* GP8/GP9 are UART1's TX/RX on the RP2040. */
#define TMC_UART      uart1
#define TMC_BAUD      115200   /* TMC2209 auto-detects baud from the sync byte */

/* TMC single-wire UART CRC (poly 0x07, LSB-first per byte). */
static uint8_t tmc_crc(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (b & 0x01))
                crc = (uint8_t)((crc << 1) ^ 0x07);
            else
                crc = (uint8_t)(crc << 1);
            b >>= 1;
        }
    }
    return crc;
}

void tmc2209_uart_init(void) {
    uart_init(TMC_UART, TMC_BAUD);
    gpio_set_function(SKR_TMC_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(SKR_TMC_UART_RX_PIN, GPIO_FUNC_UART);
}

void tmc2209_write(uint8_t addr, uint8_t reg, uint32_t value) {
    uint8_t d[8];
    d[0] = 0x05;            /* sync + reserved */
    d[1] = addr;            /* slave address  */
    d[2] = reg | 0x80;      /* register + write flag */
    d[3] = (value >> 24) & 0xFF;
    d[4] = (value >> 16) & 0xFF;
    d[5] = (value >> 8)  & 0xFF;
    d[6] = (value)       & 0xFF;
    d[7] = tmc_crc(d, 7);

    uart_write_blocking(TMC_UART, d, sizeof(d));
    uart_tx_wait_blocking(TMC_UART);  /* let the frame finish before the next */
}

void tmc2209_configure(uint8_t addr) {
    /* GCONF: I_scale_analog=0 (internal ref), pdn_disable=1 (enable UART path),
     * mstep_reg_select=1 (microsteps from CHOPCONF, not MS pins), multistep_filt=1. */
    tmc2209_write(addr, TMC_REG_GCONF, 0x000001C0u);

    /* CHOPCONF: reset default 0x10000053 (TOFF=3, intpol on) with MRES=4 => 1/16. */
    tmc2209_write(addr, TMC_REG_CHOPCONF, 0x14000053u);

    /* IHOLD_IRUN: IHOLDDELAY=2, IRUN=16/31 (~half), IHOLD=8/31. */
    tmc2209_write(addr, TMC_REG_IHOLD_IRUN, 0x00021008u);

    /* TPOWERDOWN: delay before dropping to hold current at standstill. */
    tmc2209_write(addr, TMC_REG_TPOWERDOWN, 20u);
}
