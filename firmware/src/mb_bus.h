#pragma once

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "mb_pins.h"

#define MB_SM_ADDR   0
#define MB_SM_WRITE  1
#define MB_SM_READ   2
#define MB_SM_SYNC0  0   // pio1
#define MB_SM_SYNC1  1
#define MB_SM_PHI2   2   // pio1: PHI2 cycle counter

#define MB_ACC_ADDR(w)    ((uint8_t)((w) & 0xFF))   // A0..A7
#define MB_ACC_IOSEL(w)   (((w) >> 8) & 1u)         // bit8: 0 = asserted
#define MB_ACC_IS_READ(w) (((w) >> 9) & 1u)         // bit9: 1 = read, 0 = write

void mb_bus_init(void);

// (re)arm a channel to a PHI0-cycle period, 0 stops it
void mb_sync_init(void (*tick_cb)(int chan));
void mb_sync_restart(int chan, uint32_t phi0_cycles);

// inline: on the read-response critical path, no call/divide may survive
extern volatile uint32_t g_phi2_mirror;
static inline uint32_t mb_phi2_ticks(void) {
    return 0u - g_phi2_mirror;
}

// arm 0 = cancel, irq returns the NVIC IRQ to mask while editing the shared IFR (-1 if none)
void mb_alarm_init(void (*t2_cb)(int chan));
void mb_alarm_arm(int chan, uint32_t phi2_cycles);
int  mb_alarm_irq(void);

int  mb_ar_tick_init(void (*cb)(void));

// reset_start() must run after init/via_init/reset_handler, so the bus never wakes with stale VIA state
void mb_bus_reset_handler(void (*handler)(bool asserted));
void mb_bus_reset_start(void);

static inline uint32_t __not_in_flash_func(mb_bus_get_access)(void) {
    while (pio0->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + MB_SM_ADDR))) {
        tight_loop_contents();
    }
    return pio0->rxf[MB_SM_ADDR];
}

// bounded wait, 0 if the byte never lands
static inline uint8_t __not_in_flash_func(mb_bus_get_wdata)(void) {
    uint retry = 64;
    while ((pio0->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + MB_SM_WRITE))) && --retry) {
        tight_loop_contents();
    }
    return (uint8_t)(pio0->rxf[MB_SM_WRITE] & 0xFF);
}

static inline void __not_in_flash_func(mb_bus_put_rdata)(uint8_t data) {
    pio0->txf[MB_SM_READ] = data;
}

// open-drain: assert = drive low, release = Hi-Z
static inline void __not_in_flash_func(mb_bus_irq)(bool assert) {
    gpio_set_dir(PIN_IRQ, assert ? GPIO_OUT : GPIO_IN);
}
