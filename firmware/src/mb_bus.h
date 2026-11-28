#pragma once

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "mb_pins.h"

#define MB_SM_ADDR   0
#define MB_SM_WRITE  1
#define MB_SM_READ   2
#define MB_SM_RADDR  3
// pio1: SM0/1 count Timer1 of VIA1/VIA2, SM2 counts PHI2 cycles. pio2: SM0/1 count Timer2.
#define MB_SM_PHI2   2

#define MB_ACC_ADDR(w)    ((uint8_t)((w) & 0xFF))   // A0..A7
#define MB_ACC_IOSEL(w)   (((w) >> 8) & 1u)         // bit8: 0 = asserted
#define MB_ACC_IS_READ(w) (((w) >> 9) & 1u)         // bit9: 1 = read, 0 = write

void mb_bus_init(void);

extern volatile uint32_t g_phi2_mirror;
static inline uint32_t mb_phi2_ticks(void) {
    return 0u - g_phi2_mirror;
}


// Cycle count of a write access, taken BEFORE this cycle's PHI0 fall is counted.
// Reading the mirror at a fixed latency after /IOSEL races the PHI0 fall (core0 sometimes sees the access
// just before it and sometimes just after), which made T2 read one cycle high (MB-audit 6522 test).
// PHI0 high: the fall has not happened yet and the mirror update trails it by ~50 ns, so read now.
// PHI0 low: the fall already happened; let the mirror catch up, then remove that edge from the count.
#ifndef MB_HOST_SIM
#include "hardware/gpio.h"
static inline uint32_t mb_access_ticks(void) {
    if (gpio_get(PIN_PHI0)) return mb_phi2_ticks();
    busy_wait_at_least_cycles(32);
    return mb_phi2_ticks() - 1u;
}
#else
static inline uint32_t mb_access_ticks(void) { return mb_phi2_ticks(); }
#endif

extern volatile uint8_t mb_rtable[256];

// *_arm loads n as if written on PHI2 cycle w, and returns true if its $FFFF cycle already passed.
bool mb_t1_arm(int chan, uint32_t w, uint16_t n);
void mb_t1_latch(int chan, uint16_t n);
bool mb_t2_arm(int chan, uint32_t w, uint16_t n);
void mb_t2_hold(int chan, uint16_t n);

#define MB_TICK_T1(chan) (1u << (chan))
#define MB_TICK_T2(chan) (4u << (chan))
uint32_t mb_timer_ticks(void);

// reset_start() must run after init/via_init/reset_handler, so the bus never wakes with stale VIA state
void mb_bus_reset_handler(void (*handler)(bool asserted));
void mb_bus_reset_start(void);

static inline bool __not_in_flash_func(mb_bus_access_pending)(void) {
    return !(pio0->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + MB_SM_ADDR)));
}

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

// open-drain: assert = drive low, release = Hi-Z
static inline void __not_in_flash_func(mb_bus_irq)(bool assert) {
    gpio_set_dir(PIN_IRQ, assert ? GPIO_OUT : GPIO_IN);
}
