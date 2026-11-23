/* RP2350 PIO interpreter (PIO version 1) for the host tests. Not modeled: side-set, clock dividers and FIFO joining. */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define PIO_SIM_SMS   4
#define PIO_SIM_FIFO  4

typedef struct {
    uint8_t  wrap_target, wrap;
    uint8_t  in_base, in_count, out_base, out_count, set_base, set_count, jmp_pin;
    bool     in_shift_right, autopush, out_shift_right, autopull;
    uint8_t  push_thresh, pull_thresh;
    bool     status_rx;
    uint8_t  status_n;
    bool     enabled;
    uint8_t  pc;
    uint32_t x, y, isr, osr;
    uint8_t  isr_count, osr_count;
    uint32_t delay;
    uint32_t tx[PIO_SIM_FIFO], rx[PIO_SIM_FIFO];
    uint8_t  tx_n, rx_n;
    uint32_t pushes, stalls;
} pio_sim_sm_t;

typedef struct {
    uint16_t     mem[32];
    pio_sim_sm_t sm[PIO_SIM_SMS];
    uint8_t      irq;
    uint8_t      irq_raised;
} pio_sim_t;

typedef struct {
    uint32_t in, sync1, sync2;
    uint32_t out_value, out_enable;
} pio_sim_gpio_t;

void     pio_sim_reset(pio_sim_t *p);
void     pio_sim_load(pio_sim_t *p, uint8_t offset, const uint16_t *code, unsigned len);
void     pio_sim_sm_restart(pio_sim_t *p, unsigned sm, uint8_t pc);
bool     pio_sim_tx_put(pio_sim_t *p, unsigned sm, uint32_t v);
bool     pio_sim_rx_get(pio_sim_t *p, unsigned sm, uint32_t *v);
void     pio_sim_exec(pio_sim_t *p, pio_sim_gpio_t *g, unsigned sm, uint16_t instr);
void     pio_sim_step(pio_sim_t *p, pio_sim_gpio_t *g);
void     pio_sim_gpio_clock(pio_sim_gpio_t *g);
uint32_t pio_sim_gpio_levels(const pio_sim_gpio_t *g);
