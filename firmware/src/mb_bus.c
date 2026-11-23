/*
 * Derived from Oliver Schmidt's a2pico2 (MIT, Copyright (c) 2022 Oliver Schmidt,
 * https://github.com/oliverschmidt/a2pico).
 */
#include "mb_bus.h"
#include "mb_pins.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "mb_bus.pio.h"

#define N_BUS_SM 4
#define MB_IRQ_READ 5u    // raddr->table-read doorbell, must match IRQ_READ in mb_bus.pio
#define MB_IRQ_WRITE 4u   // addr->write doorbell, must match IRQ_WRITE in mb_bus.pio

// (re)applied by mb_reset_isr on /RES
static struct {
    uint          offset;
    pio_sm_config config;
} bus_sm[N_BUS_SM];

static void (*g_reset_cb)(bool asserted) = NULL;

volatile uint32_t g_phi2_mirror;
// must be <= 0x0FFFFFFF: RP2350 TRANS_COUNT is COUNT[0-27]+MODE[28-31], so 1<<28 sets a mode bit + zero count (freezes the mirror). 1<<27 = ~134M transfers (~130 s).
static const    uint32_t g_phi2_reload = 1u << 27;
static int dma_mirror = -1, dma_restart = -1;

static void mb_phi2_start(void) {
    uint off = pio_add_program(pio1, &mb_phi2_program);
    pio_sm_config c = mb_phi2_program_get_default_config(off);
    mb_phi2_cfg(&c);
    pio_sm_init(pio1, MB_SM_PHI2, off, &c);
    pio_sm_set_enabled(pio1, MB_SM_PHI2, true);

    dma_mirror  = dma_claim_unused_channel(true);
    dma_restart = dma_claim_unused_channel(true);

    dma_channel_config mc = dma_channel_get_default_config(dma_mirror);
    channel_config_set_transfer_data_size(&mc, DMA_SIZE_32);
    channel_config_set_read_increment(&mc, false);
    channel_config_set_write_increment(&mc, false);
    channel_config_set_dreq(&mc, pio_get_dreq(pio1, MB_SM_PHI2, false)); // RX
    channel_config_set_chain_to(&mc, dma_restart);
    dma_channel_configure(dma_mirror, &mc,
        (void *)&g_phi2_mirror, &pio1->rxf[MB_SM_PHI2], g_phi2_reload, false);

    dma_channel_config rc = dma_channel_get_default_config(dma_restart);
    channel_config_set_transfer_data_size(&rc, DMA_SIZE_32);
    channel_config_set_read_increment(&rc, false);
    channel_config_set_write_increment(&rc, false);
    dma_channel_configure(dma_restart, &rc,
        &dma_hw->ch[dma_mirror].al1_transfer_count_trig, &g_phi2_reload, 1, false);

    dma_channel_start(dma_mirror);
}

// 256-aligned so the raddr SM can push base|index as a complete DMA pointer.
__attribute__((aligned(256))) volatile uint8_t mb_rtable[256];

static int dma_raddr = -1, dma_rbyte = -1;

// raddr SM's Y = table base >> 8. pio_sm_init clears it, so this follows every (re)init.
static void raddr_load_base(void) {
    pio_sm_put(pio0, MB_SM_RADDR, (uint32_t)(uintptr_t)mb_rtable >> 8);
    pio_sm_exec(pio0, MB_SM_RADDR, pio_encode_pull(false, true));
    pio_sm_exec(pio0, MB_SM_RADDR, pio_encode_mov(pio_y, pio_osr));
}

static void table_read_dma_start(void) {
    dma_raddr = dma_claim_unused_channel(true);
    dma_rbyte = dma_claim_unused_channel(true);

    dma_channel_config b = dma_channel_get_default_config(dma_rbyte);
    channel_config_set_transfer_data_size(&b, DMA_SIZE_8);
    channel_config_set_read_increment(&b, false);
    channel_config_set_write_increment(&b, false);
    channel_config_set_high_priority(&b, true);
    dma_channel_configure(dma_rbyte, &b, &pio0->txf[MB_SM_READ], mb_rtable, 1, false);

    dma_channel_config a = dma_channel_get_default_config(dma_raddr);
    channel_config_set_transfer_data_size(&a, DMA_SIZE_32);
    channel_config_set_read_increment(&a, false);
    channel_config_set_write_increment(&a, false);
    channel_config_set_dreq(&a, pio_get_dreq(pio0, MB_SM_RADDR, false));
    channel_config_set_high_priority(&a, true);
    dma_channel_configure(dma_raddr, &a, &dma_hw->ch[dma_rbyte].al3_read_addr_trig,
        &pio0->rxf[MB_SM_RADDR], dma_encode_endless_transfer_count(), true);
}

static uint          t1_off, t2_off;
static pio_sm_config t1_cfg, t2_cfg;

static void counter_dma_start(PIO pio, uint sm, volatile uint8_t *dst) {
    int ch = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
    dma_channel_configure(ch, &c, dst, &pio->rxf[sm], dma_encode_endless_transfer_count(), true);
}

static uint32_t t1_value(uint16_t n, uint32_t d) {
    uint32_t p = (d - 1u) % ((uint32_t)n + 2u);
    return p <= n ? (uint32_t)n - p : 0xFFFFFFFFu;
}
static uint32_t t2_value(uint16_t n, uint32_t d) {
    return (uint32_t)n - (d - 1u);
}

// A PHI0 fall reaches g_phi2_mirror about 10 system clocks late (synchronizer, push, DMA).
#define PHI2_MIRROR_LAG_CLOCKS 32

// Repeats until e is unchanged across the start, so a PHI0 fall during it cannot push a value a cycle late.
static uint32_t counter_start(PIO pio, uint sm, uint start, const pio_sm_config *cfg, bool t1,
                              uint16_t n, uint32_t w) {
    uint32_t e = 0;
    for (int tries = 0; tries < 4; tries++) {
        pio_sm_init(pio, sm, start, cfg);
        pio_interrupt_clear(pio, sm);
        e = mb_phi2_ticks();
        uint32_t d = e + 1u - w;
        if (t1) pio_sm_put(pio, sm, n);
        pio_sm_put(pio, sm, t1 ? t1_value(n, d) : t2_value(n, d));
        pio_sm_set_enabled(pio, sm, true);
        busy_wait_at_least_cycles(PHI2_MIRROR_LAG_CLOCKS);
        if (mb_phi2_ticks() == e) break;
    }
    return e;
}

bool __not_in_flash_func(mb_t1_arm)(int chan, uint32_t w, uint16_t n) {
    uint32_t e = counter_start(pio1, (uint)chan, t1_off + mb_t1_offset_start, &t1_cfg, true, n, w);
    return e - w >= (uint32_t)n + 2u;
}

void __not_in_flash_func(mb_t1_latch)(int chan, uint16_t n) {
    if (!pio_sm_is_tx_fifo_full(pio1, (uint)chan)) pio_sm_put(pio1, (uint)chan, n);
}

bool __not_in_flash_func(mb_t2_arm)(int chan, uint32_t w, uint16_t n) {
    uint32_t e = counter_start(pio2, (uint)chan, t2_off + mb_t2_offset_start, &t2_cfg, false, n, w);
    return e - w >= (uint32_t)n + 2u;
}

void __not_in_flash_func(mb_t2_hold)(int chan, uint16_t n) {
    pio_sm_set_enabled(pio2, (uint)chan, false);
    pio_sm_clear_fifos(pio2, (uint)chan);
    uint8_t i = (uint8_t)(chan ? 0x88 : 0x08);
    mb_rtable[i]     = (uint8_t)n;
    mb_rtable[i + 1] = (uint8_t)(n >> 8);
}

uint32_t __not_in_flash_func(mb_timer_ticks)(void) {
    uint32_t t = (pio1->irq & 3u) | ((pio2->irq & 3u) << 2);
    for (uint ch = 0; ch < 2; ch++) {
        if (t & MB_TICK_T1(ch)) pio_interrupt_clear(pio1, ch);
        if (t & MB_TICK_T2(ch)) pio_interrupt_clear(pio2, ch);
    }
    return t;
}

static void counters_init(void) {
    t1_off = pio_add_program(pio1, &mb_t1_program);
    t1_cfg = mb_t1_program_get_default_config(t1_off);
    mb_t1_cfg(&t1_cfg);
    pio_claim_sm_mask(pio2, 0b0011);
    t2_off = pio_add_program(pio2, &mb_t2_program);
    t2_cfg = mb_t2_program_get_default_config(t2_off);
    mb_t2_cfg(&t2_cfg);
    for (int ch = 0; ch < 2; ch++) {
        uint8_t i = (uint8_t)(ch ? 0x80 : 0x00);
        counter_dma_start(pio1, (uint)ch, &mb_rtable[i | 0x04]);
        counter_dma_start(pio2, (uint)ch, &mb_rtable[i | 0x08]);
    }
}

// mirrors a2pico2's a2_reset
static void __not_in_flash_func(mb_reset_isr)(uint gpio, uint32_t events) {
    (void)gpio;
    if (events & GPIO_IRQ_EDGE_FALL) {
        // reset asserted: get off the bus mid-cycle, then zero the model
        pio_set_sm_mask_enabled(pio0, (1u << N_BUS_SM) - 1u, false);
        pio_sm_set_pindirs_with_mask(pio0, 0, 0, 0xFFu << PIN_DATA_BASE);  // D0-7 -> input
        if (g_reset_cb) g_reset_cb(true);
    } else if (events & GPIO_IRQ_EDGE_RISE) {
        // reset released: fresh SM state, rejoin the bus
        for (uint sm = 0; sm < N_BUS_SM; sm++) {
            pio_sm_init(pio0, sm, bus_sm[sm].offset, &bus_sm[sm].config);
        }
        pio_interrupt_clear(pio0, MB_IRQ_WRITE);   // pio_sm_init does not clear PIO IRQ flags
        pio_interrupt_clear(pio0, MB_IRQ_READ);
        raddr_load_base();
        pio_set_sm_mask_enabled(pio0, (1u << N_BUS_SM) - 1u, true);
        if (g_reset_cb) g_reset_cb(false);
    }
}

void mb_bus_init(void) {
    // /IRQ open-drain: value 0, released (input) until asserted
    gpio_init(PIN_IRQ);
    gpio_disable_pulls(PIN_IRQ);
    gpio_put(PIN_IRQ, 0);
    gpio_set_dir(PIN_IRQ, GPIO_IN);
    gpio_set_drive_strength(PIN_IRQ, GPIO_DRIVE_STRENGTH_12MA);

    gpio_init(PIN_RES);
    gpio_disable_pulls(PIN_RES);

    for (uint g = PIN_DATA_BASE; g <= PIN_PHI0; g++) {
        pio_gpio_init(pio0, g);
        gpio_disable_pulls(g);
        // /IOSEL + PHI0 are edge-gated `wait gpio` strobes: keep Schmitt hysteresis on for clean edges. Level-sampled lines get it off.
        if (g != PIN_IOSEL && g != PIN_PHI0) {
            gpio_set_input_hysteresis_enabled(g, false);
        }
    }
    // D0-7 drive 4 mA + fast slew: a read must be valid within the 6502's setup window. 2 mA was marginal, 4 mA has margin, still 3x below a2pico's 12 mA.
    for (uint g = PIN_DATA_BASE; g < PIN_DATA_BASE + N_DATA; g++) {
        gpio_set_drive_strength(g, GPIO_DRIVE_STRENGTH_4MA);
        gpio_set_slew_rate(g, GPIO_SLEW_RATE_FAST);
    }

    // SMs stay idle until reset_start
    pio_claim_sm_mask(pio0, (1u << N_BUS_SM) - 1u);

    bus_sm[MB_SM_ADDR].offset = pio_add_program(pio0, &mb_addr_program);
    bus_sm[MB_SM_ADDR].config = mb_addr_program_get_default_config(bus_sm[MB_SM_ADDR].offset);
    mb_addr_cfg(&bus_sm[MB_SM_ADDR].config);

    bus_sm[MB_SM_WRITE].offset = pio_add_program(pio0, &mb_write_program);
    bus_sm[MB_SM_WRITE].config = mb_write_program_get_default_config(bus_sm[MB_SM_WRITE].offset);
    mb_write_cfg(&bus_sm[MB_SM_WRITE].config);

    bus_sm[MB_SM_READ].offset = pio_add_program(pio0, &mb_read_table_program);
    bus_sm[MB_SM_READ].config = mb_read_table_program_get_default_config(bus_sm[MB_SM_READ].offset);
    mb_read_table_cfg(&bus_sm[MB_SM_READ].config);
    bus_sm[MB_SM_RADDR].offset = pio_add_program(pio0, &mb_raddr_program);
    bus_sm[MB_SM_RADDR].config = mb_raddr_program_get_default_config(bus_sm[MB_SM_RADDR].offset);
    mb_raddr_cfg(&bus_sm[MB_SM_RADDR].config);

    // pio1: Timer1 counters (SM0/1) + free-running PHI2 edge counter (SM2)
    pio_claim_sm_mask(pio1, 0b0111);
    counters_init();
    table_read_dma_start();
    mb_phi2_start();
}

void mb_bus_reset_handler(void (*handler)(bool asserted)) {
    g_reset_cb = handler;
}

void mb_bus_reset_start(void) {
    gpio_set_irq_enabled_with_callback(PIN_RES,
        GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, mb_reset_isr);
    // prime by hand: rejoin the bus if /RES is already high, else stay off it
    mb_reset_isr(PIN_RES, gpio_get(PIN_RES) ? GPIO_IRQ_EDGE_RISE : GPIO_IRQ_EDGE_FALL);
}
