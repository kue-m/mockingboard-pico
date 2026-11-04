/*
 * Derived from Oliver Schmidt's a2pico2 (MIT, Copyright (c) 2022 Oliver Schmidt,
 * https://github.com/oliverschmidt/a2pico).
 */
#include "mb_bus.h"
#include "mb_pins.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/dma.h"
#include "pico/time.h"
#include "mb_bus.pio.h"

#define N_BUS_SM 3
#define MB_IRQ_WRITE 4u   // addr->write doorbell, must match IRQ_WRITE in mb_bus.pio

static uint          sync_off;
static pio_sm_config sync_cfg;
static void        (*g_tick_cb)(int chan) = NULL;

// (re)applied by mb_reset_isr on /RES
static struct {
    uint          offset;
    pio_sm_config config;
} bus_sm[N_BUS_SM];

static void (*g_reset_cb)(bool asserted) = NULL;

// PHI2 cycle-count time base: pio1 SM2 counts PHI0 edges (a scaled SysTick's +/-1 jitter breaks
// timer-calibration detection). A DMA mirrors RX into a RAM word.
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

// mb_phi2_ticks() is inlined in mb_bus.h (critical path). Y counts down, negate for an up-counter.

// one hardware alarm serves both VIA Timer2s. The VIA model masks it (via_lock) while editing IFR.
#define T2_DUE_SLACK_US 2                 // window for near-coincident deadlines
static void   (*g_t2_cb)(int chan) = NULL;
static int      t2_alarm = -1;
static uint64_t t2_deadline_us[2];        // 0 = inactive

static void __not_in_flash_func(t2_reschedule)(void) {
    uint64_t best = 0;
    for (int c = 0; c < 2; c++)
        if (t2_deadline_us[c] && (best == 0 || t2_deadline_us[c] < best)) best = t2_deadline_us[c];
    if (best) hardware_alarm_set_target(t2_alarm, from_us_since_boot(best));
    else      hardware_alarm_cancel(t2_alarm);
}

static void __not_in_flash_func(t2_alarm_isr)(uint num) {
    (void)num;
    uint64_t now = to_us_since_boot(get_absolute_time());
    for (int c = 0; c < 2; c++) {
        if (t2_deadline_us[c] && t2_deadline_us[c] <= now + T2_DUE_SLACK_US) {
            t2_deadline_us[c] = 0;
            if (g_t2_cb) g_t2_cb(c);
        }
    }
    t2_reschedule();
}

void mb_alarm_init(void (*t2_cb)(int chan)) {
    g_t2_cb = t2_cb;
    t2_deadline_us[0] = t2_deadline_us[1] = 0;
    t2_alarm = hardware_alarm_claim_unused(true);
    hardware_alarm_set_callback(t2_alarm, t2_alarm_isr);
}

void __not_in_flash_func(mb_alarm_arm)(int chan, uint32_t phi2_cycles) {
    if (phi2_cycles == 0) {               // cancel this channel
        t2_deadline_us[chan] = 0;
        t2_reschedule();
        return;
    }
    // off the bus-read hot path (T2C-H write only), so a 64-bit divide is fine
    uint64_t us = ((uint64_t)phi2_cycles * 1000000ull + 510242ull) / 1020484ull;
    t2_deadline_us[chan] = to_us_since_boot(get_absolute_time()) + (us ? us : 1);
    t2_reschedule();
}

int mb_alarm_irq(void) {
    return (t2_alarm >= 0) ? (int)hardware_alarm_get_irq_num((uint)t2_alarm) : -1;
}

// self-rearming poll -> mb_via_ar_poll. ~250 us << phoneme duration, so CA1 latency is inaudible.
#define AR_POLL_US 250
static void (*g_ar_cb)(void) = NULL;
static int      ar_alarm = -1;

static void __not_in_flash_func(ar_alarm_isr)(uint num) {
    if (g_ar_cb) g_ar_cb();
    hardware_alarm_set_target((uint)num, make_timeout_time_us(AR_POLL_US));
}

int mb_ar_tick_init(void (*cb)(void)) {
    g_ar_cb = cb;
    ar_alarm = hardware_alarm_claim_unused(true);
    hardware_alarm_set_callback((uint)ar_alarm, ar_alarm_isr);
    hardware_alarm_set_target((uint)ar_alarm, make_timeout_time_us(AR_POLL_US));
    return (int)hardware_alarm_get_irq_num((uint)ar_alarm);
}

static void __not_in_flash_func(mb_sync_isr)(void) {
    if (pio1->irq & (1u << 0)) { pio_interrupt_clear(pio1, 0); if (g_tick_cb) g_tick_cb(0); }
    if (pio1->irq & (1u << 1)) { pio_interrupt_clear(pio1, 1); if (g_tick_cb) g_tick_cb(1); }
}

// mirrors a2pico2's a2_reset
static void __not_in_flash_func(mb_reset_isr)(uint gpio, uint32_t events) {
    (void)gpio;
    if (events & GPIO_IRQ_EDGE_FALL) {
        // reset asserted: get off the bus mid-cycle, then zero the model
        pio_set_sm_mask_enabled(pio0, 0b0111, false);
        pio_sm_set_pindirs_with_mask(pio0, 0, 0, 0xFFu << PIN_DATA_BASE);  // D0-7 -> input
        if (g_reset_cb) g_reset_cb(true);
    } else if (events & GPIO_IRQ_EDGE_RISE) {
        // reset released: fresh SM state, rejoin the bus
        for (uint sm = 0; sm < N_BUS_SM; sm++) {
            pio_sm_init(pio0, sm, bus_sm[sm].offset, &bus_sm[sm].config);
        }
#ifdef MB_FAILSAFE_READ
        pio_interrupt_clear(pio0, MB_IRQ_WRITE);   // pio_sm_init does not clear PIO IRQ flags
#endif
        pio_set_sm_mask_enabled(pio0, 0b0111, true);
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
    pio_claim_sm_mask(pio0, 0b0111);

    bus_sm[MB_SM_ADDR].offset = pio_add_program(pio0, &mb_addr_program);
    bus_sm[MB_SM_ADDR].config = mb_addr_program_get_default_config(bus_sm[MB_SM_ADDR].offset);
    mb_addr_cfg(&bus_sm[MB_SM_ADDR].config);

    bus_sm[MB_SM_WRITE].offset = pio_add_program(pio0, &mb_write_program);
    bus_sm[MB_SM_WRITE].config = mb_write_program_get_default_config(bus_sm[MB_SM_WRITE].offset);
    mb_write_cfg(&bus_sm[MB_SM_WRITE].config);

#ifdef MB_FAILSAFE_READ
    bus_sm[MB_SM_READ].offset = pio_add_program(pio0, &mb_read_failsafe_program);
    bus_sm[MB_SM_READ].config = mb_read_failsafe_program_get_default_config(bus_sm[MB_SM_READ].offset);
    mb_read_failsafe_cfg(&bus_sm[MB_SM_READ].config);
#else
    bus_sm[MB_SM_READ].offset = pio_add_program(pio0, &mb_read_program);
    bus_sm[MB_SM_READ].config = mb_read_program_get_default_config(bus_sm[MB_SM_READ].offset);
    mb_read_cfg(&bus_sm[MB_SM_READ].config);
#endif

    // pio1: Timer1 counters (SM0/1) + free-running PHI2 edge counter (SM2)
    pio_claim_sm_mask(pio1, 0b0111);
    sync_off = pio_add_program(pio1, &mb_sync_program);
    sync_cfg = mb_sync_program_get_default_config(sync_off);
    mb_phi2_start();
}

void mb_sync_init(void (*tick_cb)(int chan)) {
    g_tick_cb = tick_cb;
    pio_set_irq0_source_enabled(pio1, pis_interrupt0, true);   // flag 0 = VIA1
    pio_set_irq0_source_enabled(pio1, pis_interrupt1, true);   // flag 1 = VIA2
    irq_set_exclusive_handler(PIO1_IRQ_0, mb_sync_isr);
    irq_set_enabled(PIO1_IRQ_0, true);
}

void __not_in_flash_func(mb_sync_restart)(int chan, uint32_t phi0_cycles) {
    uint sm = chan ? MB_SM_SYNC1 : MB_SM_SYNC0;
    pio_sm_set_enabled(pio1, sm, false);
    if (phi0_cycles == 0) {
        return;   // 0 stops this channel's Timer1
    }
    pio_sm_clear_fifos(pio1, sm);
    pio_sm_init(pio1, sm, sync_off, &sync_cfg);
    // mb_sync runs X+1 PHI0 cycles/tick. A 6522 free-run Timer1 fires every N+2 PHI2, so X = N+1.
    pio1->txf[sm] = phi0_cycles + 1;
    pio_sm_set_enabled(pio1, sm, true);
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
