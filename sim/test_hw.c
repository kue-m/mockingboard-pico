#include <setjmp.h>
#include <string.h>
#include "test_checks.h"
#include "sdk_fake.h"
#include "mb_bus.h"
#include "mb_audio.h"
#include "mb_pins.h"
#include "mb_speech.h"
#include "pio_sim.h"

static int g_reset_calls, g_last_reset;
static void reset_cb(bool a)       { g_reset_calls++; g_last_reset = a; }

static void reset_world(void) {
    fake_sdk_reset();
    g_reset_calls = 0; g_last_reset = -1;
}

static void test_pins(void) {
    reset_world();
    mb_bus_init();

    CHECK(fake.gpio[PIN_IRQ].init && fake.gpio[PIN_IRQ].pulls_off && !fake.gpio[PIN_IRQ].level
          && !fake.gpio[PIN_IRQ].dir_out, "/IRQ starts released: input, output latch 0, no pulls");
    CHECK(fake.gpio[PIN_IRQ].drive == GPIO_DRIVE_STRENGTH_12MA, "/IRQ pulls low with 12 mA");
    CHECK(fake.gpio[PIN_RES].init && fake.gpio[PIN_RES].pulls_off, "/RES is a plain input without pulls");

    bool bus_pio = true, hyst_ok = true, pulls = true;
    for (uint g = PIN_DATA_BASE; g <= PIN_PHI0; g++) {
        bus_pio &= fake.gpio[g].pio;
        pulls &= fake.gpio[g].pulls_off;
        bool want_hyst = (g == PIN_IOSEL || g == PIN_PHI0);
        hyst_ok &= (fake.gpio[g].hysteresis == want_hyst);
    }
    CHECK(bus_pio, "every bus line GP0-GP18 is handed to PIO0");
    CHECK(pulls, "no bus line has a pull resistor");
    CHECK(hyst_ok, "Schmitt hysteresis stays on only for the edge-waited /IOSEL and PHI0");

    bool data_drive = true;
    for (uint g = PIN_DATA_BASE; g < PIN_DATA_BASE + N_DATA; g++)
        data_drive &= (fake.gpio[g].drive == GPIO_DRIVE_STRENGTH_4MA && fake.gpio[g].slew == GPIO_SLEW_RATE_FAST);
    CHECK(data_drive, "D0-D7 drive 4 mA with fast slew");

    CHECK(fake.sm_claimed[0] == 0xF && fake.sm_enabled[0] == 0, "PIO0's four bus SMs are claimed but stay idle until reset_start");
    CHECK(fake.sm_claimed[1] == 0x7 && fake.sm_claimed[2] == 0x3, "PIO1 claims the Timer1 counters and PHI2, PIO2 the Timer2 counters");
    CHECK((fake.sm_enabled[1] & (1u << MB_SM_PHI2)) != 0, "the PHI2 counter runs from init");
}

static void test_phi2_mirror(void) {
    reset_world();
    mb_bus_init();
    int mirror = -1, restart = -1;
    for (int c = 0; c < FAKE_DMA_CH; c++) {
        if (fake.dma_write[c] == (volatile void *)&g_phi2_mirror) mirror = c;
    }
    CHECK(mirror >= 0, "a DMA channel writes the PHI2 count into g_phi2_mirror");
    if (mirror < 0) return;
    restart = (int)fake.dma_cfg[mirror].chain_to;
    CHECK(fake.dma_read[mirror] == (const volatile void *)&pio1->rxf[MB_SM_PHI2] && !fake.dma_cfg[mirror].read_inc
          && !fake.dma_cfg[mirror].write_inc && fake.dma_cfg[mirror].size == DMA_SIZE_32,
          "the mirror copies 32-bit words from the PHI2 SM's RX FIFO to one fixed address");
    CHECK(fake.dma_cfg[mirror].dreq == pio_get_dreq(pio1, MB_SM_PHI2, false), "the mirror is paced by the PHI2 SM's RX DREQ");
    CHECK(fake.dma_started[mirror], "the mirror starts at init");
    CHECK(fake.dma_count[mirror] > 0 && fake.dma_count[mirror] <= 0x0FFFFFFFu,
          "the mirror's transfer count fits RP2350 TRANS_COUNT bits 0-27 (bits 28-31 select the mode)");
    CHECK(restart != mirror && fake.dma_write[restart] == (volatile void *)&dma_hw->ch[mirror].al1_transfer_count_trig
          && fake.dma_count[restart] == 1 && *(const uint32_t *)fake.dma_read[restart] == fake.dma_count[mirror],
          "when the mirror runs out, a chained channel rewrites its count and retriggers it");

    g_phi2_mirror = 0u - 1000u;
    uint32_t a = mb_phi2_ticks();
    g_phi2_mirror -= 37u;
    CHECK(mb_phi2_ticks() - a == 37u, "mb_phi2_ticks counts up one per PHI2 cycle");
    g_phi2_mirror = 2u;
    a = mb_phi2_ticks();
    g_phi2_mirror -= 5u;
    CHECK(mb_phi2_ticks() - a == 5u, "mb_phi2_ticks stays wrap-safe across the counter wrap");
}

static void test_timer_ticks(void) {
    reset_world();
    mb_bus_init();
    mb_bus_reset_handler(reset_cb);
    fake.gpio[PIN_RES].level = true;
    mb_bus_reset_start();
    bool quiet = true;
    for (int i = 0; i < FAKE_IRQS; i++) quiet &= !fake.irq_enabled[i] && fake.irq_handler[i] == NULL;
    for (int i = 0; i < 3; i++) quiet &= fake.irq0_sources[i] == 0;
    CHECK(quiet && fake.gpio_cb != NULL, "the bus raises no interrupt but /RES: the timer timeouts are polled");

    pio1->irq = 0x3; pio2->irq = 0x2;
    CHECK(mb_timer_ticks() == (MB_TICK_T1(0) | MB_TICK_T1(1) | MB_TICK_T2(1)) && pio1->irq == 0 && pio2->irq == 0,
          "mb_timer_ticks reports every pending timeout and clears its flag");
    CHECK(mb_timer_ticks() == 0, "a timeout is reported once");
    pio1->irq = 0x4 | 0x1; pio2->irq = 0x1;
    CHECK(mb_timer_ticks() == (MB_TICK_T1(0) | MB_TICK_T2(0)) && pio1->irq == 0x4,
          "only the counter SMs' flags are timeouts; other flags stay set");
    pio1->irq = 0;
}

static void test_reset(void) {
    reset_world();
    mb_bus_init();
    mb_bus_reset_handler(reset_cb);

    fake.gpio[PIN_RES].level = true;
    mb_bus_reset_start();
    CHECK(fake.gpio_irq_enabled[PIN_RES] && fake.gpio_irq_events[PIN_RES] == (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE)
          && fake.gpio_cb != NULL, "/RES interrupts on both edges");
    CHECK((fake.sm_enabled[0] & 0xF) == 0xF && g_reset_calls == 1 && g_last_reset == 0,
          "starting with /RES high joins the bus at once");
    CHECK(fake.sm_init_count[0][MB_SM_ADDR] == 1 && fake.sm_init_count[0][MB_SM_WRITE] == 1
          && fake.sm_init_count[0][MB_SM_READ] == 1, "joining the bus re-inits the bus SMs");
    CHECK(fake.sm_init_count[0][MB_SM_RADDR] == 1 && fake.tx_log[0][MB_SM_RADDR][0] == (uint32_t)(uintptr_t)mb_rtable >> 8
          && fake.execs[0][MB_SM_RADDR] == 2 && (fake.irq_cleared[0] & (1u << 5)),
          "joining re-inits the raddr SM, reloads the table base into Y and clears the read doorbell");
    CHECK((fake.irq_cleared[0] & (1u << 4)) != 0, "joining clears the addr->write doorbell flag (pio_sm_init leaves PIO IRQ flags set)");

    fake.pindirs[0] = 0xFF;
    fake.gpio_cb(PIN_RES, GPIO_IRQ_EDGE_FALL);
    CHECK((fake.sm_enabled[0] & 0xF) == 0, "/RES falling stops the bus SMs mid-cycle");
    CHECK((fake.pindirs_mask[0] & 0xFF) == 0xFF && (fake.pindirs[0] & 0xFF) == 0, "/RES falling floats D0-D7");
    CHECK(g_reset_calls == 2 && g_last_reset == 1, "/RES falling resets the model");

    fake.gpio_cb(PIN_RES, GPIO_IRQ_EDGE_RISE);
    CHECK((fake.sm_enabled[0] & 0xF) == 0xF && g_last_reset == 0 && fake.sm_init_count[0][MB_SM_READ] == 2,
          "/RES rising rejoins the bus with fresh SMs");

    reset_world();
    mb_bus_init();
    mb_bus_reset_handler(reset_cb);
    fake.gpio[PIN_RES].level = false;
    mb_bus_reset_start();
    CHECK((fake.sm_enabled[0] & 0xF) == 0 && g_last_reset == 1, "starting with /RES low stays off the bus");
}

static int find_dma(volatile void *write) {
    for (int c = 0; c < FAKE_DMA_CH; c++) if (fake.dma_claimed[c] && fake.dma_write[c] == write) return c;
    return -1;
}

static void test_table_wiring(void) {
    reset_world();
    mb_bus_init();
    CHECK(((uintptr_t)mb_rtable & 0xFF) == 0, "the response table is 256-byte aligned (raddr pushes base|index)");

    int b = find_dma(&pio0->txf[MB_SM_READ]);
    int a = b < 0 ? -1 : find_dma(&dma_hw->ch[b].al3_read_addr_trig);
    CHECK(a >= 0 && b >= 0, "two DMA channels form the read path");
    if (a >= 0 && b >= 0) {
        CHECK(fake.dma_read[a] == (const volatile void *)&pio0->rxf[MB_SM_RADDR] && fake.dma_cfg[a].size == DMA_SIZE_32
              && fake.dma_cfg[a].dreq == pio_get_dreq(pio0, MB_SM_RADDR, false) && fake.dma_count[a] == 0xFFFFFFFFu
              && fake.dma_started[a] && fake.dma_cfg[a].high_priority && !fake.dma_cfg[a].read_inc && !fake.dma_cfg[a].write_inc,
              "channel A takes each raddr pointer for ever (endless), at high priority");
        CHECK(fake.dma_cfg[b].size == DMA_SIZE_8 && fake.dma_count[b] == 1 && !fake.dma_started[b]
              && fake.dma_cfg[b].dreq == FAKE_DREQ_FORCE && fake.dma_cfg[b].high_priority,
              "channel B moves one byte per trigger, at high priority, only when A triggers it");
    }

    struct { PIO pio; uint sm; uint8_t idx; } c[] = { { pio1, 0, 0x04 }, { pio1, 1, 0x84 }, { pio2, 0, 0x08 }, { pio2, 1, 0x88 } };
    bool counters = true;
    for (int i = 0; i < 4; i++) {
        int ch = find_dma(&mb_rtable[c[i].idx]);
        counters &= ch >= 0 && fake.dma_read[ch] == (const volatile void *)&c[i].pio->rxf[c[i].sm]
                 && fake.dma_cfg[ch].size == DMA_SIZE_16 && fake.dma_count[ch] == 0xFFFFFFFFu && fake.dma_started[ch]
                 && fake.dma_cfg[ch].dreq == pio_get_dreq(c[i].pio, c[i].sm, false);
    }
    CHECK(counters, "four endless 16-bit DMA channels store T1C and T2C of both VIAs");

    g_phi2_mirror = 0u - 1000u;
    bool passed = mb_t1_arm(1, 998, 5);
    CHECK(fake.sm_init_count[1][1] == 1 && (fake.sm_enabled[1] & 2) && fake.tx_puts[1][1] == 2
          && fake.tx_log[1][1][0] == 5 && fake.tx_log[1][1][1] == 5 - 2 && !passed,
          "a Timer1 load written 2 cycles ago starts its SM at n-2 for the next cycle");
    passed = mb_t1_arm(0, 990, 5);
    CHECK(fake.tx_log[1][0][1] == 5 - ((11 - 1) % 7) && passed, "a load whose timeout already passed reports it");
    mb_t1_arm(0, 999, 0);
    CHECK(fake.tx_log[1][0][3] == 0xFFFFFFFFu, "the $FFFF cycle starts the SM at its $FFFF entry");
    mb_t1_latch(0, 0x4321);
    CHECK(fake.tx_log[1][0][4] == 0x4321, "a latch change is posted to the Timer1 SM");

    passed = mb_t2_arm(1, 999, 0x10);
    CHECK(fake.tx_puts[2][1] == 1 && fake.tx_log[2][1][0] == 0x10 - 1 && (fake.sm_enabled[2] & 2) && !passed,
          "a Timer2 load starts its SM with the next cycle's value");
    mb_t2_arm(1, 900, 0x10);
    CHECK(fake.tx_log[2][1][1] == (uint32_t)0x10 - 100u, "a late Timer2 load keeps counting below 0, never reaching 0 again");
    mb_t2_hold(1, 0xBEEF);
    CHECK(!(fake.sm_enabled[2] & 2) && mb_rtable[0x88] == 0xEF && mb_rtable[0x89] == 0xBE,
          "holding Timer2 stops its SM and shows the value in the table");
}

void test_bus(void) {
    test_pins();
    test_phi2_mirror();
    test_table_wiring();
    test_timer_ticks();
    test_reset();
}

/* audioRun never returns. The fake speech_drain runs each scenario step and longjmps out. */

#define SAMP_BUF 256
#define PWM_MAX  1023

static int     g_sp_init_clock, g_sp_init_rate;
static int32_t g_sp_sample[2];
static int     g_sp_live[2];
void    speech_init(int clockHz, int sampleRate) { g_sp_init_clock = clockHz; g_sp_init_rate = sampleRate; }
int32_t speech_sample(int chip)                  { return g_sp_sample[chip & 1]; }
int     speech_live(int chip)                    { return g_sp_live[chip & 1]; }

static jmp_buf g_exit;
static void  (*g_step)(int pass);
static int     g_pass;
void speech_drain(void) { g_step(g_pass++); }

static void run_loop(void (*step)(int pass)) {
    g_step = step; g_pass = 0;
    memset(fake.dma_claimed, 0, sizeof fake.dma_claimed);
    if (setjmp(g_exit) == 0) audioRun();
}

static int dma_a = 0, dma_b = 1;
static const uint32_t *buf(int b) { return (const uint32_t *)fake.dma_read[b ? dma_b : dma_a]; }
static void refill(int b) { fake_dma_hw.ints0 = 1u << (b ? dma_b : dma_a); fake.irq_handler[DMA_IRQ_0](); }

static void ay(int ch, uint8_t r, uint8_t v) {
    mb_audio_post(ay_cmd(AY_OP_REG, ch, r));
    mb_audio_post(ay_cmd(AY_OP_DATA, ch, v));
}
static void tone_on(int ch) {
    ay(ch, 0, 100); ay(ch, 1, 0);
    ay(ch, 7, 0x3E);
    ay(ch, 8, 15);
}

typedef struct { uint32_t l_min, l_max, r_min, r_max; } span_t;
static span_t scan(const uint32_t *b) {
    span_t s = { 0xFFFF, 0, 0xFFFF, 0 };
    for (int i = 0; i < SAMP_BUF; i++) {
        uint32_t l = b[i] & 0xFFFF, r = b[i] >> 16;
        if (l < s.l_min) s.l_min = l;
        if (l > s.l_max) s.l_max = l;
        if (r < s.r_min) s.r_min = r;
        if (r > s.r_max) s.r_max = r;
    }
    return s;
}

static span_t g_span;
static uint32_t g_first[4];

static void step_silence(int pass) { (void)pass; g_span = scan(buf(0)); longjmp(g_exit, 1); }

static void step_tone_left(int pass) {
    if (pass == 0) { tone_on(0); return; }
    if (pass == 1) { refill(0); return; }
    g_span = scan(buf(0)); longjmp(g_exit, 1);
}
static void step_tone_right(int pass) {
    if (pass == 0) { mb_audio_post(ay_cmd(AY_OP_RESET, 0, 0)); tone_on(1); return; }
    if (pass == 1) { refill(1); return; }
    g_span = scan(buf(1)); longjmp(g_exit, 1);
}
static void step_queue_full(int pass) {
    if (pass == 0) {
        mb_audio_post(ay_cmd(AY_OP_RESET, 0, 0));
        mb_audio_post(ay_cmd(AY_OP_RESET, 1, 0));
        for (int i = 0; i < 509; i++) mb_audio_post(ay_cmd(AY_OP_REG, 0, 0));
        tone_on(0);
        return;
    }
    if (pass == 1) { refill(0); return; }
    g_span = scan(buf(0)); longjmp(g_exit, 1);
}
static void step_bias(int pass) {
    if (pass == 0) { g_sp_live[0] = 1; refill(0); return; }
    if (pass == 1) {
        g_first[0] = buf(0)[0] & 0xFFFF;
        g_first[1] = buf(0)[SAMP_BUF - 1] & 0xFFFF;
        g_sp_sample[0] = 5000; refill(0); return;
    }
    if (pass == 2) { refill(0); return; }
    if (pass == 3) { g_span = scan(buf(0)); g_sp_sample[0] = -5000; refill(0); return; }
    if (pass == 4) {
        g_first[2] = scan(buf(0)).l_max;
        g_sp_sample[0] = 0; g_sp_live[0] = 0; refill(0); refill(0); return;
    }
    if (pass == 5) { refill(0); return; }
    g_first[3] = buf(0)[SAMP_BUF - 1] & 0xFFFF;
    longjmp(g_exit, 1);
}

void test_audio(void) {
    fake_sdk_reset();
    audioInit(1020484, 50000);
    uint slice = pwm_gpio_to_slice_num(PIN_AUDIO_L);
    CHECK(fake.gpio[PIN_AUDIO_L].function == GPIO_FUNC_PWM && fake.gpio[PIN_AUDIO_R].function == GPIO_FUNC_PWM
          && pwm_gpio_to_slice_num(PIN_AUDIO_R) == slice, "GP26/GP27 are the two channels of one PWM slice");
    CHECK(fake.pwm_wrap == 1024 && fake.pwm_div_int == 1 && fake.pwm_div_frac == 0 && fake.pwm_enabled,
          "PWM runs undivided with a 1024-step wrap (carrier = clk_sys / 1024)");
    CHECK(fake.pwm_level_a == 0 && fake.pwm_level_b == 0, "PWM starts at level 0");
    CHECK(g_sp_init_clock == 1020484 && g_sp_init_rate == 50000, "speech gets the card clock and the sample rate");

    run_loop(step_silence);
    CHECK(fake.dma_timer_claimed == 0 && fake.dma_timer_num == 1 && fake.dma_timer_den == 200000000u / 50000u,
          "the DMA pacing timer divides clk_sys down to exactly 50 kHz");
    bool cfg_ok = true;
    for (int i = 0; i < 2; i++) {
        int ch = i ? dma_b : dma_a, other = i ? dma_a : dma_b;
        cfg_ok &= fake.dma_cfg[ch].size == DMA_SIZE_32 && fake.dma_cfg[ch].read_inc && !fake.dma_cfg[ch].write_inc
               && fake.dma_cfg[ch].dreq == dma_get_timer_dreq(0) && (int)fake.dma_cfg[ch].chain_to == other
               && fake.dma_write[ch] == (volatile void *)&pwm_hw->slice[slice].cc && fake.dma_count[ch] == SAMP_BUF
               && fake.dma_irq0[ch];
    }
    CHECK(cfg_ok, "two 256-sample buffers ping-pong into the PWM compare register, each raising DMA_IRQ_0");
    CHECK(fake.dma_started[dma_a] && !fake.dma_started[dma_b] && fake.irq_enabled[DMA_IRQ_0],
          "buffer A starts the chain");
    CHECK(g_span.l_max == 0 && g_span.r_max == 0, "silent PSGs and idle speech output level 0 on both channels");

    run_loop(step_tone_left);
    CHECK(g_span.l_max > g_span.l_min && g_span.r_max == 0, "PSG0 (VIA1) plays on the left channel only");
    run_loop(step_tone_right);
    CHECK(g_span.r_max > g_span.r_min && g_span.l_max == 0, "PSG1 (VIA2) plays on the right channel only, after a PSG0 reset");

    run_loop(step_queue_full);
    CHECK(g_span.l_max == 0, "a full command queue drops new commands instead of overwriting queued ones");

    run_loop(step_bias);
    CHECK(g_first[0] == 2 && g_first[1] == PWM_MAX / 2 + 1, "a live speech channel slews its mid-rail bias up 2 steps per sample to 512");
    CHECK(g_span.r_max == 0, "the bias belongs to the live channel only");
    CHECK(g_span.l_max == PWM_MAX, "an over-range speech sample clamps to the PWM maximum");
    CHECK(g_first[2] == 0, "an under-range sample clamps to 0");
    CHECK(g_first[3] == 0, "an idle speech channel slews its bias back down to 0");
}

#ifndef MB_HAVE_PIOASM
void test_pio(void) {}
#else

#define CYC   196                    /* system clocks per Apple bus cycle (~978 ns) */
#define HALF   98
#define NS(c) ((c) * 5)

static pio_sim_t      P[3];
static pio_sim_gpio_t G;
static uint64_t       clk;

static struct { bool on, read; uint8_t addr, data; uint64_t cycle; } acc;
static bool res_high = true;

static int  g_ticks[2], g_t2_ticks[2];
static int64_t g_tick_cycle[2], g_t2_tick_cycle[2];
static int  g_resets;
static bool in_poll;

static uint64_t bus_cycle(void) { return clk / CYC; }
static unsigned phase(void)     { return (unsigned)(clk % CYC); }

static uint32_t bus_inputs(void) {
    uint64_t cyc = bus_cycle(); unsigned ph = phase();
    uint32_t in = 0;
    if (ph >= HALF) in |= 1u << PIN_PHI0;
    bool during = acc.on && (cyc == acc.cycle || (cyc == acc.cycle + 1 && ph < 2));
    bool iosel_low = during && ((cyc == acc.cycle && ph >= HALF + 3) || cyc == acc.cycle + 1);
    if (!iosel_low) in |= 1u << PIN_IOSEL;
    if (during) {
        in |= (uint32_t)acc.addr << PIN_ADDR_BASE;
        if (acc.read) in |= 1u << PIN_RW;
        else if (ph >= HALF + 6) in |= acc.data;
    } else {
        in |= 1u << PIN_RW;
    }
    if (res_high) in |= 1u << PIN_RES;
    return in;
}

static uint32_t ticks_now(void) { return 0u - g_phi2_mirror; }

static void step(void) {
    G.in = bus_inputs();
    for (int i = 0; i < 3; i++) pio_sim_step(&P[i], &G);
    fake_sdk_dma_step();
    pio_sim_gpio_clock(&G);
    clk++;
    if (!in_poll && ((P[1].irq & 3) || (P[2].irq & 3))) {
        in_poll = true;
        fake_sdk_sync_irq_regs();
        uint32_t t = mb_timer_ticks();
        for (int ch = 0; ch < 2; ch++) {
            if (t & MB_TICK_T1(ch)) { g_ticks[ch]++;    g_tick_cycle[ch]    = (int64_t)ticks_now(); }
            if (t & MB_TICK_T2(ch)) { g_t2_ticks[ch]++; g_t2_tick_cycle[ch] = (int64_t)ticks_now(); }
        }
        in_poll = false;
    }
}
static void advance(int clocks) { while (clocks-- > 0) step(); }
static void to_phase(unsigned ph) { while (phase() != ph) step(); }

static void pio_reset_cb(bool a) { (void)a; g_resets++; }

static void boot(void) {
    fake_sdk_reset();
    for (int i = 0; i < 3; i++) pio_sim_reset(&P[i]);
    memset(&G, 0, sizeof G);
    clk = 0; acc.on = false; res_high = true;
    void *sims[3] = { &P[0], &P[1], &P[2] };
    fake_sdk_attach_sim(sims, &G);
    memset((void *)mb_rtable, 0, 256);
    mb_bus_init();
    mb_bus_reset_handler(pio_reset_cb);
    fake.gpio[PIN_RES].level = true;
    mb_bus_reset_start();
    fake_sdk_set_call_cost(advance, 4);
    advance(CYC * 3);
}

typedef struct { uint8_t want, at_latch; int valid_clk; bool clean, released, driven; } read_t;

static read_t bus_read(uint8_t addr, unsigned latch_ns) {
    read_t r = { 0, 0, -1, true, false, false };
    to_phase(0);
    acc.on = true; acc.read = true; acc.addr = addr; acc.cycle = bus_cycle();
    to_phase(HALF);
    r.want = mb_rtable[addr & 0xEF];
    for (unsigned k = 0; k < CYC - HALF; k++) {
        bool drv = (G.out_enable & 0xFF) == 0xFF;
        uint8_t v = (uint8_t)pio_sim_gpio_levels(&G);
        r.driven |= drv;
        if (drv && v != 0x00 && v != r.want) r.clean = false;
        if (drv && v == r.want) { if (r.valid_clk < 0) r.valid_clk = (int)k; }
        else if (r.valid_clk >= 0 && k * 5 < latch_ns) r.valid_clk = -1;
        if (k * 5 == latch_ns) r.at_latch = v;
        step();
    }
    advance(12);
    r.released = (G.out_enable & 0xFF) == 0;
    acc.on = false;
    return r;
}

static void bus_write(uint8_t addr, uint8_t data) {
    to_phase(0);
    acc.on = true; acc.read = false; acc.addr = addr; acc.data = data; acc.cycle = bus_cycle();
    advance(CYC + 4);
    acc.on = false;
}

static void reads(void) {
    boot();
    CHECK(((P[0].sm[MB_SM_RADDR].y << 8) & 0xFFFFFFFFu) == (uint32_t)(uintptr_t)mb_rtable,
          "the raddr SM holds the response table's base");
    for (int i = 0; i < 256; i++) mb_rtable[i] = (uint8_t)(i ^ 0xA5);

    int worst = 0; bool all_ok = true, all_clean = true, all_rel = true;
    for (int a = 0; a < 256; a++) {
        read_t r = bus_read((uint8_t)a, 340);
        all_ok &= r.at_latch == (uint8_t)((a & 0xEF) ^ 0xA5) && r.valid_clk >= 0;
        all_clean &= r.clean;
        all_rel &= r.released;
        if (r.valid_clk > worst) worst = r.valid_clk;
    }
    printf("  read path: table byte stable %d ns after PHI0 rises (worst of 256 reads)\n", NS(worst));
    CHECK(all_ok, "every read returns table[address with A4 cleared] by the IIgs latch point");
    CHECK(all_clean, "a read drives only 0x00 and then its own byte");
    CHECK(all_rel, "the bus is released after /IOSEL rises");
    CHECK(NS(worst) <= 160, "the byte is valid within 160 ns of PHI0 rising (IIgs spec latch: ~340 ns)");

    bool quiet = true;
    for (int i = 0; i < 16; i++) {
        bus_write((uint8_t)(i * 17), 0x3C);
        quiet &= (G.out_enable & 0xFF) == 0 && P[0].sm[MB_SM_RADDR].rx_n == 0;
    }
    read_t after = bus_read(0x01, 340);
    CHECK(quiet && after.at_latch == (0x01 ^ 0xA5) && after.clean, "writes are never driven and leave no byte behind");

    pio_sm_put(pio0, MB_SM_READ, 0xEE);
    read_t late = bus_read(0x02, 340);
    CHECK(late.at_latch == (0x02 ^ 0xA5) && late.clean, "a late byte from an earlier read is dropped");
}

static uint16_t t1_oracle(uint16_t n, uint32_t d) {
    uint32_t p = (d - 1u) % ((uint32_t)n + 2u);
    return p <= n ? (uint16_t)(n - p) : 0xFFFF;
}

static bool t1_run(uint16_t n, unsigned at_phase, unsigned late, int cycles, bool via_bus) {
    to_phase(at_phase);
    uint32_t w = ticks_now() - late;
    g_ticks[0] = 0;
    mb_t1_arm(0, w, n);
    uint32_t e = ticks_now();
    bool ok = true;
    int ffff = 0;
    for (int k = 0; k < cycles && ok; k++) {
        to_phase(HALF + 40);
        uint32_t c = ticks_now();
        if (c <= e) continue;
        uint16_t want = t1_oracle(n, c - w);
        uint16_t got = (uint16_t)(mb_rtable[4] | (mb_rtable[5] << 8));
        if (got != want) { printf("  T1 n=%u phase=%u late=%u: cycle +%u shows %04X, 6522 %04X\n", n, at_phase, late, c - w, got, want); ok = false; }
        if (want == 0xFFFF) ffff++;
        if (via_bus && k % 3 == 0) {
            read_t r = bus_read(0x04, 340);
            uint16_t bw = t1_oracle(n, ticks_now() - 1 - w);
            if (r.at_latch != (uint8_t)bw) { printf("  T1C-L over the bus: %02X, 6522 %02X\n", r.at_latch, (uint8_t)bw); ok = false; }
        }
        advance(CYC - 40 - HALF);
    }
    if (ok && g_ticks[0] < ffff) { printf("  T1 n=%u: %d ticks for %d $FFFF cycles\n", n, g_ticks[0], ffff); ok = false; }
    return ok;
}

static void timer1(void) {
    boot();
    const uint16_t ns[] = { 0, 1, 2, 5, 0x40, 0x1234 };
    bool ok = true;
    for (unsigned i = 0; i < sizeof ns / sizeof ns[0]; i++)
        ok &= t1_run(ns[i], 150, 0, ns[i] < 0x100 ? 3 * (ns[i] + 2) + 4 : 40, true);
    CHECK(ok, "Timer1 counts n..0, $FFFF, n in the table and on the bus, for n = 0, 1, 2, 5, $40, $1234");

    bool phases = true;
    for (unsigned ph = 0; ph < CYC; ph += 5)
        for (unsigned late = 0; late < 2; late++)
            phases &= t1_run(5, ph, late, 12, false);
    CHECK(phases, "a Timer1 load is cycle-exact from any point of the bus cycle, on time or a cycle late");

    to_phase(150);
    uint32_t w = ticks_now();
    g_ticks[0] = 0;
    mb_t1_arm(0, w, 9);
    advance(CYC * 14);
    CHECK(g_ticks[0] >= 1 && (uint32_t)g_tick_cycle[0] - w == 9 + 2 + (uint32_t)(g_ticks[0] - 1) * 11,
          "the Timer1 tick arrives on the $FFFF cycle, N+2 after the load");

    to_phase(150);
    w = ticks_now();
    mb_t1_arm(0, w, 0x10);
    advance(CYC * 4);
    mb_t1_latch(0, 0x30); mb_t1_latch(0, 0x40); mb_t1_latch(0, 0x08);
    bool latch_ok = true;
    for (int k = 0; k < 0x10 + 0x08 * 3; k++) {
        to_phase(HALF + 40);
        uint32_t d = ticks_now() - w;
        uint16_t want = d <= 0x12 ? t1_oracle(0x10, d) : t1_oracle(0x08, d - 0x12);
        uint16_t got = (uint16_t)(mb_rtable[4] | (mb_rtable[5] << 8));
        if (d >= 6 && got != want) { printf("  latch: cycle +%u shows %04X, want %04X\n", d, got, want); latch_ok = false; break; }
        advance(CYC - 40 - HALF);
    }
    CHECK(latch_ok, "a latch change takes effect at the next reload, and the newest of several wins");

    to_phase(150);
    w = ticks_now();
    mb_t1_arm(0, w, 7); mb_t1_arm(1, w, 3);
    advance(CYC * 3);
    to_phase(HALF + 40);
    uint32_t d = ticks_now() - w;
    CHECK((uint16_t)(mb_rtable[0x04] | (mb_rtable[0x05] << 8)) == t1_oracle(7, d)
          && (uint16_t)(mb_rtable[0x84] | (mb_rtable[0x85] << 8)) == t1_oracle(3, d),
          "VIA1 and VIA2 Timer1 count independently");
}

static void timer2(void) {
    boot();
    to_phase(150);
    uint32_t w = ticks_now();
    g_t2_ticks[0] = 0;
    mb_t2_arm(0, w, 0x30);
    bool ok = true;
    for (int k = 0; k < 0x30 + 40 && ok; k++) {
        to_phase(HALF + 40);
        uint32_t d = ticks_now() - w;
        uint16_t want = (uint16_t)(0x30 - (d - 1));
        uint16_t got = (uint16_t)(mb_rtable[8] | (mb_rtable[9] << 8));
        if (d >= 2 && got != want) { printf("  T2: cycle +%u shows %04X, 6522 %04X\n", d, got, want); ok = false; }
        advance(CYC - 40 - HALF);
    }
    CHECK(ok, "Timer2 counts n..0, $FFFF, $FFFE, ... without reloading");
    CHECK_SPEC(g_t2_ticks[0] == 1 && (uint32_t)g_t2_tick_cycle[0] - w == 0x30 + 2, "W65C22 2.9; mb-audit T6522_11",
               "Timer2 ticks once, on the $FFFF cycle N+2 PHI2 cycles after the load");

    mb_t2_hold(0, 0x1234);
    advance(CYC * 5);
    CHECK((uint16_t)(mb_rtable[8] | (mb_rtable[9] << 8)) == 0x1234, "a held Timer2 shows its load value");
    to_phase(150);
    w = ticks_now();
    mb_t2_arm(0, w, 0x1234);
    advance(CYC * 4);
    to_phase(HALF + 40);
    CHECK((uint16_t)(mb_rtable[8] | (mb_rtable[9] << 8)) == (uint16_t)(0x1234 - (ticks_now() - w - 1)),
          "re-loading a held Timer2 counts again");
}

static void reset(void) {
    boot();
    for (int i = 0; i < 256; i++) mb_rtable[i] = (uint8_t)i;
    fake.gpio_cb(PIN_RES, GPIO_IRQ_EDGE_FALL);
    res_high = false;
    advance(CYC);
    read_t r = bus_read(0x03, 340);
    CHECK(!r.driven, "while /RES is low the card stays off the bus");
    res_high = true;
    fake.gpio_cb(PIN_RES, GPIO_IRQ_EDGE_RISE);
    advance(CYC);
    r = bus_read(0x03, 340);
    CHECK(r.at_latch == 0x03 && r.clean, "after /RES the read path works again (table base reloaded)");
}

void test_pio(void) {
    reads();
    timer1();
    timer2();
    reset();
}
#endif

int g_tests, g_fails, g_xfails;

void test_bus(void);
void test_audio(void);
void test_pio(void);

int main(void) {
    test_bus();
    test_audio();
    test_pio();
    printf("\n%d/%d checks passed, %d known discrepancies (XFAIL)%s\n",
           g_tests - g_fails - g_xfails, g_tests, g_xfails, g_fails ? "  <-- FAILURES" : "  (all pass)");
    return g_fails ? 1 : 0;
}
