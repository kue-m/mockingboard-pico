#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sdk_fake.h"
#include "pio_sim.h"

fake_sdk_t fake;
pio_hw_t   fake_pio_hw[3];
dma_hw_t   fake_dma_hw;
pwm_hw_t   fake_pwm_hw;

static pio_sim_t      *g_sim[3];
static pio_sim_gpio_t *g_gpio;
static uintptr_t       g_addr_hi;      /* the high half of host pointers, which DMA addresses lack */
static void          (*g_advance)(int clocks);
static int             g_call_clocks;

static void call_cost(void) { if (g_advance) g_advance(g_call_clocks); }
void fake_sdk_set_call_cost(void (*advance)(int clocks), int clocks) { g_advance = advance; g_call_clocks = clocks; }
#define FAKE_DMA_LATENCY 3
typedef struct { bool busy; uint32_t reload, left; uintptr_t rd, wr; int wait; } dma_run_t;
static dma_run_t g_dma[FAKE_DMA_CH];

void fake_sdk_reset(void) {
    memset(&fake, 0, sizeof fake);
    memset((void *)fake_pio_hw, 0, sizeof fake_pio_hw);
    memset((void *)&fake_dma_hw, 0, sizeof fake_dma_hw);
    memset((void *)&fake_pwm_hw, 0, sizeof fake_pwm_hw);
    memset(g_dma, 0, sizeof g_dma);
    fake.sys_hz = 200000000u;
    fake.dma_timer_claimed = -1;
    for (int g = 0; g < FAKE_GPIOS; g++) { fake.gpio[g].hysteresis = true; fake.gpio[g].drive = -1; fake.gpio[g].slew = -1; }
    g_sim[0] = g_sim[1] = g_sim[2] = NULL; g_gpio = NULL; g_advance = NULL;
}

void fake_sdk_attach_sim(void *sims[3], void *gpio) {
    for (int i = 0; i < 3; i++) g_sim[i] = (pio_sim_t *)sims[i];
    g_gpio = (pio_sim_gpio_t *)gpio;
    g_addr_hi = (uintptr_t)&fake & ~(uintptr_t)0xFFFFFFFFu;
}

static int pio_index(PIO pio) { return (int)(pio - fake_pio_hw); }
static pio_sim_t *sim_of(PIO pio) { return g_sim[pio_index(pio)]; }

void gpio_init(uint g)                     { fake.gpio[g].init = true; fake.gpio[g].dir_out = false; fake.gpio[g].level = false; }
void gpio_disable_pulls(uint g)            { fake.gpio[g].pulls_off = true; }
void gpio_put(uint g, bool v)              { fake.gpio[g].level = v; }
bool gpio_get(uint g)                      { return fake.gpio[g].level; }
void gpio_set_dir(uint g, bool out)        { fake.gpio[g].dir_out = out; }
void gpio_set_drive_strength(uint g, enum gpio_drive_strength d) { fake.gpio[g].drive = (int)d; }
void gpio_set_slew_rate(uint g, enum gpio_slew_rate s)           { fake.gpio[g].slew = (int)s; }
void gpio_set_input_hysteresis_enabled(uint g, bool en)          { fake.gpio[g].hysteresis = en; }
void gpio_set_function(uint g, enum gpio_function fn)            { fake.gpio[g].function = (int)fn; }
void gpio_set_irq_enabled_with_callback(uint g, uint32_t events, bool en, gpio_irq_callback_t cb) {
    fake.gpio_irq_events[g] = events; fake.gpio_irq_enabled[g] = en; fake.gpio_cb = cb;
}

void irq_set_exclusive_handler(uint num, irq_handler_t h) { fake.irq_handler[num] = h; }
void irq_set_enabled(uint num, bool en)                    { fake.irq_enabled[num] = en; }

uint pio_add_program(PIO pio, const pio_program_t *p) {
    int i = pio_index(pio);
    uint off = fake.next_offset[i];
    if (off + p->length > 32) {
        fprintf(stderr, "fake SDK: PIO%d instruction memory overflow (%u + %u > 32)\n", i, off, p->length);
        abort();
    }
    fake.next_offset[i] += p->length;
    fake.programs_added[i]++;
    if (g_sim[i] && p->instructions) {
        for (unsigned k = 0; k < p->length; k++) {      /* the SDK relocates JMP targets */
            uint16_t in = p->instructions[k];
            if ((in >> 13) == 0) in = (uint16_t)(in + off);
            g_sim[i]->mem[off + k] = in;
        }
    }
    return off;
}
void pio_sm_init(PIO pio, uint sm, uint pc, const pio_sm_config *c) {
    call_cost();
    int i = pio_index(pio);
    fake.sm_init_count[i][sm]++; fake.sm_init_pc[i][sm] = pc; fake.sm_cfg[i][sm] = *c;
    fake.sm_enabled[i] &= ~(1u << sm);
    pio_sim_t *s = g_sim[i];
    if (!s) return;
    pio_sim_sm_t *m = &s->sm[sm];
    m->enabled = false;
    m->wrap_target = c->wrap_target; m->wrap = c->wrap;
    m->in_base = c->in_base; m->in_count = c->in_count; m->out_base = c->out_base; m->out_count = c->out_count;
    m->set_base = c->set_base; m->set_count = c->set_count; m->jmp_pin = c->jmp_pin;
    m->in_shift_right = c->in_shift_right; m->autopush = c->autopush;
    m->out_shift_right = c->out_shift_right; m->autopull = c->autopull;
    m->push_thresh = c->push_thresh; m->pull_thresh = c->pull_thresh;
    m->status_rx = c->status_rx; m->status_n = c->status_n;
    m->tx_n = m->rx_n = 0;
    pio_sim_sm_restart(s, sm, (uint8_t)pc);
}
void pio_sm_set_enabled(PIO pio, uint sm, bool en) {
    call_cost();
    int i = pio_index(pio);
    if (en) fake.sm_enabled[i] |= 1u << sm; else fake.sm_enabled[i] &= ~(1u << sm);
    if (g_sim[i]) g_sim[i]->sm[sm].enabled = en;
}
void pio_set_sm_mask_enabled(PIO pio, uint32_t mask, bool en) {
    for (uint sm = 0; sm < 4; sm++) if (mask & (1u << sm)) pio_sm_set_enabled(pio, sm, en);
}
void pio_sm_set_pindirs_with_mask(PIO pio, uint sm, uint32_t dirs, uint32_t mask) {
    (void)sm; int i = pio_index(pio);
    fake.pindirs[i] = (fake.pindirs[i] & ~mask) | (dirs & mask); fake.pindirs_mask[i] |= mask;
    if (g_gpio) g_gpio->out_enable = (g_gpio->out_enable & ~mask) | (dirs & mask);
}
void pio_interrupt_clear(PIO pio, uint num) {
    call_cost();
    int i = pio_index(pio);
    pio->irq &= ~(1u << num); fake.irq_cleared[i] |= 1u << num;
    if (g_sim[i]) g_sim[i]->irq &= (uint8_t)~(1u << num);
}
void pio_set_irq0_source_enabled(PIO pio, enum pio_interrupt_source s, bool en) {
    if (en) fake.irq0_sources[pio_index(pio)] |= 1u << s; else fake.irq0_sources[pio_index(pio)] &= ~(1u << s);
}
void pio_claim_sm_mask(PIO pio, uint mask)  { fake.sm_claimed[pio_index(pio)] |= mask; }
void pio_gpio_init(PIO pio, uint pin)       { fake.gpio[pin].pio = true; fake.gpio[pin].function = GPIO_FUNC_PIO0 + pio_index(pio); }
void pio_sm_clear_fifos(PIO pio, uint sm) {
    int i = pio_index(pio);
    fake.fifo_clears[i][sm]++;
    if (g_sim[i]) g_sim[i]->sm[sm].tx_n = g_sim[i]->sm[sm].rx_n = 0;
}
uint pio_get_dreq(PIO pio, uint sm, bool tx) { return (uint)(pio_index(pio) * 8 + (tx ? 0 : 4) + sm); }
void pio_sm_put(PIO pio, uint sm, uint32_t v) {
    call_cost();
    int i = pio_index(pio);
    fake.tx_log[i][sm][fake.tx_puts[i][sm] % 8] = v; fake.tx_puts[i][sm]++;
    if (g_sim[i]) pio_sim_tx_put(g_sim[i], sm, v);
}
bool pio_sm_is_tx_fifo_full(PIO pio, uint sm) {
    pio_sim_t *s = sim_of(pio);
    return s ? s->sm[sm].tx_n == PIO_SIM_FIFO : false;
}
bool pio_sm_is_rx_fifo_empty(PIO pio, uint sm) {
    pio_sim_t *s = sim_of(pio);
    return s ? s->sm[sm].rx_n == 0 : true;
}
uint32_t pio_sm_get(PIO pio, uint sm) {
    uint32_t v = 0;
    pio_sim_t *s = sim_of(pio);
    if (s) pio_sim_rx_get(s, sm, &v);
    return v;
}
void pio_sm_exec(PIO pio, uint sm, uint instr) {
    call_cost();
    int i = pio_index(pio);
    fake.execs[i][sm]++;
    if (g_sim[i]) pio_sim_exec(g_sim[i], g_gpio, sm, (uint16_t)instr);
}
pio_sm_config pio_get_default_sm_config(void) {
    pio_sm_config c;
    memset(&c, 0, sizeof c);
    c.wrap = 31; c.in_count = 32; c.push_thresh = 32; c.pull_thresh = 32; c.in_shift_right = true; c.out_shift_right = true;
    return c;
}
void sm_config_set_wrap(pio_sm_config *c, uint t, uint w)            { c->wrap_target = (uint8_t)t; c->wrap = (uint8_t)w; }
void sm_config_set_in_pins(pio_sm_config *c, uint base)               { c->in_base = (uint8_t)base; }
void sm_config_set_in_pin_count(pio_sm_config *c, uint n)             { c->in_count = (uint8_t)n; }
void sm_config_set_in_shift(pio_sm_config *c, bool r, bool a, uint t) { c->in_shift_right = r; c->autopush = a; c->push_thresh = (uint8_t)(t ? t : 32); }
void sm_config_set_out_shift(pio_sm_config *c, bool r, bool a, uint t){ c->out_shift_right = r; c->autopull = a; c->pull_thresh = (uint8_t)(t ? t : 32); }
void sm_config_set_out_pins(pio_sm_config *c, uint base, uint n)      { c->out_base = (uint8_t)base; c->out_count = (uint8_t)n; }
void sm_config_set_jmp_pin(pio_sm_config *c, uint pin)                { c->jmp_pin = (uint8_t)pin; }
void sm_config_set_mov_status(pio_sm_config *c, enum pio_mov_status_type s, uint n) {
    c->status_rx = (s == STATUS_RX_LESSTHAN); c->status_n = (uint8_t)n;
}

void fake_sdk_sync_irq_regs(void) {
    for (int i = 0; i < 3; i++) if (g_sim[i]) fake_pio_hw[i].irq = g_sim[i]->irq;
}

void busy_wait_us(uint64_t us)                     { fake.busy_waited_us += us; fake.now_us += us; }
void busy_wait_at_least_cycles(uint32_t cycles)    { if (g_advance) g_advance((int)cycles); }

int dma_claim_unused_channel(bool req) {
    (void)req;
    for (int c = 0; c < FAKE_DMA_CH; c++) if (!fake.dma_claimed[c]) { fake.dma_claimed[c] = true; return c; }
    return -1;
}
int dma_claim_unused_timer(bool req) { (void)req; fake.dma_timer_claimed = 0; return 0; }
dma_channel_config dma_channel_get_default_config(uint ch) {
    dma_channel_config c = { DMA_SIZE_32, FAKE_DREQ_FORCE, ch, true, false, false };
    return c;
}
void channel_config_set_transfer_data_size(dma_channel_config *c, enum dma_channel_transfer_size s) { c->size = (uint)s; }
void channel_config_set_read_increment(dma_channel_config *c, bool i)  { c->read_inc = i; }
void channel_config_set_write_increment(dma_channel_config *c, bool i) { c->write_inc = i; }
void channel_config_set_dreq(dma_channel_config *c, uint d)            { c->dreq = d; }
void channel_config_set_chain_to(dma_channel_config *c, uint ch)       { c->chain_to = ch; }
void channel_config_set_high_priority(dma_channel_config *c, bool h)   { c->high_priority = h; }

static void dma_trigger(uint ch) {
    fake.dma_started[ch] = true;
    g_dma[ch].busy = true; g_dma[ch].left = g_dma[ch].reload;
}
void dma_channel_configure(uint ch, const dma_channel_config *c, volatile void *w, const volatile void *r,
                           uint count, bool trigger) {
    fake.dma_cfg[ch] = *c; fake.dma_write[ch] = w; fake.dma_read[ch] = r; fake.dma_count[ch] = count;
    g_dma[ch].reload = count; g_dma[ch].rd = (uintptr_t)r; g_dma[ch].wr = (uintptr_t)w;
    if (trigger) dma_trigger(ch);
}
void dma_channel_start(uint ch)                                  { dma_trigger(ch); }
void dma_channel_set_read_addr(uint ch, const volatile void *r, bool trigger) {
    fake.dma_read[ch] = r; g_dma[ch].rd = (uintptr_t)r; if (trigger) dma_trigger(ch);
}
void dma_channel_set_irq0_enabled(uint ch, bool en)              { fake.dma_irq0[ch] = en; }
void dma_timer_set_fraction(uint t, uint16_t num, uint16_t den)  { (void)t; fake.dma_timer_num = num; fake.dma_timer_den = den; }
uint dma_get_timer_dreq(uint t)                                  { return 0x3B + t; }

static bool pio_fifo_at(uintptr_t a, bool tx, int *pio, uint *sm) {
    for (int i = 0; i < 3; i++)
        for (uint s = 0; s < 4; s++) {
            uintptr_t r = (uintptr_t)(tx ? &fake_pio_hw[i].txf[s] : &fake_pio_hw[i].rxf[s]);
            if (a >= r && a < r + 4) { *pio = i; *sm = s; return true; }
        }
    return false;
}

static bool dma_ready(uint ch) {
    uint d = fake.dma_cfg[ch].dreq;
    if (d == FAKE_DREQ_FORCE) return true;
    if (d >= 24) return false;
    int i = (int)d / 8; uint sm = d % 4; bool rx = (d % 8) >= 4;
    if (!g_sim[i]) return false;
    return rx ? g_sim[i]->sm[sm].rx_n > 0 : g_sim[i]->sm[sm].tx_n < PIO_SIM_FIFO;
}

static void dma_transfer(uint ch) {
    dma_run_t *d = &g_dma[ch];
    unsigned size = 1u << fake.dma_cfg[ch].size;
    uint32_t v = 0;
    int pi; uint sm;
    if (pio_fifo_at(d->rd, false, &pi, &sm)) {
        uint32_t w = 0;
        pio_sim_rx_get(g_sim[pi], sm, &w);
        v = w >> (8 * (d->rd & 3));
    } else {
        memcpy(&v, (const void *)d->rd, size);
    }
    v &= size == 4 ? 0xFFFFFFFFu : ((1u << (8 * size)) - 1u);
    if (pio_fifo_at(d->wr, true, &pi, &sm)) {
        uint32_t w = size == 1 ? v * 0x01010101u : size == 2 ? v * 0x00010001u : v;   /* narrow writes replicate */
        pio_sim_tx_put(g_sim[pi], sm, w);
    } else {
        bool alias = false;
        for (uint x = 0; x < FAKE_DMA_CH && !alias; x++) {
            if (d->wr == (uintptr_t)&fake_dma_hw.ch[x].al3_read_addr_trig) {
                g_dma[x].rd = g_addr_hi | v; dma_trigger(x); alias = true;
            } else if (d->wr == (uintptr_t)&fake_dma_hw.ch[x].al1_transfer_count_trig) {
                g_dma[x].reload = v; dma_trigger(x); alias = true;
            }
        }
        if (!alias) memcpy((void *)d->wr, &v, size);
    }
    if (fake.dma_cfg[ch].read_inc)  d->rd += size;
    if (fake.dma_cfg[ch].write_inc) d->wr += size;
    if (d->reload != 0xFFFFFFFFu && --d->left == 0) {
        d->busy = false;
        if (fake.dma_cfg[ch].chain_to != ch) dma_trigger(fake.dma_cfg[ch].chain_to);
    }
}

void fake_sdk_dma_step(void) {
    for (uint ch = 0; ch < FAKE_DMA_CH; ch++) {
        if (!g_dma[ch].busy || !dma_ready(ch)) { g_dma[ch].wait = FAKE_DMA_LATENCY; continue; }
        if (g_dma[ch].wait > 0) g_dma[ch].wait--;
    }
    for (int pass = 0; pass < 2; pass++)
        for (uint ch = 0; ch < FAKE_DMA_CH; ch++) {
            if (!g_dma[ch].busy || fake.dma_cfg[ch].high_priority != (pass == 0)) continue;
            if (!dma_ready(ch) || g_dma[ch].wait > 0) continue;
            dma_transfer(ch);
            g_dma[ch].wait = FAKE_DMA_LATENCY;
            return;
        }
}

uint pwm_gpio_to_slice_num(uint g)                               { return (g >> 1) & 7u; }
void pwm_set_clkdiv_int_frac(uint s, uint8_t i, uint8_t f)       { (void)s; fake.pwm_div_int = i; fake.pwm_div_frac = f; }
void pwm_set_wrap(uint s, uint16_t w)                            { (void)s; fake.pwm_wrap = w; }
void pwm_set_both_levels(uint s, uint16_t a, uint16_t b)         { (void)s; fake.pwm_level_a = a; fake.pwm_level_b = b; }
void pwm_set_enabled(uint s, bool en)                            { (void)s; fake.pwm_enabled = en; }
uint32_t clock_get_hz(enum clock_index c)                        { (void)c; return fake.sys_hz; }
