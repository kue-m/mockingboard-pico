#include <string.h>
#include "pio_sim.h"

enum { OP_JMP, OP_WAIT, OP_IN, OP_OUT, OP_PUSHPULL, OP_MOV, OP_IRQ, OP_SET };

void pio_sim_reset(pio_sim_t *p) {
    memset(p, 0, sizeof *p);
    for (unsigned s = 0; s < PIO_SIM_SMS; s++) {
        pio_sim_sm_t *m = &p->sm[s];
        m->wrap = 31; m->in_count = 32; m->push_thresh = 32; m->pull_thresh = 32;
        m->osr_count = 32;
    }
}

void pio_sim_load(pio_sim_t *p, uint8_t offset, const uint16_t *code, unsigned len) {
    for (unsigned i = 0; i < len; i++) p->mem[(offset + i) & 31] = code[i];
}

void pio_sim_sm_restart(pio_sim_t *p, unsigned sm, uint8_t pc) {
    pio_sim_sm_t *m = &p->sm[sm];
    m->pc = pc; m->x = m->y = m->isr = m->osr = 0;
    m->isr_count = 0; m->osr_count = 32; m->delay = 0;
}

bool pio_sim_tx_put(pio_sim_t *p, unsigned sm, uint32_t v) {
    pio_sim_sm_t *m = &p->sm[sm];
    if (m->tx_n == PIO_SIM_FIFO) return false;
    m->tx[m->tx_n++] = v;
    return true;
}

bool pio_sim_rx_get(pio_sim_t *p, unsigned sm, uint32_t *v) {
    pio_sim_sm_t *m = &p->sm[sm];
    if (m->rx_n == 0) return false;
    *v = m->rx[0];
    memmove(m->rx, m->rx + 1, (size_t)(--m->rx_n) * sizeof m->rx[0]);
    return true;
}

static bool tx_pop(pio_sim_sm_t *m, uint32_t *v) {
    if (m->tx_n == 0) return false;
    *v = m->tx[0];
    memmove(m->tx, m->tx + 1, (size_t)(--m->tx_n) * sizeof m->tx[0]);
    return true;
}

static uint32_t mask_n(unsigned n) { return n >= 32 ? 0xFFFFFFFFu : ((1u << n) - 1u); }

static uint32_t read_pins(const pio_sim_sm_t *m, const pio_sim_gpio_t *g) {
    uint32_t v = (g->sync2 >> m->in_base) | (m->in_base ? g->sync2 << (32 - m->in_base) : 0);
    return v & mask_n(m->in_count);
}

static void write_out(pio_sim_sm_t *m, pio_sim_gpio_t *g, uint32_t v, bool dirs) {
    uint32_t msk = mask_n(m->out_count) << m->out_base, bits = (v << m->out_base) & msk;
    if (dirs) g->out_enable = (g->out_enable & ~msk) | bits;
    else      g->out_value  = (g->out_value & ~msk) | bits;
}

static unsigned irq_bit(uint16_t field, unsigned sm) {
    unsigned idx = field & 7, mode = (field >> 3) & 3;
    if (mode == 2) idx = (idx & 4) | ((idx + sm) & 3);
    return 1u << idx;
}

static uint32_t bitrev(uint32_t v) {
    uint32_t r = 0;
    for (int i = 0; i < 32; i++) { r = (r << 1) | (v & 1); v >>= 1; }
    return r;
}

static bool execute(pio_sim_t *p, pio_sim_gpio_t *g, unsigned sm, uint16_t in, bool *jumped) {
    pio_sim_sm_t *m = &p->sm[sm];
    unsigned op = in >> 13, a = (in >> 5) & 7, b = in & 31;
    *jumped = false;
    switch (op) {
    case OP_JMP: {
        bool take;
        switch (a) {
            case 0: take = true; break;
            case 1: take = m->x == 0; break;
            case 2: take = m->x != 0; m->x--; break;
            case 3: take = m->y == 0; break;
            case 4: take = m->y != 0; m->y--; break;
            case 5: take = m->x != m->y; break;
            case 6: take = (g->sync2 >> m->jmp_pin) & 1; break;
            default: take = m->osr_count < m->pull_thresh; break;
        }
        if (take) { m->pc = (uint8_t)b; *jumped = true; }
        return true;
    }
    case OP_WAIT: {
        unsigned pol = (in >> 7) & 1, src = (in >> 5) & 3;
        if (src == 0) return ((g->sync2 >> b) & 1) == pol;
        if (src == 1) return ((read_pins(m, g) >> b) & 1) == pol;
        if (src == 2) {
            unsigned bit = irq_bit(in, sm);
            bool set = (p->irq & bit) != 0;
            if (set != (bool)pol) return false;
            if (pol) p->irq &= (uint8_t)~bit;
            return true;
        }
        return ((g->sync2 >> m->jmp_pin) & 1) == pol;
    }
    case OP_IN: {
        unsigned n = b ? b : 32;
        if (m->autopush && m->isr_count + n >= m->push_thresh && m->rx_n == PIO_SIM_FIFO) return false;
        uint32_t v;
        switch (a) {
            case 0: v = read_pins(m, g); break;
            case 1: v = m->x; break;
            case 2: v = m->y; break;
            case 6: v = m->isr; break;
            case 7: v = m->osr; break;
            default: v = 0; break;
        }
        v &= mask_n(n);
        if (m->in_shift_right) m->isr = (n == 32 ? 0 : m->isr >> n) | (n == 32 ? v : v << (32 - n));
        else                   m->isr = (n == 32 ? 0 : m->isr << n) | v;
        m->isr_count = (uint8_t)(m->isr_count + n > 32 ? 32 : m->isr_count + n);
        if (m->autopush && m->isr_count >= m->push_thresh) {
            m->rx[m->rx_n++] = m->isr; m->pushes++;
            m->isr = 0; m->isr_count = 0;
        }
        return true;
    }
    case OP_OUT: {
        unsigned n = b ? b : 32;
        uint32_t v;
        if (m->out_shift_right) { v = m->osr & mask_n(n); m->osr = n == 32 ? 0 : m->osr >> n; }
        else                    { v = n == 32 ? m->osr : m->osr >> (32 - n); m->osr = n == 32 ? 0 : m->osr << n; }
        m->osr_count = (uint8_t)(m->osr_count + n > 32 ? 32 : m->osr_count + n);
        switch (a) {
            case 0: write_out(m, g, v, false); break;
            case 1: m->x = v; break;
            case 2: m->y = v; break;
            case 4: write_out(m, g, v, true); break;
            case 5: m->pc = (uint8_t)v; *jumped = true; break;
            default: break;
        }
        return true;
    }
    case OP_PUSHPULL: {
        bool pull = (in >> 7) & 1, block = (in >> 5) & 1;
        if (pull) {
            uint32_t v;
            if (tx_pop(m, &v)) m->osr = v;
            else if (block) return false;
            else m->osr = m->x;
            m->osr_count = 0;
        } else {
            if (m->rx_n == PIO_SIM_FIFO) { if (block) return false; }
            else { m->rx[m->rx_n++] = m->isr; m->pushes++; }
            m->isr = 0; m->isr_count = 0;
        }
        return true;
    }
    case OP_MOV: {
        unsigned dst = a, opn = (in >> 3) & 3, src = in & 7;
        uint32_t v;
        switch (src) {
            case 0: v = read_pins(m, g); break;
            case 1: v = m->x; break;
            case 2: v = m->y; break;
            case 5: {
                unsigned level = m->status_rx ? m->rx_n : m->tx_n;
                v = level < m->status_n ? 0xFFFFFFFFu : 0;
                break;
            }
            case 6: v = m->isr; break;
            case 7: v = m->osr; break;
            default: v = 0; break;
        }
        if (opn == 1) v = ~v; else if (opn == 2) v = bitrev(v);
        switch (dst) {
            case 0: write_out(m, g, v, false); break;
            case 1: m->x = v; break;
            case 2: m->y = v; break;
            case 3: write_out(m, g, v, true); break;
            case 5: m->pc = (uint8_t)(v & 31); *jumped = true; break;
            case 6: m->isr = v; m->isr_count = 0; break;
            case 7: m->osr = v; m->osr_count = 0; break;
            default: break;
        }
        return true;
    }
    case OP_IRQ: {
        unsigned bit = irq_bit(in, sm);
        bool clr = (in >> 6) & 1;
        if (clr) p->irq &= (uint8_t)~bit;
        else { p->irq |= (uint8_t)bit; p->irq_raised |= (uint8_t)bit; }
        return true;
    }
    default: {
        switch (a) {
            case 0: { uint32_t msk = mask_n(m->set_count) << m->set_base;
                      g->out_value = (g->out_value & ~msk) | (((uint32_t)b << m->set_base) & msk); break; }
            case 1: m->x = b; break;
            case 2: m->y = b; break;
            case 4: { uint32_t msk = mask_n(m->set_count) << m->set_base;
                      g->out_enable = (g->out_enable & ~msk) | (((uint32_t)b << m->set_base) & msk); break; }
            default: break;
        }
        return true;
    }
    }
}

void pio_sim_exec(pio_sim_t *p, pio_sim_gpio_t *g, unsigned sm, uint16_t instr) {
    bool jumped;
    (void)execute(p, g, sm, instr, &jumped);
}

void pio_sim_step(pio_sim_t *p, pio_sim_gpio_t *g) {
    for (unsigned s = 0; s < PIO_SIM_SMS; s++) {
        pio_sim_sm_t *m = &p->sm[s];
        if (!m->enabled) continue;
        if (m->delay) { m->delay--; continue; }
        uint16_t in = p->mem[m->pc];
        bool jumped;
        if (!execute(p, g, s, in, &jumped)) { m->stalls++; continue; }
        if (!jumped) m->pc = (m->pc == m->wrap) ? m->wrap_target : (uint8_t)((m->pc + 1) & 31);
        m->delay = (in >> 8) & 31;
    }
}

void pio_sim_gpio_clock(pio_sim_gpio_t *g) {
    g->sync2 = g->sync1;
    g->sync1 = pio_sim_gpio_levels(g);
}

uint32_t pio_sim_gpio_levels(const pio_sim_gpio_t *g) {
    return (g->in & ~g->out_enable) | (g->out_value & g->out_enable);
}
