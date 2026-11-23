#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "test_checks.h"
#include "mb_via.h"
#include "mb_audio.h"
#include "mb_bus.h"
#include "mb_selftest.h"
#include "mb_version.h"
#include "mb_version_gen.h"

uint8_t mb_test_via_read_value(int ch, uint8_t off);

uint8_t  g_wdata;
bool     g_irq;
int      g_ay_op = -1, g_ay_ch = -1;
uint8_t  g_ay_val;

void    mb_bus_irq(bool a)            { g_irq = a; }
uint8_t mb_bus_get_wdata(void)        { return g_wdata; }
void    mb_audio_post(uint32_t c)     { g_ay_op = (c >> 16) & 3; g_ay_ch = (c >> 8) & 1; g_ay_val = c & 0xFF; }

uint32_t g_phi2 = 0;
uint32_t mb_phi2_ticks(void)          { return g_phi2; }

int     g_sp_chip = -1, g_sp_reg = -1, g_sp_count = 0;
uint8_t g_sp_val;
uint8_t g_ar_mirror[2] = { 0, 0 };
uint8_t g_ar_edges_mirror[2] = { 0, 0 };
void    speech_post(int c, int reg, uint8_t v) { g_sp_chip = c; g_sp_reg = reg; g_sp_val = v; g_sp_count++; }
uint8_t speech_ar(int c)                       { return g_ar_mirror[c & 1]; }
uint8_t speech_ar_edges(int c)                 { return g_ar_edges_mirror[c & 1]; }

volatile uint8_t mb_rtable[256];

#define FAKE_LATCH_EVENTS 64
typedef struct {
    bool     active, held;
    uint32_t start, x0;
    uint32_t ev_cycle[FAKE_LATCH_EVENTS];
    uint16_t ev_latch[FAKE_LATCH_EVENTS];
    int      ev_n;
} fake_counter_t;
static fake_counter_t g_t1c[2], g_t2c[2];
uint32_t g_t1_arm_w[2], g_t2_arm_w[2];
uint16_t g_t1_arm_n[2], g_t2_arm_n[2], g_t1_latch_posted[2], g_t2_held_n[2];
int      g_t1_arms[2], g_t2_arms[2], g_t1_latch_posts[2], g_t2_holds[2];
bool     g_arm_passed;
uint32_t g_pending_ticks;

static void counter_load(fake_counter_t *c, uint32_t w, uint16_t n) {
    c->active = true; c->held = false; c->start = w + 1; c->x0 = n;
    c->ev_n = 0; c->ev_cycle[0] = w; c->ev_latch[0] = n; c->ev_n = 1;
}
static void counter_post_latch(fake_counter_t *c, uint32_t cycle, uint16_t n) {
    if (c->ev_n < FAKE_LATCH_EVENTS) { c->ev_cycle[c->ev_n] = cycle; c->ev_latch[c->ev_n] = n; c->ev_n++; }
}
static uint16_t counter_at(fake_counter_t *c, bool reload, uint32_t cycle) {
    if (!c->active) return 0;
    if (c->held || (int32_t)(cycle - c->start) <= 0) return (uint16_t)c->x0;
    uint32_t x = c->x0;
    int ev = 0;
    for (uint32_t k = c->start; k != cycle; k++) {
        while (ev + 1 < c->ev_n && (int32_t)(c->ev_cycle[ev + 1] - k) <= 0) ev++;
        if (x == 0)                          x = 0xFFFFFFFFu;
        else if (reload && x == 0xFFFFFFFFu) x = c->ev_latch[ev];
        else                                 x--;
    }
    return (uint16_t)x;
}
static void fake_counters_to_table(void) {
    for (int ch = 0; ch < 2; ch++) {
        uint8_t i = (uint8_t)(ch ? 0x80 : 0x00);
        uint16_t t1 = counter_at(&g_t1c[ch], true, g_phi2), t2 = counter_at(&g_t2c[ch], false, g_phi2);
        mb_rtable[i | 4] = (uint8_t)t1; mb_rtable[i | 5] = (uint8_t)(t1 >> 8);
        if (!g_t2c[ch].held) { mb_rtable[i | 8] = (uint8_t)t2; mb_rtable[i | 9] = (uint8_t)(t2 >> 8); }
    }
}

bool mb_t1_arm(int ch, uint32_t w, uint16_t n) {
    g_t1_arm_w[ch] = w; g_t1_arm_n[ch] = n; g_t1_arms[ch]++;
    counter_load(&g_t1c[ch], w, n);
    bool p = g_arm_passed; g_arm_passed = false; return p;
}
void mb_t1_latch(int ch, uint16_t n) {
    g_t1_latch_posted[ch] = n; g_t1_latch_posts[ch]++;
    counter_post_latch(&g_t1c[ch], g_phi2, n);
}
bool mb_t2_arm(int ch, uint32_t w, uint16_t n) {
    g_t2_arm_w[ch] = w; g_t2_arm_n[ch] = n; g_t2_arms[ch]++;
    counter_load(&g_t2c[ch], w, n);
    bool p = g_arm_passed; g_arm_passed = false; return p;
}
void mb_t2_hold(int ch, uint16_t n) {
    g_t2_held_n[ch] = n; g_t2_holds[ch]++;
    g_t2c[ch].held = true; g_t2c[ch].x0 = n;
    mb_rtable[(ch ? 0x88 : 0x08)] = (uint8_t)n; mb_rtable[(ch ? 0x89 : 0x09)] = (uint8_t)(n >> 8);
}
uint32_t mb_timer_ticks(void) { uint32_t t = g_pending_ticks; g_pending_ticks = 0; return t; }

void t1_tick(int ch)   { g_pending_ticks |= MB_TICK_T1(ch); mb_via_poll(); }
void t2_tick(int ch)   { g_pending_ticks |= MB_TICK_T2(ch); mb_via_poll(); }
void ar_pulse(int ch)  { g_ar_edges_mirror[ch & 1]++; mb_via_poll(); }

static uint8_t table_read(uint8_t addr) {
    mb_via_poll();
    fake_counters_to_table();
    uint8_t v = mb_rtable[addr & 0xEF];
    mb_via_access((uint32_t)addr | (1u << 9));
    return v;
}
void wr(int ch, uint8_t off, uint8_t d) {
    g_wdata = d; mb_via_access((uint32_t)(((ch ? 0x80 : 0) | (off & 0x0F)) & 0xFF));
}
uint8_t rd(int ch, uint8_t off) { return table_read((uint8_t)((ch ? 0x80 : 0) | (off & 0x0F))); }
void wra(uint8_t addr, uint8_t d) { g_wdata = d; mb_via_access((uint32_t)addr); }
uint8_t rda(uint8_t addr) { return table_read(addr); }

uint16_t rd16(int ch, uint8_t off_lo) {
    uint8_t hi = rd(ch, (uint8_t)(off_lo + 1));
    return (uint16_t)((hi << 8) | rd(ch, off_lo));
}

void test_selftest(void) {
    CHECK(!mb_phi2_advanced(1000, 1000), "self-test: frozen counter (no advance) -> not running");
    CHECK(!mb_phi2_advanced(1000, 1000 + 5), "self-test: tiny advance (5) stays below threshold");
    CHECK(!mb_phi2_advanced(1000, 1000 + MB_PHI2_SELFTEST_MIN_TICKS),
          "self-test: exactly the threshold is not 'advancing' (strict >)");
    CHECK(mb_phi2_advanced(1000, 1000 + MB_PHI2_SELFTEST_MIN_TICKS + 1),
          "self-test: one past threshold counts as advancing");
    CHECK(mb_phi2_advanced(5000, 5000 + 10205), "self-test: healthy ~10,205-tick window -> advancing");
    CHECK(mb_phi2_advanced(0xFFFFF000u, 0xFFFFF000u + 10205u),
          "self-test: advance across the 32-bit wrap is counted correctly (modular sub)");
    CHECK(!mb_phi2_advanced(0xFFFFFF00u, 0x00000050u),
          "self-test: a frozen-then-wrapped read (delta 0x150) stays below threshold");
}

void test_via_t1(void) {
    mb_via_reset(true);
    g_phi2 = 100;
    wr(0, 0x0B, 0x40);
    wr(0, 0x06, 0x00);
    wr(0, 0x07, 0x10);
    wr(0, 0x05, 0x10);
    CHECK(g_t1_arm_n[0] == 0x1000 && g_t1_arm_w[0] == 100, "T1C-H loads the counter with the latch, on the write's cycle");
    CHECK(g_irq == false, "no /IRQ before IER bit6 enabled");

    wr(0, 0x0E, 0x80 | 0x40);
    CHECK(rd(0, 0x0E) == 0xC0, "IER reads back the enabled bits with bit7 forced to 1");
    t1_tick(0);
    CHECK(g_irq == true, "Timer1 timeout asserts /IRQ when enabled");
    CHECK((rd(0, 0x0D) & 0x40) != 0, "IFR bit6 set after the timeout");
    CHECK((rd(0, 0x0D) & 0x80) != 0, "IFR bit7 (any) set after the timeout");

    rd(0, 0x04);
    CHECK(g_irq == false, "reading T1C-L releases /IRQ");
    CHECK((rd(0, 0x0D) & 0x40) == 0, "reading T1C-L clears IFR bit6");

    wr(0, 0x0E, 0x40);
    t1_tick(0);
    CHECK((rd(0, 0x0D) & 0x40) != 0, "IFR bit6 still latches when IER disabled");
    CHECK(g_irq == false, "no /IRQ when IER bit6 disabled");
    rd(0, 0x04);

    wr(1, 0x0B, 0x40);
    wr(1, 0x06, 0x00); wr(1, 0x07, 0x08); wr(1, 0x05, 0x08);
    wr(0, 0x0E, 0xC0); wr(1, 0x0E, 0xC0);
    t1_tick(1);
    CHECK(g_irq == true, "VIA2 timeout asserts shared /IRQ");
    t1_tick(0);
    CHECK(g_irq == true, "both pending -> /IRQ still asserted");
    rd(1, 0x04);
    CHECK(g_irq == true, "clearing VIA2 leaves /IRQ (VIA1 still pending)");
    rd(0, 0x04);
    CHECK(g_irq == false, "clearing both releases /IRQ");

    CHECK(g_t1_arm_n[0] == 0x1000 && g_t1_arm_n[1] == 0x0800, "two VIAs hold independent periods");

    mb_via_reset(true);
    wr(0, 0x0B, 0x40);
    wr(0, 0x06, 0x00); wr(0, 0x07, 0x10);
    g_phi2 = 1000; wr(0, 0x05, 0x10);
    g_phi2 = 1001;
    CHECK(rd16(0, 0x04) == 0x1000, "T1 counter reads the latch (0x1000) on the cycle after the load");
    g_phi2 = 1001 + 0x0200;
    CHECK(rd16(0, 0x04) == 0x1000 - 0x0200, "T1 counter drops one per PHI2 cycle");

    g_phi2 = 5000;
    wr(0, 0x06, 0xFF); wr(0, 0x07, 0xFF); wr(0, 0x05, 0xFF);
    { uint8_t a = (g_phi2 = 5010, rd(0, 0x04)); uint8_t b = (g_phi2 = 5015, rd(0, 0x04));
      CHECK((uint8_t)(a - b) == 5, "detection: T1C-L drops by exactly 5 over 5 PHI2 cycles"); }

    g_phi2 = 7000;
    wr(0, 0x0B, 0x40); wr(0, 0x06, 0x04); wr(0, 0x07, 0x00); wr(0, 0x05, 0x00);
    g_phi2 = 7000 + 5;
    CHECK(rd16(0, 0x04) == 0x0000, "free-run counter hits 0 on cycle N+1 after the load");
    g_phi2 = 7000 + 6;
    CHECK(rd16(0, 0x04) == 0xFFFF, "free-run shows $FFFF on the timeout cycle");
    g_phi2 = 7000 + 7;
    CHECK(rd16(0, 0x04) == 0x0004, "free-run reloads N a period of N+2 after the first N");

    mb_via_reset(true);
    wr(0, 0x0B, 0x40); wr(0, 0x0E, 0xC0);
    wr(0, 0x06, 0x00); wr(0, 0x07, 0x08); wr(0, 0x05, 0x08);
    t1_tick(0);
    rd(0, 0x05);
    CHECK((rd(0, 0x0D) & 0x40) != 0, "reading T1C-H leaves the Timer1 flag set");
    rd(0, 0x04);
    CHECK((rd(0, 0x0D) & 0x40) == 0, "reading T1C-L then clears it");
    t1_tick(0);
    CHECK((rd(0, 0x0D) & 0x40) != 0, "free-run interrupts again at the next timeout");

    /* T1L-H write clears IFR6 (W65C22 Tbl 2-7), T1L-L does not: the Ultima V "Activate Music" freeze */
    mb_via_reset(true);
    wr(0, 0x0B, 0x40); wr(0, 0x0E, 0xC0);
    wr(0, 0x06, 0x9A); wr(0, 0x07, 0x42); wr(0, 0x05, 0x42);  /* latch 0x429A = Ultima V's ~60 Hz period */
    t1_tick(0);
    CHECK((rd(0, 0x0D) & 0x40) != 0 && g_irq == true, "Ultima V: Timer1 timeout sets flag + asserts /IRQ");
    wr(0, 0x06, 0x9A);
    CHECK((rd(0, 0x0D) & 0x40) != 0, "writing T1L-L ($06) leaves the Timer1 flag set");
    CHECK(g_irq == true, "writing T1L-L does not release /IRQ");
    wr(0, 0x07, 0x42);
    CHECK((rd(0, 0x0D) & 0x40) == 0, "writing T1L-H ($07) clears the Timer1 flag (Ultima V ack path)");
    CHECK(g_irq == false, "writing T1L-H releases /IRQ (no interrupt storm -> no freeze)");

    /* reading T1L-H does not clear IFR6. The T1C-L-read ack (Ultima III/IV) stays intact */
    t1_tick(0);
    CHECK((rd(0, 0x0D) & 0x40) != 0, "the next timeout sets the Timer1 flag again");
    rd(0, 0x07);
    CHECK((rd(0, 0x0D) & 0x40) != 0, "reading T1L-H leaves the Timer1 flag set");
    CHECK(g_irq == true, "reading T1L-H does not release /IRQ");
    rd(0, 0x04);
    CHECK((rd(0, 0x0D) & 0x40) == 0, "reading T1C-L still clears the flag (Ultima III/IV path intact)");

    /* A latch write applies at the next reload, or a game's first song plays at the wrong tempo. */
    mb_via_reset(true);
    g_phi2 = 20000;
    wr(0, 0x0B, 0x40);
    wr(0, 0x06, 0x04); wr(0, 0x07, 0x00); wr(0, 0x05, 0x00);
    int arms = g_t1_arms[0];
    g_phi2 = 20002;
    wr(0, 0x06, 0x08); wr(0, 0x07, 0x00);
    CHECK(g_t1_arms[0] == arms && g_t1_latch_posted[0] == 0x0008,
          "a bare latch write posts the new latch without restarting the count");
    g_phi2 = 20003; CHECK(rd16(0, 0x04) == 0x0002, "the count carries on from the old load");
    g_phi2 = 20007; CHECK(rd16(0, 0x04) == 0x0008, "the next reload takes the new latch");
    g_phi2 = 20010; CHECK(rd16(0, 0x04) == 0x0005, "and counts down from it");

    g_phi2 = 50; mb_via_init();
    CHECK(rd(0, 0x0B) == 0x00, "cold init: ACR = 0 (one-shot), not free-run");
    CHECK(rd(0, 0x06) == 0xFF && rd(0, 0x07) == 0xFF, "cold init: T1 latch = 0xFFFF");
    { g_phi2 = 100; uint8_t a = rd(0, 0x04); g_phi2 = 105; uint8_t b = rd(0, 0x04);
      CHECK((uint8_t)(a - b) == 5, "cold init: the T1 counter runs for detection without a load"); }

    mb_via_reset(true);
    g_phi2 = 9000;
    wr(0, 0x06, 0x04); wr(0, 0x07, 0x00); wr(0, 0x05, 0x00);
    CHECK(g_t1_arm_n[0] == 4 && g_t1_arm_w[0] == 9000, "one-shot: the load starts the counter");
    g_phi2 = 9000 + 5;
    CHECK(rd16(0, 0x04) == 0x0000, "one-shot counter reaches 0");
    g_phi2 = 9000 + 6;
    CHECK(rd16(0, 0x04) == 0xFFFF, "one-shot counter rolls to $FFFF");
    wr(0, 0x0E, 0xC0);
    t1_tick(0);
    CHECK((rd(0, 0x0D) & 0x40) != 0, "one-shot timeout sets the Timer1 flag");
    rd(0, 0x04);
    t1_tick(0);
    CHECK((rd(0, 0x0D) & 0x40) == 0 && g_irq == false, "one-shot: later timeouts raise nothing until the next load");

    mb_via_reset(true);
    g_phi2 = 30000;
    wr(0, 0x0B, 0x40); wr(0, 0x0E, 0xC0);
    wr(0, 0x06, 0xFF); wr(0, 0x07, 0x40); wr(0, 0x05, 0x40);
    wr(0, 0x0B, 0x02);
    t1_tick(0);
    CHECK((rd(0, 0x0D) & 0x40) != 0 && g_irq == true, "free-run->one-shot: the next timeout sets the flag and /IRQ");
    wr(0, 0x0D, 0x40);
    t1_tick(0);
    CHECK((rd(0, 0x0D) & 0x40) == 0 && g_irq == false, "free-run->one-shot: later timeouts raise nothing");
    wr(0, 0x06, 0x00); wr(0, 0x07, 0x08); wr(0, 0x05, 0x08);
    t1_tick(0);
    CHECK((rd(0, 0x0D) & 0x40) != 0 && g_irq == true, "a subsequent T1C-H load re-arms the one-shot normally");

    mb_via_reset(true);
    wr(0, 0x02, 0x80);
    wr(0, 0x0B, 0xC0);
    wr(0, 0x06, 0x02); wr(0, 0x07, 0x00); wr(0, 0x05, 0x00);
    CHECK((rd(0, 0x00) & 0x80) == 0x00, "PB7 low just after the load");
    t1_tick(0);
    CHECK((rd(0, 0x00) & 0x80) == 0x80, "PB7 high after one free-run period");
    t1_tick(0);
    CHECK((rd(0, 0x00) & 0x80) == 0x00, "PB7 low again after two periods");
    wr(0, 0x02, 0x00);
    CHECK((rd(0, 0x00) & 0x80) == 0x80, "DDRB bit7 input reads the pin, not the Timer1 PB7 wave");

    mb_via_reset(true);
    wr(0, 0x02, 0x80);
    wr(0, 0x0B, 0x80);
    wr(0, 0x06, 0x02); wr(0, 0x07, 0x00); wr(0, 0x05, 0x00);
    CHECK((rd(0, 0x00) & 0x80) == 0x00, "one-shot PB7 low while counting down");
    t1_tick(0);
    CHECK((rd(0, 0x00) & 0x80) == 0x80, "one-shot PB7 high after the timeout");
    t1_tick(0);
    CHECK((rd(0, 0x00) & 0x80) == 0x80, "one-shot PB7 stays high at later timeouts");
}

void test_via_t2(void) {
    mb_via_reset(true);
    g_phi2 = 20000;
    wr(0, 0x08, 0x34);
    wr(0, 0x09, 0x12);
    CHECK(g_t2_arm_n[0] == 0x1234 && g_t2_arm_w[0] == 20000, "T2C-H loads Timer2 on the write's cycle");
    g_phi2 = 20001;
    CHECK(rd16(0, 0x08) == 0x1234, "T2 counter reads the load value on the cycle after the load");

    g_phi2 = 20001 + 0x0034;
    CHECK(rd16(0, 0x08) == 0x1234 - 0x0034, "T2 counter decrements with the PHI2 count");

    g_phi2 = 20001 + 0x1234 + 3;
    CHECK(rd16(0, 0x08) == 0xFFFD, "T2 keeps counting past 0 (one-shot, no reload)");

    mb_via_reset(true);
    wr(0, 0x0E, 0x80 | 0x20);
    wr(0, 0x09, 0x00);
    t2_tick(0);
    CHECK(g_irq == true, "T2 timeout asserts /IRQ when IER bit5 enabled");
    CHECK((rd(0, 0x0D) & 0x20) != 0, "IFR bit5 (T2) set after the timeout");
    rd(0, 0x08);
    CHECK(g_irq == false, "reading T2C-L releases /IRQ");
    CHECK((rd(0, 0x0D) & 0x20) == 0, "reading T2C-L clears IFR bit5");
    t2_tick(0);
    CHECK((rd(0, 0x0D) & 0x20) == 0 && g_irq == false, "T2 interrupts once per load");

    wr(0, 0x0E, 0x20);
    wr(0, 0x09, 0x00);
    t2_tick(0);
    CHECK((rd(0, 0x0D) & 0x20) != 0, "IFR bit5 latches with IER disabled");
    CHECK(g_irq == false, "no /IRQ when T2 IER disabled");
    wr(0, 0x0D, 0x20);
    CHECK((rd(0, 0x0D) & 0x20) == 0, "IFR write-1 clears T2");

    mb_via_reset(true);
    g_phi2 = 30000;
    int holds = g_t2_holds[0];
    wr(0, 0x0B, 0x20);
    wr(0, 0x08, 0x00); wr(0, 0x09, 0x02);
    CHECK(g_t2_holds[0] == holds + 2 && g_t2_held_n[0] == 0x0200, "pulse mode holds the counter at the load (no PB6 source)");
    g_phi2 = 30000 + 0x0100;
    CHECK(rd16(0, 0x08) == 0x0200, "pulse mode: counter holds");
    t2_tick(0);
    CHECK((rd(0, 0x0D) & 0x20) == 0, "pulse mode: no timed interrupt");

    mb_via_reset(true);
    wr(0, 0x0E, 0x80 | 0x20);
    wr(1, 0x0B, 0x40); wr(1, 0x06, 0x00); wr(1, 0x07, 0x08); wr(1, 0x05, 0x08); wr(1, 0x0E, 0xC0);
    wr(0, 0x09, 0x00);
    t2_tick(0);
    CHECK(g_irq == true, "T2(ch0) asserts shared /IRQ");
    t1_tick(1);
    CHECK(g_irq == true, "T2 + Timer1 -> /IRQ");
    rd(0, 0x08);
    CHECK(g_irq == true, "clearing T2 leaves Timer1 /IRQ");
    rd(1, 0x04);
    CHECK(g_irq == false, "clearing both releases /IRQ");

    g_phi2 = 0;
}

void test_via_ports_irq(void) {
    mb_via_reset(true);
    wr(1, 0x0C, 0x0C); CHECK(rd(1, 0x0C) == 0x0C, "PCR round-trips (SSI-263 CA1 neg-edge config)");
    wr(0, 0x0C, 0xB0); CHECK(rd(0, 0x0C) == 0xB0, "PCR round-trips (SC-01 CB2-pulse config)");

    wr(0, 0x0E, 0x80 | 0x19);
    CHECK((rd(0, 0x0E) & 0x19) == 0x19, "IER accepts CA2/CB1/CB2 enables");
    CHECK((rd(0, 0x0D) & 0x19) == 0x00, "CA2/CB1/CB2 flags never set (pins unconnected)");
    CHECK(g_irq == false, "no /IRQ from the unconnected CA2/CB1/CB2 lines");

    wr(0, 0x02, 0xFF);
    wr(0, 0x0C, 0xB0);
    wr(0, 0x0D, 0x10);
    wr(0, 0x00, 0x3F);
    CHECK((rd(0, 0x0D) & 0x10) == 0x00, "SC-01 probe: CB1 flag stays clear -> correctly not found");

    mb_via_reset(true);
    wr(0, 0x0E, 0x82); ar_pulse(0);
    wr(0, 0x03, 0xFF);
    wr(0, 0x0F, 0x5C);
    CHECK((rd(0, 0x0D) & 0x02) != 0, "writing ORA_NH ($F) does not clear CA1");
    CHECK(rd(0, 0x01) == 0x5C, "writing ORA_NH updates the port A output latch");

    mb_via_reset(true);
    wr(1, 0x0A, 0x5A); CHECK(rd(1, 0x0A) == 0x5A, "SR register round-trips");
    wr(1, 0x0A, 0xA5); CHECK(rd(1, 0x0A) == 0xA5, "SR register round-trips (2)");
    wr(0, 0x0E, 0x80 | 0x04);
    CHECK((rd(0, 0x0D) & 0x04) == 0, "SR flag does not auto-fire (engine not modeled)");
    CHECK(g_irq == false, "no /IRQ from the unused SR");
    wr(0, 0x0B, 0x1C);
    wr(0, 0x0A, 0x00);
    CHECK(g_irq == false, "writing SR in a shift mode stays quiet (no false IRQ)");
}

void test_ay(void) {
    wr(0, 0x01, 0x07);
    wr(0, 0x00, 0x07);
    CHECK(g_ay_op == AY_OP_REG && g_ay_ch == 0 && g_ay_val == 0x07, "ORB=0x07 latches register index");
    wr(0, 0x01, 0x3E);
    wr(0, 0x00, 0x06);
    CHECK(g_ay_op == AY_OP_DATA && g_ay_val == 0x3E, "ORB=0x06 writes data");
    wr(1, 0x00, 0x00);
    CHECK(g_ay_op == AY_OP_RESET && g_ay_ch == 1, "ORB PB2=0 resets the AY");

    mb_via_reset(true);
    wr(0, 0x03, 0xFF);
    wr(0, 0x01, 0x02); wr(0, 0x00, 0x07); wr(0, 0x00, 0x04);
    wr(0, 0x01, 0xC5); wr(0, 0x00, 0x06); wr(0, 0x00, 0x04);
    wr(0, 0x03, 0x00);
    wr(0, 0x00, 0x05);
    CHECK(rd(0, 0x01) == 0xC5, "AY read-back returns the last-written register value (ORA)");
    CHECK(rd(0, 0x0F) == 0xC5, "AY read-back via ORA_NH ($F) too (no CA handshake)");

    wr(0, 0x03, 0xFF);
    wr(0, 0x01, 0x01); wr(0, 0x00, 0x07); wr(0, 0x00, 0x04);
    wr(0, 0x01, 0xFF); wr(0, 0x00, 0x06); wr(0, 0x00, 0x04);
    wr(0, 0x03, 0x00);
    wr(0, 0x00, 0x05);
    CHECK(rd(0, 0x01) == 0x0F, "AY read-back masks unused bits (reg 1 = 4 bits)");

    wr(0, 0x03, 0xF0);
    wr(0, 0x01, 0xAB);
    CHECK(rd(0, 0x01) == ((0xAB & 0xF0) | (0x0F & 0x0F)),
          "ORA read mixes output-latch bits with AY input bits per DDRA");

    wr(0, 0x03, 0xFF);
    wr(0, 0x01, 0x02); wr(0, 0x00, 0x07); wr(0, 0x00, 0x04);
    wr(0, 0x01, 0x33); wr(0, 0x00, 0x06); wr(0, 0x00, 0x04);
    wr(0, 0x00, 0x00);
    wr(0, 0x03, 0x00);
    wr(0, 0x00, 0x05);
    CHECK(rd(0, 0x01) == 0xFF, "AY read after reset floats (latched address invalidated)");
    wr(0, 0x01, 0x42); wr(0, 0x00, 0x06); wr(0, 0x00, 0x04);
    wr(0, 0x01, 0x02); wr(0, 0x00, 0x07); wr(0, 0x00, 0x04);
    wr(0, 0x00, 0x05); wr(0, 0x03, 0x00);
    CHECK(rd(0, 0x01) == 0x00, "data write after reset lands nowhere; reg 2 still reset-cleared");
}

void test_via_reset(void) {
    mb_via_reset(true);
    wr(0, 0x06, 0x34); wr(0, 0x07, 0x12);
    wr(1, 0x0A, 0x9C);
    wr(0, 0x03, 0xFF); wr(0, 0x0B, 0xC0); wr(0, 0x0C, 0x0C); wr(0, 0x0E, 0xC0);
    mb_via_reset(true);
    CHECK(rd(0, 0x06) == 0x34 && rd(0, 0x07) == 0x12, "reset preserves the T1 latch");
    CHECK(rd(1, 0x0A) == 0x9C, "reset preserves the SR");
    CHECK(rd(0, 0x03) == 0x00, "reset clears DDRA");
    CHECK(rd(0, 0x0B) == 0x00, "reset clears ACR");
    CHECK(rd(0, 0x0C) == 0x00, "reset clears PCR");
    CHECK(rd(0, 0x0D) == 0x00, "reset clears IFR");
    CHECK((rd(0, 0x0E) & 0x7F) == 0x00, "reset clears IER");
}

void test_speech_bus(void) {
    g_sp_count = 0; wra(0x20, 0x11);
    CHECK(g_sp_chip == 0 && g_sp_reg == 0 && g_sp_val == 0x11 && g_sp_count == 1,
          "$Cn20 -> chip0 reg0");
    wra(0x25, 0x22);
    CHECK(g_sp_chip == 0 && g_sp_reg == 5 && g_sp_val == 0x22, "$Cn25 -> chip0 reg5 (A2:A0)");
    wra(0x2C, 0x33);
    CHECK(g_sp_chip == 0 && g_sp_reg == 4 && g_sp_val == 0x33, "$Cn2C -> chip0 reg4 (A3 ignored)");
    wra(0x40, 0x44);
    CHECK(g_sp_chip == 1 && g_sp_reg == 0 && g_sp_val == 0x44, "$Cn40 -> chip1");
    wra(0xA0, 0x55);
    CHECK(g_sp_chip == 0 && g_sp_val == 0x55, "$CnA0 -> chip0 (A7 ignored for speech)");

    g_sp_count = 0; wra(0x60, 0x66);
    CHECK(g_sp_count == 2 && g_sp_val == 0x66, "$Cn60 writes BOTH chips");

    g_sp_count = 0; wr(0, 0x01, 0xAB);
    CHECK(g_sp_count == 0, "$Cn01 stays VIA (not speech)");

    /* status read D7 = A/R (SSI-263A datasheet) */
    g_ar_mirror[0] = 1; CHECK(rda(0x20) == 0x80, "$Cn20 read: A/R requesting -> D7=1");
    g_ar_mirror[0] = 0; CHECK(rda(0x20) == 0x00, "$Cn20 read: A/R idle/busy -> D7=0");
    g_ar_mirror[0] = 1; CHECK(rda(0x23) == 0x80, "$Cn23 read == $Cn20 (A2:A0 ignored)");
    g_ar_mirror[1] = 1; CHECK(rda(0x40) == 0x80, "$Cn40 read -> chip1 A/R requesting -> D7=1");
}

void test_via_ca1(void) {
    mb_via_reset(true);
    wr(0, 0x0E, 0x80 | 0x02);
    ar_pulse(0);
    CHECK(g_irq == true, "A/R event asserts /IRQ when CA1 IER enabled");
    CHECK((rd(0, 0x0D) & 0x02) != 0, "IFR bit1 (CA1) set after A/R event");
    rd(0, 0x01);
    CHECK(g_irq == false, "reading ORA clears CA1 /IRQ");
    CHECK((rd(0, 0x0D) & 0x02) == 0, "reading ORA clears IFR bit1");
    ar_pulse(0); wr(0, 0x0D, 0x02);
    CHECK(g_irq == false, "IFR write-1 clears CA1");
    ar_pulse(0); wr(0, 0x01, 0x00);
    CHECK(g_irq == false, "writing ORA clears CA1");

    wr(0, 0x0E, 0x02);
    ar_pulse(0);
    CHECK((rd(0, 0x0D) & 0x02) != 0, "CA1 latches with IER disabled");
    CHECK(g_irq == false, "no /IRQ when CA1 IER disabled");
    rd(0, 0x01);

    wr(0, 0x0E, 0x82); ar_pulse(0);
    rd(0, 0x0F);
    CHECK((rd(0, 0x0D) & 0x02) != 0, "ORA_NH read does NOT clear CA1");
    rd(0, 0x01);

    mb_via_reset(true);
    wr(0, 0x0E, 0x82);
    wr(1, 0x0B, 0x40);
    wr(1, 0x06, 0x00); wr(1, 0x07, 0x08); wr(1, 0x05, 0x08); wr(1, 0x0E, 0xC0);
    ar_pulse(0);
    CHECK(g_irq == true, "CA1(ch0) asserts shared /IRQ");
    t1_tick(1);
    CHECK(g_irq == true, "CA1 + Timer1 -> /IRQ");
    rd(0, 0x01);
    CHECK(g_irq == true, "clearing CA1 leaves Timer1 /IRQ");
    rd(1, 0x04);
    CHECK(g_irq == false, "clearing both releases /IRQ");

    mb_via_reset(true);
    wr(0, 0x0E, 0x82);
    mb_via_poll();
    CHECK(g_irq == false, "A/R poll: no edge-count change -> no /IRQ");
    g_ar_edges_mirror[0]++; mb_via_poll();
    CHECK(g_irq == true && (rd(0, 0x0D) & 0x02) != 0, "A/R poll: edge-count advance latches CA1 -> /IRQ");
    rd(0, 0x01);
    mb_via_poll();
    CHECK((rd(0, 0x0D) & 0x02) == 0, "A/R poll: no new edge does not re-latch");
    g_ar_edges_mirror[0] += 2; mb_via_poll();
    CHECK((rd(0, 0x0D) & 0x02) != 0, "A/R poll: coalesced edges still latch CA1");
    rd(0, 0x01);
    g_ar_edges_mirror[1]++; mb_via_poll();
    CHECK((rd(1, 0x0D) & 0x02) != 0, "A/R poll: chip1 edge-count advance latches ch1 CA1");
}

void test_version(void) {
    mb_via_reset(true);
    mb_info_select_page(MB_INFO_PAGE_VERSION);

    {
        size_t vlen = strlen(MB_VERSION_STR);
        int ok = 1;
        for (int i = 0; i < MB_VERSION_STR_LEN; i++) {
            uint8_t want = (i < (int)vlen) ? (uint8_t)MB_VERSION_STR[i] : (uint8_t)' ';
            wr(0, 0x0A, (uint8_t)i);
            if (rd(0, 0x0A) != want || rd(0, 0x0A) != want) ok = 0;
        }
        CHECK(ok, "info: a VIA1 SR index write stages that version byte, and repeated reads return it");
    }
    wr(0, 0x0A, MB_VERSION_STR_LEN);
    CHECK(rd(0, 0x0A) == MB_INFO_NONE, "info: an index past the version field stages 0xFF");

    wr(0, 0x0A, 3);
    CHECK(mb_rtable[0x0A] == (uint8_t)MB_VERSION_STR[3], "info: the staged byte is in the response table");
    wr(0, 0x00, 0x04);
    wr(0, 0x01, 0x55);
    CHECK(rd(0, 0x0A) == (uint8_t)MB_VERSION_STR[3], "info: AY activity leaves the staged byte alone");
    mb_via_reset(true);
    CHECK(rd(0, 0x0A) == (uint8_t)MB_VERSION_STR[3], "info: /RES preserves the staged byte (SR survives reset)");

    wr(1, 0x0A, 0x42);
    CHECK(rd(1, 0x0A) == 0x42, "info: VIA2 SR stores and reads back the written value");
    wr(0, 0x0A, 0);
    CHECK(rd(0, 0x0A) == MB_INFO_NONE, "info: a VIA2 SR write selects the page (unknown page -> 0xFF)");
    wr(1, 0x0A, MB_INFO_PAGE_VERSION);
    wr(0, 0x0A, 0);
    CHECK(rd(0, 0x0A) == (uint8_t)MB_VERSION_STR[0], "info: selecting the version page restores it");
    {
        int nonzero = 1;
        for (int pg = 0; pg < 4; pg++) {
            wr(1, 0x0A, (uint8_t)pg);
            for (int i = 0; i < 256; i++) {
                wr(0, 0x0A, (uint8_t)i);
                if (rd(0, 0x0A) == 0) nonzero = 0;
            }
        }
        CHECK(nonzero, "info: no staged byte is ever 0 (0 is a missed read)");
    }
    wr(1, 0x0A, MB_INFO_PAGE_VERSION);
}

void test_mb_audit(void) {
    /* mb-audit behavioral conformance: value-checks ported from Tom Charlesworth's mb-audit */
    for (int ch = 0; ch < 2; ch++) {
        mb_via_reset(true);
        int ddr_ok = 1;
        for (int n = 0; n < 256; n++) {
            wr(ch, 0x02, (uint8_t)n);
            wr(ch, 0x03, (uint8_t)~n);
            if (rd(ch, 0x02) != (uint8_t)n || rd(ch, 0x03) != (uint8_t)~n) { ddr_ok = 0; break; }
        }
        CHECK(ddr_ok, "mb-audit Datalines2: DDRB/DDRA read back every written value");

        mb_via_reset(true);
        g_phi2 = 1000; g_irq = false;
        wr(ch, 0x06, 0x01); wr(ch, 0x07, 0x00);
        wr(ch, 0x0B, 0x00);
        wr(ch, 0x05, 0x00);
        CHECK((rd(ch, 0x0D) & 0x40) == 0, "mb-audit IRQ: IFR.T1 clear right after arm");
        CHECK(!g_irq, "mb-audit IRQ: /IRQ not asserted before underflow");
        t1_tick(ch);
        CHECK((rd(ch, 0x0D) & 0x40) != 0, "mb-audit IRQ: IFR.T1 set on one-shot underflow");
        CHECK(!g_irq, "mb-audit IRQ: IFR.T1 alone does not assert /IRQ (IER disabled)");
        wr(ch, 0x0E, 0x80 | 0x40);
        CHECK(g_irq, "mb-audit IRQ: enabling IER.T1 with IFR.T1 set asserts /IRQ");
        (void)rd(ch, 0x04);
        CHECK((rd(ch, 0x0D) & 0x40) == 0, "mb-audit IRQ: reading T1C-L clears IFR.T1");
        CHECK(!g_irq, "mb-audit IRQ: /IRQ releases once IFR.T1 cleared");

        mb_via_reset(true);
        wr(ch, 0x0B, 0x40);
        g_phi2 = 5000;
        wr(ch, 0x06, 0x00); wr(ch, 0x07, 0x10);
        wr(ch, 0x05, 0x10);
        g_phi2 = 5001 + 0x1002;
        CHECK(((rd(ch, 0x05) << 8) | rd(ch, 0x04)) == 0x1000,
              "mb-audit continuous: counter reloads to the latch after one period");

        mb_via_reset(true);
        wr(ch, 0x08, 0x05); wr(ch, 0x09, 0x00);
        t2_tick(ch);
        CHECK((rd(ch, 0x0D) & 0x20) != 0, "mb-audit T2: IFR.T2 set on one-shot underflow");
        (void)rd(ch, 0x08);
        CHECK((rd(ch, 0x0D) & 0x20) == 0, "mb-audit T2: reading T2C-L clears IFR.T2");

        mb_via_reset(true);
        wr(ch, 0x03, 0xFF);
        wr(ch, 0x01, 0x02); wr(ch, 0x00, 0x07); wr(ch, 0x00, 0x04);
        wr(ch, 0x01, 0xC5); wr(ch, 0x00, 0x06); wr(ch, 0x00, 0x04);
        wr(ch, 0x03, 0x00); wr(ch, 0x00, 0x05);
        CHECK(rd(ch, 0x01) == 0xC5, "mb-audit AY: read-back returns the last written value");
        wr(ch, 0x03, 0xFF);
        wr(ch, 0x01, 0x06); wr(ch, 0x00, 0x07); wr(ch, 0x00, 0x04);
        wr(ch, 0x01, 0xFF); wr(ch, 0x00, 0x06); wr(ch, 0x00, 0x04);
        wr(ch, 0x03, 0x00); wr(ch, 0x00, 0x05);
        CHECK(rd(ch, 0x01) == 0x1F, "mb-audit AY: register 6 masks to 5 bits on read-back");
        wr(ch, 0x00, 0x00);
        wr(ch, 0x01, 0x42); wr(ch, 0x00, 0x06); wr(ch, 0x00, 0x04);
        wr(ch, 0x01, 0x06); wr(ch, 0x00, 0x07); wr(ch, 0x00, 0x04);
        wr(ch, 0x03, 0x00); wr(ch, 0x00, 0x05);
        CHECK(rd(ch, 0x01) == 0x00, "mb-audit AY: write after reset lands nowhere (latch invalidated)");
    }
    mb_via_reset(true);
}

#define DS   "W65C22 datasheet"
#define MBA  "mb-audit chip-6522.a"

static void t1_arm_at(int ch, uint16_t n, bool freerun, uint32_t w) {
    wr(ch, 0x0B, freerun ? 0x40 : 0x00);
    wr(ch, 0x06, (uint8_t)n);
    g_phi2 = w;
    wr(ch, 0x05, (uint8_t)(n >> 8));
}
static void t1_loaded(int ch) { wr(ch, 0x05, 0x10); }
static void t2_loaded(int ch) { wr(ch, 0x09, 0x10); }

static uint16_t t1_at(int ch, uint32_t cycle) { g_phi2 = cycle; return rd16(ch, 0x04); }
static uint16_t t2_at(int ch, uint32_t cycle) { g_phi2 = cycle; return rd16(ch, 0x08); }

static uint32_t t1_find_zero(int ch, uint32_t from, uint32_t limit) {
    for (uint32_t c = from; c < from + limit; c++) if (t1_at(ch, c) == 0) return c;
    return 0;
}

static void counters_phase_independent(void) {
    const uint16_t ns[] = { 0, 1, 2, 5, 0x100, 0x1234 };
    for (unsigned i = 0; i < sizeof ns / sizeof ns[0]; i++) {
        uint16_t n = ns[i];
        for (int ch = 0; ch < 2; ch++) {
            mb_via_reset(true);
            t1_arm_at(ch, n, true, 1000);
            uint32_t z = t1_find_zero(ch, 1001, (uint32_t)n + 4);
            bool ok = z != 0 && t1_at(ch, z + 1) == 0xFFFF && t1_at(ch, z + 2) == n
                      && t1_at(ch, z + 2 + (uint32_t)n + 2) == n && t1_at(ch, z + (uint32_t)n + 2) == 0;
            for (uint32_t k = 0; ok && k <= n && k < 64; k++) ok = (t1_at(ch, z - k) == k);
            CHECK_SPEC(ok, MBA " T6522_A/E/F", "free-run Timer1 reads N..1, 0, $FFFF, N: a period of N+2");
        }
    }

    mb_via_reset(true);
    t1_arm_at(0, 0, true, 500);
    bool only = true;
    for (uint32_t c = 501; c < 501 + 40000; c += 37) {
        uint16_t v = t1_at(0, c);
        only &= (v == 0 || v == 0xFFFF);
    }
    CHECK_SPEC(only, MBA " T6522_B", "T1L=$0000 reads only $00/$FF");

    mb_via_reset(true);
    t1_arm_at(0, 0xFFFF, true, 100);
    uint8_t a = (g_phi2 = 300, rd(0, 0x04));
    uint8_t b = (g_phi2 = 308, rd(0, 0x04));
    CHECK_SPEC((uint8_t)(a - b) == 8, MBA " SF_GetTimerL", "T1C-L drops by exactly the cycles elapsed");

    mb_via_reset(true);
    wr(0, 0x08, 0x05); g_phi2 = 2000; wr(0, 0x09, 0x00);
    uint32_t z = 0;
    for (uint32_t c = 2000; c < 2010; c++) if (t2_at(0, c) == 0) { z = c; break; }
    CHECK_SPEC(z && t2_at(0, z + 1) == 0xFFFF && t2_at(0, z + 2) == 0xFFFE && t2_at(0, z + 300) == (uint16_t)(0 - 300),
               DS " 2.9", "Timer2 rolls past $0000 to $FFFF and keeps counting down");

    mb_via_init();
    uint8_t c0 = (g_phi2 += 777, rd(1, 0x08));
    uint8_t c1 = (g_phi2 += 8, rd(1, 0x08));
    CHECK_SPEC((uint8_t)(c0 - c1) == 8, MBA " SF_GetTimerL", "Timer2 counts down without being loaded");
}

static void counters_phase(void) {
    mb_via_reset(true);
    t1_arm_at(0, 0x10, true, 4000);
    CHECK_SPEC(t1_at(0, 4001) == 0x10, MBA " T6522_F/T6522_10; " DS " 2.6",
               "Timer1 reads N one cycle after the T1C-H write");
    mb_via_reset(true);
    wr(0, 0x08, 0x10); g_phi2 = 4000; wr(0, 0x09, 0x00);
    CHECK_SPEC(t2_at(0, 4001) == 0x10, MBA " T6522_D/T6522_11",
               "Timer2 reads N one cycle after the T2C-H write");
    CHECK_SPEC(g_t2_arm_w[0] == 4000 && g_t2_arm_n[0] == 0x10, MBA " T6522_11",
               "Timer2 is loaded on the T2C-H write's own cycle");
}

static void one_shot(void) {
    mb_via_reset(true);
    t1_arm_at(0, 4, false, 6000);
    uint32_t z = t1_find_zero(0, 6000, 8);
    CHECK_SPEC(z && t1_at(0, z + 1) == 0xFFFF, MBA " T6522_A", "one-shot Timer1 rolls from $0000 to $FFFF");
    CHECK_SPEC(t1_at(0, z + 2) == 4, MBA " T6522_9/T6522_A",
               "one-shot Timer1 reloads the latch after underflow (only the interrupt is one-shot)");

    mb_via_reset(true);
    t1_arm_at(0, 8, false, 100);
    t1_tick(0);
    (void)rd(0, 0x04);
    t1_tick(0);
    CHECK_SPEC((rd(0, 0x0D) & 0x40) == 0, DS " 2.6; " MBA " T6522_0", "one-shot Timer1 raises no second interrupt");

    mb_via_reset(true);
    t1_arm_at(0, 0x4000, true, 100);
    wr(0, 0x0B, 0x00);
    t1_tick(0);
    CHECK_SPEC((rd(0, 0x0D) & 0x40) != 0, MBA " T6522_1",
               "a free-run -> one-shot switch still raises the next Timer1 flag");
    (void)rd(0, 0x04);
    t1_tick(0);
    CHECK_SPEC((rd(0, 0x0D) & 0x40) == 0, MBA " T6522_1",
               "...and no flag at the timeouts after that");

    mb_via_reset(true);
    t1_arm_at(0, 0, true, 100);
    CHECK_SPEC(g_t1_arm_n[0] == 0, MBA " T6522_17", "T1L=$0000 free-run interrupts every 2 cycles");
    mb_via_reset(true);
    t1_arm_at(0, 1, true, 100);
    CHECK_SPEC(g_t1_arm_n[0] == 1, MBA " T6522_A", "T1L=$0001 free-run interrupts every 3 cycles");
}

static void ifr_ier(void) {
    mb_via_reset(true);
    wr(0, 0x0E, 0x80 | 0x41);
    wr(0, 0x0E, 0x80 | 0x20);
    CHECK_SPEC(rd(0, 0x0E) == 0xE1, DS " Table 2-12", "IER set mode ORs in the 1 bits, read shows bit 7 = 1");
    wr(0, 0x0E, 0x01);
    CHECK_SPEC(rd(0, 0x0E) == 0xE0, DS " Table 2-12", "IER clear mode clears only the 1 bits");

    mb_via_reset(true);
    wr(0, 0x0E, 0x80 | 0x42);
    t1_loaded(0); t1_tick(0); ar_pulse(0);
    wr(0, 0x0D, 0x80);
    CHECK_SPEC((rd(0, 0x0D) & 0x42) == 0x42, DS " 2.14", "writing 1 to IFR bit 7 clears no flag");
    wr(0, 0x0D, 0x40);
    CHECK_SPEC((rd(0, 0x0D) & 0x42) == 0x02 && g_irq, DS " 2.14", "writing 1 to an IFR bit clears only that flag");
    wr(0, 0x0D, 0x02);
    CHECK_SPEC(rd(0, 0x0D) == 0x00 && !g_irq, DS " 2.14", "clearing the last enabled flag drops IFR bit 7 and /IRQ");

    mb_via_reset(true);
    t1_loaded(0); t1_tick(0);
    CHECK_SPEC((rd(0, 0x0D) & 0x40) && !g_irq, DS " 2.14", "a disabled flag still latches in IFR without /IRQ");
    KNOWN_DISCREPANCY((rd(0, 0x0D) & 0x80) == 0, DS " 2.14, Table 2-11",
                      "IFR bit 7 stays 0 while the only set flag is disabled");
    wr(0, 0x0E, 0x80 | 0x40);
    CHECK_SPEC((rd(0, 0x0D) & 0x80) && g_irq, DS " 2.14; " MBA " T6522_2",
               "enabling the flag's IER bit sets IFR bit 7 and asserts /IRQ at once");
    wr(0, 0x0E, 0x40);
    KNOWN_DISCREPANCY((rd(0, 0x0D) & 0x80) == 0 && !g_irq, DS " 2.14",
                      "disabling the IER bit drops IFR bit 7 with the flag still set");
}

static void flag_clears(void) {
    struct { uint8_t off; bool write; bool clears; const char *what; } t1[] = {
        { 0x04, false, true,  "reading T1C-L clears IFR6" },
        { 0x05, false, false, "reading T1C-H leaves IFR6" },
        { 0x06, false, false, "reading T1L-L leaves IFR6" },
        { 0x07, false, false, "reading T1L-H leaves IFR6" },
        { 0x04, true,  false, "writing T1C-L (latch only) leaves IFR6" },
        { 0x06, true,  false, "writing T1L-L leaves IFR6" },
        { 0x07, true,  true,  "writing T1L-H clears IFR6" },
        { 0x05, true,  true,  "writing T1C-H clears IFR6" },
    };
    for (unsigned i = 0; i < sizeof t1 / sizeof t1[0]; i++) {
        mb_via_reset(true);
        t1_loaded(0); t1_tick(0);
        if (t1[i].write) wr(0, t1[i].off, 0x00); else (void)rd(0, t1[i].off);
        bool cleared = (rd(0, 0x0D) & 0x40) == 0;
        CHECK_SPEC(cleared == t1[i].clears, DS " Tables 2-6/2-7; " MBA " T6522_15", t1[i].what);
    }
    struct { uint8_t off; bool write; bool clears; const char *what; } t2[] = {
        { 0x08, false, true,  "reading T2C-L clears IFR5" },
        { 0x09, false, false, "reading T2C-H leaves IFR5" },
        { 0x08, true,  false, "writing T2C-L (latch only) leaves IFR5" },
        { 0x09, true,  true,  "writing T2C-H clears IFR5" },
    };
    for (unsigned i = 0; i < sizeof t2 / sizeof t2[0]; i++) {
        mb_via_reset(true);
        t2_loaded(0); t2_tick(0);
        if (t2[i].write) wr(0, t2[i].off, 0x00); else (void)rd(0, t2[i].off);
        bool cleared = (rd(0, 0x0D) & 0x20) == 0;
        CHECK_SPEC(cleared == t2[i].clears, DS " Table 2-9", t2[i].what);
    }
}

static void decode(void) {
    mb_via_reset(true);
    wra(0x12, 0x5A); wra(0x93, 0xA5);
    CHECK_SPEC(rd(0, 0x02) == 0x5A && rd(1, 0x03) == 0xA5, MBA " T6522_12", "$Cn1x and $Cn9x reach VIA1 and VIA2");
    CHECK_SPEC(rd(0, 0x03) == 0x00 && rd(1, 0x02) == 0x00, MBA " DetectMB4CorEchoPlusorSDM",
               "the two VIAs keep independent registers");

    mb_via_reset(true);
    t1_loaded(0); t2_loaded(0); t1_tick(0); t2_tick(0);
    (void)rda(0x44);
    CHECK_SPEC((rd(0, 0x0D) & 0x40) == 0, "mb-audit chip-ssi263.a Test#1",
               "a read of $Cn44 is also a VIA1 T1C-L read and clears IFR6");
    (void)rda(0x48);
    CHECK_SPEC((rd(0, 0x0D) & 0x20) == 0, "mb-audit chip-ssi263.a Test#1",
               "a read of $Cn48 is also a VIA1 T2C-L read and clears IFR5");

    mb_via_reset(true);
    t1_loaded(1); t1_tick(1);
    (void)rda(0xC4);
    CHECK_SPEC((rd(1, 0x0D) & 0x40) == 0, "Mockingboard v1a schematic: VIA CS1 = A7",
               "a read of $CnC4 is a VIA2 T1C-L read and clears VIA2's IFR6");

    mb_via_reset(true);
    ar_pulse(0);
    (void)rda(0x21);
    CHECK_SPEC((rd(0, 0x0D) & 0x02) == 0, "W65C22 Table 2-5; Mockingboard v1a schematic",
               "a read of $Cn21 is a VIA1 ORA read and clears CA1");

    mb_via_reset(true);
    g_ar_mirror[1] = 1; mb_via_poll();
    t1_loaded(0); t1_tick(0);
    int posts = g_sp_count;
    uint8_t v = rda(0x44);
    CHECK(v == 0x80 && g_sp_count == posts,
          "a speech-range read still serves the chip status and writes nothing to the chip");
    g_ar_mirror[1] = 0; mb_via_poll();
}

/* Willy Byte in the Digital Dimension, from the AppleWin GH#52 trace. */
static void willy_byte_irq(void) {
    mb_via_reset(true);
    wra(0x0B, 0x40); wra(0x0D, 0xC0); wra(0x0E, 0xC0);
    wra(0x04, 0xFF); wra(0x05, 0x40);
    t1_tick(0); wra(0x0D, 0xC0);
    wra(0x0E, 0x02); wra(0x8E, 0x02); wra(0x0B, 0x02);
    t1_tick(0);
    wra(0x8C, 0x0C); wra(0x43, 0x80); wra(0x40, 0xC0);
    wra(0x43, 0x70); wra(0x8E, 0x82);
    ar_pulse(1);
    CHECK(g_irq, "Willy: /IRQ is asserted when the speech handler is entered");

    wra(0x8D, 0x02);
    CHECK(g_irq, "Willy: acking CA1 alone leaves the Timer1 flag holding /IRQ");
    static const uint8_t frame[5] = { 0x28, 0x70, 0xA8, 0x59, 0x14 };
    for (int x = 4; x >= 0; x--) {
        (void)rda((uint8_t)(0x40 + x));
        wra((uint8_t)(0x40 + x), frame[4 - x]);
    }
    CHECK_SPEC(!g_irq && (rd(0, 0x0D) & 0x40) == 0, "mb-audit chip-ssi263.a Test#1; AppleWin GH#52",
               "Willy: the phoneme writes' dummy read of $C444 clears IFR6 and releases /IRQ");
}

static void reset_state(void) {
    mb_via_reset(true);
    wr(0, 0x02, 0xFF); wr(0, 0x03, 0xFF); wr(0, 0x0C, 0x0C);
    t1_arm_at(0, 0x0234, true, 10);
    wr(0, 0x0E, 0x80 | 0x60);
    t1_tick(0);
    CHECK(g_irq, "set-up: a pending enabled Timer1 interrupt");
    mb_via_reset(true);
    CHECK_SPEC(!g_irq && rd(0, 0x0D) == 0x00 && rd(0, 0x0E) == 0x80, MBA " Test6522AfterReset",
               "reset clears the pending interrupt; IER reads $80");
    CHECK_SPEC(rd(0, 0x02) == 0 && rd(0, 0x03) == 0 && rd(0, 0x0B) == 0 && rd(0, 0x0C) == 0,
               DS " 3.9", "reset clears DDRB, DDRA, ACR and PCR");
    CHECK_SPEC(rd(0, 0x00) == 0xFF && rd(0, 0x01) == 0xFF, MBA " Test6522AfterReset", "after reset IRB and IRA read $FF");
    CHECK_SPEC(rd(0, 0x06) == 0x34 && rd(0, 0x07) == 0x02, DS " 3.9; " MBA " Test6522AfterReset",
               "reset preserves the Timer1 latches");
    uint8_t a = (g_phi2 = 50000, rd(0, 0x04)), b = (g_phi2 = 50003, rd(0, 0x04));
    CHECK_SPEC(a != b, DS " 3.9", "reset leaves the Timer1 counter running");
}

void test_via_spec(void) {
    counters_phase_independent();
    counters_phase();
    one_shot();
    ifr_ier();
    flag_clears();
    decode();
    willy_byte_irq();
    reset_state();
    mb_via_reset(true);
    g_phi2 = 0;
}

#undef MBA
#define AYDS "GI AY-3-8910 datasheet"
#define MBA  "mb-audit chip-ay8913.a"

enum { AY_RESET = 0, AY_INACTIVE = 4, AY_READ = 5, AY_WRITE = 6, AY_LATCH = 7 };

static void ay_init(int ch) {
    wr(ch, 0x03, 0xFF); wr(ch, 0x02, 0x07);
    wr(ch, 0x00, AY_RESET); wr(ch, 0x00, AY_INACTIVE);
}
static void ay_set(int ch, uint8_t r, uint8_t v) {
    wr(ch, 0x01, r); wr(ch, 0x00, AY_LATCH); wr(ch, 0x00, AY_INACTIVE);
    wr(ch, 0x01, v); wr(ch, 0x00, AY_WRITE); wr(ch, 0x00, AY_INACTIVE);
}
static uint8_t ay_get(int ch, uint8_t r) {
    wr(ch, 0x01, r); wr(ch, 0x00, AY_LATCH); wr(ch, 0x00, AY_INACTIVE);
    wr(ch, 0x03, 0x00); wr(ch, 0x00, AY_READ);
    uint8_t v = rd(ch, 0x01);
    wr(ch, 0x00, AY_INACTIVE); wr(ch, 0x03, 0xFF);
    return v;
}

static void bus_protocol(void) {
    for (int ch = 0; ch < 2; ch++) {
        mb_via_reset(true);
        ay_init(ch);
        bool data_ok = true;
        for (int n = 0; n < 256; n++) { ay_set(ch, 0, (uint8_t)n); data_ok &= ay_get(ch, 0) == n; }
        CHECK_SPEC(data_ok, MBA " Datalines", "R0 reads back every value written");
        bool addr_ok = true;
        for (uint8_t r = 0; r < 14; r++) { if (r >= 8 && r <= 10) continue; ay_set(ch, r, r); }
        for (uint8_t r = 0; r < 14; r++) { if (r >= 8 && r <= 10) continue; addr_ok &= ay_get(ch, r) == r; }
        CHECK_SPEC(addr_ok, MBA " Addresslines", "each register keeps its own value");

        wr(ch, 0x01, 2); wr(ch, 0x00, AY_LATCH); wr(ch, 0x00, AY_INACTIVE);
        bool latch_ok = true;
        for (int n = 0; n < 32; n++) {
            wr(ch, 0x01, (uint8_t)(n * 7)); wr(ch, 0x00, AY_WRITE); wr(ch, 0x00, AY_INACTIVE);
            wr(ch, 0x03, 0x00); wr(ch, 0x00, AY_READ);
            latch_ok &= rd(ch, 0x01) == (uint8_t)(n * 7);
            wr(ch, 0x00, AY_INACTIVE); wr(ch, 0x03, 0xFF);
        }
        CHECK_SPEC(latch_ok, AYDS " p. 5-19; " MBA " latch", "a latched address serves many writes and reads");

        ay_set(ch, 3, 0x05);
        wr(ch, 0x01, 0x0A); wr(ch, 0x00, AY_INACTIVE);
        CHECK_SPEC(ay_get(ch, 3) == 0x05, AYDS " p. 5-19", "INACTIVE writes nothing");

        ay_set(ch, 1, 0x1F); ay_set(ch, 6, 0xFF); ay_set(ch, 7, 0xFF); ay_set(ch, 8, 0xFF); ay_set(ch, 13, 0xFF);
        CHECK_SPEC(ay_get(ch, 1) == 0x0F && ay_get(ch, 6) == 0x1F && ay_get(ch, 7) == 0x3F
                   && ay_get(ch, 8) == 0x1F && ay_get(ch, 13) == 0x0F, MBA " YM2149/8913 detection",
                   "unused register bits read 0 (8913: R7 reads $3F)");

        ay_set(ch, 2, 0x33);
        wr(ch, 0x01, 2); wr(ch, 0x00, AY_LATCH); wr(ch, 0x00, AY_RESET); wr(ch, 0x00, AY_INACTIVE);
        wr(ch, 0x01, 0x42); wr(ch, 0x00, AY_WRITE); wr(ch, 0x00, AY_INACTIVE);
        bool all_zero = true;
        for (uint8_t r = 0; r < 14; r++) all_zero &= ay_get(ch, r) == 0;
        CHECK_SPEC(all_zero, AYDS " p. 5-19; " MBA " reset", "reset zeroes all registers and the unlatched write is lost");

        wr(ch, 0x00, AY_RESET); wr(ch, 0x00, AY_INACTIVE);
        wr(ch, 0x03, 0x00);
        bool hiz = rd(ch, 0x01) == 0xFF;
        wr(ch, 0x00, AY_READ); hiz &= rd(ch, 0x01) == 0xFF;
        wr(ch, 0x00, AY_INACTIVE); hiz &= rd(ch, 0x01) == 0xFF;
        CHECK_SPEC(hiz, MBA " high-Z", "an undriven port A reads $FF");
    }

    mb_via_reset(true);
    ay_init(0); ay_init(1);
    ay_set(0, 0, 0x11); ay_set(1, 0, 0x33);
    wr(0, 0x00, AY_RESET); wr(0, 0x00, AY_INACTIVE);
    CHECK_SPEC(ay_get(0, 0) == 0x00 && ay_get(1, 0) == 0x33, MBA " reset", "PB2 resets only its own AY");
}

static void discrepancies(void) {
    mb_via_reset(true);
    wr(0, 0x02, 0x00);
    g_ay_op = -1;
    wr(0, 0x00, AY_RESET);
    KNOWN_DISCREPANCY(g_ay_op != AY_OP_RESET, "W65C22 datasheet Table 2-2; " AYDS " p. 5-19",
                      "ORB = 0 with port B set to input does not reset the AY");

    mb_via_reset(true);
    ay_init(0);
    wr(0, 0x01, 2); wr(0, 0x00, AY_LATCH);
    wr(0, 0x01, 5);
    wr(0, 0x00, AY_INACTIVE);
    KNOWN_DISCREPANCY(g_ay_op == AY_OP_REG && g_ay_val == 5, AYDS " p. 5-25 fig. 9",
                      "port A changing inside the latch window latches the new address");

    mb_via_reset(true);
    ay_init(0);
    wr(0, 0x01, 0x12); wr(0, 0x00, AY_LATCH); wr(0, 0x00, AY_INACTIVE);
    wr(0, 0x01, 0x55); wr(0, 0x00, AY_WRITE); wr(0, 0x00, AY_INACTIVE);
    KNOWN_DISCREPANCY(ay_get(0, 2) == 0x00, AYDS " p. 5-19",
                      "a register address with nonzero high nibble deselects the AY");

    mb_via_reset(true);
    ay_init(0);
    ay_set(0, 2, 0x33);
    mb_via_reset(true);
    wr(0, 0x00, AY_INACTIVE);
    wr(0, 0x02, 0x07);
    wr(0, 0x03, 0xFF);
    KNOWN_DISCREPANCY(ay_get(0, 2) == 0x33, "Mockingboard schematic; " AYDS " p. 5-19 (inferred)",
                      "Apple /RES does not reset the AY, so its registers survive");

    mb_via_reset(true);
    ay_init(0);
    ay_set(0, 14, 0x55);
    KNOWN_DISCREPANCY(ay_get(0, 14) != 0x55, MBA " 8913 detection",
                      "AY-3-8913 R14 (no I/O port) does not read back the written value");
}

static void speech_wiring(void) {
    g_sp_count = 0; wra(0x30, 0x11);
    CHECK_SPEC(g_sp_chip == 0 && g_sp_reg == 0 && g_sp_count == 1, "mb-audit chip-ssi263.a Test#A",
               "$Cn30 (A4 set) still selects SSI-263 chip 0, register 0");
    g_sp_count = 0; wra(0xC7, 0x22);
    CHECK_SPEC(g_sp_chip == 1 && g_sp_reg == 7 && g_sp_count == 1, "mb-audit chip-ssi263.a Test#A",
               "$CnC7 (A7 set) selects chip 1, register 7 (an R4 alias)");

    mb_via_reset(true);
    wr(0, 0x0C, 0x0C);
    mb_via_poll();
    g_ar_edges_mirror[0]++; mb_via_poll();
    CHECK_SPEC((rd(0, 0x0D) & 0x02) != 0, "W65C22 Table 2-5; mb-audit KickSSI263", "an A/R request sets CA1 with PCR = $0C");
    rd(0, 0x01);
    wr(0, 0x0C, 0x01);
    g_ar_edges_mirror[0]++; mb_via_poll();
    KNOWN_DISCREPANCY((rd(0, 0x0D) & 0x02) == 0, "W65C22 Table 2-5",
                      "with CA1 set to positive edge, the A/R request (falling) does not set CA1");
    mb_via_reset(true);
}

void test_ay_spec(void) {
    bus_protocol();
    discrepancies();
    speech_wiring();
}

static uint32_t rng = 0x9E3779B9u;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

static bool table_consistent(char *why, size_t cap) {
    for (int ch = 0; ch < 2; ch++)
        for (uint8_t off = 0; off < 16; off++) {
            if (off == 4 || off == 5 || off == 8 || off == 9) continue;
            uint8_t want = mb_test_via_read_value(ch, off), got = mb_rtable[(ch ? 0x80 : 0) | off];
            if (got != want) { snprintf(why, cap, "VIA%d reg %X: table %02X, model %02X", ch + 1, off, got, want); return false; }
        }
    for (unsigned i = 0; i < 256; i++) {
        if ((i & 0x10) || !(i & 0x60)) continue;
        uint8_t want = g_ar_mirror[(i & 0x40) ? 1 : 0] ? 0x80 : 0x00;
        if (mb_rtable[i] != want) { snprintf(why, cap, "speech byte %02X: %02X, A/R wants %02X", i, mb_rtable[i], want); return false; }
    }
    return true;
}

static void sync_property(void) {
    char why[96] = "";
    bool ok = true;
    int step = 0;
    mb_via_reset(true);
    for (; ok && step < 20000; step++) {
        g_phi2 += 1 + (rnd() % 7);
        int ch = (int)(rnd() & 1);
        uint32_t k = rnd() % 100;
        if (k < 45) {
            uint8_t off = (uint8_t)(rnd() & 15), v = (uint8_t)rnd();
            if (off == 0) v = (uint8_t)((v & 0xF8) | (rnd() % 8));
            wr(ch, off, v);
        } else if (k < 80) {
            (void)rd(ch, (uint8_t)(rnd() & 15));
        } else if (k < 86) {
            t1_tick(ch);
        } else if (k < 90) {
            t2_tick(ch);
        } else if (k < 94) {
            ar_pulse(ch);
        } else if (k < 97) {
            g_ar_mirror[ch] ^= 1;
            mb_via_poll();
        } else if (k < 98) {
            mb_via_reset(true);
        } else {
            (void)rda((uint8_t)(0x20 | (rnd() & 0xDF)));
        }
        ok = table_consistent(why, sizeof why);
    }
    if (!ok) printf("  table diverged at step %d: %s\n", step, why);
    CHECK(ok, "after any access, tick, A/R change or reset every table byte matches the model");
    g_ar_mirror[0] = g_ar_mirror[1] = 0;
    mb_via_poll();
}

static void counter_driving(void) {
    mb_via_reset(true);
    g_phi2 = 7000; wr(1, 0x06, 0x00); wr(1, 0x05, 0x00);
    CHECK(g_t1_arm_w[1] == 7000 && g_t1_arm_n[1] == 0x0000, "T1C-H loads Timer1 with the latch on the write's cycle");
    int posts = g_t1_latch_posts[1];
    wr(1, 0x06, 0x34); wr(1, 0x07, 0x12);
    CHECK(g_t1_latch_posts[1] == posts + 2 && g_t1_latch_posted[1] == 0x1234,
          "latch writes post the new latch for the next reload, without reloading now");
    wr(1, 0x04, 0x56);
    CHECK(g_t1_latch_posted[1] == 0x1256, "a T1C-L write is a low-latch write");

    g_phi2 = 8000; wr(0, 0x08, 0x78); wr(0, 0x09, 0x56);
    CHECK(g_t2_arm_w[0] == 8000 && g_t2_arm_n[0] == 0x5678, "T2C-H loads Timer2 on the write's cycle");
    int holds = g_t2_holds[0];
    wr(0, 0x0B, 0x20);
    CHECK(g_t2_holds[0] == holds + 1 && g_t2_held_n[0] == 0x5678 && rd16(0, 0x08) == 0x5678,
          "switching Timer2 to pulse counting holds it at its load value");
    wr(0, 0x09, 0x9A);
    CHECK(g_t2_held_n[0] == 0x9A78 && rd16(0, 0x08) == 0x9A78, "a T2C-H write in pulse mode loads and holds");
    int arms = g_t2_arms[0];
    g_phi2 = 9000; wr(0, 0x0B, 0x00);
    CHECK(g_t2_arms[0] == arms + 1 && g_t2_arm_w[0] == 9000, "leaving pulse mode resumes timed counting");
    wr(0, 0x0B, 0x20);
    arms = g_t2_arms[0];
    mb_via_reset(true);
    CHECK(g_t2_arms[0] == arms + 1, "reset clears ACR, so a held Timer2 counts again");
}

static void ticks(void) {
    mb_via_reset(true);
    wr(0, 0x02, 0x80); wr(0, 0x0B, 0xC0);
    wr(0, 0x06, 0x10); wr(0, 0x05, 0x00);
    CHECK((rd(0, 0x00) & 0x80) == 0x00, "PB7 goes low on the T1C-H load");
    t1_tick(0);
    CHECK((rd(0, 0x0D) & 0x40) && (rd(0, 0x00) & 0x80), "free-run timeout: flag, PB7 high");
    (void)rd(0, 0x04);
    t1_tick(0);
    CHECK((rd(0, 0x0D) & 0x40) && !(rd(0, 0x00) & 0x80), "next free-run timeout: flag again, PB7 low");
    CHECK(mb_rtable[0x0D] == mb_test_via_read_value(0, 0x0D), "a tick updates the IFR byte at once");

    wr(0, 0x0B, 0x80); (void)rd(0, 0x04);
    wr(0, 0x05, 0x00);
    t1_tick(0);
    CHECK((rd(0, 0x0D) & 0x40) && (rd(0, 0x00) & 0x80), "one-shot timeout: flag, PB7 high");
    (void)rd(0, 0x04);
    t1_tick(0);
    CHECK(!(rd(0, 0x0D) & 0x40) && (rd(0, 0x00) & 0x80), "later one-shot timeouts: no flag, PB7 stays high");

    mb_via_reset(true);
    g_arm_passed = true;
    wr(0, 0x05, 0x00);
    CHECK((rd(0, 0x0D) & 0x40) != 0, "a Timer1 load whose timeout passed before the SM ran still flags");
    t1_tick(0); (void)rd(0, 0x04); t1_tick(0);
    CHECK((rd(0, 0x0D) & 0x40) == 0, "...and only once");
    g_arm_passed = true;
    wr(0, 0x09, 0x00);
    CHECK((rd(0, 0x0D) & 0x20) != 0, "a Timer2 load whose timeout passed still flags");
    t2_tick(0);
    (void)rd(0, 0x08); t2_tick(0);
    CHECK((rd(0, 0x0D) & 0x20) == 0, "Timer2 flags once per load");
    mb_via_reset(true);
}

static void version_through_table(void) {
    mb_via_reset(true);
    mb_info_select_page(MB_INFO_PAGE_VERSION);
    wr(0, 0x0A, 1);
    uint8_t a = rd(0, 0x0A), b = rd(0, 0x0A);
    CHECK(a == (uint8_t)MB_VERSION_STR[1] && b == a && mb_rtable[0x0A] == a,
          "an SR index write stages its byte in the table, and reads leave it in place");
}

void test_table(void) {
    sync_property();
    counter_driving();
    ticks();
    version_through_table();
}

/* Areas share the modeled VIA state. Each resets what it depends on or relies on the area before it. */

int g_tests, g_fails, g_xfails;

int main(void) {
    mb_via_init();

    test_selftest();
    test_via_t1();
    test_via_t2();
    test_via_ports_irq();
    test_ay();
    test_via_reset();
    test_speech_bus();
    test_via_ca1();
    test_version();
    test_mb_audit();
    test_via_spec();
    test_ay_spec();
    test_table();

    printf("\n%d/%d checks passed, %d known discrepancies (XFAIL)%s\n",
           g_tests - g_fails - g_xfails, g_tests, g_xfails, g_fails ? "  <-- FAILURES" : "  (all pass)");
    return g_fails ? 1 : 0;
}
