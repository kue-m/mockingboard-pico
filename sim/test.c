/* Unit tests for the real firmware mb_via.c, compiled against host shims. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "mb_via.h"
#include "mb_audio.h"
#include "mb_bus.h"
#include "mb_selftest.h"
#include "mb_version.h"
#include "mb_version_gen.h"

uint32_t mb_test_t1_reduce(uint16_t n, uint32_t e);

static uint8_t  g_wdata, g_rdata;
static bool     g_irq;
static void   (*g_tick)(int);
static uint32_t g_period[2]; static bool g_running[2]; static int g_restart_calls[2];
static int      g_ay_op = -1, g_ay_ch = -1; static uint8_t g_ay_val;

void    mb_bus_irq(bool a)            { g_irq = a; }
uint8_t mb_bus_get_wdata(void)        { return g_wdata; }
void    mb_bus_put_rdata(uint8_t d)   { g_rdata = d; }
void    mb_sync_init(void (*cb)(int)) { g_tick = cb; }
void    mb_sync_restart(int ch, uint32_t n) { g_period[ch] = n; g_running[ch] = (n > 0); g_restart_calls[ch]++; }
void    mb_audio_post(uint32_t c)     { g_ay_op = (c >> 16) & 3; g_ay_ch = (c >> 8) & 1; g_ay_val = c & 0xFF; }

static uint32_t g_phi2 = 0;
uint32_t mb_phi2_ticks(void)          { return g_phi2; }

static void  (*g_t2_cb)(int) = NULL;
static uint32_t g_t2_armed[2]; static bool g_t2_run[2];
void mb_alarm_init(void (*cb)(int))   { g_t2_cb = cb; }
void mb_alarm_arm(int ch, uint32_t n) { g_t2_armed[ch] = n; g_t2_run[ch] = (n > 0); }
int  mb_alarm_irq(void)               { return -1; }
int  mb_ar_tick_init(void (*cb)(void)) { (void)cb; return -1; }

static int     g_sp_chip = -1, g_sp_reg = -1, g_sp_count = 0;
static uint8_t g_sp_val;
static uint8_t g_ar_mirror[2] = { 0, 0 };
static uint8_t g_ar_edges_mirror[2] = { 0, 0 };
void    speech_post(int c, int reg, uint8_t v) { g_sp_chip = c; g_sp_reg = reg; g_sp_val = v; g_sp_count++; }
uint8_t speech_ar(int c)                       { return g_ar_mirror[c & 1]; }
uint8_t speech_ar_edges(int c)                 { return g_ar_edges_mirror[c & 1]; }

static void wr(int ch, uint8_t off, uint8_t d) {
    g_wdata = d; mb_via_access((uint32_t)(((ch ? 0x80 : 0) | (off & 0x0F)) & 0xFF));
}
static uint8_t rd(int ch, uint8_t off) {
    mb_via_access((uint32_t)((((ch ? 0x80 : 0) | (off & 0x0F)) & 0xFF) | (1u << 9)));
    return g_rdata;
}
static void wra(uint8_t addr, uint8_t d) { g_wdata = d; mb_via_access((uint32_t)addr); }
static uint8_t rda(uint8_t addr) { mb_via_access((uint32_t)addr | (1u << 9)); return g_rdata; }

static int fails = 0, tests = 0;
#define CHECK(cond, msg) do { tests++; if (!(cond)) { printf("  FAIL: %s\n", msg); fails++; } } while (0)

int main(void) {
    mb_via_init();

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

    wr(0, 0x0B, 0x40);
    wr(0, 0x06, 0x00);
    wr(0, 0x07, 0x10);
    wr(0, 0x05, 0x10);
    CHECK(g_running[0] && g_period[0] == 0x1000, "T1C-H arms counter with latch period");
    CHECK(g_irq == false, "no /IRQ before IER bit6 enabled");

    wr(0, 0x0E, 0x80 | 0x40);
    CHECK(rd(0, 0x0E) == 0xC0, "IER reads back the enabled bits with bit7 forced to 1");
    g_tick(0);
    CHECK(g_irq == true, "Timer1 tick asserts /IRQ when enabled");
    CHECK((rd(0, 0x0D) & 0x40) != 0, "IFR bit6 set after tick");
    CHECK((rd(0, 0x0D) & 0x80) != 0, "IFR bit7 (any) set after tick");

    rd(0, 0x04);
    CHECK(g_irq == false, "reading T1C-L releases /IRQ");
    CHECK((rd(0, 0x0D) & 0x40) == 0, "reading T1C-L clears IFR bit6");

    wr(0, 0x0E, 0x40);
    g_tick(0);
    CHECK((rd(0, 0x0D) & 0x40) != 0, "IFR bit6 still latches when IER disabled");
    CHECK(g_irq == false, "no /IRQ when IER bit6 disabled");
    rd(0, 0x04);

    wr(1, 0x0B, 0x40);
    wr(1, 0x06, 0x00); wr(1, 0x07, 0x08); wr(1, 0x05, 0x08);
    wr(0, 0x0E, 0xC0); wr(1, 0x0E, 0xC0);
    g_tick(1);
    CHECK(g_irq == true, "VIA2 tick asserts shared /IRQ");
    g_tick(0);
    CHECK(g_irq == true, "both pending -> /IRQ still asserted");
    rd(1, 0x04);
    CHECK(g_irq == true, "clearing VIA2 leaves /IRQ (VIA1 still pending)");
    rd(0, 0x04);
    CHECK(g_irq == false, "clearing both releases /IRQ");

    CHECK(g_period[0] == 0x1000 && g_period[1] == 0x0800, "two VIAs hold independent periods");

    mb_via_reset(true);
    wr(0, 0x0B, 0x40);
    g_phi2 = 1000;
    wr(0, 0x06, 0x00); wr(0, 0x07, 0x10); wr(0, 0x05, 0x10);
    CHECK(rd(0, 0x05) == 0x10 && rd(0, 0x04) == 0x00, "T1 counter reads the latch (0x1000) at arm");
    g_phi2 = 1000 + 0x0200;
    CHECK((((uint16_t)rd(0, 0x05) << 8) | rd(0, 0x04)) == 0x1000 - 0x0200,
          "T1 counter decremented by the elapsed PHI2 count");

    g_phi2 = 5000;
    wr(0, 0x06, 0xFF); wr(0, 0x07, 0xFF); wr(0, 0x05, 0xFF);
    { uint8_t a = rd(0, 0x04); g_phi2 += 5; uint8_t b = rd(0, 0x04);
      CHECK((uint8_t)(a - b) == 5, "detection: T1C-L drops by exactly 5 over 5 PHI2 cycles"); }

    g_phi2 = 7000;
    wr(0, 0x0B, 0x40); wr(0, 0x06, 0x04); wr(0, 0x07, 0x00); wr(0, 0x05, 0x00);
    g_phi2 = 7000 + 4;
    CHECK((((uint16_t)rd(0, 0x05) << 8) | rd(0, 0x04)) == 0x0000, "free-run counter hits 0 at phase N");
    g_phi2 = 7000 + 5;
    CHECK((((uint16_t)rd(0, 0x05) << 8) | rd(0, 0x04)) == 0xFFFF, "free-run shows 0xFFFF underflow cycle");
    g_phi2 = 7000 + 6;
    CHECK((((uint16_t)rd(0, 0x05) << 8) | rd(0, 0x04)) == 0x0004, "free-run reloads to N after N+2 cycles");

    mb_via_reset(true);
    wr(0, 0x0B, 0x40); wr(0, 0x0E, 0xC0);
    wr(0, 0x06, 0x00); wr(0, 0x07, 0x08); wr(0, 0x05, 0x08);
    g_tick(0);
    rd(0, 0x05);
    CHECK((rd(0, 0x0D) & 0x40) != 0, "reading T1C-H leaves the Timer1 flag set");
    rd(0, 0x04);
    CHECK((rd(0, 0x0D) & 0x40) == 0, "reading T1C-L then clears it");

    CHECK(g_running[0], "free-run keeps the PIO counter running after underflow");

    /* T1L-H write clears IFR6 (W65C22 Tbl 2-7), T1L-L does not: the Ultima V "Activate Music" freeze */
    mb_via_reset(true);
    wr(0, 0x0B, 0x40); wr(0, 0x0E, 0xC0);
    wr(0, 0x06, 0x9A); wr(0, 0x07, 0x42); wr(0, 0x05, 0x42);  /* latch 0x429A = Ultima V's ~60 Hz period */
    g_tick(0);
    CHECK((rd(0, 0x0D) & 0x40) != 0 && g_irq == true, "Ultima V: Timer1 tick sets flag + asserts /IRQ");
    wr(0, 0x06, 0x9A);
    CHECK((rd(0, 0x0D) & 0x40) != 0, "writing T1L-L ($06) leaves the Timer1 flag set");
    CHECK(g_irq == true, "writing T1L-L does not release /IRQ");
    wr(0, 0x07, 0x42);
    CHECK((rd(0, 0x0D) & 0x40) == 0, "writing T1L-H ($07) clears the Timer1 flag (Ultima V ack path)");
    CHECK(g_irq == false, "writing T1L-H releases /IRQ (no interrupt storm -> no freeze)");

    /* reading T1L-H does not clear IFR6. The T1C-L-read ack (Ultima III/IV) stays intact */
    g_tick(0);
    CHECK((rd(0, 0x0D) & 0x40) != 0, "re-tick sets the Timer1 flag again");
    rd(0, 0x07);
    CHECK((rd(0, 0x0D) & 0x40) != 0, "reading T1L-H leaves the Timer1 flag set");
    CHECK(g_irq == true, "reading T1L-H does not release /IRQ");
    rd(0, 0x04);
    CHECK((rd(0, 0x0D) & 0x40) == 0, "reading T1C-L still clears the flag (Ultima III/IV path intact)");

    /* free-run re-rates the PIO at the next underflow, not the arm ("first song too fast" otherwise) */
    mb_via_reset(true);
    g_restart_calls[0] = 0;
    wr(0, 0x0B, 0x40); wr(0, 0x0E, 0xC0);
    wr(0, 0x06, 0x45); wr(0, 0x07, 0x37); wr(0, 0x05, 0x37);
    CHECK(g_period[0] == 0x3745, "re-rate: PIO clocked at the arm latch A");
    CHECK(g_restart_calls[0] == 1, "re-rate: the arm is a single restart");
    g_tick(0);
    CHECK(g_period[0] == 0x3745 && g_restart_calls[0] == 1,
          "re-rate: unchanged latch -> no spurious restart, period held");
    wr(0, 0x06, 0x9A); wr(0, 0x07, 0x42);
    CHECK(g_period[0] == 0x3745,
          "re-rate: a bare latch write does NOT re-rate mid-period (reload is at underflow)");
    g_tick(0);
    CHECK(g_period[0] == 0x429A && g_restart_calls[0] == 2,
          "re-rate: the next underflow adopts the new latch B (one restart)");
    g_tick(0);
    CHECK(g_period[0] == 0x429A && g_restart_calls[0] == 2,
          "re-rate: a stable latch after the change -> no further restart");

    g_phi2 = 20000; mb_via_reset(true); g_restart_calls[0] = 0;
    wr(0, 0x0B, 0x40);
    wr(0, 0x06, 0x04); wr(0, 0x07, 0x00); wr(0, 0x05, 0x00);
    wr(0, 0x06, 0x08); wr(0, 0x07, 0x00);
    g_phi2 = 20050; g_tick(0);
    CHECK(g_period[0] == 0x0008 && g_restart_calls[0] == 2, "re-rate: PIO adopts N=8 at underflow");
    g_phi2 = 20050; CHECK((((uint16_t)rd(0, 0x05) << 8) | rd(0, 0x04)) == 0x0008,
          "re-rate: readable counter reloads to the new N at the re-rate epoch");
    g_phi2 = 20053; CHECK((((uint16_t)rd(0, 0x05) << 8) | rd(0, 0x04)) == 0x0005,
          "re-rate: readable counter counts down from the new N");

    g_phi2 = 50; mb_via_init();
    CHECK(rd(0, 0x0B) == 0x00, "cold init: ACR = 0 (one-shot), not free-run");
    CHECK(rd(0, 0x06) == 0xFF && rd(0, 0x07) == 0xFF, "cold init: T1 latch = 0xFFFF");
    { g_phi2 = 100; uint8_t a = rd(0, 0x04); g_phi2 = 105; uint8_t b = rd(0, 0x04);
      CHECK((uint8_t)(a - b) == 5, "cold init: free-wheeling T1 counter decrements for detection"); }

    mb_via_reset(true);
    g_phi2 = 9000;
    wr(0, 0x06, 0x04); wr(0, 0x07, 0x00); wr(0, 0x05, 0x00);
    CHECK(g_running[0], "one-shot: PIO armed at start");
    g_phi2 = 9000 + 4;
    CHECK((((uint16_t)rd(0, 0x05) << 8) | rd(0, 0x04)) == 0x0000, "one-shot counter reaches 0");
    g_phi2 = 9000 + 6;
    CHECK((((uint16_t)rd(0, 0x05) << 8) | rd(0, 0x04)) == (uint16_t)(4 - 6),
          "one-shot keeps counting past 0 (no reload)");
    wr(0, 0x0E, 0xC0);
    g_tick(0);
    CHECK((rd(0, 0x0D) & 0x40) != 0, "one-shot underflow sets the Timer1 flag");
    CHECK(!g_running[0], "one-shot halts the PIO after the single underflow");

    /* W65C22 2.6: a free-run->one-shot switch with no T1C-H reload raises no IRQ (Willy Byte freeze). */
    mb_via_reset(true);
    g_phi2 = 30000;
    wr(0, 0x0B, 0x40); wr(0, 0x0E, 0xC0);
    wr(0, 0x06, 0xFF); wr(0, 0x07, 0x40); wr(0, 0x05, 0x40);
    CHECK(g_running[0], "free-run->one-shot: counter running under music");
    wr(0, 0x0B, 0x02);
    CHECK(!g_running[0], "free-run->one-shot switch stops the counter (no ghost underflow)");
    CHECK((rd(0, 0x0D) & 0x40) == 0 && g_irq == false, "free-run->one-shot sets no Timer1 flag and no /IRQ");
    wr(0, 0x06, 0x00); wr(0, 0x07, 0x08); wr(0, 0x05, 0x08);
    CHECK(g_running[0], "a subsequent T1C-H load re-arms the one-shot normally");

    mb_via_reset(true);
    g_phi2 = 11000;
    wr(0, 0x02, 0x80);
    wr(0, 0x0B, 0xC0);
    wr(0, 0x06, 0x02); wr(0, 0x07, 0x00); wr(0, 0x05, 0x00);
    g_phi2 = 11000 + 0; CHECK((rd(0, 0x00) & 0x80) == 0x00, "PB7 low just after arm");
    g_phi2 = 11000 + 4; CHECK((rd(0, 0x00) & 0x80) == 0x80, "PB7 high after one free-run period");
    g_phi2 = 11000 + 8; CHECK((rd(0, 0x00) & 0x80) == 0x00, "PB7 low again after two periods");
    wr(0, 0x02, 0x00);
    CHECK((rd(0, 0x00) & 0x80) == 0x80, "DDRB bit7 input reads the pin, not the Timer1 PB7 wave");

    mb_via_reset(true);
    g_phi2 = 12000;
    wr(0, 0x02, 0x80);
    wr(0, 0x0B, 0x80);
    wr(0, 0x06, 0x02); wr(0, 0x07, 0x00); wr(0, 0x05, 0x00);
    g_phi2 = 12000 + 1; CHECK((rd(0, 0x00) & 0x80) == 0x00, "one-shot PB7 low while counting down");
    g_phi2 = 12000 + 4; CHECK((rd(0, 0x00) & 0x80) == 0x80, "one-shot PB7 high after the single underflow");

    mb_via_reset(true);
    g_phi2 = 20000;
    wr(0, 0x08, 0x34);
    wr(0, 0x09, 0x12);
    CHECK(g_t2_run[0] && g_t2_armed[0] == 0x1234 + 1, "T2C-H arms the one-shot at N+1 cycles");
    CHECK(rd(0, 0x09) == 0x12 && rd(0, 0x08) == 0x34, "T2 counter reads the load value at arm");

    g_phi2 = 20000 + 0x0034;
    CHECK((((uint16_t)rd(0, 0x09) << 8) | rd(0, 0x08)) == 0x1234 - 0x0034,
          "T2 counter decrements with the PHI2 count");

    g_phi2 = 20000 + 0x1234 + 3;
    CHECK((((uint16_t)rd(0, 0x09) << 8) | rd(0, 0x08)) == (uint16_t)(0x1234 - (0x1234 + 3)),
          "T2 keeps counting past 0 (one-shot, no reload)");

    mb_via_reset(true);
    wr(0, 0x0E, 0x80 | 0x20);
    mb_via_t2_event(0);
    CHECK(g_irq == true, "T2 underflow asserts /IRQ when IER bit5 enabled");
    CHECK((rd(0, 0x0D) & 0x20) != 0, "IFR bit5 (T2) set after underflow");
    rd(0, 0x08);
    CHECK(g_irq == false, "reading T2C-L releases /IRQ");
    CHECK((rd(0, 0x0D) & 0x20) == 0, "reading T2C-L clears IFR bit5");

    wr(0, 0x0E, 0x20);
    mb_via_t2_event(0);
    CHECK((rd(0, 0x0D) & 0x20) != 0, "IFR bit5 latches with IER disabled");
    CHECK(g_irq == false, "no /IRQ when T2 IER disabled");
    wr(0, 0x0D, 0x20);
    CHECK((rd(0, 0x0D) & 0x20) == 0, "IFR write-1 clears T2");

    mb_via_reset(true);
    g_phi2 = 30000;
    wr(0, 0x0B, 0x20);
    wr(0, 0x08, 0x00); wr(0, 0x09, 0x02);
    CHECK(!g_t2_run[0], "pulse mode: no timed (PHI2) IRQ armed");
    g_phi2 = 30000 + 0x0100;
    CHECK((((uint16_t)rd(0, 0x09) << 8) | rd(0, 0x08)) == 0x0200,
          "pulse mode: counter holds (no PHI2 source)");

    mb_via_reset(true);
    wr(0, 0x0E, 0x80 | 0x20);
    wr(1, 0x0B, 0x40); wr(1, 0x06, 0x00); wr(1, 0x07, 0x08); wr(1, 0x05, 0x08); wr(1, 0x0E, 0xC0);
    mb_via_t2_event(0);
    CHECK(g_irq == true, "T2(ch0) asserts shared /IRQ");
    g_tick(1);
    CHECK(g_irq == true, "T2 + Timer1 -> /IRQ");
    rd(0, 0x08);
    CHECK(g_irq == true, "clearing T2 leaves Timer1 /IRQ");
    rd(1, 0x04);
    CHECK(g_irq == false, "clearing both releases /IRQ");

    g_phi2 = 0;


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
    wr(0, 0x0E, 0x82); mb_via_ar_event(0);
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

    mb_via_reset(true);
    wr(0, 0x0E, 0x80 | 0x02);
    mb_via_ar_event(0);
    CHECK(g_irq == true, "A/R event asserts /IRQ when CA1 IER enabled");
    CHECK((rd(0, 0x0D) & 0x02) != 0, "IFR bit1 (CA1) set after A/R event");
    rd(0, 0x01);
    CHECK(g_irq == false, "reading ORA clears CA1 /IRQ");
    CHECK((rd(0, 0x0D) & 0x02) == 0, "reading ORA clears IFR bit1");
    mb_via_ar_event(0); wr(0, 0x0D, 0x02);
    CHECK(g_irq == false, "IFR write-1 clears CA1");
    mb_via_ar_event(0); wr(0, 0x01, 0x00);
    CHECK(g_irq == false, "writing ORA clears CA1");

    wr(0, 0x0E, 0x02);
    mb_via_ar_event(0);
    CHECK((rd(0, 0x0D) & 0x02) != 0, "CA1 latches with IER disabled");
    CHECK(g_irq == false, "no /IRQ when CA1 IER disabled");
    rd(0, 0x01);

    wr(0, 0x0E, 0x82); mb_via_ar_event(0);
    rd(0, 0x0F);
    CHECK((rd(0, 0x0D) & 0x02) != 0, "ORA_NH read does NOT clear CA1");
    rd(0, 0x01);

    mb_via_reset(true);
    wr(0, 0x0E, 0x82);
    wr(1, 0x0B, 0x40);
    wr(1, 0x06, 0x00); wr(1, 0x07, 0x08); wr(1, 0x05, 0x08); wr(1, 0x0E, 0xC0);
    mb_via_ar_event(0);
    CHECK(g_irq == true, "CA1(ch0) asserts shared /IRQ");
    g_tick(1);
    CHECK(g_irq == true, "CA1 + Timer1 -> /IRQ");
    rd(0, 0x01);
    CHECK(g_irq == true, "clearing CA1 leaves Timer1 /IRQ");
    rd(1, 0x04);
    CHECK(g_irq == false, "clearing both releases /IRQ");

    mb_via_reset(true);
    wr(0, 0x0E, 0x82);
    mb_via_ar_poll();                        /* sync to the current edge counts */
    CHECK(g_irq == false, "A/R poll: no edge-count change -> no /IRQ");
    g_ar_edges_mirror[0]++; mb_via_ar_poll();
    CHECK(g_irq == true && (rd(0, 0x0D) & 0x02) != 0, "A/R poll: edge-count advance latches CA1 -> /IRQ");
    rd(0, 0x01);
    mb_via_ar_poll();
    CHECK((rd(0, 0x0D) & 0x02) == 0, "A/R poll: no new edge does not re-latch");
    g_ar_edges_mirror[0] += 2; mb_via_ar_poll();
    CHECK((rd(0, 0x0D) & 0x02) != 0, "A/R poll: coalesced edges still latch CA1");
    rd(0, 0x01);
    g_ar_edges_mirror[1]++; mb_via_ar_poll();
    CHECK((rd(1, 0x0D) & 0x02) != 0, "A/R poll: chip1 edge-count advance latches ch1 CA1");

    mb_via_reset(true);
    wr(0, 0x0E, 0x80 | 0x40);
    g_tick(0);
    CHECK((rd(0, 0x0D) & 0x40) != 0, "ack-locked: Timer1 flag set before ack");
    mb_via_read_ack_locked(0, 0x05);
    CHECK((rd(0, 0x0D) & 0x40) != 0, "mb_via_read_ack_locked(off=5) leaves Timer1 flag");
    mb_via_read_ack_locked(0, 0x04);
    CHECK((rd(0, 0x0D) & 0x40) == 0, "mb_via_read_ack_locked(off=4) clears Timer1 flag");

    mb_via_reset(true);
    mb_version_disarm();

    CHECK(!mb_version_armed(), "version: idle before any read");
    (void)rd(0, 0x0A);
    CHECK(mb_version_armed(), "version: a VIA1 SR read auto-arms the readout");
    mb_version_disarm();

    {
        size_t vlen = strlen(MB_VERSION_STR);
        int ok = 1;
        for (int i = 0; i < MB_VERSION_READ_LEN; i++) {
            uint8_t got  = rd(0, 0x0A);
            uint8_t want = (i < (int)vlen) ? (uint8_t)MB_VERSION_STR[i] : (uint8_t)' ';
            if (got != want) ok = 0;
        }
        CHECK(ok, "version: VIA1 SR reads clock out the string then space padding for READ_LEN bytes");
    }

    CHECK(!mb_version_armed(), "version: readout self-disarms after READ_LEN bytes");
    CHECK(rd(0, 0x0A) == (uint8_t)MB_VERSION_STR[0], "version: the next read auto-arms from the top");
    mb_version_disarm();

    (void)rd(0, 0x0A);
    CHECK(mb_version_armed(), "version: a read arms the readout");
    wr(0, 0x00, 0x00);
    CHECK(!mb_version_armed(), "version: an ORB write disarms the readout");

    mb_version_disarm();
    wr(1, 0x0A, 0x5A);
    CHECK(rd(1, 0x0A) == 0x5A, "version: VIA2 SR read returns the stored register, not the version");
    CHECK(!mb_version_armed(), "version: a VIA2 SR read does not arm the readout");

    mb_version_disarm();
    CHECK(mb_version_read_byte() == mb_version_read_byte()
          && mb_version_read_byte() == (uint8_t)MB_VERSION_STR[0],
          "version: a read without ack is non-destructive");
    mb_version_disarm();
    CHECK(!mb_version_armed() && mb_version_read_byte() == (uint8_t)MB_VERSION_STR[0],
          "version: disarm resets state and read pointer");

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
        g_tick(ch);
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
        g_phi2 = 5000 + 0x1002;
        CHECK(((rd(ch, 0x05) << 8) | rd(ch, 0x04)) == 0x1000,
              "mb-audit continuous: counter reloads to the latch after one period");

        mb_via_reset(true);
        wr(ch, 0x08, 0x05); wr(ch, 0x09, 0x00);
        mb_via_t2_event(ch);
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

    /* division-free free-run reduce must equal e % (latch+2) for every divisor */
    {
        uint32_t bad = 0, worst_n = 0, worst_e = 0, lcg = 0x12345678u;
        for (uint32_t n = 0; n <= 0xFFFFu; n++) {
            uint32_t P = n + 2u;
            uint32_t es[10] = { 0u, 1u, P - 1u, P, P + 1u, 2u * P - 1u,
                                0xFFFFFFFFu, 0x80000000u, P * 1000u + 7u, lcg };
            for (int i = 0; i < 10; i++) {
                if (mb_test_t1_reduce((uint16_t)n, es[i]) != (es[i] % P)) {
                    if (!bad) { worst_n = n; worst_e = es[i]; }
                    bad++;
                }
            }
            lcg = lcg * 1664525u + 1013904223u;
        }
        for (uint32_t k = 0; k < 2000000u; k++) {
            lcg = lcg * 1664525u + 1013904223u;
            uint16_t n = (uint16_t)(lcg >> 8);
            uint32_t e = lcg ^ (lcg << 13) ^ (lcg >> 7);
            if (mb_test_t1_reduce(n, e) != (e % ((uint32_t)n + 2u))) {
                if (!bad) { worst_n = n; worst_e = e; }
                bad++;
            }
        }
        if (bad) printf("  reduce mismatch: first n=%u e=%u (total %u)\n", worst_n, worst_e, bad);
        CHECK(bad == 0, "free-run reciprocal reduce matches e % (n+2) exhaustively");
    }

    printf("\n%d/%d checks passed%s\n", tests - fails, tests, fails ? "  <-- FAILURES" : "  (all pass)");
    return fails ? 1 : 0;
}
