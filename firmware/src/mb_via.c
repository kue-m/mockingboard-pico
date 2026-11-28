#include "mb_via.h"
#include "mb_bus.h"
#include "mb_audio.h"
#include "mb_speech.h"
#include "mb_version.h"

// WDC W65C22 register offsets.
#define R_ORB    0x0
#define R_ORA    0x1
#define R_DDRB   0x2
#define R_DDRA   0x3
#define R_T1CL   0x4
#define R_T1CH   0x5
#define R_T1LL   0x6
#define R_T1LH   0x7
#define R_T2CL   0x8
#define R_T2CH   0x9
#define R_SR     0xA
#define R_ACR    0xB
#define R_PCR    0xC
#define R_IFR    0xD
#define R_IER    0xE
#define R_ORA_NH 0xF

// Only CA1 (SSI-263 A/R) is a wired input. The rest never get set.
#define IFR_CA2  0x01
#define IFR_CA1  0x02
#define IFR_SR   0x04
#define IFR_CB2  0x08
#define IFR_CB1  0x10
#define IFR_T2   0x20
#define IFR_T1   0x40
#define IFR_ANY  0x80

#define ACR_T2_PULSE   0x20   // 1 = PB6 pulse counting, 0 = timed one-shot
#define ACR_T1_FREERUN 0x40   // 1 = free-run (reload), 0 = one-shot
#define ACR_T1_PB7     0x80

static volatile uint8_t  reg[2][16];
static volatile uint8_t  ora[2];
static volatile uint8_t  t1_oneshot_armed[2];
static volatile uint8_t  t2_armed[2];
static volatile uint8_t  t1_pb7_level[2];
static uint32_t          g_access_cycle;
static uint8_t           g_ar_shown[2];
static uint8_t           g_ar_edges[2];

// A port A read returns the last value written to the register, masked as emu2149 does, so read-back needs no cross-core access.
static const uint8_t ay_regmsk[16] = {
    0xff, 0x0f, 0xff, 0x0f, 0xff, 0x0f, 0x1f, 0x3f,
    0x1f, 0x1f, 0x1f, 0xff, 0xff, 0x0f, 0xff, 0xff,
};
static volatile uint8_t ay_shadow[2][16];
static volatile uint8_t ay_addr[2];
static volatile uint8_t ay_addr_valid[2];
static volatile uint8_t ay_read[2];   // 1 = AY PSG read mode (drives port A inputs)

static uint8_t via_read_value(int ch, uint8_t off);

static inline uint8_t rt_index(int ch, uint8_t off) { return (uint8_t)((ch ? 0x80 : 0x00) | off); }

static void __not_in_flash_func(rtable_sync)(int ch, uint8_t off) {
    if (off == R_T1CL || off == R_T1CH || off == R_T2CL || off == R_T2CH) return;
    mb_rtable[rt_index(ch, off)] = via_read_value(ch, off);
}
static void rtable_sync_via(int ch) {
    for (uint8_t off = 0; off < 16; off++) rtable_sync(ch, off);
}

static void __not_in_flash_func(rtable_sync_speech)(int chip) {
    uint8_t v = mb_speech_status(chip);
    for (unsigned i = 0; i < 256; i++) {
        if ((i & 0x10) || !(i & 0x60)) continue;
        if (((i & 0x40) ? 1 : 0) == chip) mb_rtable[i] = v;
    }
}


static void __not_in_flash_func(recompute_irq)(void) {
    bool any = ((reg[0][R_IFR] & reg[0][R_IER] & 0x7F) != 0) ||
               ((reg[1][R_IFR] & reg[1][R_IER] & 0x7F) != 0);
    mb_bus_irq(any);
}

// The IFR table byte is stored before /IRQ moves. A read in the cycle a timeout begins needs it.
static void __not_in_flash_func(ifr_set)(int ch, uint8_t bits) {
    uint8_t v = reg[ch][R_IFR] | bits;
    v = (v & 0x7F) ? (v | IFR_ANY) : (v & ~IFR_ANY);
    reg[ch][R_IFR] = v;
    rtable_sync(ch, R_IFR);
    recompute_irq();
}

static void __not_in_flash_func(ifr_clear)(int ch, uint8_t bits) {
    uint8_t v = reg[ch][R_IFR] & ~bits;
    v = (v & 0x7F) ? (v | IFR_ANY) : (v & ~IFR_ANY);
    reg[ch][R_IFR] = v;
    rtable_sync(ch, R_IFR);
    recompute_irq();
}

// AY control off port B: PB0=BC1, PB1=BDIR, PB2=/RESET (BC2 tied high).
static void __not_in_flash_func(ay_control)(int ch, uint8_t pb) {
    if (!(pb & 0x04)) {
        for (int r = 0; r < 16; r++) ay_shadow[ch][r] = 0;
        ay_read[ch] = 0;
        ay_addr_valid[ch] = 0;                 // reset invalidates the latched address
        mb_audio_post(ay_cmd(AY_OP_RESET, ch, 0));
        return;
    }
    ay_read[ch] = ((pb & 0x03) == 0x01);       // 01 = PSG read
    switch (pb & 0x03) {                        // {BDIR, BC1}
        case 0x03:                             // 11 -> latch register address
            ay_addr[ch] = ora[ch] & 0x0F;
            ay_addr_valid[ch] = 1;
            mb_audio_post(ay_cmd(AY_OP_REG, ch, ay_addr[ch]));
            break;
        case 0x02:                             // 10 -> write data
            if (!ay_addr_valid[ch]) break;     // no valid address since reset -> lands nowhere
            ay_shadow[ch][ay_addr[ch]] = ora[ch] & ay_regmsk[ay_addr[ch]];
            mb_audio_post(ay_cmd(AY_OP_DATA, ch, ora[ch]));
            break;
        default:
            break;
    }
}

// DDR-correct reads: output bits from the latch, input bits read 1 (port A inputs read AY data in read mode). Latch/0 instead breaks detection (Ultima IV, DDR=0).
static inline uint8_t __not_in_flash_func(read_porta)(int ch) {
    uint8_t ddr = reg[ch][R_DDRA];
    uint8_t in  = (ay_read[ch] && ay_addr_valid[ch]) ? ay_shadow[ch][ay_addr[ch]] : 0xFF;
    return (uint8_t)((reg[ch][R_ORA] & ddr) | (in & (uint8_t)~ddr));
}
static inline uint8_t __not_in_flash_func(read_portb)(int ch) {
    uint8_t ddr = reg[ch][R_DDRB];
    uint8_t v   = (uint8_t)((reg[ch][R_ORB] & ddr) | (uint8_t)~ddr);
    if ((reg[ch][R_ACR] & ACR_T1_PB7) && (ddr & 0x80))
        v = (v & 0x7F) | t1_pb7_level[ch];
    return v;
}

static inline uint16_t t1_latch(int ch) {
    return (uint16_t)(reg[ch][R_T1LL] | ((uint16_t)reg[ch][R_T1LH] << 8));
}

static inline uint16_t t2_load(int ch) {
    return (uint16_t)(reg[ch][R_T2CL] | ((uint16_t)reg[ch][R_T2CH] << 8));
}

static void __not_in_flash_func(timer2_start)(int ch) {
    uint16_t n = t2_load(ch);
    ifr_clear(ch, IFR_T2);
    if (reg[ch][R_ACR] & ACR_T2_PULSE) {
        t2_armed[ch] = 0;
        mb_t2_hold(ch, n);
        return;
    }
    t2_armed[ch] = 1;
    if (mb_t2_arm(ch, g_access_cycle, n)) { t2_armed[ch] = 0; ifr_set(ch, IFR_T2); }
}

static inline void t1_latch_changed(int ch) {
    mb_t1_latch(ch, t1_latch(ch));
}

static void __not_in_flash_func(timer1_start)(int ch) {
    uint16_t n = t1_latch(ch);
    ifr_clear(ch, IFR_T1);
    t1_oneshot_armed[ch] = 1;
    t1_pb7_level[ch] = 0x00;
    if (mb_t1_arm(ch, g_access_cycle, n)) {
        t1_oneshot_armed[ch] = 0;
        t1_pb7_level[ch] = 0x80;
        ifr_set(ch, IFR_T1);
    }
}

static void __not_in_flash_func(via_write)(int ch, uint8_t off, uint8_t d) {
    switch (off) {
        case R_ORA:
            ora[ch] = d; reg[ch][R_ORA] = d;
#if MB_SPEECH_AR_IRQ
            ifr_clear(ch, IFR_CA1);            // handshake access clears CA1 (A/R ack)
#endif
            break;
        case R_ORA_NH:
            ora[ch] = d; reg[ch][R_ORA] = d;   // no-handshake alias: does not clear CA1
            break;
        case R_ORB:
            reg[ch][R_ORB] = d; ay_control(ch, d);
            break;
        case R_T1CL:
        case R_T1LL:
            reg[ch][R_T1LL] = d;
            t1_latch_changed(ch);
            break;
        case R_T1LH:
            reg[ch][R_T1LH] = d;               // high latch only, no latch->counter transfer
            t1_latch_changed(ch);
            ifr_clear(ch, IFR_T1);             // W65C22 Table 2-7: a T1L-H write resets IFR6
            break;
        case R_T1CH:
            reg[ch][R_T1LH] = d;               // T1C-H also writes the high latch
            t1_latch_changed(ch);
            timer1_start(ch);
            break;
        case R_T2CL:
            reg[ch][R_T2CL] = d;
            break;
        case R_T2CH:
            reg[ch][R_T2CH] = d;
            timer2_start(ch);
            break;
        case R_IFR:
            ifr_clear(ch, d & 0x7F);           // write-1-to-clear
            break;
        case R_IER:
            if (d & 0x80) reg[ch][R_IER] |= (d & 0x7F);
            else          reg[ch][R_IER] &= ~(d & 0x7F);
            recompute_irq();
            break;
        case R_SR:
            if (ch == 0) {
                reg[0][R_SR] = mb_info_stage(d);
            } else {
                reg[1][R_SR] = d;
                mb_info_select_page(d);
            }
            break;
        case R_ACR: {
            uint8_t prev = reg[ch][R_ACR];
            reg[ch][R_ACR] = d;
            // A free-run -> one-shot switch still interrupts at the next timeout, once (mb-audit T6522_1).
            if ((prev & ACR_T1_FREERUN) && !(d & ACR_T1_FREERUN)) {
                t1_oneshot_armed[ch] = 1;
            }
            if ((prev ^ d) & ACR_T2_PULSE) {
                if (d & ACR_T2_PULSE) { t2_armed[ch] = 0; mb_t2_hold(ch, t2_load(ch)); }
                else                  mb_t2_arm(ch, g_access_cycle, t2_load(ch));
            }
            break;
        }
        default:
            reg[ch][off] = d;
            break;
    }
}

static uint8_t __not_in_flash_func(via_read_value)(int ch, uint8_t off) {
    switch (off) {
        case R_ORB:
            return read_portb(ch);
        case R_ORA:
        case R_ORA_NH:
            return read_porta(ch);
        case R_T1CL:
        case R_T1CH:
        case R_T2CL:
        case R_T2CH:
            return mb_rtable[rt_index(ch, off)];
        case R_IER:
            return reg[ch][R_IER] | 0x80;      // IER reads back with bit7 = 1
        case R_SR:
            return reg[ch][R_SR];
        default:
            return reg[ch][off];
    }
}

static void __not_in_flash_func(via_read_ack)(int ch, uint8_t off) {
    switch (off) {
        case R_ORA:
#if MB_SPEECH_AR_IRQ
            ifr_clear(ch, IFR_CA1);
#endif
            break;
        case R_T1CL:
            ifr_clear(ch, IFR_T1);
            break;
        case R_T2CL:
            ifr_clear(ch, IFR_T2);
            break;
        default:
            break;
    }
}

static void __not_in_flash_func(t1_timeout)(int ch) {
    if (reg[ch][R_ACR] & ACR_T1_FREERUN) {
        t1_pb7_level[ch] ^= 0x80;
    } else if (t1_oneshot_armed[ch]) {
        t1_oneshot_armed[ch] = 0;
        t1_pb7_level[ch] = 0x80;
    } else {
        return;
    }
    ifr_set(ch, IFR_T1);
    rtable_sync(ch, R_ORB);
}

static void __not_in_flash_func(t2_timeout)(int ch) {
    if (!t2_armed[ch]) return;
    t2_armed[ch] = 0;
    ifr_set(ch, IFR_T2);
}

// An SSI-263 A/R edge latches CA1 on that chip's VIA: chip0 -> ch0, chip1 -> ch1.
static void __not_in_flash_func(ar_edge)(int ch) {
    ifr_set(ch, IFR_CA1);
}

// Timeouts first, so a read that clears a flag clears it after its timeout set it.
// Kept apart from the speech polling so the main loop can run it every turn: the delay from a timeout to the
// IFR table byte is the loop turn time, and mb-audit T6522_10 reads IFR within ~1 cycle of that.
void __not_in_flash_func(mb_via_poll_timers)(void) {
    uint32_t t = mb_timer_ticks();
    if (t) {
        for (int c = 0; c < 2; c++) {
            if (t & MB_TICK_T1(c)) t1_timeout(c);
            if (t & MB_TICK_T2(c)) t2_timeout(c);
        }
    }
}

void __not_in_flash_func(mb_via_poll)(void) {
    mb_via_poll_timers();
    mb_via_poll_slow();
}

void __not_in_flash_func(mb_via_poll_slow)(void) {
    for (int c = 0; c < 2; c++) {
#if MB_SPEECH_AR_IRQ
        // An advance of the A/R edge count latches CA1, so a request already dropped is still caught.
        uint8_t e = speech_ar_edges(c);
        if (e != g_ar_edges[c]) { g_ar_edges[c] = e; ar_edge(c); }
#endif
        uint8_t ar = speech_ar(c) ? 1 : 0;
        if (ar != g_ar_shown[c]) { g_ar_shown[c] = ar; rtable_sync_speech(c); }
    }
}

void __not_in_flash_func(mb_via_reset)(bool asserted) {
    if (!asserted) return;
    bool t2_held[2] = { (reg[0][R_ACR] & ACR_T2_PULSE) != 0, (reg[1][R_ACR] & ACR_T2_PULSE) != 0 };
    for (int c = 0; c < 2; c++) {
        // W65C22S: RES preserves the T1/T2 counters+latches and SR, so reg[6..0xA] stay.
        reg[c][R_ORB]  = 0; reg[c][R_ORA] = 0;
        reg[c][R_DDRB] = 0; reg[c][R_DDRA] = 0;
        reg[c][R_ACR]  = 0; reg[c][R_PCR] = 0;
        reg[c][R_IFR]  = 0; reg[c][R_IER] = 0;
        ora[c] = 0;
        ay_addr[c] = 0; ay_addr_valid[c] = 0; ay_read[c] = 0;
        for (int i = 0; i < 16; i++) ay_shadow[c][i] = 0;
    }
    mb_bus_irq(false);
    for (int c = 0; c < 2; c++) {
        t1_oneshot_armed[c] = 0; t2_armed[c] = 0; t1_pb7_level[c] = 0;
        if (t2_held[c]) mb_t2_arm(c, mb_phi2_ticks(), t2_load(c));
        rtable_sync_via(c);
    }
}

#ifdef MB_HOST_SIM
uint8_t mb_test_via_read_value(int ch, uint8_t off) { return via_read_value(ch, off); }
#endif

void mb_via_init(void) {
    mb_via_reset(true);

    // The T1 latch powers up undefined. $FFFF keeps detection from seeing a tiny period.
    for (int c = 0; c < 2; c++) {
        reg[c][R_T1LL] = 0xFF; reg[c][R_T1LH] = 0xFF;
        mb_t1_arm(c, mb_phi2_ticks(), 0xFFFF);
        mb_t2_arm(c, mb_phi2_ticks(), t2_load(c));
        rtable_sync_via(c);
        g_ar_shown[c] = speech_ar(c) ? 1 : 0;
        rtable_sync_speech(c);
    }
}

void __not_in_flash_func(mb_via_access)(uint32_t acc) {
    uint32_t cyc = MB_ACC_IS_READ(acc) ? 0u : mb_access_ticks();
    uint8_t a   = MB_ACC_ADDR(acc);
    uint8_t sel = a & 0x60;                      // A6,A5 = SSI-263 chip select
    if (sel && !MB_ACC_IS_READ(acc)) {
        uint8_t val = mb_speech_bus_write(a);
        (void)val;
        return;
    }

    // The 6522s decode only A7 and A3-A0, so a speech-range read is also a read of the VIA register under it.
    int     ch  = (a & 0x80) ? 1 : 0;           // A7 selects VIA1/VIA2
    uint8_t off = a & 0x0F;
    if (MB_ACC_IS_READ(acc)) {
        via_read_ack(ch, off);
        rtable_sync(ch, off);
    } else {
        uint8_t val = mb_bus_get_wdata();
        g_access_cycle = cyc;
        via_write(ch, off, val);
        rtable_sync_via(ch);
    }
}
