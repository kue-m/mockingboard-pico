/* The bus-service and Timer1 paths live in RAM (__not_in_flash_func) so a cross-core XIP-cache miss can't stall the 6502 read deadline or jitter the Timer1 period. */
#include "mb_via.h"
#include "mb_bus.h"
#include "mb_audio.h"
#include "mb_speech.h"
#include "mb_version.h"

#include "hardware/irq.h"

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

// not static: the frozen fast-path (mb_fastread.S) reads these directly.
volatile uint8_t  reg[2][16];
static volatile uint8_t  ora[2];
volatile uint32_t t1_arm[2];          // PHI2 tick at the last Timer1 arm
volatile uint32_t t2_arm[2];
static volatile uint16_t t1_run_n[2]; // latch the PIO period is currently clocked at

// Reciprocal magic for the free-run modulo e % (latch+2): avoids the UDIV whose timing variance tips the read margin. not static (mb_fastread.S reads it).
#define T1_RECIP_ADD  0x40
#define T1_RECIP_POW2 0x80
volatile uint32_t t1_recip[2];
volatile uint8_t  t1_recip_more[2];   // bits0-4 shift, bit6 add-case, bit7 pow2 (ASM contract)


// A port A read returns the last value written to the register, masked as emu2149 does, so read-back needs no cross-core access.
static const uint8_t ay_regmsk[16] = {
    0xff, 0x0f, 0xff, 0x0f, 0xff, 0x0f, 0x1f, 0x3f,
    0x1f, 0x1f, 0x1f, 0xff, 0xff, 0x0f, 0xff, 0xff,
};
static volatile uint8_t ay_shadow[2][16];
static volatile uint8_t ay_addr[2];
static volatile uint8_t ay_addr_valid[2];
static volatile uint8_t ay_read[2];   // 1 = AY PSG read mode (drives port A inputs)

static uint8_t t1_pb7(int ch);

#ifndef MB_HOST_SIM
static int g_t2_irq = -1;             // NVIC IRQ, masked under via_lock
static int g_ar_irq = -1;
#endif


// Mask core0's IFR producers while the bus path edits reg[][], so an RMW can't tear.
static inline void __not_in_flash_func(via_lock)(void) {
    irq_set_enabled(PIO1_IRQ_0, false);
#ifndef MB_HOST_SIM
    if (g_t2_irq >= 0) irq_set_enabled((uint)g_t2_irq, false);
#if MB_SPEECH_AR_IRQ
    if (g_ar_irq >= 0) irq_set_enabled((uint)g_ar_irq, false);
#endif
#endif
}
static inline void __not_in_flash_func(via_unlock)(void) {
#ifndef MB_HOST_SIM
#if MB_SPEECH_AR_IRQ
    if (g_ar_irq >= 0) irq_set_enabled((uint)g_ar_irq, true);
#endif
    if (g_t2_irq >= 0) irq_set_enabled((uint)g_t2_irq, true);
#endif
    irq_set_enabled(PIO1_IRQ_0, true);
}

static void __not_in_flash_func(recompute_irq)(void) {
    bool any = ((reg[0][R_IFR] & reg[0][R_IER] & 0x7F) != 0) ||
               ((reg[1][R_IFR] & reg[1][R_IER] & 0x7F) != 0);
    mb_bus_irq(any);
}

static void __not_in_flash_func(ifr_set)(int ch, uint8_t bits) {
    uint8_t v = reg[ch][R_IFR] | bits;
    v = (v & 0x7F) ? (v | IFR_ANY) : (v & ~IFR_ANY);
    reg[ch][R_IFR] = v;
    recompute_irq();
}

static void __not_in_flash_func(ifr_clear)(int ch, uint8_t bits) {
    uint8_t v = reg[ch][R_IFR] & ~bits;
    v = (v & 0x7F) ? (v | IFR_ANY) : (v & ~IFR_ANY);
    reg[ch][R_IFR] = v;
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
        v = (v & 0x7F) | t1_pb7(ch);
    return v;
}

static inline uint16_t t1_latch(int ch) {
    return (uint16_t)(reg[ch][R_T1LL] | ((uint16_t)reg[ch][R_T1LH] << 8));
}

// Reciprocal magic for divisor P = latch+2 (64-bit divide, off the read hot path).
static void __not_in_flash_func(t1_recip_update)(int ch) {
    uint32_t P = (uint32_t)t1_latch(ch) + 2u;
    if ((P & (P - 1u)) == 0u) {
        uint32_t L = 0; while ((1u << L) < P) L++;
        t1_recip[ch] = 0;
        t1_recip_more[ch] = (uint8_t)(L | T1_RECIP_POW2);
        return;
    }
    uint32_t L   = 31u - (uint32_t)__builtin_clz(P);    // floor(log2 P)
    uint64_t num = (uint64_t)1u << (32 + L);
    uint32_t m   = (uint32_t)(num / P);
    uint32_t rem = (uint32_t)(num - (uint64_t)m * P);
    if (P - rem < (1u << L)) {
        t1_recip[ch]      = m + 1u;
        t1_recip_more[ch] = (uint8_t)L;
    } else {
        uint32_t m2 = m + m, rem2 = rem + rem;
        if (rem2 >= P || rem2 < rem) m2 += 1u;
        t1_recip[ch]      = m2 + 1u;
        t1_recip_more[ch] = (uint8_t)(L | T1_RECIP_ADD);
    }
}

// e % P via the magic (no UDIV). mb_fastread.S .Lt1 reproduces this exactly: keep in lockstep.
static inline uint32_t __not_in_flash_func(t1_divmod_reduce)(int ch, uint32_t e) {
    uint8_t  more = t1_recip_more[ch];
    uint32_t P    = (uint32_t)t1_latch(ch) + 2u;
    uint32_t q;
    if (more & T1_RECIP_POW2) {
        q = e >> (more & 0x1F);
    } else {
        uint32_t qh = (uint32_t)(((uint64_t)t1_recip[ch] * e) >> 32);   // MULHI
        if (more & T1_RECIP_ADD) {
            uint32_t t = ((e - qh) >> 1) + qh;
            q = t >> (more & 0x1F);
        } else {
            q = qh >> (more & 0x1F);
        }
    }
    return e - q * P;
}

#ifdef MB_HOST_SIM
// Test hook: host suite checks the reduce vs true e % (n+2). Clobbers ch0's latch.
uint32_t mb_test_t1_reduce(uint16_t n, uint32_t e) {
    reg[0][R_T1LL] = (uint8_t)(n & 0xFF);
    reg[0][R_T1LH] = (uint8_t)(n >> 8);
    t1_recip_update(0);
    return t1_divmod_reduce(0, e);
}
#endif

// Live Timer1 counter, phase-aligned to the /IRQ PIO. Free-run wraps on N+2. One-shot counts the full 16-bit space.
static uint16_t __not_in_flash_func(t1_counter)(int ch) {
    uint16_t n = t1_latch(ch);
    uint32_t e = mb_phi2_ticks() - t1_arm[ch];          // wrap-safe
    if (reg[ch][R_ACR] & ACR_T1_FREERUN) {
        return (uint16_t)(n - t1_divmod_reduce(ch, e));
    }
    return (uint16_t)(n - e);
}

// Free-run toggles PB7 every underflow. One-shot goes high after the single underflow.
static uint8_t __not_in_flash_func(t1_pb7)(int ch) {
    uint16_t n = t1_latch(ch);
    uint32_t e = mb_phi2_ticks() - t1_arm[ch];
    if (reg[ch][R_ACR] & ACR_T1_FREERUN) {
        uint32_t period = (uint32_t)n + 2u;
        return ((e / period) & 1u) ? 0x80 : 0x00;
    }
    return (e > n) ? 0x80 : 0x00;
}

// Timer2: 16-bit one-shot (no reload). Timed mode counts PHI2. Pulse mode counts PB6 (unsourced, so it holds).
static inline uint16_t t2_load(int ch) {
    return (uint16_t)(reg[ch][R_T2CL] | ((uint16_t)reg[ch][R_T2CH] << 8));
}
static uint16_t __not_in_flash_func(t2_counter)(int ch) {
    uint16_t n = t2_load(ch);
    if (reg[ch][R_ACR] & ACR_T2_PULSE) return n;
    uint32_t e = mb_phi2_ticks() - t2_arm[ch];
    return (uint16_t)(n - e);                           // free-wheels past 0
}

static void __not_in_flash_func(timer2_start)(int ch) {
    uint16_t n = t2_load(ch);
    ifr_clear(ch, IFR_T2);
    t2_arm[ch] = mb_phi2_ticks();

    via_unlock();                               // mb_alarm_arm is alarm setup (no reg[][])
    if (reg[ch][R_ACR] & ACR_T2_PULSE) mb_alarm_arm(ch, 0);
    else                               mb_alarm_arm(ch, (uint32_t)n + 1u);  // underflow at N+1
    via_lock();
}

static void __not_in_flash_func(timer1_start)(int ch) {
    uint16_t n = t1_latch(ch);
    ifr_clear(ch, IFR_T1);
    t1_arm[ch] = mb_phi2_ticks();               // phase-align the readable counter to the PIO
    t1_run_n[ch] = n;

    // mb_sync_restart is PIO setup. Drop the mask so a pending via_tick on the other channel isn't lost to PIO's sticky irq flag.
    via_unlock();
    mb_sync_restart(ch, n ? n : 1);
    via_lock();
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
            mb_version_disarm();               // AY activity cancels a pending version readout
            break;
        case R_T1CL:
        case R_T1LL:
            reg[ch][R_T1LL] = d;
            t1_recip_update(ch);
            break;
        case R_T1LH:
            reg[ch][R_T1LH] = d;               // high latch only, no latch->counter transfer
            t1_recip_update(ch);
            ifr_clear(ch, IFR_T1);             // W65C22 Table 2-7: a T1L-H write resets IFR6
            break;
        case R_T1CH:
            reg[ch][R_T1LH] = d;               // T1C-H also writes the high latch
            t1_recip_update(ch);
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
            reg[ch][R_SR] = d;                 // store-only on this card
            break;
        case R_ACR: {
            uint8_t prev = reg[ch][R_ACR];
            reg[ch][R_ACR] = d;
            // one-shot T1 fires an IRQ only from a T1C-H load (W65C22 2.6). A free-run->one-shot switch with no reload raises none.
            if ((prev & ACR_T1_FREERUN) && !(d & ACR_T1_FREERUN))
                mb_sync_restart(ch, 0);
            break;
        }
        default:
            reg[ch][off] = d;
            break;
    }
}

// The read value, no IFR side-effect: lock-free so it runs before via_lock() and drives D0-7 inside the 6502's ~0.5 us deadline. Flag-clears deferred to via_read_ack.
static uint8_t __not_in_flash_func(via_read_value)(int ch, uint8_t off) {
    switch (off) {
        case R_ORB:
            return read_portb(ch);
        case R_ORA:
        case R_ORA_NH:
            return read_porta(ch);
        case R_T1CL:
            return (uint8_t)(t1_counter(ch) & 0xFF);
        case R_T1CH:
            return (uint8_t)(t1_counter(ch) >> 8);
        case R_T2CL:
            return (uint8_t)(t2_counter(ch) & 0xFF);
        case R_T2CH:
            return (uint8_t)(t2_counter(ch) >> 8);
        case R_IER:
            return reg[ch][R_IER] | 0x80;      // IER reads back with bit7 = 1
        case R_SR:
            if (ch == 0)
                return mb_version_read_byte();  // VIA1 SR read -> firmware version
            return reg[ch][R_SR];
        default:
            return reg[ch][off];
    }
}

// Read flag-clears, under via_lock() after the byte reached the SM. ORA_NH ($F) is absent: it does not clear CA1.
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
        case R_SR:
            if (ch == 0) mb_version_advance();
            break;
        default:
            break;
    }
}

// mb_fastread.S drives the byte itself, then calls this for the flag-clear ack under the lock.
void __not_in_flash_func(mb_via_read_ack_locked)(int ch, uint8_t off) {
    via_lock();
    via_read_ack(ch, off);
    via_unlock();
}

static void __not_in_flash_func(via_tick)(int ch) {   // Timer1 underflow, ISR context
    ifr_set(ch, IFR_T1);
    if (!(reg[ch][R_ACR] & ACR_T1_FREERUN)) {   // one-shot: a single IRQ, then halt the PIO
        mb_sync_restart(ch, 0);                 // the software counter keeps free-running
        return;
    }
    // The PIO captured its period at the arm, so re-rate when the latch changed. Guard is load-bearing: steady music rewrites the same latch (no restart), else a title's first song sticks at the arm period.
    uint16_t n = t1_latch(ch);
    if (n != t1_run_n[ch]) {
        t1_run_n[ch] = n;
        t1_arm[ch]   = mb_phi2_ticks();
        mb_sync_restart(ch, n ? n : 1);
    }
}

// An SSI-263 A/R edge latches CA1 on that chip's VIA: chip0 -> ch0, chip1 -> ch1.
void __not_in_flash_func(mb_via_ar_event)(int ch) {
    ifr_set(ch & 1, IFR_CA1);
}

void __not_in_flash_func(mb_via_t2_event)(int ch) {   // Timer2 one-shot underflow, ISR context
    ifr_set(ch & 1, IFR_T2);
}

// Latch CA1 on an advance of core1's A/R rising-edge count: matches the W65C22's edge-latched
// CA1, so a brief request whose level has collapsed by poll time is still caught. A core0 alarm.
static uint8_t ar_edges[2];
void __not_in_flash_func(mb_via_ar_poll)(void) {
    for (int c = 0; c < 2; c++) {
        uint8_t e = speech_ar_edges(c);
        if (e != ar_edges[c]) { mb_via_ar_event(c); ar_edges[c] = e; }
    }
}

// No via_lock: the /RES GPIO IRQ can preempt a held lock anyway, and the ROM re-inits the VIA.
void __not_in_flash_func(mb_via_reset)(bool asserted) {
    if (!asserted) return;
    for (int c = 0; c < 2; c++) {
        // W65C22S: RES preserves the T1/T2 counters+latches and SR. Leave reg[6..0xA] and t1_arm/t2_arm so the readable counters keep free-wheeling.
        reg[c][R_ORB]  = 0; reg[c][R_ORA] = 0;
        reg[c][R_DDRB] = 0; reg[c][R_DDRA] = 0;
        reg[c][R_ACR]  = 0; reg[c][R_PCR] = 0;
        reg[c][R_IFR]  = 0; reg[c][R_IER] = 0;
        ora[c] = 0;
        ay_addr[c] = 0; ay_addr_valid[c] = 0; ay_read[c] = 0;
        for (int i = 0; i < 16; i++) ay_shadow[c][i] = 0;
    }
    mb_bus_irq(false);
    mb_sync_restart(0, 0);                      // 0 = disable Timer1 IRQ
    mb_sync_restart(1, 0);
    mb_alarm_arm(0, 0);                         // 0 = disable Timer2 IRQ
    mb_alarm_arm(1, 0);
}

void mb_via_init(void) {
    mb_sync_init(via_tick);
    mb_alarm_init(mb_via_t2_event);
#ifndef MB_HOST_SIM
    g_t2_irq = mb_alarm_irq();
#if MB_SPEECH_AR_IRQ
    // A/R -> CA1 via a core0 poll alarm (not the inter-core FIFO), avoiding SIO FIFO machinery.
    g_ar_irq = mb_ar_tick_init(mb_via_ar_poll);
#endif
#endif
    mb_via_reset(true);

    // Cold power-on only (not /RES): the T1 latch powers up undefined, so seed 0xFFFF so the free-wheeling detection counter never reads a tiny/zero period.
    for (int c = 0; c < 2; c++) {
        reg[c][R_T1LL] = 0xFF; reg[c][R_T1LH] = 0xFF;
        t1_arm[c]      = mb_phi2_ticks();
        t1_run_n[c]    = 0xFFFF;
        t1_recip_update(c);
    }
}

// aligned(16): pins the timer-read critical path to a fixed fetch-boundary alignment, so read timing stays stable regardless of linker placement.
__attribute__((aligned(16)))
void __not_in_flash_func(mb_via_access)(uint32_t acc) {
    uint8_t a   = MB_ACC_ADDR(acc);
    uint8_t sel = a & 0x60;                      // A6,A5 = SSI-263 chip select

    // Speech region, decoded in its own TU so editing speech never recompiles the timer-read path.
    if (sel) {
        mb_speech_bus_access(acc, a);
        return;
    }

    int     ch  = (a & 0x80) ? 1 : 0;           // A7 selects VIA1/VIA2
    uint8_t off = a & 0x0F;

    uint8_t val;
    if (MB_ACC_IS_READ(acc)) {
        // Drive the byte before the lock: the read SM must present D0-7 within the 6502's ~0.5 us window or the CPU latches the floating bus.
        val = via_read_value(ch, off);
        mb_bus_put_rdata(val);
        via_lock();
        via_read_ack(ch, off);
        via_unlock();
    } else {
        via_lock();
        val = mb_bus_get_wdata();
        via_write(ch, off, val);
        via_unlock();
    }
}
