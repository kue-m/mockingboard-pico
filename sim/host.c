/* Host harness: compiles mb_via.c + emu2149 against host shims, renders a tune to WAV. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "emu2149.h"
#include "mb_via.h"
#include "mb_audio.h"
#include "mb_bus.h"

#define SR 44100

static PSG *psg0, *psg1;
static uint8_t psg0Reg, psg1Reg;

void audioWritePsg0Reg(uint8_t r) { psg0Reg = r; }
void audioWritePsg1Reg(uint8_t r) { psg1Reg = r; }
void audioWritePsg0(uint8_t v)    { PSG_writeReg(psg0, psg0Reg, v); }
void audioWritePsg1(uint8_t v)    { PSG_writeReg(psg1, psg1Reg, v); }
void audioResetPsg0(void)         { PSG_reset(psg0); }
void audioResetPsg1(void)         { PSG_reset(psg1); }

#define AYQ 4096
static uint32_t ayq[AYQ];
static uint32_t ayq_head = 0, ayq_tail = 0;
void mb_audio_post(uint32_t cmd) {
    uint32_t n = (ayq_head + 1) & (AYQ - 1);
    if (n == ayq_tail) return;
    ayq[ayq_head] = cmd; ayq_head = n;
}
static void ay_drain(void) {
    while (ayq_tail != ayq_head) {
        uint32_t c = ayq[ayq_tail]; ayq_tail = (ayq_tail + 1) & (AYQ - 1);
        int op = (c >> 16) & 3, ch = (c >> 8) & 1; uint8_t v = c & 0xFF;
        switch (op) {
            case AY_OP_REG:   ch ? audioWritePsg1Reg(v) : audioWritePsg0Reg(v); break;
            case AY_OP_DATA:  ch ? audioWritePsg1(v)    : audioWritePsg0(v);    break;
            case AY_OP_RESET: ch ? audioResetPsg1()     : audioResetPsg0();     break;
        }
    }
}

static uint8_t g_wdata, g_rdata;
static bool    g_irq;
static void  (*g_tick)(int) = NULL;
void    mb_bus_irq(bool a)                   { g_irq = a; }
uint8_t mb_bus_get_wdata(void)               { return g_wdata; }
void    mb_bus_put_rdata(uint8_t d)          { g_rdata = d; }
void    mb_sync_init(void (*cb)(int))        { g_tick = cb; }
struct { uint32_t period; double acc; bool run; } tmr[2];
void    mb_sync_restart(int ch, uint32_t n)  { tmr[ch].period = n; tmr[ch].run = (n > 0); tmr[ch].acc = 0; }
static uint32_t g_phi2 = 0;
uint32_t mb_phi2_ticks(void)                 { return g_phi2; }
void mb_alarm_init(void (*cb)(int))          { (void)cb; }
void mb_alarm_arm(int ch, uint32_t n)        { (void)ch; (void)n; }
int  mb_alarm_irq(void)                      { return -1; }
int  mb_ar_tick_init(void (*cb)(void))       { (void)cb; return -1; }
void    speech_post(int c, int reg, uint8_t v) { (void)c; (void)reg; (void)v; }
uint8_t speech_ar(int c)                       { (void)c; return 0; }
uint8_t speech_ar_edges(int c)                       { (void)c; return 0; }

static void busw(int chan, uint8_t off, uint8_t data) {
    g_wdata = data;
    uint32_t acc = (uint32_t)(((chan ? 0x80 : 0) | (off & 0x0F)) & 0xFF);
    mb_via_access(acc);
}
/* AY write via the 6522 BC1/BDIR sequence: latch address (ORB 0x07), write data (ORB 0x06) */
static void ay_write(int chan, uint8_t reg, uint8_t val) {
    busw(chan, 0x01, reg);
    busw(chan, 0x00, 0x07);
    busw(chan, 0x00, 0x04);
    busw(chan, 0x01, val);
    busw(chan, 0x00, 0x06);
    busw(chan, 0x00, 0x04);
}

static double lpL, lpR;      /* leaky integrators = coupling-cap DC block */
static int16_t *wavbuf;
static long wavcap, wavlen;

static void wav_reserve(long frames) {
    wavcap = frames; wavlen = 0;
    wavbuf = malloc((size_t)frames * 2 * sizeof(int16_t));
}
static inline int16_t clamp16(double x) {
    if (x >  32767.0) return  32767;
    if (x < -32768.0) return -32768;
    return (int16_t)x;
}
static void render(long frames) {
    for (long i = 0; i < frames && wavlen < wavcap; i++) {
        g_phi2 += 23;   /* ~PHI2 cycles per 44.1 kHz frame (1020484/44100) */
        ay_drain();
        PSG_calc(psg0);
        PSG_calc(psg1);
        /* hard-pan: AY#0 -> left, AY#1 -> right */
        int32_t L = ((int32_t)psg0->ch_out[0] + psg0->ch_out[1] + psg0->ch_out[2]) * 1024 >> 14;
        int32_t R = ((int32_t)psg1->ch_out[0] + psg1->ch_out[1] + psg1->ch_out[2]) * 1024 >> 14;
        lpL += (L - lpL) * 0.0008;  lpR += (R - lpR) * 0.0008;   /* ~5 Hz HPF */
        wavbuf[wavlen * 2 + 0] = clamp16((L - lpL) * 26.0);
        wavbuf[wavlen * 2 + 1] = clamp16((R - lpR) * 26.0);
        wavlen++;
    }
}

static void write_wav(const char *path) {
    FILE *f = fopen(path, "wb");
    uint32_t data = (uint32_t)(wavlen * 2 * 2), riff = 36 + data, rate = SR, byterate = SR * 2 * 2;
    uint16_t ch = 2, bits = 16, block = 4, fmt = 1;
    fwrite("RIFF", 1, 4, f); fwrite(&riff, 4, 1, f); fwrite("WAVE", 1, 4, f);
    fwrite("fmt ", 1, 4, f); uint32_t sz = 16; fwrite(&sz, 4, 1, f);
    fwrite(&fmt, 2, 1, f); fwrite(&ch, 2, 1, f); fwrite(&rate, 4, 1, f);
    fwrite(&byterate, 4, 1, f); fwrite(&block, 2, 1, f); fwrite(&bits, 2, 1, f);
    fwrite("data", 1, 4, f); fwrite(&data, 4, 1, f);
    fwrite(wavbuf, 1, data, f);
    fclose(f);
}

static uint32_t rd32(const uint8_t *p) { return p[0] | p[1]<<8 | p[2]<<16 | (uint32_t)p[3]<<24; }

static uint8_t *read_file(const char *path, long *outlen) {
    int gz = 0; { FILE *t = fopen(path, "rb"); if (t){ int a=fgetc(t),b=fgetc(t); gz=(a==0x1f&&b==0x8b); fclose(t);} }
    FILE *f;
    if (gz) { char cmd[1024]; snprintf(cmd,sizeof cmd,"gzip -dc '%s'",path); f=popen(cmd,"r"); }
    else f = fopen(path, "rb");
    if (!f) return NULL;
    size_t cap = 1<<20, len = 0; uint8_t *buf = malloc(cap);
    for (;;) { if (len+65536 > cap){cap*=2; buf=realloc(buf,cap);} size_t n=fread(buf+len,1,65536,f); len+=n; if(n<65536) break; }
    if (gz) pclose(f); else fclose(f);
    *outlen = (long)len; return buf;
}

static int play_vgm(const char *path) {
    long len; uint8_t *d = read_file(path, &len);
    if (!d || len < 0x40 || memcmp(d, "Vgm ", 4) != 0) { fprintf(stderr,"not a VGM: %s\n", path); return 0; }
    uint32_t version = rd32(d + 0x08);
    uint32_t ayclk   = rd32(d + 0x74) & 0x3FFFFFFFu;  /* mask VGM flag bits: bit30 dual-chip, bit31 AY-type */
    uint32_t off     = (version >= 0x150) ? (0x34 + rd32(d + 0x34)) : 0x40;
    if (ayclk == 0) ayclk = 1020484;   /* default to the Apple II / Mockingboard clock */
    if (psg0) PSG_delete(psg0);
    if (psg1) PSG_delete(psg1);
    psg0 = PSG_new(ayclk, SR); PSG_setVolumeMode(psg0, EMU2149_VOL_AY_3_8910); PSG_reset(psg0);
    psg1 = PSG_new(ayclk, SR); PSG_setVolumeMode(psg1, EMU2149_VOL_AY_3_8910); PSG_reset(psg1);
    printf("VGM v%x  AY clock %u Hz  data@0x%x\n", version, ayclk, off);

    long budget = (long)SR * 600;
    wav_reserve(budget);
    long p = off;
    while (p < len && wavlen < wavcap) {
        uint8_t cmd = d[p++];
        if (cmd == 0xA0) { uint8_t reg = d[p], val = d[p+1]; p += 2;
            int chip = (reg & 0x80) ? 1 : 0; ay_write(chip, reg & 0x7F, val); }
        else if (cmd == 0x61) { uint32_t w = d[p] | d[p+1]<<8; p += 2; render(w); }
        else if (cmd == 0x62) { render(735); }
        else if (cmd == 0x63) { render(882); }
        else if (cmd >= 0x70 && cmd <= 0x7F) { render((cmd & 0x0F) + 1); }
        else if (cmd == 0x66) { break; }
        else if (cmd == 0x67) { uint32_t sz = rd32(d + p + 2); p += 6 + sz; }
        else if (cmd >= 0x30 && cmd <= 0x3F) { p += 1; }
        else if (cmd >= 0x40 && cmd <= 0x4E) { p += 2; }
        else if (cmd >= 0x51 && cmd <= 0x5F) { p += 2; }
        else if (cmd >= 0x80 && cmd <= 0x8F) { render(0); }
        else if (cmd == 0xE0) { p += 4; }
        else { break; }
    }
    free(d);
    return 1;
}

static uint16_t tp(double f) { double t = 1020484.0 / (16.0 * f); return (uint16_t)(t + 0.5); }
static void note(int chan, int ayreg_base, double freq) {
    uint16_t t = tp(freq);
    ay_write(chan, ayreg_base,     t & 0xFF);
    ay_write(chan, ayreg_base + 1, (t >> 8) & 0x0F);
}
static void play_synth(void) {
    psg0 = PSG_new(1020484, SR); PSG_setVolumeMode(psg0, EMU2149_VOL_AY_3_8910); PSG_reset(psg0);
    psg1 = PSG_new(1020484, SR); PSG_setVolumeMode(psg1, EMU2149_VOL_AY_3_8910); PSG_reset(psg1);
    wav_reserve((long)SR * 8);

    static const double mel[] = {440,554.37,659.25,880, 659.25,554.37,440,554.37};
    static const double bas[] = {110,110,146.83,146.83, 164.81,164.81,110,110};
    ay_write(0, 0x07, 0x38);
    ay_write(0, 0x08, 0x0F);
    ay_write(0, 0x09, 0x0C);
    ay_write(1, 0x07, 0x07);
    ay_write(1, 0x06, 0x0F);
    ay_write(1, 0x08, 0x0A);
    for (int step = 0; step < 16; step++) {
        note(0, 0x00, mel[step % 8]);
        note(0, 0x02, bas[step % 8]);
        note(1, 0x00, mel[(step + 2) % 8] * 2.0);
        render(SR / 4);
    }
}

int main(int argc, char **argv) {
    mb_via_init();

    int ok = 0;
    if (argc > 1) ok = play_vgm(argv[1]);
    if (!ok) { printf("rendering synthetic tune (author-written stimulus)\n"); play_synth(); }

    write_wav("out.wav");
    printf("wrote out.wav: %ld frames, %.2f s @ %d Hz stereo\n", wavlen, (double)wavlen / SR, SR);
    return 0;
}
