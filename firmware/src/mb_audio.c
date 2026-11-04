/*
 * Core from Troy Schrapel's pico-56 (MIT, Copyright (c) 2023 Troy Schrapel,
 * https://github.com/visrealm/pico-56). Repinned + core-loop added here.
 */
#include "mb_audio.h"
#include "mb_pins.h"
#include "mb_speech.h"
#include "emu2149.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include <stdlib.h>

#define PWM_WRAP 1024

// lock-free SPSC: core0 producer, core1 consumer
#define AYQ_SIZE 512   // power of two, one music-tick burst
static volatile uint32_t ayq[AYQ_SIZE];
static volatile uint32_t ayq_head = 0;
static volatile uint32_t ayq_tail = 0;

void mb_audio_post(uint32_t cmd) {
    uint32_t h = ayq_head;
    uint32_t n = (h + 1) & (AYQ_SIZE - 1);
    if (n == ayq_tail) return;           // full: drop
    ayq[h] = cmd;
    __dmb();                             // publish payload before advancing head
    ayq_head = n;
}

static void ay_drain(void) {
    while (ayq_tail != ayq_head) {
        __dmb();                       // acquire before reading the payload head guards
        uint32_t cmd = ayq[ayq_tail];
        ayq_tail = (ayq_tail + 1) & (AYQ_SIZE - 1);
        int     op = (cmd >> 16) & 3;
        int     ch = (cmd >> 8) & 1;
        uint8_t v  = cmd & 0xFF;
        switch (op) {
            case AY_OP_REG:   ch ? audioWritePsg1Reg(v) : audioWritePsg0Reg(v); break;
            case AY_OP_DATA:  ch ? audioWritePsg1(v)    : audioWritePsg0(v);    break;
            case AY_OP_RESET: ch ? audioResetPsg1()     : audioResetPsg0();     break;
        }
    }
}

static PSG*    psg0 = NULL;
static PSG*    psg1 = NULL;
static uint    audioSlice = 0;
static uint8_t psg0Reg = 0;
static uint8_t psg1Reg = 0;
static int     g_sampleUs = 20;
static int     g_sampleRate = 50000;

static PSG* createPSG(int psgClock, int sampleRate) {
    PSG* psg = PSG_new(psgClock, sampleRate);
    PSG_setVolumeMode(psg, EMU2149_VOL_AY_3_8910);
    PSG_reset(psg);
    return psg;
}

void audioInit(int psgClock, int sampleRate) {
    psg0 = createPSG(psgClock, sampleRate);
    psg1 = createPSG(psgClock, sampleRate);
    g_sampleUs = 1000000 / sampleRate;
    g_sampleRate = sampleRate;

    gpio_set_function(PIN_AUDIO_L, GPIO_FUNC_PWM);
    gpio_set_function(PIN_AUDIO_R, GPIO_FUNC_PWM);   // same slice, channel B
    audioSlice = pwm_gpio_to_slice_num(PIN_AUDIO_L);

    pwm_set_clkdiv_int_frac(audioSlice, 1, 0);       // carrier = clk_sys / PWM_WRAP (ultrasonic)
    pwm_set_wrap(audioSlice, PWM_WRAP);
    pwm_set_both_levels(audioSlice, 0, 0);
    pwm_set_enabled(audioSlice, true);

    speech_init(psgClock, sampleRate);

#ifdef MB_TONE_TEST
    // equal tone both channels: any L/R difference is analog HW, not firmware
    {
        const uint16_t TP = 145;                     // ~440 Hz at the 1,020,484 Hz PSG clock
        audioWritePsg0Reg(0); audioWritePsg0(TP & 0xFF);
        audioWritePsg0Reg(1); audioWritePsg0((TP >> 8) & 0x0F);
        audioWritePsg0Reg(7); audioWritePsg0(0xFE);  // mixer: tone A on (active low)
        audioWritePsg0Reg(8); audioWritePsg0(0x0F);
        audioWritePsg1Reg(0); audioWritePsg1(TP & 0xFF);
        audioWritePsg1Reg(1); audioWritePsg1((TP >> 8) & 0x0F);
        audioWritePsg1Reg(7); audioWritePsg1(0xFE);
        audioWritePsg1Reg(8); audioWritePsg1(0x0F);
    }
#endif
}

static inline uint16_t clamp_pwm(int32_t v) {
    if (v < 0) return 0;
    if (v > PWM_WRAP - 1) return PWM_WRAP - 1;
    return (uint16_t)v;
}


static uint32_t __not_in_flash_func(next_sample_word)(void) {
    PSG_calc(psg0);
    PSG_calc(psg1);
    int32_t leftAudio  = ((int32_t)psg0->ch_out[0] + (int32_t)psg0->ch_out[1]
                         + (int32_t)psg0->ch_out[2]) * PWM_WRAP >> 14;
    int32_t rightAudio = ((int32_t)psg1->ch_out[0] + (int32_t)psg1->ch_out[1]
                         + (int32_t)psg1->ch_out[2]) * PWM_WRAP >> 14;
    int32_t sp0 = speech_sample(0);
    int32_t sp1 = speech_sample(1);
#ifdef SPEECH_SWAP_CH
    int32_t spL = sp1, spR = sp0;
    int     lvL = speech_live(1), lvR = speech_live(0);
#else
    int32_t spL = sp0, spR = sp1;            // hard-pan: chip0 -> L, chip1 -> R
    int     lvL = speech_live(0), lvR = speech_live(1);
#endif
    // live channel needs a slew-limited mid-rail bias (~5 ms): a hard step pops (DC transient)
    static int32_t bias_l = 0, bias_r = 0;
    const int32_t BIAS_TGT = PWM_WRAP / 2, BIAS_SLEW = 2;
    int32_t tL = lvL ? BIAS_TGT : 0, tR = lvR ? BIAS_TGT : 0;
    int32_t dL = tL - bias_l, dR = tR - bias_r;
    bias_l += (dL > BIAS_SLEW) ? BIAS_SLEW : (dL < -BIAS_SLEW) ? -BIAS_SLEW : dL;
    bias_r += (dR > BIAS_SLEW) ? BIAS_SLEW : (dR < -BIAS_SLEW) ? -BIAS_SLEW : dR;
    int32_t Lv = leftAudio  + spL + bias_l;
    int32_t Rv = rightAudio + spR + bias_r;
    uint16_t L = clamp_pwm(Lv);
    uint16_t R = clamp_pwm(Rv);
    return (uint32_t)L | ((uint32_t)R << 16);
}

// DMA-paced: output holds 50 kHz through generation spikes (core1 fills two buffers ahead)
#define SAMP_BUF 256                      // 5.12 ms per buffer at 50 kHz
static uint32_t dmabuf[2][SAMP_BUF];
static volatile int need_fill[2] = { 0, 0 };
static int dma_a, dma_b;

static void fill_buffer(int b) {
    for (int i = 0; i < SAMP_BUF; i++)
        dmabuf[b][i] = next_sample_word();
}

static void __not_in_flash_func(dma_irq)(void) {
    if (dma_hw->ints0 & (1u << dma_a)) {
        dma_hw->ints0 = 1u << dma_a;
        dma_channel_set_read_addr(dma_a, dmabuf[0], false);
        need_fill[0] = 1;
    }
    if (dma_hw->ints0 & (1u << dma_b)) {
        dma_hw->ints0 = 1u << dma_b;
        dma_channel_set_read_addr(dma_b, dmabuf[1], false);
        need_fill[1] = 1;
    }
}

static void dma_delivery_start(void) {
    fill_buffer(0);
    fill_buffer(1);

    dma_a = dma_claim_unused_channel(true);
    dma_b = dma_claim_unused_channel(true);
    int timer = dma_claim_unused_timer(true);
    // DREQ div from clk_sys keeps the sample rate exact at any system clock
    uint32_t dreq_div = clock_get_hz(clk_sys) / (uint32_t)g_sampleRate;
    dma_timer_set_fraction(timer, 1, (uint16_t)dreq_div);
    uint dreq = dma_get_timer_dreq(timer);
    volatile void *cc = &pwm_hw->slice[audioSlice].cc; // 32-bit: chan A | (chan B << 16)

    for (int i = 0; i < 2; i++) {
        int ch = i ? dma_b : dma_a, other = i ? dma_a : dma_b;
        dma_channel_config c = dma_channel_get_default_config(ch);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_dreq(&c, dreq);
        channel_config_set_chain_to(&c, other);        // ping-pong: each kicks the other
        dma_channel_configure(ch, &c, (void *)cc, dmabuf[i], SAMP_BUF, false);
    }
    dma_channel_set_irq0_enabled(dma_a, true);
    dma_channel_set_irq0_enabled(dma_b, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_channel_start(dma_a);
}

void audioRun(void) {
    dma_delivery_start();
    while (true) {
#ifndef MB_TONE_TEST
        ay_drain();
        speech_drain();
#endif
        if (need_fill[0]) { need_fill[0] = 0; fill_buffer(0); }
        if (need_fill[1]) { need_fill[1] = 0; fill_buffer(1); }
    }
}

#ifdef MB_PHI2_SELFTEST
// PHI2 self-test tone: advancing -> ~1518 Hz beep, frozen -> ~180 Hz buzz
void audioSelfTestPhi2(bool advancing) {
    uint16_t tp = advancing ? 42u : 354u;
    audioWritePsg0Reg(0); audioWritePsg0(tp & 0xFF);
    audioWritePsg0Reg(1); audioWritePsg0((tp >> 8) & 0x0F);
    audioWritePsg0Reg(7); audioWritePsg0(0xFE);              // mixer: tone A on (active low)
    audioWritePsg0Reg(8); audioWritePsg0(0x0F);
    audioWritePsg1Reg(0); audioWritePsg1(tp & 0xFF);
    audioWritePsg1Reg(1); audioWritePsg1((tp >> 8) & 0x0F);
    audioWritePsg1Reg(7); audioWritePsg1(0xFE);
    audioWritePsg1Reg(8); audioWritePsg1(0x0F);

    uint32_t samples = 700000u / (uint32_t)g_sampleUs;
    for (uint32_t i = 0; i < samples; i++) {
        pwm_hw->slice[audioSlice].cc = next_sample_word();
        busy_wait_us((uint64_t)g_sampleUs);
    }

    audioResetPsg0();
    audioResetPsg1();
    pwm_set_both_levels(audioSlice, 0, 0);
}
#endif

void    audioWritePsg0Reg(uint8_t r) { psg0Reg = r; }
void    audioWritePsg1Reg(uint8_t r) { psg1Reg = r; }
void    audioWritePsg0(uint8_t v)    { PSG_writeReg(psg0, psg0Reg, v); }
void    audioWritePsg1(uint8_t v)    { PSG_writeReg(psg1, psg1Reg, v); }
void    audioResetPsg0(void)         { PSG_reset(psg0); }
void    audioResetPsg1(void)         { PSG_reset(psg1); }
