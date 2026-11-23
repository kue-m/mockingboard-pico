/* Golden CRCs recorded from mb_audio.c at 6bc9116. -DMB_AUDIO_GOLDEN_PRINT prints them. */
#include "mb_audio.c"

#include <stdio.h>
#include <string.h>

pwm_hw_t g_pwm_hw;
dma_hw_t g_dma_hw;
int g_dma_next_channel;

static uint32_t g_us;
uint32_t time_us_32(void) { return g_us += 1000; }

static int g_live[2];
void    speech_init(int clockHz, int sampleRate) { (void)clockHz; (void)sampleRate; }
void    speech_drain(void) {}
int     speech_live(int c) { return g_live[c & 1]; }
int32_t speech_sample(int c) {
    static int32_t t[2];
    if (!g_live[c & 1]) return 0;
    t[c & 1] = (t[c & 1] + 37) % 200;
    return t[c & 1] - 100;
}

static int g_pass, g_fail;
#define CHECK(cond, msg) do { if (cond) g_pass++; else { g_fail++; printf("FAIL: %s\n", msg); } } while (0)

static uint32_t crc32_update(uint32_t crc, const uint32_t *w, int n) {
    for (int i = 0; i < n; i++) {
        uint32_t v = w[i];
        for (int k = 0; k < 4; k++) {
            crc ^= (v >> (8 * k)) & 0xFF;
            for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1)));
        }
    }
    return crc;
}

static void ay(int ch, int reg, int val) {
    mb_audio_post(ay_cmd(AY_OP_REG, ch, (uint8_t)reg));
    mb_audio_post(ay_cmd(AY_OP_DATA, ch, (uint8_t)val));
}

static uint32_t render(int buffers, uint32_t crc) {
    for (int i = 0; i < buffers; i++) {
        ay_drain();
        fill_buffer(i & 1);
        crc = crc32_update(crc, dmabuf[i & 1], SAMP_BUF);
    }
    return crc;
}

static void reset_chips(void) {
    mb_audio_post(ay_cmd(AY_OP_RESET, 0, 0));
    mb_audio_post(ay_cmd(AY_OP_RESET, 1, 0));
    ay_drain();
    g_live[0] = g_live[1] = 0;
}

static uint32_t scenario_silence(void) {
    reset_chips();
    return render(4, 0xFFFFFFFFu);
}

static uint32_t scenario_tones(void) {
    reset_chips();
    ay(0, 0, 145); ay(0, 1, 0); ay(0, 7, 0x3E); ay(0, 8, 15);
    ay(1, 2, 0x60); ay(1, 3, 0); ay(1, 7, 0x3D); ay(1, 9, 10);
    return render(8, 0xFFFFFFFFu);
}

static uint32_t scenario_envelope(void) {
    reset_chips();
    ay(0, 0, 200); ay(0, 1, 0); ay(0, 7, 0x3E); ay(0, 8, 0x10);
    ay(0, 11, 0x00); ay(0, 12, 0x02); ay(0, 13, 0x0E);
    return render(16, 0xFFFFFFFFu);
}

static uint32_t scenario_noise(void) {
    reset_chips();
    ay(1, 6, 0x08); ay(1, 7, 0x1F); ay(1, 10, 12);
    return render(8, 0xFFFFFFFFu);
}

static uint32_t scenario_speech_bias(void) {
    reset_chips();
    ay(0, 0, 145); ay(0, 7, 0x3E); ay(0, 8, 8);
    g_live[0] = 1;
    uint32_t crc = render(6, 0xFFFFFFFFu);
    g_live[0] = 0;
    g_live[1] = 1;
    crc = render(6, crc);
    g_live[1] = 0;
    return render(4, crc);
}

static uint32_t scenario_volume_pcm(void) {
    reset_chips();
    ay(0, 0, 0); ay(0, 1, 0); ay(0, 7, 0x3E);
    uint32_t crc = 0xFFFFFFFFu;
    for (int i = 0; i < 16; i++) {
        ay(0, 8, (i * 5) & 15);
        crc = render(1, crc);
    }
    return crc;
}

static uint32_t scenario_reset_midstream(void) {
    reset_chips();
    ay(0, 0, 100); ay(0, 7, 0x3E); ay(0, 8, 15);
    uint32_t crc = render(3, 0xFFFFFFFFu);
    mb_audio_post(ay_cmd(AY_OP_RESET, 0, 0));
    return render(3, crc);
}

static const struct { const char *name; uint32_t (*run)(void); uint32_t golden; } SCENARIOS[] = {
    { "silence",          scenario_silence,         0x38E3FFEEu },
    { "tones",            scenario_tones,           0x90DD7075u },
    { "envelope",         scenario_envelope,        0x1D977FCEu },
    { "noise",            scenario_noise,           0x980F2D34u },
    { "speech bias",      scenario_speech_bias,     0x02C51E78u },
    { "volume pcm",       scenario_volume_pcm,      0x6D57D4ADu },
    { "reset midstream",  scenario_reset_midstream, 0x5C46DD43u },
};
#define N_SCENARIOS ((int)(sizeof(SCENARIOS) / sizeof(SCENARIOS[0])))

static void test_dma_ping_pong(void) {
    g_dma_next_channel = 0;
    dma_delivery_start();
    CHECK(dma_a == 0 && dma_b == 1, "dma: two channels claimed");
    need_fill[0] = need_fill[1] = 0;
    dma_hw->ints0 = 1u << dma_a;
    dma_irq();
    CHECK(need_fill[0] == 1 && need_fill[1] == 0, "dma: channel A completion requests buffer 0");
    dma_hw->ints0 = 1u << dma_b;
    dma_irq();
    CHECK(need_fill[1] == 1, "dma: channel B completion requests buffer 1");
}


int main(void) {
    audioInit(1020484, 50000);
    test_dma_ping_pong();
    for (int i = 0; i < N_SCENARIOS; i++) {
        uint32_t crc = SCENARIOS[i].run();
#ifdef MB_AUDIO_GOLDEN_PRINT
        printf("    { \"%s\", 0x%08Xu },\n", SCENARIOS[i].name, crc);
#else
        char msg[96];
        snprintf(msg, sizeof msg, "golden: %s matches the main-branch mixer", SCENARIOS[i].name);
        CHECK(crc == SCENARIOS[i].golden, msg);
#endif
    }
    printf("%s: %d/%d checks passed\n", g_fail ? "FAIL" : "PASS", g_pass, g_pass + g_fail);
    return g_fail ? 1 : 0;
}
