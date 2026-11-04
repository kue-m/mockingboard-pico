#include "mb_speech.h"
#include "ssi263.h"

#include "pico/stdlib.h"
#include "hardware/sync.h"


/* Clean-room glottal source (not MAME's SC-01 wave): keeps the shipped firmware zero-GPL. */
static const double OUR_GLOTTAL[] = {
    0.750000, 0.000632, 0.002476, 0.005384, 0.009119, 0.013378, 0.017818, 0.022078,
    0.025813, 0.028721, 0.030565, 0.031197, 0.028822, 0.022059, 0.011938,
};
#define OUR_GLOTTAL_N ((int)(sizeof(OUR_GLOTTAL) / sizeof(OUR_GLOTTAL[0])))

/* voice +/- swing in PWM counts (wrap 1024) */
#define SPEECH_PWM_GAIN 400.0f

#define SSI_R0_DP 0
#define SSI_R3_CTL 3  /* D7 = CTL (1 = power down) */

static ssi263_params g_params;
static const unsigned char *g_rom;
static int g_rate = 50000;
static ssi263 *chip[2] = { 0, 0 };

/* g_ar is the only cross-core field read by core0 */
static int            g_live[2]    = { 0, 0 };
static int            g_ringout[2] = { 0, 0 };
static volatile uint8_t g_ar[2]    = { 0, 0 };

/* lock-free SPSC queue, core0 -> core1 */
#define SPQ_SIZE 256   // power of two (used as mask)
static volatile uint32_t spq[SPQ_SIZE];
static volatile uint32_t spq_head = 0;   // core0 producer
static volatile uint32_t spq_tail = 0;   // core1 consumer

static inline uint32_t sp_cmd(int c, int reg, uint8_t v) {
    return ((uint32_t)(c & 1) << 16) | ((uint32_t)(reg & 7) << 8) | (uint32_t)v;
}

void speech_post(int c, int reg, uint8_t val) {
    uint32_t h = spq_head;
    uint32_t n = (h + 1) & (SPQ_SIZE - 1);
    if (n == spq_tail) return;
    spq[h] = sp_cmd(c, reg, val);
    __dmb();                                              // publish before advancing head
    spq_head = n;
}

uint8_t speech_ar(int c) { return g_ar[c & 1]; }

static uint8_t g_prev_ctl[2] = { 0x80, 0x80 };

/* Core1 is the sole writer. Count A/R rising edges rather than sampling the level. */
static uint8_t g_last_req[2];
static volatile uint8_t g_ar_edges[2];
uint8_t speech_ar_edges(int c) { return g_ar_edges[c & 1]; }
static void note_req(int c) {
    int r = ssi263_request(chip[c]);
    if (r && !g_last_req[c]) {
        g_ar_edges[c]++;
    }
    g_last_req[c] = (uint8_t)r;
    g_ar[c] = (uint8_t)r;
}

static void speech_build(int c) {
    if (chip[c]) ssi263_free(chip[c]);
    chip[c] = ssi263_new(&g_params, g_rom, (double)g_rate);
    g_live[c] = 0; g_ringout[c] = 0; g_ar[c] = 0; g_last_req[c] = 0;
}

void speech_init(int clockHz, int sampleRate) {
    g_rate = sampleRate;
    ssi263_default_params(&g_params);
    g_params.xck_hz = (double)clockHz;   // SSI-263 shares the card's phase-1 clock with the AYs
    g_params.glottal_n = OUR_GLOTTAL_N;
    for (int i = 0; i < OUR_GLOTTAL_N; i++)
        g_params.glottal_wave[i] = OUR_GLOTTAL[i];
    // output decimation FIR dominates CPU, oversample 1 keeps it light
    g_params.output_oversample = 1.0f;
    g_rom = ssi263_default_rom();
    speech_build(0);
    speech_build(1);
}

/* track liveness from CTL so an un-driven chip is never run (CPU budget) */
static void apply_write(int c, int reg, uint8_t v) {
    ssi263_write(chip[c], reg, v);
    if (reg == SSI_R3_CTL) {
        uint8_t prevctl = g_prev_ctl[c]; g_prev_ctl[c] = v;
        if (v & 0x80) {
            if (g_live[c] && g_ringout[c] == 0)
                g_ringout[c] = g_rate / 5;                // ~0.2 s release tail, then idle
        } else {
            g_live[c] = 1; g_ringout[c] = 0;
            if (prevctl & 0x80) ssi263_write(chip[c], 0, ssi263_reg(chip[c], 0));  // CTL H->L starts the phoneme (as real HW)
        }
    }
    if (g_live[c]) note_req(c);                            // catch an A/R edge made inside this batch
}

void speech_drain(void) {
    while (spq_tail != spq_head) {
        __dmb();                                          // acquire before reading payload
        uint32_t cmd = spq[spq_tail];
        spq_tail = (spq_tail + 1) & (SPQ_SIZE - 1);
        apply_write((cmd >> 16) & 1, (cmd >> 8) & 7, cmd & 0xFF);
    }
}

int speech_live(int c) { return g_live[c & 1]; }

int32_t __not_in_flash_func(speech_sample)(int c) {
    if (!g_live[c]) { g_ar[c] = 0; g_last_req[c] = 0; return 0; }

    float s = 0.0f;
    ssi263_run(chip[c], 1, &s);                           // always run: keeps A/R / card detection alive

    note_req(c);

    if (g_ringout[c] > 0 && --g_ringout[c] == 0) g_live[c] = 0;

    float v = s * (float)g_params.output_gain;
    if (v > 1.0f) v = 1.0f; else if (v < -1.0f) v = -1.0f;
    int32_t out = (int32_t)(v * SPEECH_PWM_GAIN);
    return out;
}

