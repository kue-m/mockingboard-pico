/* Regression for the SSI-263 power-up accuracy fix (mb_speech.c): a real SSI-263 raises A/R
   only after the phoneme loaded on CTL H->L completes, so the engine must restart the phoneme
   on CTL H->L rather than leave a stale timer_done. Drives the real engine. */
#include <stdio.h>
#include "mb_speech.h"

static int fails = 0, tests = 0;
#define CHECK(c, m) do { tests++; if (!(c)) { printf("  FAIL: %s\n", m); fails++; } } while (0)

/* one word's control load + first phoneme, as the driver writes it */
static void setup_word(void) {
    speech_post(1, 3, 0x80);
    speech_post(1, 1, 0x8D);
    speech_post(1, 2, 0xB8);
    speech_post(1, 4, 0xEB);
    speech_post(1, 0, 0xC0);
    speech_post(1, 3, 0x5C);
}

int main(void) {
    speech_init(1020484, 50000);

    setup_word();
    speech_drain();
    int g = 0;
    while (!speech_ar(1) && g++ < 200000) speech_sample(1);
    CHECK(speech_ar(1), "A/R rises after the first phoneme completes");

    /* second word's power cycle, drained in one batch with the prior timer_done still latched */
    setup_word();
    speech_drain();
    speech_sample(1);
    CHECK(speech_ar(1) == 0, "no premature A/R at power-up: phoneme restarts (as real HW)");

    g = 0;
    while (!speech_ar(1) && g++ < 200000) speech_sample(1);
    CHECK(speech_ar(1), "A/R rises after the power-cycle's phoneme completes");

    printf("%s: %d/%d checks passed\n", fails ? "FAIL" : "PASS", tests - fails, tests);
    return fails ? 1 : 0;
}
