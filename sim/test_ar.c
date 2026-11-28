/* The SSI-263 driver (mb_speech.c) over the vendored engine, against the SSI-263A datasheet and mb-audit. */
#include "test_checks.h"
#include "mb_speech.h"

int g_tests, g_fails, g_xfails;

#define SSI  "SSI-263A datasheet"
#define MBA  "mb-audit chip-ssi263.a"
#define RATE 50000
#define XCK  1020484

/* one word's control load + first phoneme, as the driver writes it */
static void setup_word(void) {
    speech_post(1, 3, 0x80);
    speech_post(1, 1, 0x8D);
    speech_post(1, 2, 0xB8);
    speech_post(1, 4, 0xEB);
    speech_post(1, 0, 0xC0);
    speech_post(1, 3, 0x5C);
}

static void post(int c, int reg, uint8_t v) { speech_post(c, reg, v); speech_drain(); }

static void kick(int c, uint8_t r0, uint8_t r2) {
    post(c, 3, 0x80);
    post(c, 2, r2);
    post(c, 0, r0);
    post(c, 3, 0x00);
}

static int until_request(int c, int limit) {
    for (int n = 0; n < limit; n++) {
        if (speech_ar(c)) return n;
        speech_sample(c);
    }
    return speech_ar(c) ? limit : -1;
}

static void power_up_regression(void) {
    setup_word();
    speech_drain();
    int g = 0;
    while (!speech_ar(1) && g++ < 200000) speech_sample(1);
    CHECK(speech_ar(1), "A/R rises after the first phoneme completes");

    setup_word();
    speech_drain();
    speech_sample(1);
    CHECK(speech_ar(1) == 0, "no premature A/R at power-up: phoneme restarts (as real HW)");
    g = 0;
    while (!speech_ar(1) && g++ < 200000) speech_sample(1);
    CHECK(speech_ar(1), "A/R rises after the power-cycle's phoneme completes");
}

static void request_and_ack(void) {
    kick(0, 0xC0, 0xF0);
    CHECK_SPEC(!speech_ar(0), MBA " Test#2 (S9)", "no request right after the CTL H->L kick");
    CHECK_SPEC(until_request(0, RATE) >= 0, MBA " Test#2 (S9)", "the request appears when the phoneme ends");

    (void)speech_ar(0);
    CHECK_SPEC(speech_ar(0), MBA " S6", "reading the status leaves the request set");

    post(0, 0, 0xC0);
    CHECK_SPEC(!speech_ar(0), MBA " Test#6 (S4)", "an R0 write acknowledges the request");
    until_request(0, RATE);
    post(0, 3, 0x70);
    CHECK_SPEC(speech_ar(0), MBA " Test#6 (S4)", "an R3 write with CTL = 0 leaves the request");
    post(0, 4, 0xE0);
    CHECK_SPEC(speech_ar(0), MBA " Test#6 (S4)", "an R4 (filter) write leaves the request");
    post(0, 1, 0x80);
    CHECK_SPEC(!speech_ar(0), MBA " Test#6 (S4/S5)", "an R1 write acknowledges the request");
    post(0, 0, 0xC0);
    until_request(0, RATE);
    post(0, 2, 0xF0);
    KNOWN_DISCREPANCY(!speech_ar(0), MBA " Test#6 (S4/S5)", "an R2 write acknowledges the request");

    post(0, 0, 0xC0);
    until_request(0, RATE);
    post(0, 3, 0x80);
    speech_sample(0);
    CHECK_SPEC(!speech_ar(0), SSI " CTL; " MBA " Test#7 (S10)", "CTL = 1 drops the request");
}

static void mode_00(void) {
    kick(1, 0xC0, 0xF0);
    until_request(1, RATE);
    post(1, 3, 0x80);
    post(1, 0, 0x00);
    post(1, 3, 0x00);
    KNOWN_DISCREPANCY(until_request(1, RATE) >= 0, SSI " mode table; " MBA " Test#2 (S8)",
                      "in mode 00 the status D7 still reports the end of the phoneme");
}

static void phoneme_time(void) {
    /* A single hardware measurement, so the datasheet formula (4096 cycles) stays the passing check. */
    kick(0, 0xC0, 0xF0);
    int n = until_request(0, RATE);
    long cycles = (long)n * XCK / RATE;
    KNOWN_DISCREPANCY(n >= 0 && cycles > 0x1ED8 * 85 / 100 && cycles < 0x1ED8 * 115 / 100,
                      MBA " S11", "phoneme $00 at R=$F, D=3 lasts about $1ED8 cycles on hardware");
    CHECK_SPEC(n >= 0 && cycles >= 4096 * 85 / 100 && cycles <= 4096 * 115 / 100, SSI " phoneme duration formula",
               "the phoneme lasts 4096 x (16-R) x (4-D) / XCK per the datasheet formula");
}

static void liveness(void) {
    CHECK(speech_live(0), "a chip that received CTL = 0 is live");
    post(0, 3, 0x80);
    int n = 0;
    while (speech_live(0) && n < RATE) { speech_sample(0); n++; }
    CHECK(!speech_live(0) && n == RATE / 5, "after CTL = 1 the chip rings out for 0.2 s, then idles");
    CHECK(speech_sample(0) == 0 && !speech_ar(0), "an idle chip outputs 0 and requests nothing");

    for (int i = 0; i < 300; i++) speech_post(0, 4, 0xE0);
    speech_post(0, 3, 0x00);
    speech_drain();
    CHECK(!speech_live(0), "a full speech queue drops new writes");
}

int main(void) {
    speech_init(XCK, RATE);
    power_up_regression();
    request_and_ack();
    mode_00();
    phoneme_time();
    liveness();
    printf("\n%d/%d checks passed, %d known discrepancies (XFAIL)%s\n",
           g_tests - g_fails - g_xfails, g_tests, g_xfails, g_fails ? "  <-- FAILURES" : "  (all pass)");
    return g_fails ? 1 : 0;
}
