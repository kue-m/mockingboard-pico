/*
 * Adapted from Troy Schrapel's pico-56 audio.c (MIT, Copyright (c) 2023 Troy Schrapel), repinned to GP26/27.
 */
#pragma once

#include <inttypes.h>
#include <stdbool.h>

void audioInit(int psgClock, int sampleRate);

void audioRun(void);             // never returns

#ifdef MB_PHI2_SELFTEST
// run after audioInit(), before audioRun()
void audioSelfTestPhi2(bool advancing);
#endif

void    audioWritePsg0Reg(uint8_t reg);
void    audioWritePsg1Reg(uint8_t reg);
void    audioWritePsg0(uint8_t val);
void    audioWritePsg1(uint8_t val);
void    audioResetPsg0(void);
void    audioResetPsg1(void);

// lock-free: core0 posts, only core1 touches emu2149
#define AY_OP_REG    0
#define AY_OP_DATA   1
#define AY_OP_RESET  2
static inline uint32_t ay_cmd(int op, int ch, uint8_t v) {
    return ((uint32_t)op << 16) | ((uint32_t)ch << 8) | (uint32_t)v;
}
void mb_audio_post(uint32_t cmd);
