#pragma once

#include <stdbool.h>
#include <stdint.h>

#define MB_PHI2_SELFTEST_WINDOW_US 10000u   // ~10 ms: a healthy counter advances ~10,205 ticks (1,020,484 Hz), frozen 0
#define MB_PHI2_SELFTEST_MIN_TICKS 1000u    // between the healthy ~10,205 and noise, so outcomes are unambiguous

// modular subtraction is wrap-correct (free-running 32-bit counter)
static inline bool mb_phi2_advanced(uint32_t t0, uint32_t t1) {
    return (uint32_t)(t1 - t0) > MB_PHI2_SELFTEST_MIN_TICKS;
}
