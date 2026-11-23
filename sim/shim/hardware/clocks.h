#pragma once
#include <stdint.h>
enum clock_index { clk_sys = 5 };
static inline uint32_t clock_get_hz(enum clock_index c) { (void)c; return 200000000u; }
