#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef struct { volatile uint32_t csr, div, ctr, cc, top; } pwm_slice_hw_t;
typedef struct { pwm_slice_hw_t slice[12]; } pwm_hw_t;
extern pwm_hw_t g_pwm_hw;
#define pwm_hw (&g_pwm_hw)
static inline unsigned pwm_gpio_to_slice_num(unsigned pin) { return (pin >> 1) & 7; }
static inline void pwm_set_clkdiv_int_frac(unsigned s, uint8_t i, uint8_t f) { (void)s; (void)i; (void)f; }
static inline void pwm_set_wrap(unsigned s, uint16_t w) { (void)s; (void)w; }
static inline void pwm_set_both_levels(unsigned s, uint16_t a, uint16_t b) { pwm_hw->slice[s].cc = a | ((uint32_t)b << 16); }
static inline void pwm_set_enabled(unsigned s, bool e) { (void)s; (void)e; }
