#pragma once
#include <stdint.h>
#include <stdbool.h>

/* __not_in_flash_func is RP2350 RAM placement. A no-op on the host. */
#ifndef __not_in_flash_func
#define __not_in_flash_func(f) f
#endif
#define MB_ACC_ADDR(w)    ((uint8_t)((w) & 0xFF))
#define MB_ACC_IOSEL(w)   (((w) >> 8) & 1u)
#define MB_ACC_IS_READ(w) (((w) >> 9) & 1u)
void     mb_bus_irq(bool assert);
uint8_t  mb_bus_get_wdata(void);
uint32_t mb_phi2_ticks(void);

extern volatile uint8_t mb_rtable[256];
bool mb_t1_arm(int chan, uint32_t w, uint16_t n);
void mb_t1_latch(int chan, uint16_t n);
bool mb_t2_arm(int chan, uint32_t w, uint16_t n);
void mb_t2_hold(int chan, uint16_t n);
#define MB_TICK_T1(chan) (1u << (chan))
#define MB_TICK_T2(chan) (4u << (chan))
uint32_t mb_timer_ticks(void);
