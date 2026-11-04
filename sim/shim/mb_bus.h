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
void    mb_bus_irq(bool assert);
uint8_t mb_bus_get_wdata(void);
void    mb_bus_put_rdata(uint8_t d);
void    mb_sync_init(void (*cb)(int chan));
void    mb_sync_restart(int chan, uint32_t phi0_cycles);
uint32_t mb_phi2_ticks(void);
void     mb_alarm_init(void (*t2_cb)(int chan));
void     mb_alarm_arm(int chan, uint32_t phi2_cycles);
int      mb_alarm_irq(void);
int      mb_ar_tick_init(void (*cb)(void));
