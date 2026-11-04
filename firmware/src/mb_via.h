#pragma once

#include <stdint.h>
#include <stdbool.h>

void mb_via_init(void);
void mb_via_access(uint32_t acc);

// firmware entry: the frozen ASM fast-path (mb_fastread.S) answers Timer1 reads ($Cn04/$Cn05) so recompiles can't move the 6502 timing. All else tail-calls mb_via_access
void mb_bus_dispatch(uint32_t acc);
void mb_via_read_ack_locked(int ch, uint8_t off);
void mb_via_reset(bool asserted);  // true zeroes the model + stops timers

void mb_via_ar_event(int ch);
void mb_via_t2_event(int ch);
void mb_via_ar_poll(void);

