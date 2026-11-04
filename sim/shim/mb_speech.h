#pragma once
#include <stdint.h>
#ifndef MB_SPEECH_AR_IRQ
#define MB_SPEECH_AR_IRQ 1   /* CA1-on-ORA ack is part of the decode logic under test */
#endif
void    speech_post(int chip, int reg, uint8_t val);
uint8_t speech_ar(int chip);
uint8_t speech_ar_edges(int chip);
uint8_t mb_speech_bus_access(uint32_t acc, uint8_t a);
void    speech_init(int clockHz, int sampleRate);
void    speech_drain(void);
int32_t speech_sample(int chip);
