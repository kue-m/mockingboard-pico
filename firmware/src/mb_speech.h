#ifndef MB_SPEECH_H
#define MB_SPEECH_H

#include <stdint.h>

/* -DMB_SPEECH_AR_IRQ=0 compiles out the A/R -> 6522 CA1 path (A/B isolation) */
#ifndef MB_SPEECH_AR_IRQ
#define MB_SPEECH_AR_IRQ 1
#endif

void speech_init(int clockHz, int sampleRate);
void speech_post(int chip, int reg, uint8_t val);
uint8_t mb_speech_bus_access(uint32_t acc, uint8_t a);
uint8_t speech_ar(int chip);
uint8_t speech_ar_edges(int chip);
void speech_drain(void);
int32_t speech_sample(int chip);
int speech_live(int chip);

#endif
