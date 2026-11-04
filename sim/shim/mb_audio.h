#pragma once
#include <stdint.h>
#define AY_OP_REG    0
#define AY_OP_DATA   1
#define AY_OP_RESET  2
static inline uint32_t ay_cmd(int op, int ch, uint8_t v) {
    return ((uint32_t)op << 16) | ((uint32_t)ch << 8) | (uint32_t)v;
}
void mb_audio_post(uint32_t cmd);
