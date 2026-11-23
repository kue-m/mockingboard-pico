#include "mb_bus.h"
#include "mb_speech.h"

uint8_t __not_in_flash_func(mb_speech_status)(int chip) {
    return speech_ar(chip) ? 0x80 : 0x00;
}

uint8_t __not_in_flash_func(mb_speech_bus_write)(uint8_t a) {
    uint8_t sel = a & 0x60;                      // A6,A5 = chip select
    uint8_t val = mb_bus_get_wdata();
    uint8_t reg = a & 0x07;                      // A2:A0 selects R0-R4 (R4-R7 alias)
    if (sel & 0x40) speech_post(1, reg, val);
    if (sel & 0x20) speech_post(0, reg, val);    // $Cn60 -> both
    return val;
}
