/* SSI-263 bus decode in its own TU so editing it never recompiles the timing-critical timer-read path. */
#include "mb_bus.h"
#include "mb_speech.h"

uint8_t __not_in_flash_func(mb_speech_bus_access)(uint32_t acc, uint8_t a) {
    uint8_t sel = a & 0x60;                      // A6,A5 = chip select
    uint8_t val;
    if (MB_ACC_IS_READ(acc)) {
        int c = (sel & 0x40) ? 1 : 0;            // A6 -> chip1 else chip0
        // D7 = A/R (1 = requesting), off the detect path
        val = speech_ar(c) ? 0x80 : 0x00;
        mb_bus_put_rdata(val);
    } else {
        val = mb_bus_get_wdata();
        uint8_t reg = a & 0x07;                  // A2:A0 selects R0-R4 (R4-R7 alias)
        if (sel & 0x40) speech_post(1, reg, val);
        if (sel & 0x20) speech_post(0, reg, val);    // $Cn60 -> both
    }
    return val;
}
