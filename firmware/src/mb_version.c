#include "mb_version.h"

#include "mb_version_gen.h"   // generated at build time

#ifndef MB_VERSION_STR
#define MB_VERSION_STR "unknown"
#endif

#ifndef MB_HOST_SIM
#include "pico/binary_info.h"
bi_decl(bi_program_version_string(MB_VERSION_STR));   // so `picotool info` reports it
#endif

static const char VERSION[] = MB_VERSION_STR;
#define VLEN (sizeof(VERSION) - 1)

static bool     armed;
static uint16_t rp;

uint8_t mb_version_read_byte(void) {
    if (!armed) {                                 // auto-arm on the first read
        armed = true;
        rp    = 0;
    }
    return (uint8_t)(rp < VLEN ? VERSION[rp] : ' ');   // past the end, space-pad to the fixed width
}

bool mb_version_armed(void) {
    return armed;
}

void mb_version_advance(void) {
    if (!armed) return;
    if (++rp >= MB_VERSION_READ_LEN) {            // fixed-length field consumed -> disarm
        armed = false;
        rp    = 0;
    }
}

void mb_version_disarm(void) {
    armed = false;
    rp    = 0;
}
