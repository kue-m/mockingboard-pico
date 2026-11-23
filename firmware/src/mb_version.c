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

static uint8_t page;



void mb_info_select_page(uint8_t p) {
    page = p;
}

uint8_t mb_info_stage(uint8_t i) {
    if (page == MB_INFO_PAGE_VERSION) {
        if (i >= MB_VERSION_STR_LEN) return MB_INFO_NONE;
        return (uint8_t)(i < VLEN ? VERSION[i] : ' ');
    }
    return MB_INFO_NONE;
}
