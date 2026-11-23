/* An index write to VIA1 SR ($Cn0A) stages that byte of the page selected by VIA2 SR ($Cn8A). Never 0. */
#pragma once

#include <stdint.h>

#define MB_VERSION_STR_LEN 32   // fixed field width, space-padded/truncated

#define MB_INFO_PAGE_VERSION 0
#define MB_INFO_NONE         0xFF

void    mb_info_select_page(uint8_t page);
uint8_t mb_info_stage(uint8_t index);
