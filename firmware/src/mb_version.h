/* Version readout: each Apple read of VIA1's Shift Register ($Cn0A) clocks out one byte of the version string. */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#define MB_VERSION_READ_LEN 32   // fixed field width, space-padded/truncated

// auto-arms on first read. Does not advance (deferred to mb_version_advance, so an un-acked read is harmless)
uint8_t mb_version_read_byte(void);
bool mb_version_armed(void);
void mb_version_advance(void);
void mb_version_disarm(void);
