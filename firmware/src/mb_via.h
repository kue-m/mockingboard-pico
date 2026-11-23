#pragma once

#include <stdint.h>
#include <stdbool.h>

void mb_via_init(void);
void mb_via_poll(void);
void mb_via_access(uint32_t acc);
void mb_via_reset(bool asserted);

