/* Stand-in for the pioasm output of mb_bus.pio when pioasm is not installed. It holds symbols only, no instructions. */
#pragma once
#include "sdk_fake.h"

#define MB_FAKE_PROGRAM(name, len) \
    static const pio_program_t name##_program = { NULL, len, -1, 1, 0 }; \
    static inline pio_sm_config name##_program_get_default_config(uint o) { \
        pio_sm_config c = pio_get_default_sm_config(); sm_config_set_wrap(&c, o, o + len - 1); return c; }

MB_FAKE_PROGRAM(mb_addr, 5)
MB_FAKE_PROGRAM(mb_write, 3)
MB_FAKE_PROGRAM(mb_phi2, 4)
MB_FAKE_PROGRAM(mb_raddr, 11)
MB_FAKE_PROGRAM(mb_read_table, 12)
MB_FAKE_PROGRAM(mb_t1, 26)
MB_FAKE_PROGRAM(mb_t2, 16)
#define mb_t1_offset_start 0u
#define mb_t2_offset_start 0u

static inline void mb_addr_cfg(pio_sm_config *c)          { (void)c; }
static inline void mb_write_cfg(pio_sm_config *c)         { (void)c; }
static inline void mb_phi2_cfg(pio_sm_config *c)          { (void)c; }
static inline void mb_raddr_cfg(pio_sm_config *c)         { (void)c; }
static inline void mb_read_table_cfg(pio_sm_config *c)    { (void)c; }
static inline void mb_t1_cfg(pio_sm_config *c)            { (void)c; }
static inline void mb_t2_cfg(pio_sm_config *c)            { (void)c; }
