/* Host shim: only __not_in_flash_func matters on the host, as a no-op. */
#ifndef MB_SHIM_PICO_STDLIB_H
#define MB_SHIM_PICO_STDLIB_H
#include <stdint.h>
#include <stdlib.h>
#ifndef __not_in_flash_func
#define __not_in_flash_func(f) f
#endif
#endif
