/* Host shim: placement attributes are no-ops and the few SDK calls the audio path makes are inert. */
#ifndef MB_SHIM_PICO_STDLIB_H
#define MB_SHIM_PICO_STDLIB_H
#include <stdint.h>
#include <stdlib.h>
#ifndef __not_in_flash_func
#define __not_in_flash_func(f) f
#endif
#ifndef __scratch_x
#define __scratch_x(group)
#endif
typedef unsigned int uint;
#define GPIO_FUNC_PWM 4
static inline void gpio_set_function(unsigned pin, int fn) { (void)pin; (void)fn; }
static inline void busy_wait_us(uint64_t us) { (void)us; }
uint32_t time_us_32(void);
#endif
