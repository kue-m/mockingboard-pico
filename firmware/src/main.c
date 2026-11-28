#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "mb_bus.h"
#include "mb_audio.h"
#include "mb_speech.h"
#include "mb_via.h"
#ifdef MB_PHI2_SELFTEST
#include "mb_selftest.h"
#endif


#ifdef MB_CORE1_GATED
static volatile bool g_bus_ready;
#endif

static void core1_main(void) {
#ifdef MB_PHI2_SELFTEST
    while (!g_bus_ready) tight_loop_contents();
    uint32_t t0 = mb_phi2_ticks();
    busy_wait_us(MB_PHI2_SELFTEST_WINDOW_US);
    bool phi2_ok = mb_phi2_advanced(t0, mb_phi2_ticks());
#endif

    audioInit(1020484, 50000);   // AY clock = Apple II CPU 1.0205 MHz (14318180*65/912)

#ifdef MB_PHI2_SELFTEST
    audioSelfTestPhi2(phi2_ok);
#endif

    audioRun();
}

int main(void) {
#ifndef MB_SYSCLK_KHZ
#define MB_SYSCLK_KHZ 200000
#endif
    set_sys_clock_khz(MB_SYSCLK_KHZ, true);

#ifdef PICO_SMPS_MODE_PIN
    // fixed-PWM mode: lower 3V3 ripple than the default PFM (undefined on Pico 2 W)
    gpio_init(PICO_SMPS_MODE_PIN);
    gpio_set_dir(PICO_SMPS_MODE_PIN, GPIO_OUT);
    gpio_put(PICO_SMPS_MODE_PIN, 1);
#endif

    stdio_init_all();      // USB and UART stdio are both disabled in the ship build


    multicore_launch_core1(core1_main);

    mb_bus_init();
#ifdef MB_CORE1_GATED
    g_bus_ready = true;
#endif
    mb_via_init();
    mb_bus_reset_handler(mb_via_reset);
    mb_bus_reset_start();

    uint32_t turn = 0;
    while (true) {
        mb_via_poll_timers();
        if (mb_bus_access_pending()) mb_via_access(mb_bus_get_access());
        if ((++turn & 15u) == 0) mb_via_poll_slow();
    }
}
