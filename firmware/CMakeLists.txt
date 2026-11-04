cmake_minimum_required(VERSION 3.13)

# Production board: non-wireless Pico 2 (set before pico_sdk_init; local-pre.cmake may override).
set(PICO_BOARD pico2 CACHE STRING "Board type")
include(${CMAKE_CURRENT_LIST_DIR}/local-pre.cmake OPTIONAL)

include(pico_sdk_import.cmake)

project(mockingboard_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(mockingboard_pico
    src/main.c
    src/mb_bus.c
    src/mb_audio.c
    src/mb_via.c
    src/mb_speech.c
    src/mb_speech_bus.c
    src/mb_version.c
    src/mb_fastread.S
    vendor/emu2149/emu2149.c
    vendor/ssi263/ssi263.c
    vendor/ssi263/ssi263dsp.c
)

pico_generate_pio_header(mockingboard_pico ${CMAKE_CURRENT_LIST_DIR}/src/mb_bus.pio)

# Version header, regenerated each build from the top-level VERSION file + git state; rewritten
# only when the value changes, so an unchanged version forces no relink.
find_package(Git QUIET)
set(MB_VERSION_GEN ${CMAKE_CURRENT_BINARY_DIR}/mb_version_gen.h)
add_custom_target(mb_version_gen_tgt
    BYPRODUCTS ${MB_VERSION_GEN}
    COMMAND ${CMAKE_COMMAND}
        -DGIT_EXECUTABLE=${GIT_EXECUTABLE}
        -DSRC_DIR=${CMAKE_CURRENT_LIST_DIR}
        -DVERSION_FILE=${CMAKE_CURRENT_LIST_DIR}/../VERSION
        -DOUT=${MB_VERSION_GEN}
        -P ${CMAKE_CURRENT_LIST_DIR}/cmake/mb_version.cmake
    COMMENT "Generating firmware version header"
    VERBATIM
)
add_dependencies(mockingboard_pico mb_version_gen_tgt)

# SSI-263 engine is float32 (no double FPU): keep float literals single-precision, and cut the FIR
# to 13 taps (~15x cheaper than the 193-tap reference, still intelligible) to fit the sample budget.
set_source_files_properties(
    vendor/ssi263/ssi263.c vendor/ssi263/ssi263dsp.c
    PROPERTIES COMPILE_OPTIONS "-fsingle-precision-constant;-DSSI263_FIR_TAPS_PER_OS=12"
)

target_include_directories(mockingboard_pico PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/vendor/emu2149
    ${CMAKE_CURRENT_LIST_DIR}/vendor/ssi263
)

target_link_libraries(mockingboard_pico
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_pwm
    hardware_dma
    hardware_clocks
)

# Run from RAM: no XIP/flash-fetch stalls in the per-sample audio path.
pico_set_binary_type(mockingboard_pico copy_to_ram)

# System clock. 200 MHz holds the slot read-response inside the 6502 window (150 MHz missed); the
# PWM carrier and 50 kHz sample pacing derive from clk_sys, so this stays overridable.
set(MB_SYSCLK_KHZ 200000 CACHE STRING "System clock in kHz (200000 production)")
target_compile_definitions(mockingboard_pico PRIVATE MB_SYSCLK_KHZ=${MB_SYSCLK_KHZ})

# A/B isolation: OFF compiles out the SSI-263 A/R -> CA1 interrupt path (keeps decode + writes).
option(SPEECH_AR_IRQ "Enable the SSI-263 A/R -> 6522 CA1 interrupt path" ON)
if(NOT SPEECH_AR_IRQ)
    target_compile_definitions(mockingboard_pico PRIVATE MB_SPEECH_AR_IRQ=0)
endif()

# Boot-time audible PHI2-counter self-test (no WiFi).
option(MB_PHI2_SELFTEST "Boot-time audible PHI2 counter self-test (no WiFi)" OFF)
if(MB_PHI2_SELFTEST)
    target_compile_definitions(mockingboard_pico PRIVATE MB_PHI2_SELFTEST=1 MB_CORE1_GATED=1)
endif()

# Steady equal ~440 Hz tone on both channels via the AY path; any L/R difference is analog HW.
option(MB_TONE_TEST "Steady equal tone on both channels via the AY/music path (HW test)" OFF)
if(MB_TONE_TEST)
    target_compile_definitions(mockingboard_pico PRIVATE MB_TONE_TEST=1)
endif()

# Route SSI-263 speech to the opposite channel (for a board with an impaired right output).
option(SPEECH_SWAP_CH "Route SSI-263 speech to the opposite channel" OFF)
if(SPEECH_SWAP_CH)
    target_compile_definitions(mockingboard_pico PRIVATE SPEECH_SWAP_CH=1)
endif()

# Default read drive = 0x00, so a read-response miss reads "not ready", never the floating bus.
option(MB_FAILSAFE_READ "Fail-safe read state machine (never floats the data bus)" ON)
if(MB_FAILSAFE_READ)
    target_compile_definitions(mockingboard_pico PRIVATE MB_FAILSAFE_READ=1)
endif()

# Optional untracked local build extensions (local-post.cmake, absent in the standard tree).
include(${CMAKE_CURRENT_LIST_DIR}/local-post.cmake OPTIONAL)

pico_add_extra_outputs(mockingboard_pico)

# USB serial stdio: OFF by default (the card is BOOTSEL-flashed, and the USB SOF IRQ perturbs the
# bus-service core). Bring-up builds that print over USB pass -DMB_STDIO_USB=ON.
option(MB_STDIO_USB "Enable USB serial stdio (bring-up profiling)" OFF)
if(MB_STDIO_USB)
    pico_enable_stdio_usb(mockingboard_pico 1)
else()
    pico_enable_stdio_usb(mockingboard_pico 0)
endif()
pico_enable_stdio_uart(mockingboard_pico 0)
