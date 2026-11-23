/* Recording fake of the Pico SDK calls mb_bus.c and mb_audio.c use, with an optional PIO interpreter and DMA engine. */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef unsigned int uint;

#ifndef __not_in_flash_func
#define __not_in_flash_func(f) f
#endif
#define tight_loop_contents() ((void)0)
#define __dmb() ((void)0)
#define PICO_PIO_VERSION 1

typedef struct {
    volatile uint32_t fstat;
    volatile uint32_t txf[4];
    volatile uint32_t rxf[4];
    volatile uint32_t irq;
} pio_hw_t;
typedef pio_hw_t *PIO;
extern pio_hw_t fake_pio_hw[3];
#define pio0 (&fake_pio_hw[0])
#define pio1 (&fake_pio_hw[1])
#define pio2 (&fake_pio_hw[2])
#define PIO_FSTAT_RXEMPTY_LSB 8

typedef struct {
    volatile uint32_t ints0;
    struct { volatile uint32_t al1_transfer_count_trig, al3_read_addr_trig; } ch[16];
} dma_hw_t;
extern dma_hw_t fake_dma_hw;
#define dma_hw (&fake_dma_hw)

typedef struct { struct { volatile uint32_t cc; } slice[12]; } pwm_hw_t;
extern pwm_hw_t fake_pwm_hw;
#define pwm_hw (&fake_pwm_hw)

enum { GPIO_IN = 0, GPIO_OUT = 1 };
enum gpio_function { GPIO_FUNC_PWM = 4, GPIO_FUNC_PIO0 = 6 };
enum gpio_drive_strength { GPIO_DRIVE_STRENGTH_2MA, GPIO_DRIVE_STRENGTH_4MA, GPIO_DRIVE_STRENGTH_8MA, GPIO_DRIVE_STRENGTH_12MA };
enum gpio_slew_rate { GPIO_SLEW_RATE_SLOW, GPIO_SLEW_RATE_FAST };
enum { GPIO_IRQ_EDGE_FALL = 0x4, GPIO_IRQ_EDGE_RISE = 0x8 };
enum { PIO1_IRQ_0 = 17, PIO2_IRQ_0 = 19, DMA_IRQ_0 = 10 };
enum dma_channel_transfer_size { DMA_SIZE_8 = 0, DMA_SIZE_16 = 1, DMA_SIZE_32 = 2 };
enum pio_interrupt_source { pis_interrupt0 = 8, pis_interrupt1 = 9 };
enum clock_index { clk_sys = 5 };
enum pio_mov_status_type { STATUS_TX_LESSTHAN = 0, STATUS_RX_LESSTHAN = 1 };
enum pio_src_dest { pio_pins = 0, pio_x = 1, pio_y = 2, pio_null = 3, pio_isr = 6, pio_osr = 7 };
#define FAKE_DREQ_FORCE 0x3F

typedef void (*irq_handler_t)(void);
typedef void (*gpio_irq_callback_t)(uint gpio, uint32_t event_mask);

typedef struct pio_program {
    const uint16_t *instructions;
    uint8_t length;
    int8_t origin;
    uint8_t pio_version;
    uint32_t used_gpio_ranges;
} pio_program_t;

typedef struct {
    uint8_t wrap_target, wrap;
    uint8_t in_base, in_count, out_base, out_count, set_base, set_count, jmp_pin;
    bool    in_shift_right, autopush, out_shift_right, autopull;
    uint8_t push_thresh, pull_thresh;
    bool    status_rx;
    uint8_t status_n;
} pio_sm_config;

typedef struct { uint size, dreq, chain_to; bool read_inc, write_inc, high_priority; } dma_channel_config;

#define FAKE_GPIOS 32
#define FAKE_DMA_CH 16
#define FAKE_IRQS 32
typedef struct {
    struct { bool init, pio, pulls_off, hysteresis, level, dir_out; int drive, slew, function; } gpio[FAKE_GPIOS];
    gpio_irq_callback_t gpio_cb;
    uint32_t gpio_irq_events[FAKE_GPIOS];
    bool gpio_irq_enabled[FAKE_GPIOS];
    irq_handler_t irq_handler[FAKE_IRQS];
    bool irq_enabled[FAKE_IRQS];
    uint32_t sm_claimed[3], sm_enabled[3];
    int sm_init_count[3][4];
    uint sm_init_pc[3][4];
    pio_sm_config sm_cfg[3][4];
    uint32_t pindirs[3], pindirs_mask[3];
    uint32_t irq_cleared[3];
    uint32_t irq0_sources[3];
    int programs_added[3];
    uint next_offset[3];
    int fifo_clears[3][4];
    uint32_t tx_log[3][4][8];
    int tx_puts[3][4];
    int execs[3][4];
    uint64_t now_us;
    bool dma_claimed[FAKE_DMA_CH];
    dma_channel_config dma_cfg[FAKE_DMA_CH];
    volatile void *dma_write[FAKE_DMA_CH];
    const volatile void *dma_read[FAKE_DMA_CH];
    uint32_t dma_count[FAKE_DMA_CH];
    bool dma_started[FAKE_DMA_CH], dma_irq0[FAKE_DMA_CH];
    int dma_timer_claimed;
    uint16_t dma_timer_num, dma_timer_den;
    uint pwm_wrap, pwm_div_int, pwm_div_frac;
    bool pwm_enabled;
    uint16_t pwm_level_a, pwm_level_b;
    uint32_t sys_hz;
    uint64_t busy_waited_us;
} fake_sdk_t;
extern fake_sdk_t fake;
void fake_sdk_reset(void);

struct pio_sim_t_;
void fake_sdk_attach_sim(void *pio_sims[3], void *gpio);   /* pio_sim_t *[3], pio_sim_gpio_t * */
void fake_sdk_dma_step(void);
void fake_sdk_sync_irq_regs(void);
void fake_sdk_set_call_cost(void (*advance)(int clocks), int clocks);

void gpio_init(uint gpio);
void gpio_disable_pulls(uint gpio);
void gpio_put(uint gpio, bool value);
bool gpio_get(uint gpio);
void gpio_set_dir(uint gpio, bool out);
void gpio_set_drive_strength(uint gpio, enum gpio_drive_strength drive);
void gpio_set_slew_rate(uint gpio, enum gpio_slew_rate slew);
void gpio_set_input_hysteresis_enabled(uint gpio, bool enabled);
void gpio_set_function(uint gpio, enum gpio_function fn);
void gpio_set_irq_enabled_with_callback(uint gpio, uint32_t events, bool enabled, gpio_irq_callback_t cb);

void irq_set_exclusive_handler(uint num, irq_handler_t handler);
void irq_set_enabled(uint num, bool enabled);

uint pio_add_program(PIO pio, const pio_program_t *program);
void pio_sm_init(PIO pio, uint sm, uint initial_pc, const pio_sm_config *config);
void pio_sm_set_enabled(PIO pio, uint sm, bool enabled);
void pio_set_sm_mask_enabled(PIO pio, uint32_t mask, bool enabled);
void pio_sm_set_pindirs_with_mask(PIO pio, uint sm, uint32_t pindirs, uint32_t mask);
void pio_interrupt_clear(PIO pio, uint num);
void pio_set_irq0_source_enabled(PIO pio, enum pio_interrupt_source source, bool enabled);
void pio_claim_sm_mask(PIO pio, uint mask);
void pio_gpio_init(PIO pio, uint pin);
void pio_sm_clear_fifos(PIO pio, uint sm);
uint pio_get_dreq(PIO pio, uint sm, bool is_tx);
void pio_sm_put(PIO pio, uint sm, uint32_t data);
bool pio_sm_is_tx_fifo_full(PIO pio, uint sm);
bool pio_sm_is_rx_fifo_empty(PIO pio, uint sm);
uint32_t pio_sm_get(PIO pio, uint sm);
void pio_sm_exec(PIO pio, uint sm, uint instr);
pio_sm_config pio_get_default_sm_config(void);
void sm_config_set_wrap(pio_sm_config *c, uint wrap_target, uint wrap);
void sm_config_set_in_pins(pio_sm_config *c, uint in_base);
void sm_config_set_in_pin_count(pio_sm_config *c, uint in_count);
void sm_config_set_in_shift(pio_sm_config *c, bool shift_right, bool autopush, uint push_threshold);
void sm_config_set_out_shift(pio_sm_config *c, bool shift_right, bool autopull, uint pull_threshold);
void sm_config_set_out_pins(pio_sm_config *c, uint out_base, uint out_count);
void sm_config_set_jmp_pin(pio_sm_config *c, uint pin);
void sm_config_set_mov_status(pio_sm_config *c, enum pio_mov_status_type status_sel, uint status_n);
static inline uint pio_encode_jmp(uint addr) { return addr; }
static inline uint pio_encode_pull(bool if_empty, bool block) {
    return 0x8080u | ((((if_empty ? 2u : 0u) | (block ? 1u : 0u))) << 5);
}
static inline uint pio_encode_mov(enum pio_src_dest dest, enum pio_src_dest src) {
    return 0xA000u | (((uint)dest & 7u) << 5) | ((uint)src & 7u);
}

void busy_wait_us(uint64_t us);
void busy_wait_at_least_cycles(uint32_t cycles);

int  dma_claim_unused_channel(bool required);
int  dma_claim_unused_timer(bool required);
dma_channel_config dma_channel_get_default_config(uint channel);
void channel_config_set_transfer_data_size(dma_channel_config *c, enum dma_channel_transfer_size size);
void channel_config_set_read_increment(dma_channel_config *c, bool incr);
void channel_config_set_write_increment(dma_channel_config *c, bool incr);
void channel_config_set_dreq(dma_channel_config *c, uint dreq);
void channel_config_set_chain_to(dma_channel_config *c, uint chain_to);
void channel_config_set_high_priority(dma_channel_config *c, bool high_priority);
void dma_channel_configure(uint channel, const dma_channel_config *config, volatile void *write_addr,
                           const volatile void *read_addr, uint transfer_count, bool trigger);
void dma_channel_start(uint channel);
void dma_channel_set_read_addr(uint channel, const volatile void *read_addr, bool trigger);
void dma_channel_set_irq0_enabled(uint channel, bool enabled);
void dma_timer_set_fraction(uint timer, uint16_t numerator, uint16_t denominator);
uint dma_get_timer_dreq(uint timer_num);
static inline uint32_t dma_encode_endless_transfer_count(void) { return 0xFFFFFFFFu; }

uint pwm_gpio_to_slice_num(uint gpio);
void pwm_set_clkdiv_int_frac(uint slice, uint8_t integer, uint8_t fract);
void pwm_set_wrap(uint slice, uint16_t wrap);
void pwm_set_both_levels(uint slice, uint16_t level_a, uint16_t level_b);
void pwm_set_enabled(uint slice, bool enabled);
uint32_t clock_get_hz(enum clock_index clk);
