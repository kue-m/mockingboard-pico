/* Host shim: channel setup is inert. The harness raises completions itself through ints0. */
#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef struct { uint32_t ctrl; } dma_channel_config;
enum dma_channel_transfer_size { DMA_SIZE_8, DMA_SIZE_16, DMA_SIZE_32 };
typedef struct { volatile uint32_t read_addr, write_addr, transfer_count, ctrl_trig, al1_transfer_count_trig; } dma_channel_hw_t;
typedef struct { volatile uint32_t ints0; dma_channel_hw_t ch[16]; } dma_hw_t;
extern dma_hw_t g_dma_hw;
#define dma_hw (&g_dma_hw)
extern int g_dma_next_channel;
static inline int dma_claim_unused_channel(bool required) { (void)required; return g_dma_next_channel++; }
static inline int dma_claim_unused_timer(bool required) { (void)required; return 0; }
static inline void dma_timer_set_fraction(unsigned t, uint16_t n, uint16_t d) { (void)t; (void)n; (void)d; }
static inline unsigned dma_get_timer_dreq(unsigned t) { return t; }
static inline dma_channel_config dma_channel_get_default_config(unsigned ch) { (void)ch; dma_channel_config c = { 0 }; return c; }
static inline void channel_config_set_transfer_data_size(dma_channel_config *c, enum dma_channel_transfer_size s) { (void)c; (void)s; }
static inline void channel_config_set_read_increment(dma_channel_config *c, bool v) { (void)c; (void)v; }
static inline void channel_config_set_write_increment(dma_channel_config *c, bool v) { (void)c; (void)v; }
static inline void channel_config_set_dreq(dma_channel_config *c, unsigned d) { (void)c; (void)d; }
static inline void channel_config_set_chain_to(dma_channel_config *c, unsigned ch) { (void)c; (void)ch; }
static inline void dma_channel_configure(unsigned ch, const dma_channel_config *c, volatile void *w,
                                         const volatile void *r, unsigned n, bool trig) {
    (void)ch; (void)c; (void)w; (void)r; (void)n; (void)trig;
}
static inline void dma_channel_set_read_addr(unsigned ch, const volatile void *r, bool trig) { (void)ch; (void)r; (void)trig; }
static inline void dma_channel_set_irq0_enabled(unsigned ch, bool e) { (void)ch; (void)e; }
static inline void dma_channel_start(unsigned ch) { (void)ch; }
