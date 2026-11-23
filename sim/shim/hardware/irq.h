/* no ISR preemption in the single-threaded sim: the lock is a no-op */
#pragma once
#define PIO1_IRQ_0 0
#define SIO_IRQ_FIFO 25
static inline void irq_set_enabled(unsigned n, int e) { (void)n; (void)e; }
#define DMA_IRQ_0 10
typedef void (*irq_handler_t)(void);
static inline void irq_set_exclusive_handler(unsigned n, irq_handler_t h) { (void)n; (void)h; }
