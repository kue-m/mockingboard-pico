/* no ISR preemption in the single-threaded sim: the lock is a no-op */
#pragma once
#define PIO1_IRQ_0 0
#define SIO_IRQ_FIFO 25
static inline void irq_set_enabled(unsigned n, int e) { (void)n; (void)e; }
