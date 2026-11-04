/* Bus lines are contiguous (GP0..GP18) so the PIO grabs the whole snapshot with one `in pins`. */
#pragma once

#define PIN_DATA_BASE   0    // D0..D7  = GP0..GP7   (bidirectional)
#define PIN_ADDR_BASE   8    // A0..A7  = GP8..GP15  (input)
#define PIN_IOSEL      16    // /IOSEL  = GP16       (low = our $Cn00 page)
#define PIN_RW         17    // R/W     = GP17       (1=read 0=write)
#define PIN_PHI0       18    // PHI0    = GP18       (phase-0 clock)
#define PIN_RES        19    // /RES    = GP19       (low = reset)
#define PIN_IRQ        20    // /IRQ    = GP20       (open-drain output)
#define PIN_AUDIO_L    26    // PWM L   = GP26       (output only, non-FT ADC pin)
#define PIN_AUDIO_R    27    // PWM R   = GP27       (output only, non-FT ADC pin)

#define N_DATA          8
#define N_ADDR          8
