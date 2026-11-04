/* Frozen hand-ASM Timer1/Timer2 read: recompiling the C bus path shifts the ~0.5 us slot-read timing and breaks autodetect, so this path never moves (keep in lockstep with mb_via.c). */
    .syntax unified
#ifndef FASTREAD_TEST
    .cpu cortex-m33          /* firmware target. The qemu correctness test overrides via -mcpu */
#endif
    .thumb

    .section .time_critical.mb_bus_dispatch, "ax", %progbits
    .balign 16
    .global mb_bus_dispatch
    .thumb_func
mb_bus_dispatch:
    /* ---- decode (caller-saved regs only, no frame, so the slow path can tail-call) ---- */
    uxtb    r1, r0                  /* r1 = a = acc & 0xFF                     */
    tst     r0, #0x200              /* read? (bit 9)                           */
    beq     .Lslow                  /*   write -> C handler                    */
    tst     r1, #0x60               /* sel = a & 0x60 (speech region)          */
    bne     .Lslow                  /*   speech -> C handler                   */
    and     r2, r1, #0x0F           /* r2 = off = a & 0x0F                     */
    cmp     r2, #4                  /* T1C-L?                                  */
    beq     .Lt1
    cmp     r2, #5                  /* T1C-H?                                  */
    beq     .Lt1
    cmp     r2, #8                  /* T2C-L?                                  */
    beq     .Lt2
    cmp     r2, #9                  /* T2C-H?                                  */
    beq     .Lt2
    /* Plain register reads: drive reg[ch][off] directly to beat the C-handler latency. Ports
     * (0/1/0F) and SR (0A) stay on the C path (they compute a value). No read side-effect, no frame. */
    cmp     r2, #2
    blo     .Lslow                  /*   0,1 = ORB/ORA ports -> C              */
    cmp     r2, #0x0A
    beq     .Lslow                  /*   A = SR (version readout) -> C         */
    cmp     r2, #0x0F
    beq     .Lslow                  /*   F = ORA no-handshake port -> C        */
    lsrs    r3, r1, #7              /* r3 = ch = a >> 7                        */
    ldr     r12, =reg
    add     r12, r12, r3, lsl #4    /* r12 = &reg[ch][0]                       */
    ldrb    r0, [r12, r2]           /* r0 = reg[ch][off]                       */
    cmp     r2, #0x0E              /* IER reads back with bit7 = 1            */
    it      eq
    orreq   r0, r0, #0x80
#ifdef FASTREAD_TEST
    ldr     r3, =fastread_test_sink
#else
    ldr     r3, =0x50200018         /* pio0->txf[MB_SM_READ]                   */
#endif
    str     r0, [r3]                /* put_rdata(reg[ch][off])                 */
    bx      lr
.Lslow:
    b       mb_via_access           /* tail call, r0 = acc preserved           */

.Lt1:
    /* ---- fast path. r1=a, r2=off(4|5). frame: we call the C ack at the end ---- */
    push    {r4, r5, r6, r7, lr}
    lsrs    r3, r1, #7              /* r3 = ch = a >> 7  (0 or 1)              */

    ldr     r7, =reg                /* r7 = &reg[0][0]                         */
    add     r7, r7, r3, lsl #4      /* r7 = &reg[ch][0]   (ch * 16)            */
    ldrb    r4, [r7, #6]            /* T1LL                                    */
    ldrb    r5, [r7, #7]            /* T1LH                                    */
    orr     r4, r4, r5, lsl #8      /* r4 = n = latch (16-bit)                 */

    ldr     r6, =g_phi2_mirror
    ldr     r6, [r6]                /* r6 = g_phi2_mirror                      */
    rsb     r6, r6, #0              /* r6 = now = 0 - g_phi2_mirror            */
    ldr     r5, =t1_arm
    ldr     r5, [r5, r3, lsl #2]    /* r5 = t1_arm[ch]                         */
    subs    r5, r6, r5             /* r5 = e = now - arm  (wrap-safe 32-bit)  */

    ldrb    r6, [r7, #0x0B]         /* r6 = ACR                                */
    tst     r6, #0x40              /* free-run? (ACR bit 6)                   */
    beq     .Lsub                   /*   one-shot: skip the modulo             */
    /* free-run reduce: e % (n+2) via reciprocal multiply (no UDIV). Mirror of t1_divmod_reduce()
     * in mb_via.c -- keep in lockstep. live: r2=off r3=ch r4=n r5=e r7=&reg[ch], scratch r0 r1 r6 r12. */
    ldr     r0, =t1_recip_more
    ldrb    r12, [r0, r3]           /* r12 = more (shift | add<<6 | pow2<<7)   */
    tst     r12, #0x80              /* power-of-two divisor? (shift only)      */
    bne     .Lt1_pow2
    ldr     r0, =t1_recip
    ldr     r0, [r0, r3, lsl #2]    /* r0 = magic                              */
    umull   r1, r6, r0, r5          /* r6 = qh = MULHI(magic, e)               */
    tst     r12, #0x40              /* add-case correction needed?             */
    beq     .Lt1_shift              /*   simple: shift qh                      */
    subs    r0, r5, r6              /* e - qh                                  */
    lsrs    r0, r0, #1              /* (e - qh) >> 1                           */
    add     r6, r0, r6              /* r6 = t = ((e - qh) >> 1) + qh           */
.Lt1_shift:
    and     r0, r12, #0x1F          /* shift amount                            */
    lsr     r6, r6, r0              /* q = (qh|t) >> shift                     */
    b       .Lt1_mod
.Lt1_pow2:
    and     r0, r12, #0x1F
    lsr     r6, r5, r0              /* q = e >> shift                          */
.Lt1_mod:
    add     r0, r4, #2              /* P = n + 2                               */
    mls     r5, r6, r0, r5          /* r5 = e - q*P = e % P                    */
.Lsub:
    subs    r0, r4, r5             /* r0 = c = n - (e or e%period)            */

    cmp     r2, #5                  /* high byte?                              */
    beq     .Lhigh
    and     r0, r0, #0xFF           /* off==4: low byte                        */
    b       .Ldrive
.Lhigh:
    ubfx    r0, r0, #8, #8          /* off==5: (c >> 8) & 0xFF                 */
.Ldrive:
#ifdef FASTREAD_TEST
    ldr     r6, =fastread_test_sink /* qemu correctness test: drive into a buffer */
#else
    ldr     r6, =0x50200018         /* pio0->txf[MB_SM_READ=2]                 */
#endif
    str     r0, [r6]                /* put_rdata(byte) -- drive, before ack    */

    mov     r0, r3                  /* arg0 = ch                               */
    mov     r1, r2                  /* arg1 = off                              */
    bl      mb_via_read_ack_locked  /* flag-clear side effect (not window-critical) */
    pop     {r4, r5, r6, r7, pc}

.Lt2:
    /* ---- T2 fast path. r1=a, r2=off(8|9). timed or PB6-pulse, no latch reload ---- */
    push    {r4, r5, r6, r7, lr}
    lsrs    r3, r1, #7              /* r3 = ch = a >> 7  (0 or 1)              */

    ldr     r7, =reg                /* r7 = &reg[0][0]                         */
    add     r7, r7, r3, lsl #4      /* r7 = &reg[ch][0]   (ch * 16)            */
    ldrb    r4, [r7, #8]            /* T2C-L (low-order latch/counter)         */
    ldrb    r5, [r7, #9]            /* T2C-H                                   */
    orr     r4, r4, r5, lsl #8      /* r4 = n = T2 load (16-bit)               */

    ldrb    r6, [r7, #0x0B]         /* r6 = ACR                                */
    tst     r6, #0x20              /* PB6 pulse mode? (ACR bit 5)             */
    bne     .Lt2byte                /*   pulse: no time source, c = n          */
    ldr     r6, =g_phi2_mirror
    ldr     r6, [r6]                /* r6 = g_phi2_mirror                      */
    rsb     r6, r6, #0              /* r6 = now = 0 - g_phi2_mirror            */
    ldr     r5, =t2_arm
    ldr     r5, [r5, r3, lsl #2]    /* r5 = t2_arm[ch]                         */
    subs    r5, r6, r5             /* r5 = e = now - arm  (wrap-safe 32-bit)  */
    subs    r4, r4, r5             /* r4 = c = n - e  (timed, 16-bit wrap)    */
.Lt2byte:
    cmp     r2, #9                  /* high byte?                              */
    beq     .Lt2high
    and     r0, r4, #0xFF           /* off==8: low byte                        */
    b       .Ldrive
.Lt2high:
    ubfx    r0, r4, #8, #8          /* off==9: (c >> 8) & 0xFF                 */
    b       .Ldrive
    .pool
