/* Host shim: __dmb is an RP2350 data memory barrier. A full compiler/thread fence on host. */
#ifndef MB_SHIM_HW_SYNC_H
#define MB_SHIM_HW_SYNC_H
#ifndef __dmb
#define __dmb() __sync_synchronize()
#endif
#endif
