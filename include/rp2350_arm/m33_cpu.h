/*
 * ARM Cortex-M33 CPU Engine for RP2350 Emulation
 *
 * ARMv8-M Mainline core. The M33 is a superset of M0+ — it executes
 * the same Thumb-1 instructions plus the full Thumb-2 instruction set.
 *
 * Implementation strategy:
 *   The M33 reuses the existing M0+ CPU engine (cpu.c + instructions.c +
 *   thumb32.c) since all M0+ instructions are valid on M33, and thumb32.c
 *   already implements the Thumb-2 extensions (MOVW/MOVT, B.W, SDIV/UDIV,
 *   CLZ, MLA/MLS, SMULL/UMULL, UBFX/SBFX, BFI/BFC, wide loads/stores,
 *   data processing with shifted registers, TBB/TBH, etc.).
 *
 * M33-specific additions over M0+:
 *   - BASEPRI register (priority masking for interrupts)
 *   - More NVIC priority bits (8 bits vs M0+'s 2 bits)
 *   - CPUID returns Cortex-M33 identifier (0x410FD210)
 *   - RP2350 memory map (520KB SRAM, 32KB ROM via rv_membus)
 *   - 52 external interrupt sources (vs 26 on RP2040)
 *   - VTOR at any 128-byte boundary (vs 256-byte on M0+)
 *   - FPU (optional, not yet implemented)
 *   - TrustZone (not implemented — all code runs in secure world)
 *   - DSP instructions (saturating arithmetic — partially stubbed)
 *
 * The M33 mode is selected via `-arch m33` and uses the RP2350 peripheral
 * set while running on the existing ARM CPU engine.
 */

#ifndef M33_CPU_H
#define M33_CPU_H

#include <stdint.h>

/* Cortex-M33 CPUID value */
#define M33_CPUID  0x410FD210  /* ARM Cortex-M33 r0p0 */

/* BASEPRI register — blocks interrupts at or below this priority.
 * 0 = no masking (all interrupts pass). Non-zero = mask threshold.
 * Only the top N bits are implemented (depends on priority bits). */
#define M33_BASEPRI_MASK  0xFF

/* Initialize M33-specific state (BASEPRI, CPUID, etc.)
 * Call after standard cpu_init() to overlay M33 features. */
void m33_init_overlay(void);

/* Read/write BASEPRI for MSR/MRS */
extern uint32_t m33_basepri;

/* Check if an interrupt at 'priority' is masked by BASEPRI.
 * Returns 1 if blocked, 0 if allowed. */
static inline int m33_basepri_blocks(uint32_t priority) {
    if (m33_basepri == 0) return 0;  /* No masking */
    return priority >= m33_basepri;
}

#endif /* M33_CPU_H */
