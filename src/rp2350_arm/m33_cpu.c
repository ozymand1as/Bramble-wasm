/*
 * ARM Cortex-M33 CPU Engine for RP2350 Emulation
 *
 * Provides M33-specific overlay on top of the existing M0+ CPU engine:
 *   - BASEPRI register for priority-based interrupt masking
 *   - M33 CPUID value
 *   - MSR/MRS support for BASEPRI/BASEPRI_MAX
 *
 * The M33 mode reuses cpu_step()/dual_core_step() from cpu.c with the
 * full Thumb-2 instruction set from thumb32.c. This file only adds the
 * M33-specific registers and initialization.
 */

#include <string.h>
#include <stdio.h>
#include "rp2350_arm/m33_cpu.h"
#include "emulator.h"
#include "nvic.h"

/* BASEPRI register (per M33 spec, affects interrupt delivery) */
uint32_t m33_basepri = 0;

/* M33 overlay: call after cpu_init() to configure M33-specific state */
void m33_init_overlay(void) {
    m33_basepri = 0;
    nvic_cpuid_value = M33_CPUID;
    fprintf(stderr, "[M33] Cortex-M33 overlay initialized (BASEPRI=0, CPUID=0x%08X)\n",
            M33_CPUID);
}
