/*
 * RP2350 RISC-V Bootrom
 *
 * Provides:
 *   - Reset vector at 0x00000000 (jump to boot code)
 *   - Boot code: set SP, GP, mtvec, jump to flash
 *   - ROM function table with lookup mechanism
 *   - Function stubs at well-known addresses (intercepted by rv_cpu_step)
 *   - Trap handler for unhandled exceptions
 *
 * ROM Function Table Layout (RP2350-compatible):
 *   0x0010: Magic ('R', 'P', 0x02) — RP2350 identifier
 *   0x0014: Pointer to function table
 *   0x0018: Pointer to data table
 *   0x001C: Pointer to table lookup function
 *   0x0100: Function table entries [32-bit code, 32-bit func_ptr] ...
 *   0x0200: Data table entries
 *   0x0400: ROM function stubs (JALR x0, ra, 0 — intercepted by PC check)
 *
 * ROM Function Interception:
 *   Functions at 0x0400-0x04FF are stub RET instructions.
 *   rv_rom_intercept() checks PC and performs native C operations
 *   for memcpy, memset, flash operations, etc.
 */

#ifndef RV_BOOTROM_H
#define RV_BOOTROM_H

#include <stdint.h>
#include "rp2350_rv/rv_cpu.h"

/* ROM function addresses (stubs placed here, intercepted by PC) */
#define RV_ROM_FN_TABLE_LOOKUP  0x0300
#define RV_ROM_FN_MEMCPY        0x0400
#define RV_ROM_FN_MEMSET        0x0404
#define RV_ROM_FN_MEMCPY4       0x0408
#define RV_ROM_FN_MEMSET4       0x040C
#define RV_ROM_FN_POPCOUNT32    0x0410
#define RV_ROM_FN_CLZ32         0x0414
#define RV_ROM_FN_CTZ32         0x0418
#define RV_ROM_FN_REVERSE32     0x041C
#define RV_ROM_FN_FLASH_ENTER   0x0420
#define RV_ROM_FN_FLASH_EXIT    0x0424
#define RV_ROM_FN_FLASH_ERASE   0x0428
#define RV_ROM_FN_FLASH_PROGRAM 0x042C
#define RV_ROM_FN_REBOOT        0x0430
#define RV_ROM_FN_SET_STACK     0x0434
#define RV_ROM_FN_LAST          0x0438

/* ROM function table codes (used by rom_func_lookup) */
#define RV_ROM_CODE_MEMCPY       0x4350  /* 'CP' */
#define RV_ROM_CODE_MEMSET       0x5453  /* 'ST' */
#define RV_ROM_CODE_POPCOUNT     0x5043  /* 'PC' */
#define RV_ROM_CODE_CLZ          0x5A4C  /* 'ZL' */
#define RV_ROM_CODE_CTZ          0x5A54  /* 'ZT' */
#define RV_ROM_CODE_REVERSE      0x5652  /* 'RV' */
#define RV_ROM_CODE_FLASH_ERASE  0x4552  /* 'RE' */
#define RV_ROM_CODE_FLASH_PROG   0x5052  /* 'RP' */
#define RV_ROM_CODE_REBOOT       0x4252  /* 'RB' */

/* Populate the RP2350 ROM buffer with RISC-V bootrom code and function table. */
uint32_t rv_bootrom_init(uint8_t *rom, uint32_t rom_size,
                         uint32_t flash_base, uint32_t sram_end);

/* Check if PC is at a ROM function stub and execute it natively.
 * Returns 1 if intercepted (PC advanced to return address), 0 if not a ROM function. */
int rv_rom_intercept(rv_cpu_state_t *cpu);

#endif /* RV_BOOTROM_H */
