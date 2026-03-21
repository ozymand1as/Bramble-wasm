/*
 * Picobin IMAGE_DEF Block Parser for RP2350
 *
 * The RP2350 bootrom uses picobin blocks (IMAGE_DEF) in the first 4KB of
 * flash to determine how to boot the firmware:
 *   - CPU mode: ARM (Cortex-M33) or RISC-V (Hazard3)
 *   - Entry point: initial PC
 *   - Stack pointer: initial SP
 *
 * Block format:
 *   0xFFFFDED3          - Block start marker
 *   [items...]          - Typed items (IMAGE_TYPE, ENTRY_POINT, etc.)
 *   block_loop_offset   - Offset to next block (0 = single block)
 *   0xAB123579          - Block end marker
 *
 * Item encoding (packed 32-bit word):
 *   bits[7:0]   = type
 *   bits[15:8]  = size (in 32-bit words, including header)
 *   bits[31:16] = type-specific flags
 */

#ifndef PICOBIN_H
#define PICOBIN_H

#include <stdint.h>

/* Block markers */
#define PICOBIN_BLOCK_START  0xFFFFDED3
#define PICOBIN_BLOCK_END    0xAB123579

/* Item types */
#define PICOBIN_ITEM_IMAGE_TYPE   0x42
#define PICOBIN_ITEM_ENTRY_POINT  0x44
#define PICOBIN_ITEM_VECTOR_TABLE 0x03
#define PICOBIN_ITEM_LAST         0xFF

/* IMAGE_TYPE flags */
#define PICOBIN_IMAGE_TYPE_EXE     0x0001
#define PICOBIN_IMAGE_TYPE_DATA    0x0002
#define PICOBIN_EXE_CPU_ARM        (0 << 8)
#define PICOBIN_EXE_CPU_RISCV      (1 << 8)
#define PICOBIN_EXE_CHIP_RP2040    (0 << 12)
#define PICOBIN_EXE_CHIP_RP2350    (1 << 12)

/* Parsed IMAGE_DEF result */
typedef struct {
    int found;           /* 1 if a valid block was found */
    int is_riscv;        /* 1 if CPU type is RISC-V */
    int is_arm;          /* 1 if CPU type is ARM */
    int is_rp2350;       /* 1 if chip is RP2350 */
    uint32_t entry_pc;   /* Entry point PC (from ENTRY_POINT item) */
    uint32_t entry_sp;   /* Entry SP (from ENTRY_POINT item) */
    uint32_t vtor;       /* Vector table address (from VECTOR_TABLE item, ARM only) */
} picobin_info_t;

/* Scan the first 4KB of flash for a picobin IMAGE_DEF block.
 * Returns parsed info. flash must be at least scan_size bytes. */
picobin_info_t picobin_scan(const uint8_t *flash, uint32_t scan_size);

#endif /* PICOBIN_H */
