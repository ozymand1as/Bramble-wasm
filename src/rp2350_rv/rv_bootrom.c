/*
 * RP2350 RISC-V Bootrom
 *
 * Generates RISC-V bootrom with:
 *   - Reset vector → boot code (SP init, mtvec, flash jump)
 *   - ROM function table compatible with Pico SDK rom_func_lookup()
 *   - Function stubs at well-known addresses (intercepted by rv_rom_intercept)
 *   - Trap handler at 0x0004
 */

#include <string.h>
#include <stdio.h>
#include "rp2350_rv/rv_bootrom.h"
#include "rp2350_rv/rv_membus.h"
#include "rp2350_rv/rp2350_memmap.h"
#include "emulator.h"

/* Helper: write 32-bit LE word to ROM buffer */
static void rom_w32(uint8_t *rom, uint32_t off, uint32_t val) {
    rom[off+0] = (uint8_t)(val);
    rom[off+1] = (uint8_t)(val >> 8);
    rom[off+2] = (uint8_t)(val >> 16);
    rom[off+3] = (uint8_t)(val >> 24);
}

static void rom_w16(uint8_t *rom, uint32_t off, uint16_t val) {
    rom[off+0] = (uint8_t)(val);
    rom[off+1] = (uint8_t)(val >> 8);
}

/* RISC-V instruction encoders */
static uint32_t rv_jal(uint32_t rd, int32_t off) {
    uint32_t imm = (uint32_t)off;
    uint32_t enc = (rd << 7) | 0x6F;
    enc |= ((imm >> 12) & 0xFF) << 12;
    enc |= ((imm >> 11) & 1) << 20;
    enc |= ((imm >> 1) & 0x3FF) << 21;
    enc |= ((imm >> 20) & 1) << 31;
    return enc;
}

static uint32_t rv_lui(uint32_t rd, uint32_t imm_upper) {
    return (imm_upper & 0xFFFFF000) | (rd << 7) | 0x37;
}

static uint32_t rv_addi(uint32_t rd, uint32_t rs1, int32_t imm) {
    return ((uint32_t)imm << 20) | (rs1 << 15) | (0 << 12) | (rd << 7) | 0x13;
}

static uint32_t rv_jalr(uint32_t rd, uint32_t rs1, int32_t imm) {
    return ((uint32_t)imm << 20) | (rs1 << 15) | (0 << 12) | (rd << 7) | 0x67;
}

/* CSRRW rd, csr, rs1 */
static uint32_t rv_csrrw(uint32_t rd, uint16_t csr, uint32_t rs1) {
    return ((uint32_t)csr << 20) | (rs1 << 15) | (1 << 12) | (rd << 7) | 0x73;
}

/* RET = JALR x0, x1, 0 */
static uint32_t rv_ret(void) {
    return rv_jalr(0, 1, 0);
}

/* ========================================================================
 * ROM Initialization
 * ======================================================================== */

uint32_t rv_bootrom_init(uint8_t *rom, uint32_t rom_size,
                         uint32_t flash_base, uint32_t sram_end) {
    memset(rom, 0, rom_size);

    /* 0x0000: JAL x0, 0x20 (jump to boot code at 0x20) */
    rom_w32(rom, 0x0000, rv_jal(0, 0x20));

    /* 0x0004: Trap handler — C.J self (infinite loop) */
    rom_w16(rom, 0x0004, 0xA001);

    /* 0x0010: Magic and table pointers (RP2350 ROM header) */
    rom[0x10] = 'R'; rom[0x11] = 'P'; rom[0x12] = 0x02; rom[0x13] = 0x00;
    rom_w32(rom, 0x14, 0x0100);  /* Function table at 0x0100 */
    rom_w32(rom, 0x18, 0x0200);  /* Data table at 0x0200 */
    rom_w32(rom, 0x1C, RV_ROM_FN_TABLE_LOOKUP); /* Lookup function */

    /* 0x0020: Boot code */
    uint32_t sp_upper = sram_end & 0xFFFFF000;
    int32_t sp_lower = (int32_t)(sram_end & 0xFFF);
    if (sp_lower & 0x800) { sp_upper += 0x1000; sp_lower -= 0x1000; }

    uint32_t pc = 0x20;
    /* Set SP (x2) */
    rom_w32(rom, pc, rv_lui(2, sp_upper)); pc += 4;
    if (sp_lower != 0) {
        rom_w32(rom, pc, rv_addi(2, 2, sp_lower)); pc += 4;
    }
    /* Set GP (x3) to SRAM base */
    rom_w32(rom, pc, rv_lui(3, RP2350_SRAM_BASE)); pc += 4;
    /* Set mtvec to 0x0004 (trap handler) — CSRRW x0, mtvec, t0 */
    rom_w32(rom, pc, rv_addi(5, 0, 4)); pc += 4;       /* t0 = 4 */
    rom_w32(rom, pc, rv_csrrw(0, 0x305, 5)); pc += 4;  /* mtvec = t0 */
    /* Jump to flash */
    rom_w32(rom, pc, rv_lui(5, flash_base)); pc += 4;
    rom_w32(rom, pc, rv_jalr(0, 5, 0)); pc += 4;

    /* 0x0100: Function table entries [32-bit code, 32-bit address] */
    struct { uint32_t code; uint32_t addr; } fn_table[] = {
        { RV_ROM_CODE_MEMCPY,      RV_ROM_FN_MEMCPY },
        { RV_ROM_CODE_MEMSET,      RV_ROM_FN_MEMSET },
        { RV_ROM_CODE_POPCOUNT,    RV_ROM_FN_POPCOUNT32 },
        { RV_ROM_CODE_CLZ,         RV_ROM_FN_CLZ32 },
        { RV_ROM_CODE_CTZ,         RV_ROM_FN_CTZ32 },
        { RV_ROM_CODE_REVERSE,     RV_ROM_FN_REVERSE32 },
        { RV_ROM_CODE_FLASH_ERASE, RV_ROM_FN_FLASH_ERASE },
        { RV_ROM_CODE_FLASH_PROG,  RV_ROM_FN_FLASH_PROGRAM },
        { RV_ROM_CODE_REBOOT,      RV_ROM_FN_REBOOT },
        { 0, 0 }  /* End sentinel */
    };
    uint32_t tbl = 0x0100;
    for (int i = 0; fn_table[i].code != 0; i++) {
        rom_w32(rom, tbl, fn_table[i].code);
        rom_w32(rom, tbl + 4, fn_table[i].addr);
        tbl += 8;
    }
    rom_w32(rom, tbl, 0);  /* End sentinel */

    /* 0x0200: Data table (empty for now) */
    rom_w32(rom, 0x0200, 0);

    /* 0x0300: Table lookup function (RISC-V code)
     * a0 = table pointer, a1 = code to find
     * Returns function pointer in a0 (or 0 if not found)
     */
    pc = RV_ROM_FN_TABLE_LOOKUP;
    /* loop: LW t0, 0(a0) — load entry code */
    rom_w32(rom, pc, 0x00052283); pc += 4;  /* lw t0, 0(a0) */
    /* BEQ t0, x0, not_found (+20) */
    rom_w32(rom, pc, 0x00028a63); pc += 4;  /* beq t0, zero, +20 */
    /* BEQ t0, a1, found (+12) */
    rom_w32(rom, pc, 0x00b28663); pc += 4;  /* beq t0, a1, +12 */
    /* ADDI a0, a0, 8 — next entry */
    rom_w32(rom, pc, 0x00850513); pc += 4;  /* addi a0, a0, 8 */
    /* JAL x0, loop (-16) */
    rom_w32(rom, pc, rv_jal(0, -16)); pc += 4;
    /* found: LW a0, 4(a0) — load function pointer */
    rom_w32(rom, pc, 0x00452503); pc += 4;  /* lw a0, 4(a0) */
    /* RET */
    rom_w32(rom, pc, rv_ret()); pc += 4;
    /* not_found: ADDI a0, x0, 0 */
    rom_w32(rom, pc, rv_addi(10, 0, 0)); pc += 4;
    /* RET */
    rom_w32(rom, pc, rv_ret()); pc += 4;

    /* 0x0400+: Function stubs — all are simple RET (intercepted by rv_rom_intercept) */
    for (uint32_t addr = RV_ROM_FN_MEMCPY; addr < RV_ROM_FN_LAST && addr + 4 <= rom_size; addr += 4) {
        rom_w32(rom, addr, rv_ret());
    }

    fprintf(stderr, "[RV-BOOT] ROM initialized: SP=0x%08X, entry=0x%08X, %d ROM functions\n",
            sram_end, flash_base, (int)(sizeof(fn_table)/sizeof(fn_table[0]) - 1));

    return 0x00000000;
}

/* ========================================================================
 * ROM Function Interception
 *
 * When PC hits a ROM function stub, perform the operation natively in C
 * and return to caller. Arguments in a0-a2 (x10-x12), result in a0 (x10).
 * ======================================================================== */

int rv_rom_intercept(rv_cpu_state_t *cpu) {
    uint32_t pc = cpu->pc;
    if (pc < RV_ROM_FN_MEMCPY || pc >= RV_ROM_FN_LAST) return 0;

    rv_membus_state_t *bus = (rv_membus_state_t *)cpu->bus;
    if (!bus) return 0;

    uint32_t a0 = cpu->x[10];
    uint32_t a1 = cpu->x[11];
    uint32_t a2 = cpu->x[12];

    switch (pc) {
    case RV_ROM_FN_MEMCPY:
    case RV_ROM_FN_MEMCPY4: {
        /* memcpy(dst, src, len) — a0=dst, a1=src, a2=len */
        for (uint32_t i = 0; i < a2; i++) {
            uint8_t b = rv_mem_read8(bus, a1 + i);
            rv_mem_write8(bus, a0 + i, b);
        }
        /* Return dst in a0 (already there) */
        break;
    }

    case RV_ROM_FN_MEMSET:
    case RV_ROM_FN_MEMSET4: {
        /* memset(dst, val, len) — a0=dst, a1=val, a2=len */
        uint8_t val = (uint8_t)(a1 & 0xFF);
        for (uint32_t i = 0; i < a2; i++) {
            rv_mem_write8(bus, a0 + i, val);
        }
        break;
    }

    case RV_ROM_FN_POPCOUNT32:
        cpu->x[10] = (uint32_t)__builtin_popcount(a0);
        break;

    case RV_ROM_FN_CLZ32:
        cpu->x[10] = a0 ? (uint32_t)__builtin_clz(a0) : 32;
        break;

    case RV_ROM_FN_CTZ32:
        cpu->x[10] = a0 ? (uint32_t)__builtin_ctz(a0) : 32;
        break;

    case RV_ROM_FN_REVERSE32: {
        uint32_t v = a0;
        v = ((v >> 1) & 0x55555555) | ((v & 0x55555555) << 1);
        v = ((v >> 2) & 0x33333333) | ((v & 0x33333333) << 2);
        v = ((v >> 4) & 0x0F0F0F0F) | ((v & 0x0F0F0F0F) << 4);
        v = ((v >> 8) & 0x00FF00FF) | ((v & 0x00FF00FF) << 8);
        v = (v >> 16) | (v << 16);
        cpu->x[10] = v;
        break;
    }

    case RV_ROM_FN_FLASH_ENTER:
    case RV_ROM_FN_FLASH_EXIT:
        /* Flash connect/exit XIP — no-op in emulator */
        break;

    case RV_ROM_FN_FLASH_ERASE: {
        /* flash_range_erase(offset, count) — a0=offset, a1=count */
        uint32_t offset = a0;
        uint32_t count = a1;
        if (offset + count <= bus->flash_size) {
            memset(&bus->flash[offset], 0xFF, count);
            if (cpu->debug_enabled)
                fprintf(stderr, "[RV-ROM] flash_range_erase(0x%X, %u)\n", offset, count);
        }
        break;
    }

    case RV_ROM_FN_FLASH_PROGRAM: {
        /* flash_range_program(offset, data, count) — a0=offset, a1=data_ptr, a2=count */
        uint32_t offset = a0;
        uint32_t count = a2;
        if (offset + count <= bus->flash_size) {
            for (uint32_t i = 0; i < count; i++) {
                bus->flash[offset + i] = rv_mem_read8(bus, a1 + i);
            }
            if (cpu->debug_enabled)
                fprintf(stderr, "[RV-ROM] flash_range_program(0x%X, %u bytes)\n", offset, count);
        }
        break;
    }

    case RV_ROM_FN_REBOOT:
        fprintf(stderr, "[RV-ROM] Reboot requested\n");
        cpu->is_halted = 1;
        break;

    case RV_ROM_FN_SET_STACK:
        cpu->x[2] = a0;  /* Set SP */
        break;

    default:
        return 0;  /* Not a recognized ROM function */
    }

    /* Return to caller: PC = ra (x1) */
    cpu->pc = cpu->x[1];
    cpu->x[0] = 0;
    cpu->step_count++;
    cpu->cycle_count++;
    cpu->instret_count++;
    return 1;
}
