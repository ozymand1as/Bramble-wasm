#include <string.h>
#include <stdio.h>
#include <math.h>
#include "rom.h"
#include "emulator.h"
#include "storage.h"

/* ROM image buffer */
uint8_t rom_image[ROM_SIZE];

/*
 * ROM Layout:
 *   0x0010: Magic ('M', 'u', 0x01)
 *   0x0014: 16-bit pointer to function table (0x0100)
 *   0x0016: 16-bit pointer to data table (0x0180)
 *   0x0018: 16-bit pointer to lookup function (0x0201, Thumb bit set)
 *   0x0100: Function table entries [16-bit code, 16-bit func_ptr] ...
 *   0x0180: Data table entries [16-bit code, 16-bit data_ptr] ...
 *   0x0200: Lookup function (Thumb code)
 *   0x0300: memcpy (Thumb code)
 *   0x0320: memset (Thumb code)
 *   0x0340: popcount32 (Thumb code)
 *   0x0360: clz32 (Thumb code)
 *   0x0380: ctz32 (Thumb code)
 *   0x03A0: reverse32 stub (returns 0)
 *   0x03B0: Flash function stubs
 *   0x0400: soft_float_table (array of 16-bit function pointers)
 *   0x0440: soft_double_table (array of 16-bit function pointers)
 *   0x0500: Float function stubs (BX LR, intercepted by cpu_step)
 *   0x0540: Double function stubs (BX LR, intercepted by cpu_step)
 */

/* Helper: write a 16-bit value at a ROM offset (little-endian) */
static void rom_write16(uint32_t offset, uint16_t val) {
    rom_image[offset]     = val & 0xFF;
    rom_image[offset + 1] = (val >> 8) & 0xFF;
}

/* Place Thumb code for the table lookup function at ROM offset 0x0200.
 *
 * rom_table_lookup(uint16_t *table, uint32_t code):
 *   r0 = table pointer, r1 = code to find
 *   Returns function pointer in r0 (or 0 if not found)
 *
 * 0x0200: ldrh r2, [r0, #0]     ; Load entry code
 * 0x0202: cmp  r2, #0           ; End of table?
 * 0x0204: beq  not_found        ; -> 0x0212
 * 0x0206: cmp  r2, r1           ; Match?
 * 0x0208: beq  found            ; -> 0x020E
 * 0x020A: adds r0, #4           ; Next entry
 * 0x020C: b    loop             ; -> 0x0200
 * found:
 * 0x020E: ldrh r0, [r0, #2]     ; Load function pointer
 * 0x0210: bx   lr
 * not_found:
 * 0x0212: movs r0, #0
 * 0x0214: bx   lr
 */
static void rom_place_lookup_fn(void) {
    rom_write16(0x0200, 0x8802);  /* ldrh r2, [r0, #0] */
    rom_write16(0x0202, 0x2A00);  /* cmp r2, #0 */
    rom_write16(0x0204, 0xD005);  /* beq +5 -> 0x0212 */
    rom_write16(0x0206, 0x428A);  /* cmp r2, r1 */
    rom_write16(0x0208, 0xD001);  /* beq +1 -> 0x020E */
    rom_write16(0x020A, 0x3004);  /* adds r0, #4 */
    rom_write16(0x020C, 0xE7F8);  /* b -8 -> 0x0200 */
    rom_write16(0x020E, 0x8840);  /* ldrh r0, [r0, #2] */
    rom_write16(0x0210, 0x4770);  /* bx lr */
    rom_write16(0x0212, 0x2000);  /* movs r0, #0 */
    rom_write16(0x0214, 0x4770);  /* bx lr */
}

/* memcpy at 0x0300: r0=dst, r1=src, r2=count (bytes). Returns r0=dst. */
static void rom_place_memcpy(void) {
    rom_write16(0x0300, 0xB510);  /* push {r4, lr} */
    rom_write16(0x0302, 0x0003);  /* lsls r3, r0, #0 (movs r3, r0) */
    rom_write16(0x0304, 0x2A00);  /* cmp r2, #0 */
    rom_write16(0x0306, 0xDD05);  /* ble +5 -> 0x0314 */
    rom_write16(0x0308, 0x780C);  /* ldrb r4, [r1, #0] */
    rom_write16(0x030A, 0x701C);  /* strb r4, [r3, #0] */
    rom_write16(0x030C, 0x3301);  /* adds r3, #1 */
    rom_write16(0x030E, 0x3101);  /* adds r1, #1 */
    rom_write16(0x0310, 0x3A01);  /* subs r2, #1 */
    rom_write16(0x0312, 0xDCF9);  /* bgt -7 -> 0x0308 */
    rom_write16(0x0314, 0xBD10);  /* pop {r4, pc} */
}

/* memset at 0x0320: r0=dst, r1=value, r2=count (bytes). Returns r0=dst. */
static void rom_place_memset(void) {
    rom_write16(0x0320, 0x0003);  /* lsls r3, r0, #0 (movs r3, r0) */
    rom_write16(0x0322, 0x2A00);  /* cmp r2, #0 */
    rom_write16(0x0324, 0xDD03);  /* ble +3 -> 0x032E */
    rom_write16(0x0326, 0x7019);  /* strb r1, [r3, #0] */
    rom_write16(0x0328, 0x3301);  /* adds r3, #1 */
    rom_write16(0x032A, 0x3A01);  /* subs r2, #1 */
    rom_write16(0x032C, 0xDCFB);  /* bgt -5 -> 0x0326 */
    rom_write16(0x032E, 0x4770);  /* bx lr */
}

/* popcount32 at 0x0340 */
static void rom_place_popcount(void) {
    rom_write16(0x0340, 0x2100);
    rom_write16(0x0342, 0x2800);
    rom_write16(0x0344, 0xD004);
    rom_write16(0x0346, 0x0002);
    rom_write16(0x0348, 0x3A01);
    rom_write16(0x034A, 0x4010);
    rom_write16(0x034C, 0x3101);
    rom_write16(0x034E, 0xE7F8);
    rom_write16(0x0350, 0x0008);
    rom_write16(0x0352, 0x4770);
}

/* clz32 at 0x0360 */
static void rom_place_clz(void) {
    rom_write16(0x0360, 0x2100);
    rom_write16(0x0362, 0x2800);
    rom_write16(0x0364, 0xD005);
    rom_write16(0x0366, 0x0040);
    rom_write16(0x0368, 0xD201);
    rom_write16(0x036A, 0x3101);
    rom_write16(0x036C, 0xE7FB);
    rom_write16(0x036E, 0x0008);
    rom_write16(0x0370, 0x4770);
    rom_write16(0x0372, 0x2020);
    rom_write16(0x0374, 0x4770);
}

/* ctz32 at 0x0380 */
static void rom_place_ctz(void) {
    rom_write16(0x0380, 0x2100);
    rom_write16(0x0382, 0x2800);
    rom_write16(0x0384, 0xD007);
    rom_write16(0x0386, 0x2201);
    rom_write16(0x0388, 0x4210);
    rom_write16(0x038A, 0xD102);
    rom_write16(0x038C, 0x0840);
    rom_write16(0x038E, 0x3101);
    rom_write16(0x0390, 0xE7F9);
    rom_write16(0x0392, 0x0008);
    rom_write16(0x0394, 0x4770);
    rom_write16(0x0396, 0x2020);
    rom_write16(0x0398, 0x4770);
}

/* reverse32 stub at 0x03A0: returns 0 (not commonly needed) */
static void rom_place_reverse_stub(void) {
    rom_write16(0x03A0, 0x2000);
    rom_write16(0x03A2, 0x4770);
}

/* Flash function stubs.
 * connect_internal_flash, flash_exit_xip, flash_flush_cache,
 * flash_enter_cmd_xip: just 'bx lr' (no-ops).
 * flash_range_erase and flash_range_program: also 'bx lr' but
 * intercepted by rom_intercept() to modify cpu.flash[]. */
static void rom_place_flash_stubs(void) {
    for (uint32_t addr = 0x03B0; addr <= 0x03C4; addr += 4) {
        rom_write16(addr, 0x4770);  /* bx lr */
    }
}

/* ========================================================================
 * Soft-Float / Soft-Double Function Tables and Stubs
 *
 * The SDK looks up these tables via rom_data_lookup('SF'/'SD').
 * Each table is an array of 16-bit function pointers in ROM.
 * The actual function stubs are BX LR, but rom_intercept()
 * catches execution at these addresses and performs the operation
 * natively in C using host float/double.
 * ======================================================================== */

static void rom_place_float_double_tables(void) {
    /* soft_float_table at 0x0400: array of 16-bit pointers */
    for (int i = 0; i < ROM_FLOAT_FUNC_COUNT; i++) {
        uint16_t addr = ROM_FLOAT_FUNC_BASE + (i * 2);
        rom_write16(0x0400 + i * 2, addr | 1);  /* Thumb bit */
    }

    /* soft_double_table at 0x0440: array of 16-bit pointers */
    for (int i = 0; i < ROM_DOUBLE_FUNC_COUNT; i++) {
        uint16_t addr = ROM_DOUBLE_FUNC_BASE + (i * 2);
        rom_write16(0x0440 + i * 2, addr | 1);  /* Thumb bit */
    }

    /* Place BX LR at each float stub address */
    for (int i = 0; i < ROM_FLOAT_FUNC_COUNT; i++) {
        rom_write16(ROM_FLOAT_FUNC_BASE + i * 2, 0x4770);
    }

    /* Place BX LR at each double stub address */
    for (int i = 0; i < ROM_DOUBLE_FUNC_COUNT; i++) {
        rom_write16(ROM_DOUBLE_FUNC_BASE + i * 2, 0x4770);
    }
}

/* Build the function table at 0x0100.
 * Each entry: [16-bit code][16-bit func_ptr with Thumb bit] */
static void rom_build_func_table(void) {
    uint32_t off = 0x0100;

    rom_write16(off, ROM_FUNC_MEMCPY);    rom_write16(off + 2, 0x0301); off += 4;
    rom_write16(off, ROM_FUNC_MEMCPY44);  rom_write16(off + 2, 0x0301); off += 4;
    rom_write16(off, ROM_FUNC_MEMSET);    rom_write16(off + 2, 0x0321); off += 4;
    rom_write16(off, ROM_FUNC_MEMSET4);   rom_write16(off + 2, 0x0321); off += 4;
    rom_write16(off, ROM_FUNC_POPCOUNT32); rom_write16(off + 2, 0x0341); off += 4;
    rom_write16(off, ROM_FUNC_CLZ32);     rom_write16(off + 2, 0x0361); off += 4;
    rom_write16(off, ROM_FUNC_CTZ32);     rom_write16(off + 2, 0x0381); off += 4;
    rom_write16(off, ROM_FUNC_REVERSE32); rom_write16(off + 2, 0x03A1); off += 4;
    rom_write16(off, ROM_FUNC_CONNECT_INTERNAL_FLASH); rom_write16(off + 2, 0x03B1); off += 4;
    rom_write16(off, ROM_FUNC_FLASH_EXIT_XIP);         rom_write16(off + 2, 0x03B5); off += 4;
    rom_write16(off, ROM_FUNC_FLASH_RANGE_ERASE);      rom_write16(off + 2, 0x03B9); off += 4;
    rom_write16(off, ROM_FUNC_FLASH_RANGE_PROGRAM);    rom_write16(off + 2, 0x03BD); off += 4;
    rom_write16(off, ROM_FUNC_FLASH_FLUSH_CACHE);      rom_write16(off + 2, 0x03C1); off += 4;
    rom_write16(off, ROM_FUNC_FLASH_ENTER_CMD_XIP);    rom_write16(off + 2, 0x03C5); off += 4;
    /* End marker */
    rom_write16(off, 0x0000);             rom_write16(off + 2, 0x0000);
}

/* Build data table at 0x0180 with soft_float and soft_double entries */
static void rom_build_data_table(void) {
    uint32_t off = 0x0180;

    /* soft_float_table ('SF') -> 0x0400 */
    rom_write16(off, ROM_DATA_SOFT_FLOAT);  rom_write16(off + 2, 0x0400); off += 4;
    /* soft_double_table ('SD') -> 0x0440 */
    rom_write16(off, ROM_DATA_SOFT_DOUBLE); rom_write16(off + 2, 0x0440); off += 4;
    /* End marker */
    rom_write16(off, 0x0000);               rom_write16(off + 2, 0x0000);
}

/* Initialize ROM image */
void rom_init(void) {
    memset(rom_image, 0, ROM_SIZE);

    /* Magic at offset 0x10: 'M', 'u', version=1 */
    rom_image[0x10] = 'M';
    rom_image[0x11] = 'u';
    rom_image[0x12] = 0x01;

    /* Pointers at 0x14/0x16/0x18 */
    rom_write16(ROM_FUNC_TABLE_PTR, 0x0100);
    rom_write16(ROM_DATA_TABLE_PTR, 0x0180);
    rom_write16(ROM_LOOKUP_FN_PTR,  0x0201);

    /* Build tables */
    rom_build_func_table();
    rom_build_data_table();

    /* Place Thumb code stubs */
    rom_place_lookup_fn();
    rom_place_memcpy();
    rom_place_memset();
    rom_place_popcount();
    rom_place_clz();
    rom_place_ctz();
    rom_place_reverse_stub();
    rom_place_flash_stubs();
    rom_place_float_double_tables();

    fprintf(stderr, "[ROM] Initialized function table with 14 entries + float/double tables\n");
}

/* ========================================================================
 * ROM Function Interception
 *
 * Called from cpu_step before executing an instruction at a ROM address.
 * Handles: float/double operations (native C), flash erase/program.
 * Returns 1 if intercepted (caller should skip normal execution).
 * ======================================================================== */

/* Helper: reinterpret uint32_t as float */
static inline float u2f(uint32_t u) { float f; memcpy(&f, &u, 4); return f; }
static inline uint32_t f2u(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }

/* Helper: reinterpret uint32_t pair as double */
static inline double u2d(uint32_t lo, uint32_t hi) {
    uint64_t bits = (uint64_t)hi << 32 | lo;
    double d; memcpy(&d, &bits, 8); return d;
}
static inline void d2u(double d, uint32_t *lo, uint32_t *hi) {
    uint64_t bits; memcpy(&bits, &d, 8);
    *lo = (uint32_t)bits;
    *hi = (uint32_t)(bits >> 32);
}

static int rom_intercept_float(int idx) {
    float a, b, result;

    switch (idx) {
    case ROM_FLOAT_FADD:
        a = u2f(cpu.r[0]); b = u2f(cpu.r[1]);
        cpu.r[0] = f2u(a + b);
        return 1;
    case ROM_FLOAT_FSUB:
        a = u2f(cpu.r[0]); b = u2f(cpu.r[1]);
        cpu.r[0] = f2u(a - b);
        return 1;
    case ROM_FLOAT_FMUL:
        a = u2f(cpu.r[0]); b = u2f(cpu.r[1]);
        cpu.r[0] = f2u(a * b);
        return 1;
    case ROM_FLOAT_FDIV:
        a = u2f(cpu.r[0]); b = u2f(cpu.r[1]);
        cpu.r[0] = f2u(a / b);
        return 1;
    case ROM_FLOAT_FSQRT:
        a = u2f(cpu.r[0]);
        cpu.r[0] = f2u(sqrtf(a));
        return 1;
    case ROM_FLOAT_FLOAT2INT:
        a = u2f(cpu.r[0]);
        cpu.r[0] = (uint32_t)(int32_t)a;
        return 1;
    case ROM_FLOAT_FLOAT2FIX:
        a = u2f(cpu.r[0]);
        result = a * (float)(1 << cpu.r[1]);
        cpu.r[0] = (uint32_t)(int32_t)result;
        return 1;
    case ROM_FLOAT_FLOAT2UINT:
        a = u2f(cpu.r[0]);
        cpu.r[0] = (a < 0) ? 0 : (uint32_t)a;
        return 1;
    case ROM_FLOAT_FLOAT2UFIX:
        a = u2f(cpu.r[0]);
        result = a * (float)(1 << cpu.r[1]);
        cpu.r[0] = (result < 0) ? 0 : (uint32_t)result;
        return 1;
    case ROM_FLOAT_INT2FLOAT:
        cpu.r[0] = f2u((float)(int32_t)cpu.r[0]);
        return 1;
    case ROM_FLOAT_FIX2FLOAT:
        cpu.r[0] = f2u((float)(int32_t)cpu.r[0] / (float)(1 << cpu.r[1]));
        return 1;
    case ROM_FLOAT_UINT2FLOAT:
        cpu.r[0] = f2u((float)cpu.r[0]);
        return 1;
    case ROM_FLOAT_UFIX2FLOAT:
        cpu.r[0] = f2u((float)cpu.r[0] / (float)(1 << cpu.r[1]));
        return 1;
    case ROM_FLOAT_FCOS:
        cpu.r[0] = f2u(cosf(u2f(cpu.r[0])));
        return 1;
    case ROM_FLOAT_FSIN:
        cpu.r[0] = f2u(sinf(u2f(cpu.r[0])));
        return 1;
    case ROM_FLOAT_FTAN:
        cpu.r[0] = f2u(tanf(u2f(cpu.r[0])));
        return 1;
    case ROM_FLOAT_FEXP:
        cpu.r[0] = f2u(expf(u2f(cpu.r[0])));
        return 1;
    case ROM_FLOAT_FLN:
        cpu.r[0] = f2u(logf(u2f(cpu.r[0])));
        return 1;
    default:
        return 0;
    }
}

static int rom_intercept_double(int idx) {
    double a, b, result;

    switch (idx) {
    case ROM_FLOAT_FADD:
        a = u2d(cpu.r[0], cpu.r[1]); b = u2d(cpu.r[2], cpu.r[3]);
        d2u(a + b, &cpu.r[0], &cpu.r[1]);
        return 1;
    case ROM_FLOAT_FSUB:
        a = u2d(cpu.r[0], cpu.r[1]); b = u2d(cpu.r[2], cpu.r[3]);
        d2u(a - b, &cpu.r[0], &cpu.r[1]);
        return 1;
    case ROM_FLOAT_FMUL:
        a = u2d(cpu.r[0], cpu.r[1]); b = u2d(cpu.r[2], cpu.r[3]);
        d2u(a * b, &cpu.r[0], &cpu.r[1]);
        return 1;
    case ROM_FLOAT_FDIV:
        a = u2d(cpu.r[0], cpu.r[1]); b = u2d(cpu.r[2], cpu.r[3]);
        d2u(a / b, &cpu.r[0], &cpu.r[1]);
        return 1;
    case ROM_FLOAT_FSQRT:
        a = u2d(cpu.r[0], cpu.r[1]);
        d2u(sqrt(a), &cpu.r[0], &cpu.r[1]);
        return 1;
    case ROM_FLOAT_FLOAT2INT:
        a = u2d(cpu.r[0], cpu.r[1]);
        cpu.r[0] = (uint32_t)(int32_t)a;
        return 1;
    case ROM_FLOAT_FLOAT2FIX:
        a = u2d(cpu.r[0], cpu.r[1]);
        result = a * (double)(1u << cpu.r[2]);
        cpu.r[0] = (uint32_t)(int32_t)result;
        return 1;
    case ROM_FLOAT_FLOAT2UINT:
        a = u2d(cpu.r[0], cpu.r[1]);
        cpu.r[0] = (a < 0) ? 0 : (uint32_t)a;
        return 1;
    case ROM_FLOAT_FLOAT2UFIX:
        a = u2d(cpu.r[0], cpu.r[1]);
        result = a * (double)(1u << cpu.r[2]);
        cpu.r[0] = (result < 0) ? 0 : (uint32_t)result;
        return 1;
    case ROM_FLOAT_INT2FLOAT:
        d2u((double)(int32_t)cpu.r[0], &cpu.r[0], &cpu.r[1]);
        return 1;
    case ROM_FLOAT_FIX2FLOAT:
        d2u((double)(int32_t)cpu.r[0] / (double)(1u << cpu.r[1]),
            &cpu.r[0], &cpu.r[1]);
        return 1;
    case ROM_FLOAT_UINT2FLOAT:
        d2u((double)cpu.r[0], &cpu.r[0], &cpu.r[1]);
        return 1;
    case ROM_FLOAT_UFIX2FLOAT:
        d2u((double)cpu.r[0] / (double)(1u << cpu.r[1]),
            &cpu.r[0], &cpu.r[1]);
        return 1;
    case ROM_FLOAT_FCOS:
        d2u(cos(u2d(cpu.r[0], cpu.r[1])), &cpu.r[0], &cpu.r[1]);
        return 1;
    case ROM_FLOAT_FSIN:
        d2u(sin(u2d(cpu.r[0], cpu.r[1])), &cpu.r[0], &cpu.r[1]);
        return 1;
    case ROM_FLOAT_FTAN:
        d2u(tan(u2d(cpu.r[0], cpu.r[1])), &cpu.r[0], &cpu.r[1]);
        return 1;
    case ROM_FLOAT_FEXP:
        d2u(exp(u2d(cpu.r[0], cpu.r[1])), &cpu.r[0], &cpu.r[1]);
        return 1;
    case ROM_FLOAT_FLN:
        d2u(log(u2d(cpu.r[0], cpu.r[1])), &cpu.r[0], &cpu.r[1]);
        return 1;
    default:
        return 0;
    }
}

static int rom_intercept_flash(uint32_t pc) {
    if (pc == ROM_FLASH_RANGE_ERASE_ADDR) {
        uint32_t offs = cpu.r[0];
        uint32_t count = cpu.r[1];
        if (offs + count <= FLASH_SIZE) {
            memset(&cpu.flash[offs], 0xFF, count);
            flash_persist_sync(offs, count);
        }
        return 1;
    }

    if (pc == ROM_FLASH_RANGE_PROGRAM_ADDR) {
        uint32_t offs = cpu.r[0];
        uint32_t src = cpu.r[1];
        uint32_t count = cpu.r[2];
        if (offs + count <= FLASH_SIZE) {
            for (uint32_t i = 0; i < count; i++) {
                cpu.flash[offs + i] = mem_read8(src + i);
            }
            flash_persist_sync(offs, count);
        }
        return 1;
    }

    return 0;
}

int rom_intercept(uint32_t pc) {
    /* Float function interception */
    if (pc >= ROM_FLOAT_FUNC_BASE &&
        pc < ROM_FLOAT_FUNC_BASE + ROM_FLOAT_FUNC_COUNT * 2) {
        int idx = (pc - ROM_FLOAT_FUNC_BASE) / 2;
        if (rom_intercept_float(idx)) {
            /* Simulate BX LR */
            cpu.r[15] = cpu.r[14] & ~1u;
            cpu.step_count++;
            return 1;
        }
    }

    /* Double function interception */
    if (pc >= ROM_DOUBLE_FUNC_BASE &&
        pc < ROM_DOUBLE_FUNC_BASE + ROM_DOUBLE_FUNC_COUNT * 2) {
        int idx = (pc - ROM_DOUBLE_FUNC_BASE) / 2;
        if (rom_intercept_double(idx)) {
            cpu.r[15] = cpu.r[14] & ~1u;
            cpu.step_count++;
            return 1;
        }
    }

    /* Flash function interception */
    if (pc == ROM_FLASH_RANGE_ERASE_ADDR ||
        pc == ROM_FLASH_RANGE_PROGRAM_ADDR) {
        if (rom_intercept_flash(pc)) {
            /* Let the BX LR execute normally */
            return 0;
        }
    }

    return 0;
}

/* Read from ROM */
uint32_t rom_read32(uint32_t addr) {
    uint32_t off = addr & (ROM_SIZE - 1);
    if (off + 3 < ROM_SIZE) {
        return rom_image[off] |
               ((uint32_t)rom_image[off + 1] << 8) |
               ((uint32_t)rom_image[off + 2] << 16) |
               ((uint32_t)rom_image[off + 3] << 24);
    }
    return 0;
}

uint16_t rom_read16(uint32_t addr) {
    uint32_t off = addr & (ROM_SIZE - 1);
    if (off + 1 < ROM_SIZE) {
        return rom_image[off] | ((uint16_t)rom_image[off + 1] << 8);
    }
    return 0;
}

uint8_t rom_read8(uint32_t addr) {
    uint32_t off = addr & (ROM_SIZE - 1);
    if (off < ROM_SIZE) {
        return rom_image[off];
    }
    return 0;
}
