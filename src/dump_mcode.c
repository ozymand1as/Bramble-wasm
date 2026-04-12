#include <stdio.h>
#include <stdint.h>
#include "emulator.h"

void dump_mcode(uint32_t addr, int count) {
    if (__builtin_expect(mem_debug_unmapped, 0)) {
        printf("Dumping %d bytes at 0x%08X:\n", count, addr);
        for (int i = 0; i < count; i += 2) {
            printf("0x%08X: %04X\n", addr + i, mem_read16(addr + i));
        }
    }
}
